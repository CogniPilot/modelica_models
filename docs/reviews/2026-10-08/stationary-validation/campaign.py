import concurrent.futures
import hashlib
import itertools
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np

repository=Path.cwd()
root=Path.home()/'scratch/modelica_models/stationary-independent'
snapshot=root/'tools'
sys.path.insert(0,str(snapshot))
from generate import generate
from flow_exposure import generate as exposure, integrate_windows
from check_preintegration_noise import rotation
from score import read
from compare_exposure import eskf
from check_joint_replay_covariance import check as check_joint

sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
plan=json.loads((root/'declaration.json').read_text())['held_out_plan']
conditions=list(itertools.product(plan['seeds'],plan['speeds'],plan['heights_m']))
python=sys.executable
reference=repository/'docs/reviews/2026-10-07/native-releases/native-exposure.json'
native_reference=repository/'docs/reviews/2026-10-08/native-innovations/native-innovations.json'
harness=Path.home()/'scratch/estimator-comparison/matched-sensor-replay/harness'
innovation=Path.home()/'scratch/modelica_models/native-innovations'
covariance=Path.home()/'scratch/modelica_models/native-consistency'
build=Path.home()/'scratch/modelica_models/common-sensor-noise/build'
cxx='/nix/store/l5qkpzsr4gxvksh45b3nhxbkyr5cviar-gcc-wrapper-15.3.0/bin/g++'


def command(script, arguments):
    result=[python,str(snapshot/script)]
    for key,value in arguments.items():
        result.extend(['--'+key,str(value)])
    return result


def audit(capture):
    truth=read(capture/'truth.csv')
    attitude=np.stack([rotation(np.array([r[k] for k in ('qw','qx','qy','qz')])) for r in truth])
    velocity=np.column_stack([truth[k] for k in ('ve_m_s','vn_m_s','vu_m_s')])
    body_velocity=np.einsum('nji,nj->ni',attitude,velocity)
    distance=(truth['u_m']+1)/attitude[:,2,2]
    rates=np.column_stack((-body_velocity[:,1],body_velocity[:,0]))/distance[:,None]
    flow=read(capture/'flow.csv')
    ideal=integrate_windows(truth['t_s'],rates,flow['t_s'],0.1)
    measured=np.column_stack([flow['integrated_los_'+axis+'_rad']+flow['integrated_gyro_'+axis+'_rad'] for axis in ('x','y')])
    error=(measured-ideal)/0.1
    np.testing.assert_allclose(np.std(error,axis=0),.05,rtol=.1)
    np.testing.assert_allclose(np.mean(error,axis=0),0,atol=.007)
    declared=np.column_stack([flow['los_variance_'+a+'_rad2']+flow['gyro_variance_'+a+'_rad2'] for a in ('x','y')])/.1**2
    np.testing.assert_allclose(declared,.05**2,rtol=1e-10)
    gps=read(capture/'gps.csv')
    valid=gps['t_s']>=plan['gps_fix_after_s']
    assert np.all(gps['pos_valid'][~valid]==0) and np.all(gps['pos_valid'][valid]==1)
    indices=np.searchsorted(truth['t_s'],gps['t_s'])
    np.testing.assert_allclose(truth['t_s'][indices],gps['t_s'],rtol=0,atol=1e-8)
    measured_velocity=np.column_stack((gps['ve_m_s'],gps['vn_m_s'],-gps['vd_m_s']))
    gps_error=measured_velocity-velocity[indices]
    np.testing.assert_allclose(np.std(gps_error[valid],axis=0),[.05,.05,.075],rtol=.1)
    return dict(independent_world_velocity_camera_std_rad_s=np.std(error,axis=0).tolist(),independent_world_velocity_camera_mean_rad_s=np.mean(error,axis=0).tolist(),gps_velocity_error_std_m_s=np.std(gps_error[valid],axis=0).tolist(),input_sha256={p.name:sha(p) for p in capture.iterdir() if p.is_file()})


def prepare():
    cases=[]
    for seed,speed,height in conditions:
        name=f'{seed}-{speed:g}-{height:g}m'
        work=root/name
        work.mkdir()
        generate(work/'raw',seed=seed,speed=speed,climb_height_m=height,warmup_s=120,common_native_floors=True,gps_fix_after_s=21)
        exposure(work/'raw',work/'capture')
        audit_result=audit(work/'capture')
        case=dict(name=name,seed=seed,speed=speed,height_m=height,audit=audit_result)
        cases.append(case)
        (root/'captures.json').write_text(json.dumps(dict(complete=False,cases=cases),indent=2)+'\n')
        print('Prepared and independently audited',name,flush=True)
    (root/'captures.json').write_text(json.dumps(dict(complete=True,cases=cases),indent=2)+'\n')


def replay(case):
    work=root/case['name']
    common={'reference':reference,'harness':harness,'cxx':cxx}
    pilot=dict(common,capture=work/'capture',horizon=build/'horizon-rest',retrodiction=build/'retrodiction-rest',**{'horizon-joint':build/'horizon-rest-joint','retrodiction-joint':build/'retrodiction-rest-joint','transport-trace':build/'transport_trace','px4-source':innovation/'px4','ardupilot-source':innovation/'ardupilot','px4-observer':innovation/'px4-observer.json','ardupilot-observer':innovation/'ardupilot-observer.json','native-reference':native_reference,'px4-library':innovation/'build/px4/libekf2.a','ap-replay':innovation/'build/ardupilot/sitl/tool/Replay'},work=work/'pilot',output=work/'pilot.json')
    with (work/'pilot.log').open('w') as log:
        subprocess.run(command('compare_readiness.py',pilot),stdout=log,stderr=subprocess.STDOUT,check=True)
    native=dict(common,pilot=work/'pilot.json',cases=work/'pilot',**{'px4-source':covariance/'px4','ardupilot-source':covariance/'ardupilot','px4-observer':covariance/'px4-observer-v2.json','ardupilot-observer':covariance/'ardupilot-observer-v4.json','px4-library':covariance/'build/px4/libekf2.a','ap-replay':covariance/'build/ardupilot/sitl/tool/Replay'},work=work/'covariance',output=work/'covariance.json')
    with (work/'covariance.log').open('w') as log:
        subprocess.run(command('compare_readiness_covariance.py',native),stdout=log,stderr=subprocess.STDOUT,check=True)
    pilot_result=json.loads((work/'pilot.json').read_text())
    native_result=json.loads((work/'covariance.json').read_text())
    destination=root/'stationary/campaign'/case['name'];destination.mkdir(parents=True)
    stationary_build=Path.home()/'scratch/modelica_models/stationary-readiness'
    binaries=json.loads((root/'stationary/build.json').read_text())
    variants={'horizon':'horizon-rest','retrodiction':'retrodiction-rest','horizon_joint':'horizon-rest-joint','retrodiction_joint':'retrodiction-rest-joint'}
    scores=[]
    for scenario in plan['scenarios']:
        capture=work/'pilot'/scenario/'capture'
        assert {k:sha(capture/k) for k in pilot_result['input_sha256']}==pilot_result['input_sha256']
        transport=next(s['transport'] for s in pilot_result['scores'] if s['scenario']==scenario)
        for name,variant in variants.items():
            replay_work=destination/scenario/name;replay_work.mkdir(parents=True)
            assert sha(stationary_build/variant)==binaries[name]['binary_sha256']
            options=SimpleNamespace(work=replay_work,arm_after_s=120,delay_profile='exposure',seed=case['seed'],imu_noise_density=pilot_result.get('imu_noise_density',json.loads(reference.read_text())['imu_noise_density']),mission_offset_s=107,measurement_noise=(1e-6,.075),accuracy_end_s=166.7,consistency_windows=pilot_result['mission']['windows'],retain_covariance=name.endswith('_joint'))
            score=eskf(stationary_build/variant,capture,scenario,options,True,transport)
            if name.endswith('_joint'):score['full16_covariance']=check_joint(replay_work/'covariance.csv',name.startswith('horizon'),117,166.7)
            scores.append(dict(name=name,scenario=scenario,**score))
            (destination/'scores.json').write_text(json.dumps(dict(complete=False,reference_sha256=sha(work/'pilot.json'),input_sha256=pilot_result['input_sha256'],scores=scores),indent=2)+'\n')
    (destination/'scores.json').write_text(json.dumps(dict(complete=True,reference_sha256=sha(work/'pilot.json'),input_sha256=pilot_result['input_sha256'],scores=scores),indent=2)+'\n')
    return dict(name=case['name'],complete=pilot_result['complete'] and native_result['complete'],readiness_qualified=pilot_result['readiness_qualified'],native_covariance_valid=native_result['covariance_valid'],pilot_sha256=sha(work/'pilot.json'),covariance_sha256=sha(work/'covariance.json'),stationary_sha256=sha(destination/'scores.json'))


if sys.argv[1]=='prepare':
    prepare()
elif sys.argv[1]=='run':
    capture_manifest=json.loads((root/'captures.json').read_text())
    assert capture_manifest['complete'] and len(capture_manifest['cases'])==16
    snapshot_manifest=json.loads((root/'source-snapshot.json').read_text())
    for name,digest in snapshot_manifest.items():assert sha(snapshot/name)==digest
    result=dict(complete=False,declared_conditions=16,declared_state_replays=288,declared_native_covariance_replays=96,source_snapshot_sha256=sha(root/'source-snapshot.json'),capture_manifest_sha256=sha(root/'captures.json'),results=[])
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        pending={executor.submit(replay,case):case['name'] for case in capture_manifest['cases']}
        for future in concurrent.futures.as_completed(pending):
            try:row=future.result()
            except Exception as error:row=dict(name=pending[future],complete=False,error=repr(error))
            result['results'].append(row)
            (root/'campaign.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(row),flush=True)
    result['complete']=all(row['complete'] for row in result['results'])
    (root/'campaign.json').write_text(json.dumps(result,indent=2)+'\n')
    candidate=dict(complete=result['complete'],planned_replays=192,conditions=[dict(condition=r['name'],complete=r['complete'],sha256=r.get('stationary_sha256'),error=r.get('error')) for r in result['results']])
    (root/'stationary/campaign.json').write_text(json.dumps(candidate,indent=2)+'\n')
else:
    raise ValueError('Choose prepare or run')
