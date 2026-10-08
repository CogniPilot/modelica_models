import concurrent.futures,json,hashlib,sys
from pathlib import Path
from types import SimpleNamespace
root=Path.home()/'scratch/modelica_models/stationary-readiness';sys.path.insert(0,str(root/'tools'))
from compare_exposure import eskf
from check_joint_replay_covariance import check
base=Path.home()/'scratch/modelica_models/ekf3-imu-integrity/held-out';repo=Path.cwd();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
conditions=json.loads((base/'captures.json').read_text())['cases'];noise=json.loads((repo/'docs/reviews/2026-10-07/native-releases/native-exposure.json').read_text())['imu_noise_density'];build=json.loads((root/'build.json').read_text())
variants={'horizon':'horizon-rest','retrodiction':'retrodiction-rest','horizon_joint':'horizon-rest-joint','retrodiction_joint':'retrodiction-rest-joint'}
def run(c):
 reference_path=base/c['name']/'pilot.json';reference=json.loads(reference_path.read_text());assert reference['complete'] and reference['readiness_qualified'];destination=root/'campaign'/c['name'];destination.mkdir(parents=True);scores=[]
 for scenario in ['gps','denied','transition']:
  capture=base/c['name']/'pilot'/scenario/'capture'
  assert {k:sha(capture/k) for k in reference['input_sha256']}==reference['input_sha256']
  transport=next(s['transport'] for s in reference['scores'] if s['scenario']==scenario)
  for name,variant in variants.items():
   work=destination/scenario/name;work.mkdir(parents=True)
   assert sha(root/variant)==build[name]['binary_sha256']
   options=SimpleNamespace(work=work,arm_after_s=120,delay_profile='exposure',seed=c['seed'],imu_noise_density=noise,mission_offset_s=107,measurement_noise=(1e-6,.075),accuracy_end_s=166.7,consistency_windows=reference['mission']['windows'],retain_covariance=name.endswith('_joint'))
   score=eskf(root/variant,capture,scenario,options,True,transport)
   if name.endswith('_joint'):score['full16_covariance']=check(work/'covariance.csv',name.startswith('horizon'),117,166.7)
   scores.append(dict(name=name,scenario=scenario,**score))
   (destination/'scores.json').write_text(json.dumps(dict(complete=False,reference_sha256=sha(reference_path),input_sha256=reference['input_sha256'],scores=scores),indent=2)+'\n')
 (destination/'scores.json').write_text(json.dumps(dict(complete=True,reference_sha256=sha(reference_path),input_sha256=reference['input_sha256'],scores=scores),indent=2)+'\n')
 print('Completed',c['name'],flush=True)
 return dict(condition=c['name'],complete=True,sha256=sha(destination/'scores.json'))
result=dict(complete=False,planned_replays=96,conditions=[])
with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
 pending={executor.submit(run,c):c['name'] for c in conditions}
 for future in concurrent.futures.as_completed(pending):
  try:row=future.result()
  except Exception as error:row=dict(condition=pending[future],complete=False,error=repr(error))
  result['conditions'].append(row);(root/'campaign.json').write_text(json.dumps(result,indent=2)+'\n')
result['complete']=len(result['conditions'])==len(conditions) and all(r['complete'] for r in result['conditions']);(root/'campaign.json').write_text(json.dumps(result,indent=2)+'\n')
