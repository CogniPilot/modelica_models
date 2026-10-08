import hashlib,json,subprocess,sys
from pathlib import Path
from types import SimpleNamespace
repo=Path.cwd();sys.path.insert(0,str(repo/'tools/estimator_comparison'))
from compare_exposure import eskf
root=Path.home()/'scratch/modelica_models/magnetic-consider';build=Path.home()/'scratch/modelica_models/joint-barometer/build-v2';base=Path.home()/'scratch/modelica_models/ekf3-imu-integrity/fix-study';pilot=json.loads((base/'pilot.json').read_text())
reference=json.loads((repo/'docs/reviews/2026-10-08/common-sensor-noise/replay-build.json').read_text());noise=json.loads((repo/'docs/reviews/2026-10-07/native-releases/native-exposure.json').read_text())['imu_noise_density'];sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
variants={'horizon':'horizon-rest','retrodiction':'retrodiction-rest','horizon_joint':'horizon-rest-joint','retrodiction_joint':'retrodiction-rest-joint'}
results={};scores=[]
for name,variant in variants.items():
 config=reference['variants'][variant]
 for obj,expected in config['generated_object_sha256'].items():assert sha(build/obj)==expected
 deps=['Vehicles_Rdd2_NavigationEstimator','Tests_PreintegrationReplay']
 if name.startswith('horizon'):deps+=['Estimation_FusionHorizon_OutputPredictor','Estimation_FusionHorizon_AidingBuffer']
 command=['/nix/store/29qjlshvklnyr67nhpprzgb9igmsfsjj-gcc-wrapper-15.2.0/bin/gcc','-pipe','-O2',*config['definitions'],*['-I'+str(build/d/d/'ProductionCode') for d in deps],str(repo/'tools/estimator_comparison/replay.c'),str(root/'predicted.o'),*[str(build/(d+'.o')) for d in deps[1:]],str(build/'rumoca_galec_kernels.o'),'-lm','-o',str(root/variant)]
 subprocess.run(command,check=True);results[name]=dict(command=command,binary_sha256=sha(root/variant),object_sha256=sha(root/'predicted.o'))
(root/'build.json').write_text(json.dumps(results,indent=2)+'\n')
for scenario in ['gps','denied','transition']:
 capture=base/'pilot'/scenario/'capture';transport=next(s['transport'] for s in pilot['scores'] if s['scenario']==scenario)
 for name,variant in variants.items():
  work=root/'pilot'/scenario/name;work.mkdir(parents=True)
  options=SimpleNamespace(work=work,arm_after_s=120,delay_profile='exposure',seed=911,imu_noise_density=noise,mission_offset_s=107,measurement_noise=(1e-6,.075),accuracy_end_s=166.7,consistency_windows=pilot['mission']['windows'])
  score=eskf(root/variant,capture,scenario,options,True,transport);scores.append(dict(name=name,scenario=scenario,**score))
  (root/'pilot.json').write_text(json.dumps(dict(complete=False,scores=scores),indent=2)+'\n')
  original=next(s for s in pilot['scores'] if s['name']==name and s['scenario']==scenario)
  print(scenario,name, 'old',original['flight'],'new',score['flight'],flush=True)
(root/'pilot.json').write_text(json.dumps(dict(complete=True,scores=scores),indent=2)+'\n')
