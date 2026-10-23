import os,time,subprocess,json,re
from pathlib import Path
root=Path('/tmp/elfin-master-pack-baseline-g3')
out=root/'docs/status/evidence/occ_owner_b1/release_pose_publication_20261023/box_probe'
out.mkdir(parents=True,exist_ok=True)
env=dict(os.environ,PHASE2_STARTUP_ONLY='1',PHASE2_RUN_OUT=str(out/'startup'))
with (out/'runner_console.log').open('w') as log:
 runner=subprocess.Popen(['bash','scripts/g3/n3_functional_run.sh'],cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT)
 try:
  deadline=time.monotonic()+180
  while time.monotonic()<deadline:
   launch=out/'startup/launch.log'
   if launch.exists() and 'Publishing pose messages on' in launch.read_text(errors='replace'):break
   if runner.poll() is not None:raise RuntimeError('startup exited before publisher')
   time.sleep(.1)
  else:raise RuntimeError('publisher startup timeout')
  request='sdf_filename: "'+str(root/'docs/status/evidence/occ_owner_b1/direct_portal_first_20261022/live_02/spawn.sdf')+'" allow_renaming: false pose {position { x: -2.03 y: 0.535 z: 0.665525 } orientation { w: 1 }}'
  (out/'create_request.txt').write_text(request+'\n')
  result=subprocess.run(['ign','service','-s','/world/airport_loading/create','--reqtype','ignition.msgs.EntityFactory','--reptype','ignition.msgs.Boolean','--timeout','3000','--req',request],capture_output=True,text=True,timeout=5)
  (out/'create_response.txt').write_text(result.stdout+result.stderr)
  if result.returncode:raise RuntimeError('create failed')
  time.sleep(.3)
  commands=[('topic_info',['ign','topic','-i','-t','/world/airport_loading/pose/info']),('raw_pose',['ign','topic','-e','-t','/world/airport_loading/pose/info','--num','3'])]
  processes=[]
  for name,cmd in commands:
   f=(out/(name+'.txt')).open('w');err=(out/(name+'.stderr')).open('w')
   processes.append((name,subprocess.Popen(cmd,stdout=f,stderr=err),f,err))
  results={}
  for name,p,f,err in processes:
   try:code=p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.terminate();code=p.wait(timeout=2)
   f.close();err.close();results[name]=code
  (out/'probe_result.json').write_text(json.dumps({'commands':commands,'exit_codes':results,'scored_cases':0},indent=2)+'\n')
 finally:
  try:runner.wait(timeout=130)
  except subprocess.TimeoutExpired:runner.terminate();runner.wait(timeout=30)
 print('runner_exit',runner.returncode,'output',out)
