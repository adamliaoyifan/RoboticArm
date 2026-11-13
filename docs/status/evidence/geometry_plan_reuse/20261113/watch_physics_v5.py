"""Eval-only phase/10-second-triggered Gazebo reference capture; never an online input."""
import argparse,gzip,json,subprocess,time,hashlib,os
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('diagnostic');a=p.parse_args();root=Path(a.diagnostic);out=root/'reference';out.mkdir(exist_ok=True);rows=[];seen=set();end=time.monotonic()+3600;last_sample=0.;contact_seen=set();contact_rows=[]
while time.monotonic()<end and len(rows)<300:
 manifest=root/'run_manifest.json'
 if manifest.exists():
  try:run=json.loads(manifest.read_text())
  except ValueError:time.sleep(.1);continue
  if run.get('finished'):break
 phase='startup'
 for q in sorted((root/'run/execution').glob('*/manifest.json'),key=lambda q:q.stat().st_mtime):
  try:
   current=json.loads(q.read_text());phase=current['phase']
   stage_rows=[row for row in current.get('records',[]) if row['file'].endswith(('stage.json.gz','stage.json'))]
   if stage_rows:
    stage_file=q.parent/stage_rows[-1]['file'];raw=stage_file.read_bytes()
    stage=json.loads(gzip.decompress(raw) if stage_file.suffix=='.gz' else raw)
    phase+='_'+str(stage.get('next_leg_index',stage.get('index','na')))
  except (ValueError,KeyError):continue
 if phase not in seen or time.monotonic()-last_sample>=10:
  seen.add(phase);begin=time.time();last_sample=time.monotonic()
  try:
   result=subprocess.run(['ign','topic','-e','-t','/world/airport_loading/pose/info','--num','1'],capture_output=True,timeout=3.,env=dict(os.environ,IGN_PARTITION=run.get('ign_partition',os.environ.get('IGN_PARTITION',''))))
   raw=result.stdout;name=f'{len(rows):02d}_{phase}.pose.txt.gz';(out/name).write_bytes(gzip.compress(raw,compresslevel=1,mtime=0))
   rows.append(dict(file=name,phase=phase,wall_started=begin,wall_finished=time.time(),exit_code=result.returncode,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),stderr=result.stderr.decode(errors='replace')[:500]))
  except subprocess.TimeoutExpired as error:
   rows.append(dict(phase=phase,error='REFERENCE_TOPIC_TIMEOUT',wall_started=begin))
  (out/'manifest.json').write_text(json.dumps(dict(scope='eval_only_never_perception_or_planning_input',records=rows),indent=2)+'\n')
 if phase.startswith('COMPLETE') and run.get('eval_contact_topic'):
  key=q.parent.name
  if key not in contact_seen:
   contact_seen.add(key);begin=time.time()
   try:
    result=subprocess.run(['ign','topic','-e','-t',run['eval_contact_topic'],'--num','20'],capture_output=True,timeout=4.,env=dict(os.environ,IGN_PARTITION=run['ign_partition']))
    raw=result.stdout;name=f'contacts_{len(contact_rows):02d}_{key}.txt.gz';(out/name).write_bytes(gzip.compress(raw,compresslevel=1,mtime=0))
    contact_rows.append(dict(file=name,execution=key,topic=run['eval_contact_topic'],wall_started=begin,wall_finished=time.time(),exit_code=result.returncode,bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),stderr=result.stderr.decode(errors='replace')[:500]))
   except subprocess.TimeoutExpired as error:
    raw=error.stdout or b'';name=f'contacts_{len(contact_rows):02d}_{key}.partial.txt.gz';(out/name).write_bytes(gzip.compress(raw,compresslevel=1,mtime=0));contact_rows.append(dict(file=name,execution=key,error='CONTACT_TOPIC_TIMEOUT',bytes=len(raw),wall_started=begin))
   (out/'contacts_manifest.json').write_text(json.dumps(dict(scope='actual_physics_contact_eval_only_never_online_input',records=contact_rows),indent=2)+'\n')
 time.sleep(.2)
