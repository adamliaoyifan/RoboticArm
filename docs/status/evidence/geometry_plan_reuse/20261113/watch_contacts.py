"""Eval-only bounded contact-pair ledger; never a planning/perception input.

Read actual physics contacts continuously, aggregate counts by execution phase,
and preserve first examples per pair/phase plus the final 100-message window.
Absence of a pair is not proof of no physical contact outside sensor coverage.
"""
import collections,gzip,hashlib,json,os,re,select,subprocess,sys,time
from pathlib import Path
root=Path(sys.argv[1]);out=root/'reference'/(sys.argv[2] if len(sys.argv)>2 else 'contact_stream');out.mkdir(exist_ok=False)
run=json.loads((root/'run_manifest.json').read_text());command=['ign','topic','-e','-t',run['eval_contact_topic']]
process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=(out/'stderr.log').open('wb'),env=dict(os.environ,IGN_PARTITION=run['ign_partition']))
os.set_blocking(process.stdout.fileno(),False)
pairs={};tail=collections.deque(maxlen=100);buffer=b'';count=0;seen_bytes=0;started=time.time();phase='startup';last_phase=0.;last_flush=0.;end=time.monotonic()+3600;error='';saved_bytes=0
pattern=re.compile(r'collision1 \{(.*?)\}\s*collision2 \{(.*?)\}',re.S)
def flush():
 (out/'summary.json').write_text(json.dumps(dict(scope=__doc__,command=command,partition=run['ign_partition'],started=started,updated=time.time(),messages=count,received_bytes=seen_bytes,example_bytes=saved_bytes,pairs=list(pairs.values()),error=error,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n')
try:
 while time.monotonic()<end:
  now=time.monotonic()
  if now-last_phase>1.:
   last_phase=now
   try:
    finished=json.loads((root/'run_manifest.json').read_text()).get('finished')
   except (ValueError,OSError):finished=False
   if finished:break
   manifests=sorted((root/'run/execution').glob('*/manifest.json'))
   if manifests:
    p=manifests[-1]
    try:v=json.loads(p.read_text())
    except (ValueError,OSError):continue
    phase=p.parent.name+':'+v['phase']
    stages=[x['file'] for x in v.get('records',[]) if x['file'].endswith('stage.json.gz')]
    if stages:
     s=json.loads(gzip.decompress((p.parent/stages[-1]).read_bytes()));phase+=':'+str(s.get('next_leg_index',s.get('index','na')))
  ready,_,_=select.select([process.stdout],[],[],.2)
  if ready:
   chunk=os.read(process.stdout.fileno(),262144)
   if not chunk:
    if process.poll() is not None:break
    continue
   seen_bytes+=len(chunk);buffer+=chunk
   if len(buffer)>16*1024*1024:raise ValueError('CONTACT_MESSAGE_BUFFER_LIMIT')
   while b'\n\n' in buffer:
    raw,buffer=buffer.split(b'\n\n',1);count+=1;tail.append(raw)
    text=raw.decode(errors='replace')
    # Top-level header is the first acquisition stamp. Keep the exact example.
    stamp=re.search(r'stamp \{\s*sec: (\d+)(?:\s*nsec: (\d+))?',text)
    stamp_ns=int(stamp[1])*10**9+int(stamp[2] or 0) if stamp else None
    for a,b in pattern.findall(text):
     names=tuple(sorted([re.search(r'name: "([^"]+)"',x)[1] for x in (a,b)]));key=(phase,*names)
     if key not in pairs:
      if len(pairs)>=256:raise ValueError('CONTACT_PAIR_PHASE_LIMIT')
      name=f'{len(pairs):03d}_first.txt.gz';data=gzip.compress(raw,compresslevel=1,mtime=0);(out/name).write_bytes(data);saved_bytes+=len(data)
      pairs[key]=dict(phase=phase,objects=names,messages=0,first_stamp_ns=stamp_ns,example=name,example_sha256=hashlib.sha256(raw).hexdigest())
     pairs[key]['messages']+=1;pairs[key]['last_stamp_ns']=stamp_ns
  if now-last_flush>5.:flush();last_flush=now
except Exception as e:error=repr(e)
finally:
 process.terminate()
 try:process.wait(timeout=3.)
 except subprocess.TimeoutExpired:process.kill();process.wait()
 (out/'last_window.txt.gz').write_bytes(gzip.compress(b'\n\n'.join(tail),compresslevel=1,mtime=0));flush()
