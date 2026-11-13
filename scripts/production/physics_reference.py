#!/usr/bin/env python3
"""Bounded Gazebo reference recorder; never provides online planning inputs.

Contact examples retain original messages and phase IDs. Poses are sampled on
phase transitions and every ten seconds. Contact absence proves only the capture
scope, not physical collision freedom. Run as an owned child of simulation.py.
"""
import argparse
from collections import deque
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import time


class ContactLedger:
    def __init__(self, output):
        self.output=Path(output)
        self.buffer=b''
        self.tail=deque(maxlen=100)
        self.pairs={}
        self.messages=0
        self.received_bytes=0

    def add(self, chunk, phase):
        self.received_bytes+=len(chunk)
        self.buffer+=chunk
        if len(self.buffer)>16*1024*1024:raise ValueError('CONTACT_MESSAGE_BUFFER_LIMIT')
        while b'\n\n' in self.buffer:
            raw,self.buffer=self.buffer.split(b'\n\n',1)
            if not raw.strip():continue
            self.messages+=1
            self.tail.append(raw)
            text=raw.decode(errors='replace')
            stamp=re.search(r'stamp \{\s*sec: (\d+)(?:\s*nsec: (\d+))?',text)
            stamp_ns=int(stamp[1])*10**9+int(stamp[2] or 0) if stamp else None
            for a,b in re.findall(r'collision1 \{(.*?)\}\s*collision2 \{(.*?)\}',text,re.S):
                names=tuple(sorted(re.search(r'name: "([^"]+)"',x)[1] for x in (a,b)))
                key=(phase,*names)
                if key not in self.pairs:
                    if len(self.pairs)>=256:raise ValueError('CONTACT_PAIR_PHASE_LIMIT')
                    name=f'{len(self.pairs):03d}_contact.txt.gz'
                    (self.output/name).write_bytes(gzip.compress(raw,compresslevel=1,mtime=0))
                    self.pairs[key]=dict(phase=phase,objects=names,messages=0,
                        first_stamp_ns=stamp_ns,example=name,
                        example_sha256=hashlib.sha256(raw).hexdigest())
                row=self.pairs[key]
                row['messages']+=1
                row['last_stamp_ns']=stamp_ns

    def summary(self):
        return dict(messages=self.messages,received_bytes=self.received_bytes,
                    pairs=list(self.pairs.values()))


def current_phase(run_dir):
    manifests=sorted((run_dir/'execution').glob('*/manifest.json'),key=lambda p:p.stat().st_mtime)
    if not manifests:return 'startup'
    path=manifests[-1]
    try:
        record=json.loads(path.read_text())
        phase=path.parent.name+':'+record['phase']
        stages=[r['file'] for r in record.get('records',[]) if r['file'].endswith(('stage.json','stage.json.gz'))]
        if stages:
            data=(path.parent/stages[-1]).read_bytes()
            stage=json.loads(gzip.decompress(data) if stages[-1].endswith('.gz') else data)
            phase+=':'+str(stage.get('next_leg_index',stage.get('index','na')))
        return phase
    except (OSError,ValueError,KeyError):return path.parent.name+':updating'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--contact-topic',required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    ledger=ContactLedger(args.output)
    rows=[]
    stopped=False
    def stop_requested(*_):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop_requested)
    signal.signal(signal.SIGTERM,stop_requested)
    command=['ign','topic','-e','-t',args.contact_topic]
    error=''
    started=time.time()
    def flush():
        value=dict(ledger.summary(),pose_samples=rows,error=error,started=started,
            updated=time.time(),partition=os.environ.get('IGN_PARTITION',''),
            command=command,scope=__doc__,
            capture_complete=bool(ledger.messages and rows and not error
                and all(row.get('exit_code')==0 and row.get('bytes',0)>0 for row in rows)))
        (args.output/'summary.json').write_text(json.dumps(value,indent=2)+'\n')
    process=None
    try:
        with (args.output/'contact_stderr.log').open('wb') as log:
            process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=log)
            os.set_blocking(process.stdout.fileno(),False)
            phase='startup'
            last_phase=last_pose=last_flush=0.
            sampled=set()
            deadline=time.monotonic()+3600.
            while not stopped and time.monotonic()<deadline:
                now=time.monotonic()
                if now-last_phase>=1.:
                    phase=current_phase(args.run_dir)
                    last_phase=now
                if (phase not in sampled or now-last_pose>=10.) and len(rows)<300:
                    sampled.add(phase)
                    last_pose=now
                    begin=time.time()
                    try:
                        result=subprocess.run(['ign','topic','-e','-t',
                            '/world/airport_loading/pose/info','--num','1'],
                            capture_output=True,timeout=3.)
                        raw=result.stdout
                        name=f'{len(rows):03d}_pose.txt.gz'
                        (args.output/name).write_bytes(gzip.compress(raw,compresslevel=1,mtime=0))
                        rows.append(dict(file=name,phase=phase,wall_started=begin,
                            exit_code=result.returncode,bytes=len(raw),
                            sha256=hashlib.sha256(raw).hexdigest()))
                    except subprocess.TimeoutExpired:
                        rows.append(dict(phase=phase,wall_started=begin,error='REFERENCE_TOPIC_TIMEOUT'))
                ready,_,_=select.select([process.stdout],[],[],.2)
                if ready:
                    chunk=os.read(process.stdout.fileno(),262144)
                    if not chunk:
                        if process.poll() is not None:raise RuntimeError('CONTACT_STREAM_EXIT')
                    else:ledger.add(chunk,phase)
                if now-last_flush>=5.:flush();last_flush=now
    except Exception as exc:error=type(exc).__name__+':'+str(exc)
    finally:
        if process is not None:
            process.terminate()
            try:process.wait(timeout=3.)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        (args.output/'last_contacts.txt.gz').write_bytes(gzip.compress(
            b'\n\n'.join(ledger.tail),compresslevel=1,mtime=0))
        flush()
    return 1 if error else 0


if __name__=='__main__':
    raise SystemExit(main())
