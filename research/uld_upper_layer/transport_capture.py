"""Bounded compressed Ignition topic capture with an observation-only callback."""
import json
from bounded_trace import BoundedTrace
from pathlib import Path
import subprocess
import threading


class JsonTopicCapture:
    def __init__(self,env,output,name,topic,accept,max_bytes=75_000_000,trace_options=None):
        self.env=env;self.output=Path(output);self.name=name;self.topic=topic
        self.accept=accept;self.max_bytes=max_bytes;self.error=None;self.count=0
        self.process=None;self.thread=None;self.closing=False
        self.trace_options=trace_options or {};self.trace=None

    def start(self):
        self.stderr=(self.output/f'{self.name}_stderr.log').open('w')
        self.process=subprocess.Popen(['stdbuf','-oL','ign','topic','-e','--json-output',
            '-t',self.topic],env=self.env,stdout=subprocess.PIPE,stderr=self.stderr,
            text=True,bufsize=1,start_new_session=True)
        self.thread=threading.Thread(target=self._read,daemon=True);self.thread.start()

    def _read(self):
        decoder=json.JSONDecoder();buffer=''
        path=self.output/f'{self.name}.jsonl.gz'
        try:
            self.trace=BoundedTrace(path,**self.trace_options)
            try:
                for line in self.process.stdout:
                    buffer+=line
                    while buffer.strip():
                        buffer=buffer.lstrip()
                        try:row,end=decoder.raw_decode(buffer)
                        except json.JSONDecodeError:break
                        buffer=buffer[end:]
                        stamp=self.accept(row)
                        self.trace.append(row,stamp);self.count+=1
                        if self.count%100==0:
                            if path.stat().st_size>self.max_bytes:
                                raise RuntimeError('COMPRESSED_CAPTURE_BUDGET:'+self.name)
            finally:
                self.trace.close()
            if buffer.strip() and not self.closing:raise RuntimeError('TRUNCATED_TRANSPORT_MESSAGE:'+self.name)
        except Exception as exc:self.error=str(exc)

    def stop(self):
        self.closing=True
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        if self.thread:self.thread.join(timeout=5)
        if hasattr(self,'stderr'):self.stderr.close()
