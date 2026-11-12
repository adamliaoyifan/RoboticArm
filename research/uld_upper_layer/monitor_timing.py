"""Bounded slow-stage and cyclic-GC diagnostics; never changes GC or physics."""
from collections import deque
from contextlib import contextmanager
import gc
import gzip
import json
import threading
import time


class MonitorTiming:
    def __init__(self,path,slow_s=.020,max_events=20000):
        self._stream=gzip.open(path,'wt',compresslevel=3)
        self._slow_s=slow_s;self._max_events=max_events;self._written=0;self._omitted=0
        self._stamp=None;self._active_gc={};self._gc_events=deque(maxlen=128)
        self._gc_count=[0,0,0];self._gc_max_s=[0.,0.,0.];self._gc_dropped=0
        self._max_write_s=0.;self._stages={}
        self._gc_callback=self._on_gc
        gc.callbacks.append(self._gc_callback)

    def _on_gc(self,event,info):
        key=threading.get_ident()
        if event=='start':self._active_gc[key]=time.perf_counter()
        elif event=='stop':
            started=self._active_gc.pop(key,None)
            if started is None:return
            duration=time.perf_counter()-started;generation=info['generation']
            self._gc_count[generation]+=1;self._gc_max_s[generation]=max(self._gc_max_s[generation],duration)
            if duration>=self._slow_s:
                if len(self._gc_events)==self._gc_events.maxlen:self._gc_dropped+=1
                self._gc_events.append(dict(event='gc',generation=generation,wall_s=duration,
                    stamp_s=self._stamp,thread_id=key,collected=info['collected'],uncollectable=info['uncollectable']))

    def _write(self,event):
        if self._written>=self._max_events:self._omitted+=1;return
        started=time.perf_counter();self._stream.write(json.dumps(event)+'\n');self._written+=1
        self._max_write_s=max(self._max_write_s,time.perf_counter()-started)

    @contextmanager
    def stage(self,name,stamp_s):
        self._stamp=stamp_s;wall=time.perf_counter();cpu=time.thread_time()
        try:yield
        finally:
            elapsed=time.perf_counter()-wall;thread_cpu=time.thread_time()-cpu
            stats=self._stages.setdefault(name,dict(count=0,total_wall_s=0.,max_wall_s=0.,max_thread_cpu_s=0.))
            stats['count']+=1;stats['total_wall_s']+=elapsed
            stats['max_wall_s']=max(stats['max_wall_s'],elapsed);stats['max_thread_cpu_s']=max(stats['max_thread_cpu_s'],thread_cpu)
            while self._gc_events:self._write(self._gc_events.popleft())
            if elapsed>=self._slow_s:self._write(dict(event='slow_stage',stage=name,stamp_s=stamp_s,
                wall_s=elapsed,thread_cpu_s=thread_cpu,wall_end_s=time.time()))

    def metrics(self):
        return dict(stages=self._stages,gc_counts=self._gc_count,gc_max_pause_s=self._gc_max_s,
            gc_dropped=self._gc_dropped,events_written=self._written,events_omitted=self._omitted,
            max_timing_write_s=self._max_write_s,scope='instrumented wall/thread CPU/GC; no GC policy changes')

    def close(self):
        gc.callbacks.remove(self._gc_callback)
        while self._gc_events:self._write(self._gc_events.popleft())
        self._stream.close()
