"""Lossless decision inputs plus a bounded full-rate native failure window.

Only the archival baseline is sampled; transport callbacks always receive every
message. The raw ring retains the pre-window plus permitted processing lag.
"""
from collections import deque
import gzip
import json
import math
from pathlib import Path
import threading


class BoundedTrace:
    def __init__(self,path,period_s=0.,pre_s=0.,post_s=0.,lag_s=0.):
        self.path=Path(path);self._period=period_s;self._pre=pre_s;self._post=post_s;self._lag=lag_s
        self._baseline=gzip.open(self.path,'wt',compresslevel=3)
        self._ring=deque();self._lock=threading.Lock();self._last_archive=None;self._last=None
        self._trigger=None;self._trigger_stamp=None;self._first_trigger=None;self._last_trigger=None
        self._max_gap=0.;self.seen=0;self.archived=0;self.trigger_samples=0;self.complete=False

    def append(self,row,stamp=None):
        line=json.dumps(row,separators=(',',':'))+'\n'
        with self._lock:
            if self._period and (stamp is None or not math.isfinite(stamp)):
                raise ValueError('STAMP_REQUIRED_FOR_SAMPLED_ARCHIVE')
            if stamp is not None:
                if self._last is not None:
                    if stamp<self._last:raise ValueError('TRACE_STAMP_WENT_BACKWARDS')
                    self._max_gap=max(self._max_gap,stamp-self._last)
                self._last=stamp
            self.seen+=1
            if not self._period or self._last_archive is None or stamp-self._last_archive>=self._period-1e-9:
                self._baseline.write(line);self.archived+=1;self._last_archive=stamp
            if self._pre and stamp is not None:
                self._ring.append((stamp,line))
                while self._ring and stamp-self._ring[0][0]>self._pre+self._lag:
                    self._ring.popleft()
            if self._trigger is not None:self._write_trigger(stamp,line)
            if self.seen%100==0:
                self._baseline.flush()
                if self._trigger is not None:self._trigger.flush()

    def _write_trigger(self,stamp,line):
        if stamp is None or stamp<self._trigger_stamp-self._pre-1e-9:return
        self._trigger.write(line);self.trigger_samples+=1
        if self._first_trigger is None:self._first_trigger=stamp
        self._last_trigger=stamp
        if stamp>=self._trigger_stamp+self._post-1e-9:
            self._trigger.close();self._trigger=None
            self.complete=(self._first_trigger<=self._trigger_stamp-self._pre+self._max_gap+1e-9)

    def start_trigger(self,stamp):
        with self._lock:
            if self._trigger_stamp is not None:return
            self._trigger_stamp=stamp
            self._trigger=gzip.open(self.path.with_name(self.path.name.replace('.jsonl.gz','_trigger.jsonl.gz')),
                                    'wt',compresslevel=3)
            for old_stamp,line in self._ring:
                if self._trigger is None:break
                self._write_trigger(old_stamp,line)
            if self._trigger is not None:self._trigger.flush()

    def metrics(self):
        with self._lock:
            return dict(seen=self.seen,archived=self.archived,archive_period_s=self._period,
                trigger_stamp_s=self._trigger_stamp,trigger_first_stamp_s=self._first_trigger,
                trigger_last_stamp_s=self._last_trigger,trigger_samples=self.trigger_samples,
                max_received_gap_s=self._max_gap,trigger_complete=self.complete)

    def close(self):
        with self._lock:
            self._baseline.close()
            if self._trigger is not None:self._trigger.close();self._trigger=None
