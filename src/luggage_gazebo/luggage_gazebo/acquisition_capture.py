"""Passive, acquisition-keyed perception evidence; never feeds the detector."""
from collections import deque
import gzip
import hashlib
import json
from pathlib import Path
import threading


class AcquisitionCapture:
    """Retain bounded input bytes, export the actual selected detection window.

    The receipt distinguishes exact-acquisition completeness from recent-window
    history. It cannot reconstruct older retained extrema or missing messages.
    No latest-image substitution is allowed for a missing selected acquisition.
    """
    def __init__(self, *, retention_ns=8_000_000_000, memory_limit=256*1024*1024):
        self.retention_ns,self.memory_limit=retention_ns,memory_limit
        self.rows=deque()
        self.bytes=0
        self.latest_ns=0
        self.lock=threading.Lock()

    def add(self, topic, ros_type, stamp_ns, raw):
        if stamp_ns<=0:return
        raw=bytes(raw)
        with self.lock:
            self.latest_ns=max(self.latest_ns,stamp_ns)
            self.rows.append((topic,ros_type,stamp_ns,raw))
            self.bytes+=len(raw)
            while self.rows and (self.bytes>self.memory_limit
                    or self.rows[0][2]<self.latest_ns-self.retention_ns):
                self.bytes-=len(self.rows.popleft()[3])

    def write(self, folder, stamp_ns, *, required_topics, history_ns=1_000_000_000):
        folder=Path(folder)
        folder.mkdir(parents=True,exist_ok=False)
        with self.lock:
            rows=[row for row in self.rows if stamp_ns-history_ns<=row[2]<=stamp_ns]
        exact={topic for topic,_,stamp,_ in rows if stamp==stamp_ns}
        missing=sorted(set(required_topics)-exact)
        records=[]
        for topic,kind,stamp,raw in rows:
            name=f'{len(records):04d}.cdr.gz'
            (folder/name).write_bytes(gzip.compress(raw,compresslevel=1,mtime=0))
            records.append(dict(file=name,topic=topic,ros_type=kind,acquisition_stamp_ns=stamp,
                raw_bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest()))
        result=dict(acquisition_stamp_ns=stamp_ns,history_ns=history_ns,records=records,
            missing_at_acquisition=missing,selected_acquisition_complete=not missing,
            scope='Exact selected inputs plus recent window; older retained extrema not replayed')
        (folder/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
