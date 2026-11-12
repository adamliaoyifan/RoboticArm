"""Bounded diagnostic pre-trigger window; never supplies planning inputs."""
from collections import deque
import gzip
import hashlib
import json
import threading
import time


class ObserverFailureCapture:
    """Keep ten seconds in RAM, then persist independently compressed CDR blocks.

    The index records the exact topic/type and byte ranges for offline replay.
    Caps bound both memory and disk; dropped records are explicit evidence gaps.
    """
    def __init__(self, folder, *, window_s=10., memory_limit=96*1024*1024,
                 disk_limit=256*1024*1024, clock=time.monotonic):
        self.folder, self.window_s = folder, window_s
        self.memory_limit, self.disk_limit, self.clock = memory_limit, disk_limit, clock
        self.rows, self.static, self.memory_bytes = deque(), {}, 0
        self.lock = threading.Lock()
        self.stream = self.index = None
        self.written = self.dropped = 0
        self.closed = False

    def add(self, topic, ros_type, raw, *, persistent=False):
        row = (self.clock(), topic, ros_type, bytes(raw))
        with self.lock:
            if self.closed:return
            if self.stream is not None:
                self._write(row)
                return
            if persistent:
                key = (topic, persistent)
                old = self.static.get(key)
                if old:self.memory_bytes -= len(old[3])
                self.static[key] = row
            else:
                self.rows.append(row)
            self.memory_bytes += len(raw)
            while self.rows and (row[0]-self.rows[0][0] > self.window_s
                                 or self.memory_bytes > self.memory_limit):
                self.memory_bytes -= len(self.rows.popleft()[3])
            if self.memory_bytes > self.memory_limit:
                self.static.clear();self.memory_bytes = 0
                self.dropped += 1

    def trigger(self):
        with self.lock:
            if self.closed or self.stream is not None:return
            self.folder.mkdir(exist_ok=False)
            self.stream = (self.folder/'cdr.gzblocks').open('wb')
            self.index = (self.folder/'index.jsonl').open('w')
            for row in sorted((*self.static.values(), *self.rows), key=lambda value:value[0]):
                self._write(row)
            self.rows.clear();self.static.clear();self.memory_bytes = 0

    def _write(self, row):
        stamp, topic, ros_type, raw = row
        block = gzip.compress(raw, compresslevel=1)
        if self.written + len(block) > self.disk_limit:
            self.dropped += 1
            return
        self.stream.write(block);self.stream.flush()
        self.index.write(json.dumps(dict(monotonic=stamp, topic=topic, ros_type=ros_type,
            offset=self.written, stored_bytes=len(block), raw_bytes=len(raw),
            sha256=hashlib.sha256(raw).hexdigest()))+'\n')
        self.index.flush();self.written += len(block)

    def close(self):
        with self.lock:
            self.closed = True
            if self.stream is None:return
            self.stream.close();self.index.close()
            (self.folder/'summary.json').write_text(json.dumps(dict(
                stored_bytes=self.written, dropped_records=self.dropped,
                complete=self.dropped == 0, window_s=self.window_s))+'\n')
