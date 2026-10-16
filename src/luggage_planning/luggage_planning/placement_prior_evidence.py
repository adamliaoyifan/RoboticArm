"""Opt-in bounded exact ROS boundary capture for a single prior experiment."""
import json
from pathlib import Path
import threading


class PriorEvidenceRecorder:
    def __init__(self, directory, *, max_bytes=256*1024*1024):
        self._root = Path(directory)
        self._root.mkdir(parents=True, exist_ok=False)
        self._limit = max_bytes
        self._written = 0
        self._sequence = 0
        self._lock = threading.Lock()

    def __call__(self, boundary, message):
        from rclpy.serialization import serialize_message
        data = serialize_message(message)
        with self._lock:
            if self._written + len(data) > self._limit:
                raise RuntimeError('PRIOR_EVIDENCE_BUDGET_EXHAUSTED')
            self._sequence += 1
            name = '%04d.cdr' % self._sequence
            (self._root / name).write_bytes(data)
            self._written += len(data)
            with (self._root / 'boundaries.jsonl').open('a') as stream:
                stream.write(json.dumps(dict(file=name, boundary=boundary,
                    request_id=getattr(message, 'request_id', ''),
                    type=type(message).__module__+'.'+type(message).__name__,
                    bytes=len(data)))+'\n')
