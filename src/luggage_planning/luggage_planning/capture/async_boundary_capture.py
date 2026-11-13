"""Bounded callback snapshots and a single disk worker; no callback disk I/O.

CDR retains the complete source message, including optional empty arrays. Local
receive clocks are separate from source stamps. A sticky fault invalidates the
capture; queue overflow never silently drops a record. close() is not a callback.
"""
import hashlib
import json
import os
from pathlib import Path
import queue
import struct
import threading
import time

from luggage_planning.capture.ros_boundary_capture import _goal_id


class AsyncBoundaryCapture:
    def __init__(self, directory, *, run_id, capacity=4096,
                 max_bytes=512 * 1024 * 1024, max_queued_bytes=64 * 1024 * 1024,
                 on_error=None, opener=open):
        if not run_id or min(capacity, max_bytes, max_queued_bytes) <= 0:
            raise ValueError("run_id and positive capture bounds required")
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=False)
        self.run_id = run_id
        self.queue = queue.Queue(maxsize=capacity)
        self.max_bytes = max_bytes
        self.max_queued_bytes = max_queued_bytes
        self.queued_bytes = 0
        self.on_error = on_error
        self.opener = opener
        self.lock = threading.Lock()
        self.counts = dict(received=0, enqueued=0, written=0, dropped=0, failed=0,
                           written_bytes=0)
        self.errors = []
        self.closed = False
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self._write, name='capture-disk', daemon=True)
        self.worker.start()

    def fault(self, name, error):
        with self.lock:
            fresh = name not in {item['name'] for item in self.errors}
            if fresh and len(self.errors) < 16:
                self.errors.append(dict(name=name, detail=str(error)[:1000]))
        if fresh and self.on_error:
            try:
                self.on_error(name, str(error))
            except Exception:
                # Error reporting must not prevent cancellation/health reporting.
                pass

    @property
    def healthy(self):
        with self.lock:
            return not self.errors and not self.closed

    def record(self, boundary, message, *, goal_id=None, correlation=None):
        from rclpy.serialization import serialize_message
        receive_mono = time.monotonic_ns()
        with self.lock:
            self.counts['received'] += 1
            sequence = self.counts['received']
            if self.closed or self.errors:
                self.counts['dropped'] += 1
                return False
        try:
            payload = bytes(serialize_message(message))
            embedded = getattr(message, 'goal_id', None)
            identity = _goal_id(embedded) if embedded is not None else None
            supplied = _goal_id(goal_id) if goal_id is not None else None
            if identity and supplied and identity != supplied:
                raise ValueError('UUID mismatch')
            stamp = getattr(getattr(message, 'header', None), 'stamp', None)
            if stamp is None:
                stamp = getattr(message, 'clock', None)
            metadata = dict(sequence=sequence, run_id=self.run_id, boundary=boundary,
                            type=type(message).__module__ + '.' + type(message).__name__,
                            goal_id=identity or supplied, correlation=correlation or {},
                            receive_monotonic_ns=receive_mono, receive_wall_ns=time.time_ns(),
                            source_stamp_ns=(stamp.sec * 10**9 + stamp.nanosec
                                             if stamp is not None else None),
                            sha256=hashlib.sha256(payload).hexdigest())
            # Serialize metadata too: caller mutation must not alter queued data.
            header = json.dumps(metadata, separators=(',', ':')).encode()
            with self.lock:
                if self.closed or self.errors:
                    self.counts['dropped'] += 1
                    return False
                size = len(header) + len(payload)
                if self.queued_bytes + size > self.max_queued_bytes:
                    raise queue.Full()
                self.queue.put_nowait((header, payload))
                self.queued_bytes += size
                self.counts['enqueued'] += 1
            return True
        except queue.Full:
            with self.lock:
                self.counts['dropped'] += 1
            self.fault('CAPTURE_QUEUE_FULL', 'bounded queue capacity exceeded')
        except Exception as error:
            with self.lock:
                self.counts['failed'] += 1
            self.fault('CAPTURE_SERIALIZATION_FAILED', error)
        return False

    def health(self):
        with self.lock:
            result = dict(self.counts, errors=list(self.errors), closed=self.closed,
                          queued_bytes=self.queued_bytes,
                          run_id=self.run_id, worker_alive=self.worker.is_alive())
        result['pending'] = result['received'] - result['written'] - result['dropped'] - result['failed']
        result['recorder_io_complete'] = (self.closed and not result['worker_alive']
                                          and not result['errors']
                                          and result['received'] == result['written'])
        # Boundary coverage needs a separate same-run audit.
        result['run_capture_complete'] = False
        return result

    def _write(self):
        try:
            with self.opener(self.root / 'messages.bin', 'xb') as stream:
                while not self.stop.is_set() or not self.queue.empty():
                    try:
                        header, payload = self.queue.get(timeout=.05)
                    except queue.Empty:
                        stream.flush()
                        continue
                    with self.lock:
                        self.queued_bytes -= len(header) + len(payload)
                    frame = struct.pack('!II', len(header), len(payload)) + header + payload
                    try:
                        if self.counts['written_bytes'] + len(frame) > self.max_bytes:
                            raise OSError('CAPTURE_BYTE_BUDGET_EXCEEDED')
                        stream.write(frame)
                        with self.lock:
                            self.counts['written'] += 1
                            self.counts['written_bytes'] += len(frame)
                    except Exception:
                        with self.lock:
                            self.counts['failed'] += 1
                        raise
                    finally:
                        self.queue.task_done()
                stream.flush()
                os.fsync(stream.fileno())
        except Exception as error:
            self.fault('CAPTURE_WRITE_FAILED', error)
        finally:
            while True:
                try:
                    header, payload = self.queue.get_nowait()
                    with self.lock:
                        self.queued_bytes -= len(header) + len(payload)
                        self.counts['failed'] += 1
                    self.queue.task_done()
                except queue.Empty:
                    break
            self._write_health()

    def _write_health(self):
        try:
            temporary = self.root / 'health.tmp'
            with open(temporary, 'w') as stream:
                json.dump(self.health(), stream, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.root / 'health.json')
        except Exception as error:
            self.fault('CAPTURE_HEALTH_WRITE_FAILED', error)

    def close(self, timeout=10.):
        with self.lock:
            self.closed = True
        self.stop.set()
        self.worker.join(timeout)
        if self.worker.is_alive():
            self.fault('CAPTURE_DRAIN_TIMEOUT', timeout)
        self._write_health()
        return self.health()


def read_stream(directory):
    """Yield verified metadata and authoritative CDR; incomplete frames are errors."""
    with (Path(directory) / 'messages.bin').open('rb') as stream:
        while True:
            prefix = stream.read(8)
            if not prefix:
                return
            if len(prefix) != 8:
                raise ValueError('truncated capture frame')
            header_size, payload_size = struct.unpack('!II', prefix)
            if header_size > 1024 * 1024 or payload_size > 128 * 1024 * 1024:
                raise ValueError('invalid capture frame length')
            header, payload = stream.read(header_size), stream.read(payload_size)
            if len(header) != header_size or len(payload) != payload_size:
                raise ValueError('truncated capture payload')
            metadata = json.loads(header)
            if hashlib.sha256(payload).hexdigest() != metadata['sha256']:
                raise ValueError('capture hash mismatch')
            yield metadata, payload
