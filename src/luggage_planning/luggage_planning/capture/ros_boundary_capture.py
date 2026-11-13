"""Node-layer, lossless ROS boundary recording for diagnostic preparation.

This module does not create nodes, forward actions, or alter messages. Callers
must provide messages from real boundaries; recording a constructed goal here
alone does not prove a controller received it. Empty arrays are data, not gaps.
CDR is authoritative; the full field view helps inspect headers and UUIDs.
"""

import hashlib
import json
import math
from numbers import Integral
import os
from pathlib import Path
import shutil
import tempfile
import threading
import time


def _json_fields(value):
    """Keep array contents and integer timestamps; encode nonfinite diagnostics."""
    if isinstance(value, dict):
        return {key: _json_fields(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_fields(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite_float": repr(value)}
    return value


def _json_bytes(value):
    return (json.dumps(value, sort_keys=True, allow_nan=False,
                       separators=(",", ":")) + "\n").encode()


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _durable_write(path, data):
    with path.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def _sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _goal_id(value):
    """Normalize an explicitly supplied or embedded action UUID without guessing."""
    values = list(getattr(value, "uuid", value))
    if len(values) != 16 or any(
            not isinstance(v, Integral) or isinstance(v, bool) or not 0 <= v <= 255
            for v in values):
        raise ValueError("action goal ID must contain exactly 16 bytes")
    return bytes(values).hex()


class RosBoundaryRecorder:
    """One process owns a fresh evidence directory; calls are thread safe.

    A committed record is an atomically renamed directory containing CDR,
    complete fields and a hash index. A record error is sticky in health.json.
    close() certifies only recorder I/O, never whole-run boundary coverage.
    Storage exhaustion raises; it never truncates a message or reports success.
    Intended for opt-in diagnosis; synchronous durable I/O has overhead which
    must be measured before attaching this writer to production callbacks.
    """

    def __init__(self, directory, *, run_id, max_bytes=256 * 1024 * 1024):
        if not run_id or max_bytes <= 0:
            raise ValueError("nonempty run_id and positive max_bytes required")
        self._root = Path(directory)
        self._root.mkdir(parents=True, exist_ok=False)
        self._run_id = str(run_id)
        self._max_bytes = int(max_bytes)
        self._written_bytes = 0
        self._count = 0
        self._errors = 0
        self._first_error = None
        self._closed = False
        self._lock = threading.Lock()
        self._write_health()

    def _write_health(self):
        # Metadata is bounded and deliberately separate from the payload quota.
        health = dict(run_id=self._run_id, records=self._count,
                      written_bytes=self._written_bytes, max_bytes=self._max_bytes,
                      errors=self._errors, first_error=self._first_error,
                      closed=self._closed,
                      recorder_io_complete=self._closed and self._errors == 0,
                      run_capture_complete=False)
        temporary = self._root / ".health.tmp"
        temporary.unlink(missing_ok=True)
        _durable_write(temporary, _json_bytes(health))
        os.replace(temporary, self._root / "health.json")
        _sync_directory(self._root)

    def record(self, boundary, message, *, goal_id=None, correlation=None):
        """Snapshot one actual boundary message before its owner mutates it.

        For action Goal (without a UUID field), pass the UUID used by the real
        client. Prefer a SendGoal request, whose UUID is part of the payload.
        A supplied UUID conflicting with an embedded one is an evidence error.
        Source stamps stay in the unmodified message fields; local receive
        wall/monotonic timestamps are recorded separately, never as substitutes.
        """
        from rclpy.serialization import deserialize_message, serialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict

        with self._lock:
            if self._closed:
                raise RuntimeError("recorder closed")
            temporary = None
            try:
                receive_mono_ns = time.monotonic_ns()
                receive_wall_ns = time.time_ns()
                if not boundary:
                    raise ValueError("boundary name required")
                payload = bytes(serialize_message(message))
                snapshot = deserialize_message(payload, type(message))
                embedded = getattr(snapshot, "goal_id", None)
                embedded_id = _goal_id(embedded) if embedded is not None else None
                explicit_id = _goal_id(goal_id) if goal_id is not None else None
                if embedded_id and explicit_id and embedded_id != explicit_id:
                    raise ValueError("action goal ID mismatch")
                fields = _json_bytes(_json_fields(message_to_ordereddict(snapshot)))
                entry = dict(schema="luggage.ros_boundary/1", run_id=self._run_id,
                             boundary=str(boundary), sequence=self._count + 1,
                             type=type(message).__module__ + "." + type(message).__name__,
                             goal_id=embedded_id or explicit_id,
                             correlation=dict(correlation or {}),
                             receive_monotonic_ns=receive_mono_ns,
                             receive_wall_ns=receive_wall_ns,
                             cdr_sha256=_digest(payload), fields_sha256=_digest(fields),
                             cdr_bytes=len(payload), fields_bytes=len(fields))
                index = _json_bytes(entry)
                size = len(payload) + len(fields) + len(index)
                if self._written_bytes + size > self._max_bytes:
                    raise RuntimeError("BOUNDARY_CAPTURE_BUDGET_EXHAUSTED")
                temporary = Path(tempfile.mkdtemp(prefix=".pending-", dir=self._root))
                _durable_write(temporary / "message.cdr", payload)
                _durable_write(temporary / "fields.json", fields)
                _durable_write(temporary / "index.json", index)
                _sync_directory(temporary)
                destination = self._root / ("%06d" % (self._count + 1))
                os.rename(temporary, destination)
                temporary = None
                _sync_directory(self._root)
                self._count += 1
                self._written_bytes += size
                self._write_health()
                return destination
            except Exception as error:
                self._errors += 1
                if self._first_error is None:
                    self._first_error = "%s: %s" % (type(error).__name__, error)
                    self._first_error = self._first_error[:1000]
                self._write_health()
                raise
            finally:
                if temporary is not None:
                    shutil.rmtree(temporary)

    def close(self):
        with self._lock:
            self._closed = True
            self._write_health()


def load_boundary_record(directory, message_type):
    """Verify storage integrity and semantic agreement before returning a message.

    Compare decoded fields, not fresh CDR bytes: serializer alignment padding
    need not be reproducible across calls. No absent arrays are synthesized.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict

    directory = Path(directory)
    entry = json.loads((directory / "index.json").read_text())
    payload = (directory / "message.cdr").read_bytes()
    fields = (directory / "fields.json").read_bytes()
    if _digest(payload) != entry["cdr_sha256"] or _digest(fields) != entry["fields_sha256"]:
        raise ValueError("capture hash mismatch")
    expected_type = message_type.__module__ + "." + message_type.__name__
    if entry["type"] != expected_type:
        raise ValueError("capture message type mismatch")
    message = deserialize_message(payload, message_type)
    if _json_bytes(_json_fields(message_to_ordereddict(message))) != fields:
        raise ValueError("decoded CDR differs from recorded fields")
    embedded = getattr(message, "goal_id", None)
    if embedded is not None and _goal_id(embedded) != entry["goal_id"]:
        raise ValueError("decoded action goal ID mismatch")
    return message
