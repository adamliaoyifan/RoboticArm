"""Event ledger for T1: one JSONL stream per run, joinable per case.

Schema (schema_version 1). Every event carries the full case join key:

  run_id backend consumer hull case_id subprobe request_id event_seq
  event_kind stage state reason mono_ns payload...

Join keys follow the contract §7: every input -> store commit ->
validity publication -> snapshot read -> real consumer -> candidate ->
planning commit must be joinable across events. ``mono_ns`` is real
CLOCK_MONOTONIC (deadlines and ordering); ROS-clock stamps of fixtures
ride in payload fields and are never compared against mono_ns.
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path

SCHEMA_VERSION = 1


def mono_ns() -> int:
    return time.monotonic_ns()


class EventLedger:
    """Append-only JSONL ledger; one writer thread, fsync on flush."""

    def __init__(self, path, run_id):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = open(self.path, "a", encoding="utf-8")
        self._lock = threading.Lock()
        self._seq = 0
        self.run_id = str(run_id)

    def emit(self, *, backend, consumer, hull, case_id, subprobe,
             event_kind, stage, state, reason="", request_id="",
             deadline_ns=None, **payload):
        with self._lock:
            self._seq += 1
            row = {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.run_id,
                "event_seq": self._seq,
                "mono_ns": mono_ns(),
                "backend": str(backend),
                "consumer": str(consumer),
                "hull": str(hull),
                "case_id": str(case_id),
                "subprobe": str(subprobe or ""),
                "request_id": str(request_id or ""),
                "event_kind": str(event_kind),
                "stage": str(stage),
                "state": str(state),
                "reason": str(reason or ""),
            }
            if deadline_ns is not None:
                row["deadline_ns"] = int(deadline_ns)
            row.update(payload)
            self._stream.write(json.dumps(row, sort_keys=True) + "\n")
            self._stream.flush()
            os.fsync(self._stream.fileno())
            return self._seq

    def close(self):
        with self._lock:
            self._stream.close()

    # ---- terminal conventions ------------------------------------------
    #: A case terminal is one of these; exactly one per (case, subprobe).
    TERMINALS = ("PASS", "FAIL", "BLOCKED", "NOT_EVALUATED",
                 "INCOMPLETE_EVIDENCE")

    def terminal(self, *, backend, consumer, hull, case_id, subprobe,
                 state, reason="", **payload):
        if state not in self.TERMINALS:
            raise ValueError("terminal state %r not in %s"
                             % (state, self.TERMINALS))
        return self.emit(
            backend=backend, consumer=consumer, hull=hull, case_id=case_id,
            subprobe=subprobe, event_kind="case_terminal", stage="case",
            state=state, reason=reason, **payload)


def load_events(archive):
    """Read every events.jsonl under an archive/run dir (judge path)."""
    events = []
    for path in sorted(Path(archive).glob("**/events.jsonl")):
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    events.append(json.loads(line))
    return events


def case_key(event):
    return (event.get("run_id"), event.get("backend"),
            event.get("consumer"), event.get("hull"),
            event.get("case_id"), event.get("subprobe"))
