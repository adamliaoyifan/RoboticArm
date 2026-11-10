"""Backend registry and the four reserved real-machine interfaces.

Plan §3 (docs/plans/joint_command_boundary_capture.md):
- the real machine must not be assumed to go through JTC;
- vendor-internal servo data that cannot be obtained is marked unknown —
  "command received" is never treated as "motor executed";
- choosing an unimplemented backend is an EXPLICIT refusal, never a silent
  fallback;
- normal operation keeps a lightweight summary; high-frequency debug capture
  is opt-in per investigation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Mapping
import json
import os
import time


@dataclass(frozen=True)
class BackendDescription:
    """What a backend can and cannot observe (describe_backend)."""

    backend: str
    execution_path: str          # "jtc" | "vendor_executor" | "sim_gz" | "unknown"
    fields_command: tuple
    fields_feedback: tuple
    command_means_motor_executed: bool
    notes: str = ""


class CaptureBackend:
    """Reserved interface for real-machine adapters (four functions)."""

    def describe_backend(self) -> BackendDescription:
        raise NotImplementedError

    def record_command(self, record: Mapping) -> None:
        """Record an actually-issued command (not a plan/reference)."""
        raise NotImplementedError

    def record_feedback(self, record: Mapping) -> None:
        raise NotImplementedError

    def record_event(self, record: Mapping) -> None:
        raise NotImplementedError


class SimGzBackend(CaptureBackend):
    """Sim backend: driver/context-side events into the same run directory.

    The per-cycle plugin records are produced by the C++ recorder inside the
    gz process; this backend only appends context-layer JSONL rows.
    """

    def __init__(self, run_dir: str | Path):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self._commands = self.run_dir / "context_commands.jsonl"
        self._feedback = self.run_dir / "context_feedback.jsonl"
        self._events = self.run_dir / "context_events.jsonl"

    def describe_backend(self) -> BackendDescription:
        return BackendDescription(
            backend="sim_gz",
            execution_path="sim_gz",
            fields_command=("cycle", "joint_names", "positions", "source"),
            fields_feedback=("cycle", "positions", "velocities", "efforts"),
            # Velocity commands are enforced by the physics engine's velocity
            # constraint, which is a solver compromise under load — the motor
            # analogy does not transfer to hardware.
            command_means_motor_executed=False,
            notes="per-cycle plugin records come from the C++ recorder",
        )

    def _append(self, path: Path, record: Mapping) -> None:
        row = dict(record)
        row.setdefault("wall", time.time())
        row.setdefault("mono", time.monotonic())
        try:
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(row) + "\n")
        except OSError:
            return  # observation only; failure must never block the caller

    def record_command(self, record: Mapping) -> None:
        self._append(self._commands, record)

    def record_feedback(self, record: Mapping) -> None:
        self._append(self._feedback, record)

    def record_event(self, record: Mapping) -> None:
        self._append(self._events, record)


_BACKENDS: Dict[str, type] = {
    "sim_gz": SimGzBackend,
}


def available_backends() -> tuple:
    return tuple(sorted(_BACKENDS))


def get_backend(name: str, *args, **kwargs) -> CaptureBackend:
    """Explicit refusal for unimplemented backends — no silent fallback."""
    try:
        cls = _BACKENDS[name]
    except KeyError:
        raise NotImplementedError(
            f"capture backend '{name}' is not implemented; available: "
            f"{', '.join(available_backends())} (no fallback was applied)"
        ) from None
    return cls(*args, **kwargs)


# --------------------------------------------------------------- triggers ---
def allocate_event_id(run_dir: str | Path) -> int:
    """Next monotonic external trigger id for a run (>=1, never re-used)."""
    ctl = Path(run_dir) / "ctl"
    ctl.mkdir(parents=True, exist_ok=True)
    used = [0]
    for entry in ctl.iterdir():
        name = entry.name
        if name.startswith("trigger-"):
            tail = name[len("trigger-"):].split(".", 1)[0]
            try:
                used.append(int(tail))
            except ValueError:
                continue
    return max(used) + 1


def write_trigger(run_dir: str | Path, event_id: int, reason: str) -> Path:
    """Manual-service / settle-failure trigger via the unified ctl protocol.

    Same numbering and idempotency domain as the recorder's ctl polling.
    """
    ctl = Path(run_dir) / "ctl"
    ctl.mkdir(parents=True, exist_ok=True)
    path = ctl / f"trigger-{int(event_id)}"
    if path.exists() or Path(str(path) + ".done").exists():
        return path  # idempotent: this id was already delivered
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(reason.replace("\n", " ")[:96] + "\n")
    return path
