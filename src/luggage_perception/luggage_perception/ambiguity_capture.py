"""Same-stamp capture of inputs the detector already consumed.

A second subscription is not used to pair detections with the cloud.
The caller passes the components it has at the consume boundary, and
names the ones it does not. An incomplete set is written. Waiting does
not drop the stamp from memory without a file.

Per epoch the first anomaly and the first valid control stay on disk.
Later repeats do not replace them. Unsealed stamps that exceed the
pending bound, or that pass the timeout, are written as incomplete
records with the reason. Disk writes run on ``flush`` (tests) or a
background thread (the detector), not on the consume call itself.
"""

import json
import queue
import threading
from pathlib import Path

import numpy as np

ROLES = ("anomaly", "control")


def _epoch_key(epoch):
    epoch = epoch or {}
    return "%s:%s" % (
        epoch.get("instance_id") or "",
        int(epoch.get("generation") or 0))


def _safe(text):
    out = []
    for ch in str(text):
        out.append(ch if ch.isalnum() or ch in "._-" else "_")
    return "".join(out) or "epoch"


class FrameCapture(object):
    def __init__(self, root, max_pending=8, timeout_sec=1.0, writer=None):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.max_pending = int(max_pending)
        self.timeout_sec = float(timeout_sec)
        self._writer = writer or self._write_job
        self.counts = {
            "received": 0,
            "matched": 0,
            "expired": 0,
            "overflowed": 0,
            "written": 0,
            "write_failures": 0,
            "repeats": 0,
        }
        self._pending = {}
        self._pins = {}
        self._jobs = queue.Queue()
        self._lock = threading.Lock()
        self._thread = None

    def start(self):
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def receive(self, stamp_ns, epoch, component, payload, now):
        """One component already in hand. ``payload is None`` is not used."""
        with self._lock:
            self.counts["received"] += 1
            self._ensure_locked(int(stamp_ns), epoch, float(now))
            slot = self._pending[int(stamp_ns)]
            slot["components"][component] = payload
            slot["missing"].pop(component, None)

    def mark_missing(self, stamp_ns, epoch, component, reason, now):
        with self._lock:
            self._ensure_locked(int(stamp_ns), epoch, float(now))
            slot = self._pending[int(stamp_ns)]
            if component not in slot["components"]:
                slot["missing"][component] = str(reason)

    def seal(self, stamp_ns, role, now):
        """The caller will not add more for this stamp. Write what exists."""
        with self._lock:
            stamp_ns = int(stamp_ns)
            slot = self._pending.pop(stamp_ns, None)
            if slot is None:
                return "absent"
            slot["role"] = role
            slot["closed"] = "sealed"
            if not slot["missing"]:
                self.counts["matched"] += 1
            job = self._job(stamp_ns, slot)
        return self._accept(job)

    def expire(self, now):
        """Write unsealed stamps whose wait has ended. The file stays."""
        jobs = []
        with self._lock:
            now = float(now)
            due = [
                stamp for stamp, slot in self._pending.items()
                if now - slot["opened_at"] >= self.timeout_sec]
            for stamp in due:
                slot = self._pending.pop(stamp)
                slot["closed"] = "timeout"
                slot["missing"]["timeout"] = "unsealed_at_timeout"
                self.counts["expired"] += 1
                jobs.append(self._job(stamp, slot))
        results = [self._accept(job) for job in jobs]
        return results

    def already_pinned(self, epoch, role):
        return role in self._pins.get(_epoch_key(epoch), {})

    def note_repeat(self, epoch, role):
        if role not in ROLES:
            return
        with self._lock:
            self.counts["repeats"] += 1
        self._write_status()

    def consume(self, stamp_ns, epoch, role, components, missing, now=0.0):
        """Detector boundary: every consumed input, in one call."""
        stamp_ns = int(stamp_ns)
        for name, payload in (components or {}).items():
            self.receive(stamp_ns, epoch, name, payload, now)
        for name, reason in (missing or {}).items():
            self.mark_missing(stamp_ns, epoch, name, reason, now)
        return self.seal(stamp_ns, role, now)

    def flush(self):
        while True:
            try:
                job = self._jobs.get_nowait()
            except queue.Empty:
                break
            self._run_job(job)
        self._write_status()

    def _loop(self):
        while True:
            job = self._jobs.get()
            self._run_job(job)
            self._write_status()

    def _ensure_locked(self, stamp_ns, epoch, now):
        if stamp_ns in self._pending:
            return
        while len(self._pending) >= self.max_pending:
            oldest = min(self._pending, key=lambda s: self._pending[s]["opened_at"])
            slot = self._pending.pop(oldest)
            slot["closed"] = "overflow_before_seal"
            slot["missing"]["pending"] = "overflow_before_seal"
            self.counts["overflowed"] += 1
            job = self._job(oldest, slot)
            self._jobs.put(job)
        self._pending[stamp_ns] = {
            "epoch": dict(epoch or {}),
            "opened_at": now,
            "components": {},
            "missing": {},
            "role": None,
            "closed": None,
        }

    def _job(self, stamp_ns, slot):
        return {
            "stamp_ns": int(stamp_ns),
            "epoch": dict(slot["epoch"]),
            "role": slot.get("role"),
            "closed": slot.get("closed"),
            "components": dict(slot["components"]),
            "missing": dict(slot["missing"]),
        }

    def _accept(self, job):
        role = job.get("role")
        epoch = _epoch_key(job["epoch"])
        if role in ROLES:
            pins = self._pins.setdefault(epoch, {})
            if role in pins:
                self.counts["repeats"] += 1
                self._write_status()
                return "repeat"
            pins[role] = job["stamp_ns"]
        self._jobs.put(job)
        return "queued"

    def _run_job(self, job):
        try:
            self._writer(job)
        except Exception as exc:  # noqa: BLE001 - count and leave a stub
            self.counts["write_failures"] += 1
            self._write_failure_stub(job, exc)
            return
        self.counts["written"] += 1

    def _directory(self, job):
        epoch = _safe(_epoch_key(job["epoch"]))
        role = job.get("role") or job.get("closed") or "record"
        return self.root / epoch / ("%s_%d" % (_safe(role), job["stamp_ns"]))

    def _write_job(self, job):
        out = self._directory(job)
        if out.exists():
            return
        out.mkdir(parents=True, exist_ok=True)
        present = {}
        arrays = {}
        for name, value in job["components"].items():
            present[name] = {"present": True}
            if isinstance(value, np.ndarray):
                arrays[name] = value
            else:
                present[name]["value"] = value
        for name, reason in job["missing"].items():
            present[name] = {"present": False, "reason": reason}
        meta = {
            "stamp_ns": job["stamp_ns"],
            "epoch": job["epoch"],
            "role": job.get("role"),
            "closed": job.get("closed"),
            "components": present,
            "evidence_complete": not job["missing"],
        }
        (out / "frame.json").write_text(
            json.dumps(meta, indent=2, sort_keys=True) + "\n")
        if arrays:
            np.savez_compressed(out / "arrays.npz", **arrays)

    def _write_failure_stub(self, job, exc):
        out = self._directory(job)
        out.mkdir(parents=True, exist_ok=True)
        (out / "frame.json").write_text(json.dumps({
            "stamp_ns": job["stamp_ns"],
            "epoch": job["epoch"],
            "closed": "write_failed",
            "error": str(exc),
            "evidence_complete": False,
        }, indent=2, sort_keys=True) + "\n")

    def _write_status(self):
        pins = {
            epoch: dict(roles) for epoch, roles in self._pins.items()}
        payload = {
            "counts": dict(self.counts),
            "pins": pins,
            "evidence_incomplete": (
                self.counts["expired"] + self.counts["overflowed"]
                + self.counts["write_failures"]) > 0,
        }
        (self.root / "status.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n")


def selected_detection_index(detections):
    """Highest confidence, then bbox area. The index is recorded, not applied."""
    best_i = None
    best_key = None
    for i, det in enumerate(detections):
        bbox = det.get("bbox") or (0, 0, 0, 0)
        area = max(0, int(bbox[2]) - int(bbox[0])) * max(
            0, int(bbox[3]) - int(bbox[1]))
        key = (float(det.get("confidence") or 0.0), area)
        if best_key is None or key > best_key:
            best_key = key
            best_i = i
    return best_i
