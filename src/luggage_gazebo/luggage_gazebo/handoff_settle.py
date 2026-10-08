"""Bounded wait between two certified legs: measured joints must arrive.

Why this exists
---------------
``FollowJointTrajectory`` success is not arrival. Run08 (2026-10-08) saw the
controller report success for a 26.0 s transit while joint 2 was still
0.298 rad short of the leg's end; it kept creeping at about 0.077 rad/s for
4.08 s more. The next leg was submitted 1.65 s after success and the executor
refused it (``QP_START_STATE_MISMATCH``, drift 0.172 rad > 0.01 rad). The
executor gate was right. The handoff submitted too early.

Contract of this module
-----------------------
Pure logic, no ROS imports, injected clock and feedback source, so every
behaviour below is tested offline.

1. Continuity first. The previous leg's last submitted knot is compared with
   the next leg's first knot using the same 0.01 rad tolerance the executor
   applies. A structural mismatch is ``HANDOFF_ENDPOINT_MISMATCH`` at once. It
   is never converted into a settle timeout by waiting.
2. Then wait, without sending any motion and without widening the start gate.
   All six measured joints must be within the original tolerance of BOTH
   knots, and the arm must be still, continuously for ``hold_sec``.
3. Stillness is judged from two independent signals, and both must pass:
   the reported ``velocity`` (all six present, finite, ``|v| <= max``), and
   the finite difference of reported positions over the hold window.
   Missing velocity is never treated as zero. The finite-difference check is
   not redundant: in Run08 the reported velocity field read about -0.0007 rad/s
   (and with the wrong sign) while joint 2 was moving at +0.077 rad/s, so the
   velocity field alone would have declared the arm still.
4. Feedback must be fresh and strictly advancing in stamp. A duplicate or
   reordered stamp is ignored and counted. A gap longer than
   ``max_sample_gap_sec`` between accepted samples breaks the hold run. A
   newest sample older than ``max_feedback_age_sec`` (receive clock) cannot
   satisfy the verdict. Stalled feedback is never "stable".
5. The wait has a hard monotonic wall-clock deadline measured from the moment
   the previous action's success result was received. Expiry returns
   ``HANDOFF_SETTLE_TIMEOUT``; the caller must not submit the next leg.
6. ``should_abort`` is polled every loop (cancellation, vacuum loss, scene or
   admission invalidation). Any abort stops the wait with that reason.

The defaults are a bounded engineering choice for non-scored simulation
rehearsal. They are not a measured convergence bound (the only observed tail
is one 4.08 s sample) and they are not approved for hardware.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass

HANDOFF_SETTLE_TIMEOUT = "HANDOFF_SETTLE_TIMEOUT"
HANDOFF_ENDPOINT_MISMATCH = "HANDOFF_ENDPOINT_MISMATCH"

ARM_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))

# Executor start gate (motion_planner_node / QP start check). Never widen it.
START_TOLERANCE_RAD = 0.01

# Cap on retained per-sample rows so a pathological feedback rate cannot
# grow the record without bound. 6 s at 100 Hz is 600 rows.
MAX_TRACE_ROWS = 1500


@dataclass(frozen=True)
class HandoffPolicy:
    timeout_sec: float = 6.0
    max_abs_velocity: float = 0.01
    hold_sec: float = 0.3
    tolerance_rad: float = START_TOLERANCE_RAD
    max_sample_gap_sec: float = 0.25
    max_feedback_age_sec: float = 0.5
    min_hold_samples: int = 3

    def as_dict(self):
        return asdict(self)


def angular_gap(left, right):
    """Shortest absolute joint distance; same metric as the executor gate."""
    gap = abs(float(left) - float(right))
    return min(gap, (2.0 * math.pi) - gap)


def canonical_positions(names, values, wanted=ARM_JOINTS):
    """``values`` ordered as elfin_joint1..6, or None if names cannot."""
    if names is None or values is None:
        return None
    try:
        names = [str(name) for name in names]
        values = [float(value) for value in values]
    except (TypeError, ValueError):
        return None
    if len(names) != len(values) or len(names) != len(wanted):
        return None
    if len(set(names)) != len(names):
        return None
    if any(not math.isfinite(value) for value in values):
        return None
    if tuple(names) == tuple(wanted):
        return values
    index = dict(zip(names, values))
    if any(name not in index for name in wanted):
        return None
    return [index[name] for name in wanted]


def _round(values):
    if values is None:
        return None
    return [round(float(value), 6) for value in values]


def check_endpoint_continuity(previous_names, previous_last, next_names,
                              next_first,
                              tolerance=START_TOLERANCE_RAD):
    """Compare the previous leg's last knot with the next leg's first knot.

    ``status`` is one of ``continuous``, ``discontinuous``, ``joint_order``,
    ``missing_endpoint``. Only ``continuous`` permits waiting.
    """
    record = {
        "status": "missing_endpoint",
        "tolerance_rad": float(tolerance),
        "previous_last_joints": None,
        "next_first_joints": None,
        "delta_per_joint": None,
        "max_delta": None,
    }
    if previous_last is None or next_first is None:
        return record
    previous = canonical_positions(previous_names, previous_last)
    following = canonical_positions(next_names, next_first)
    if previous is None or following is None:
        record["status"] = "joint_order"
        return record
    deltas = [angular_gap(a, b) for a, b in zip(previous, following)]
    record["previous_last_joints"] = _round(previous)
    record["next_first_joints"] = _round(following)
    record["delta_per_joint"] = _round(deltas)
    record["max_delta"] = round(max(deltas), 6)
    record["status"] = (
        "continuous" if max(deltas) <= float(tolerance) else "discontinuous")
    return record


def _velocity_vector(sample):
    """Six finite reported velocities in canonical order, or None."""
    raw = sample.get("velocity")
    if raw is None:
        return None
    names = sample.get("velocity_name") or sample.get("name")
    return canonical_positions(names, raw)


class HandoffSettleTracker:
    """Accumulates fresh feedback and decides when the arm has arrived."""

    def __init__(self, policy, previous_last, next_first):
        self.policy = policy
        self.previous_last = list(previous_last)
        self.next_first = list(next_first)
        self.rows = []
        self.rejected = {}
        self.last_stamp = None
        self.last_rx_mono = None
        self._run = []
        self.accepted = 0

    def _reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason, 0) + 1
        return reason

    def observe(self, sample, rx_mono):
        """Feed one joint-state sample. Returns the row, or a reject reason."""
        try:
            stamp = float(sample.get("stamp"))
        except (AttributeError, TypeError, ValueError):
            return self._reject("bad_stamp")
        if not math.isfinite(stamp):
            return self._reject("bad_stamp")
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # Duplicate or reordered: carries no new information.
            return self._reject("non_advancing_stamp")
        positions = canonical_positions(
            sample.get("name"), sample.get("position"))
        if positions is None:
            self._run = []
            self.last_stamp = stamp
            self.last_rx_mono = rx_mono
            return self._reject("joint_order_or_nonfinite")
        if (self.last_stamp is not None
                and stamp - self.last_stamp > self.policy.max_sample_gap_sec):
            self._run = []
            self.rejected["feedback_gap_resets"] = (
                self.rejected.get("feedback_gap_resets", 0) + 1)
        self.last_stamp = stamp
        self.last_rx_mono = rx_mono
        self.accepted += 1

        versus_previous = [
            angular_gap(a, b) for a, b in zip(positions, self.previous_last)]
        versus_next = [
            angular_gap(a, b) for a, b in zip(positions, self.next_first)]
        tol = self.policy.tolerance_rad
        in_tolerance = max(versus_previous) <= tol and max(versus_next) <= tol
        velocity = _velocity_vector(sample)
        velocity_present = velocity is not None
        velocity_ok = velocity_present and all(
            abs(value) <= self.policy.max_abs_velocity for value in velocity)

        if in_tolerance and velocity_ok:
            self._run.append((stamp, positions))
        else:
            self._run = []
        row = {
            "stamp": stamp,
            "rx_mono": rx_mono,
            "position": _round(positions),
            "velocity": _round(velocity),
            "velocity_present": velocity_present,
            "max_delta_previous_last": round(max(versus_previous), 6),
            "max_delta_next_first": round(max(versus_next), 6),
            "in_tolerance": in_tolerance,
            "velocity_ok": velocity_ok,
            "run_sec": round(self._run_span(), 4),
        }
        if len(self.rows) < MAX_TRACE_ROWS:
            self.rows.append(row)
        else:
            self.rejected["trace_rows_dropped"] = (
                self.rejected.get("trace_rows_dropped", 0) + 1)
        return row

    def _run_span(self):
        if len(self._run) < 2:
            return 0.0
        return self._run[-1][0] - self._run[0][0]

    def _window_rate(self):
        """Max per-joint (max-min)/duration over the trailing hold window."""
        if not self._run:
            return None, 0
        end = self._run[-1][0]
        window = [item for item in self._run
                  if item[0] >= end - self.policy.hold_sec]
        if len(window) < 2:
            return None, len(window)
        duration = window[-1][0] - window[0][0]
        if duration <= 0.0:
            return None, len(window)
        worst = 0.0
        for joint in range(len(ARM_JOINTS)):
            column = [item[1][joint] for item in window]
            worst = max(worst, max(column) - min(column))
        return worst / duration, len(window)

    def verdict(self, now_mono):
        """``ready`` only when the hold window is satisfied by fresh data."""
        reasons = []
        if self.last_rx_mono is None:
            reasons.append("no_feedback")
        elif now_mono - self.last_rx_mono > self.policy.max_feedback_age_sec:
            reasons.append("feedback_stalled")
        rate, window_samples = self._window_rate()
        if self._run_span() < self.policy.hold_sec:
            reasons.append("hold_not_reached")
        elif window_samples < self.policy.min_hold_samples:
            reasons.append("too_few_hold_samples")
        elif rate is None or rate > self.policy.max_abs_velocity:
            reasons.append("position_still_moving")
        newest = self.rows[-1] if self.rows else None
        if newest is not None and not newest["in_tolerance"]:
            reasons.append("outside_tolerance")
        if newest is not None and not newest["velocity_present"]:
            reasons.append("velocity_missing")
        elif newest is not None and not newest["velocity_ok"]:
            reasons.append("velocity_above_limit")
        return {
            "status": "ready" if not reasons else "waiting",
            "reasons": reasons,
            "window_rate_rad_s": None if rate is None else round(rate, 6),
            "hold_run_sec": round(self._run_span(), 4),
            "window_samples": window_samples,
        }


def await_handoff(policy, previous, following, *, read_samples, now_mono,
                  sleep, started_mono, should_abort=None, poll_sec=0.02):
    """Run the continuity check, then the bounded measured-state wait.

    ``previous`` and ``following`` are ``(names, joints)`` tuples: the last
    knot of the previous submitted leg and the first knot of the next leg.
    ``read_samples()`` returns new ``(sample, rx_mono)`` pairs since the last
    call (the first call may return history received after ``started_mono``).
    ``now_mono`` and ``sleep`` are injected so tests control time.
    ``should_abort()`` returns a reason string to stop, or a falsy value.

    The returned ``status`` is ``ready``, ``timeout``, ``endpoint_mismatch``
    or ``aborted``. ``code`` is the failure code for the last three. This
    function never submits or commands motion.
    """
    continuity = check_endpoint_continuity(
        previous[0], previous[1], following[0], following[1],
        tolerance=policy.tolerance_rad)
    record = {
        "policy": policy.as_dict(),
        "continuity": continuity,
        "status": None,
        "code": "",
        "waited_sec": 0.0,
        "deadline_mono": float(started_mono) + float(policy.timeout_sec),
        "samples": [],
        "rejected": {},
        "last_verdict": None,
        "final_start_check": None,
    }
    if continuity["status"] != "continuous":
        record["status"] = "endpoint_mismatch"
        record["code"] = HANDOFF_ENDPOINT_MISMATCH
        return record

    canonical_previous = canonical_positions(previous[0], previous[1])
    canonical_next = canonical_positions(following[0], following[1])
    tracker = HandoffSettleTracker(policy, canonical_previous, canonical_next)
    begun = now_mono()
    deadline = float(started_mono) + float(policy.timeout_sec)
    verdict = None
    while True:
        reason = should_abort() if should_abort is not None else None
        if reason:
            record["status"] = "aborted"
            record["code"] = str(reason)
            break
        for sample, rx_mono in read_samples():
            tracker.observe(sample, rx_mono)
        now = now_mono()
        verdict = tracker.verdict(now)
        if verdict["status"] == "ready":
            record["status"] = "ready"
            break
        if now >= deadline:
            record["status"] = "timeout"
            record["code"] = HANDOFF_SETTLE_TIMEOUT
            break
        sleep(poll_sec)
    record["waited_sec"] = round(now_mono() - begun, 3)
    record["last_verdict"] = verdict
    record["rejected"] = dict(tracker.rejected)
    record["samples"] = list(tracker.rows)
    newest = tracker.rows[-1] if tracker.rows else None
    if newest is not None:
        record["final_start_check"] = {
            "sample_stamp": newest["stamp"],
            "max_delta_next_first": newest["max_delta_next_first"],
            "max_delta_previous_last": newest["max_delta_previous_last"],
            "tolerance_rad": policy.tolerance_rad,
            "within_tolerance": bool(newest["in_tolerance"]),
        }
    if (record["status"] == "ready"
            and not (record["final_start_check"] or {}).get(
                "within_tolerance")):
        # Defensive: ready is derived from the same rows, so this is
        # unreachable unless the tracker is changed. Fail closed.
        record["status"] = "timeout"
        record["code"] = HANDOFF_SETTLE_TIMEOUT
    return record


def summarize_for_extras(record, max_rows=40):
    """Small success-path summary; keeps first, last and an even sample."""
    rows = list(record.get("samples") or [])
    if len(rows) > max_rows:
        step = max(1, len(rows) // max_rows)
        picked = rows[::step]
        if picked[-1] is not rows[-1]:
            picked.append(rows[-1])
        rows = picked
    slim = dict(record)
    slim["samples"] = rows
    slim["samples_total"] = len(record.get("samples") or [])
    return slim
