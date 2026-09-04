#!/usr/bin/env python3
"""Decide whether the arm has settled, from a recorded or live joint trace.

Split out of ``motion_planner_node._wait_robot_settled`` so the decision can be
replayed against a recorded trace instead of only against a live Gazebo run.
``SettleTracker`` is the only implementation of the rule; the live poller feeds
it samples as they arrive and ``settle_decision`` replays a recording through
the same object, so there is no second copy to drift.

Why there are two criteria
--------------------------

The safety property is "the payload is not moving when the vacuum lets go".
The obvious signal for that is ``/joint_states.velocity``, and that is what the
original gate used. In this simulation it is the wrong signal.

The arm runs in Gazebo's kinematic position mode (``pid_gains: {}`` ->
``SetPosition``). In that mode the reported joint velocity carries a static,
configuration-dependent bias -- largest on the shoulder joint, growing with arm
extension -- that never decays. Measured on a provably motionless arm
(phase 8): 0.0012 rad/s parked at observe, 0.0275 rad/s with the shoulder
extended, while the actual position drifted by 1e-8 rad over ten seconds. Past
0.03 rad/s the gate rejects an arm that is not moving at all, which is what
stopped every placement run from committing a box.

``displacement`` measures the same physical property directly: over the hold
window, no joint may move more than ``vel_tol * hold_time``. That is the same
two numbers the velocity rule used, so nothing is loosened -- a joint genuinely
creeping at the tolerance still travels the full budget and is still rejected.
Peak-to-peak is used rather than start-to-end so an oscillation that returns to
where it started cannot slip through.

No ROS imports.
"""

from __future__ import division

# Every sample in the hold window must be below tolerance (the original rule).
STRICT_QUANTILE = 1.0

VELOCITY = "velocity"
DISPLACEMENT = "displacement"
CRITERIA = (VELOCITY, DISPLACEMENT)


def percentile(values, fraction):
    """Nearest-rank percentile. ``fraction`` in [0, 1]."""
    if not values:
        return 0.0
    ordered = sorted(values)
    if fraction <= 0.0:
        return ordered[0]
    if fraction >= 1.0:
        return ordered[-1]
    index = int(round(fraction * (len(ordered) - 1)))
    return ordered[index]


def sample_peak(velocities):
    """(max |v|, joint name) for one sample. ``velocities`` is {joint: v}."""
    peak_joint = ""
    peak = 0.0
    for name, value in velocities.items():
        magnitude = abs(float(value))
        if magnitude > peak:
            peak = magnitude
            peak_joint = name
    return peak, peak_joint


class SettleTracker(object):
    """Incremental settle rule, shared by the live poller and offline replay."""

    def __init__(self, vel_tol, hold_time, quantile=STRICT_QUANTILE,
                 criterion=DISPLACEMENT):
        self.vel_tol = float(vel_tol)
        self.hold_time = float(hold_time)
        self.quantile = float(quantile)
        self.criterion = criterion if criterion in CRITERIA else DISPLACEMENT
        # Same physical budget as the velocity rule: how far a joint may move
        # while it is supposed to be holding still.
        self.displacement_tol = self.vel_tol * self.hold_time

        self._first_elapsed = None
        self._last_elapsed = 0.0
        self._stable_since = None
        self._window = []
        self._position_window = []
        self._peaks = []
        self._excursions = []
        self.settled_at = None
        self.last_peak = 0.0
        self.peak_velocity = 0.0
        self.peak_joint = ""
        self.peak_time = 0.0
        self.peak_excursion = 0.0
        self.excursion_joint = ""

    def update(self, elapsed, velocities, positions=None):
        """Feed one sample. Returns True once the arm counts as settled."""
        peak, peak_joint = sample_peak(velocities)
        if self._first_elapsed is None:
            self._first_elapsed = elapsed
        self._last_elapsed = elapsed
        self.last_peak = peak
        self._peaks.append((elapsed, peak, peak_joint))
        if peak > self.peak_velocity:
            self.peak_velocity = peak
            self.peak_joint = peak_joint
            self.peak_time = elapsed

        if positions:
            self._position_window.append((elapsed, dict(positions)))
            while (self._position_window
                   and elapsed - self._position_window[0][0] > self.hold_time):
                self._position_window.pop(0)

        if self.criterion == DISPLACEMENT:
            settled = self._update_displacement(elapsed)
        elif self.quantile >= STRICT_QUANTILE:
            settled = self._update_strict(elapsed, peak)
        else:
            settled = self._update_quantile(elapsed, peak)
        if settled and self.settled_at is None:
            self.settled_at = elapsed
        return settled

    def _update_strict(self, elapsed, peak):
        if peak < self.vel_tol:
            if self._stable_since is None:
                self._stable_since = elapsed
            elif elapsed - self._stable_since >= self.hold_time:
                return True
        else:
            self._stable_since = None
        return False

    def _update_quantile(self, elapsed, peak):
        self._window.append((elapsed, peak))
        while self._window and elapsed - self._window[0][0] > self.hold_time:
            self._window.pop(0)
        if elapsed - self._first_elapsed < self.hold_time:
            return False
        if not self._window:
            return False
        below = sum(1 for _, value in self._window if value < self.vel_tol)
        return below >= self.quantile * len(self._window)

    def _update_displacement(self, elapsed):
        excursion, joint = self._window_excursion()
        self._excursions.append(excursion)
        if excursion > self.peak_excursion:
            self.peak_excursion = excursion
            self.excursion_joint = joint
        # The window has to actually span hold_time, or a single early sample
        # would satisfy the rule.
        if elapsed - self._first_elapsed < self.hold_time:
            return False
        if len(self._position_window) < 2:
            return False
        return excursion < self.displacement_tol

    def _window_excursion(self):
        """Largest peak-to-peak joint movement inside the hold window."""
        if len(self._position_window) < 2:
            return 0.0, ""
        extremes = {}
        for _elapsed, positions in self._position_window:
            for name, value in positions.items():
                value = float(value)
                low, high = extremes.get(name, (value, value))
                extremes[name] = (min(low, value), max(high, value))
        worst = 0.0
        worst_joint = ""
        for name, (low, high) in extremes.items():
            span = high - low
            if span > worst:
                worst = span
                worst_joint = name
        return worst, worst_joint

    @property
    def elapsed(self):
        if self._first_elapsed is None:
            return 0.0
        return self._last_elapsed - self._first_elapsed

    def diagnostics(self):
        """What a single peak-velocity number could not say."""
        base = {
            "criterion": self.criterion,
            "vel_tol": self.vel_tol,
            "displacement_tol": self.displacement_tol,
            "quantile": self.quantile,
        }
        if not self._peaks:
            base.update({
                "sample_count": 0,
                "peak_velocity": 0.0,
                "peak_joint": "",
                "peak_time": 0.0,
                "tail_velocity": 0.0,
                "tail_ratio": 0.0,
                "fraction_below_tol": 0.0,
                "p50": 0.0,
                "p95": 0.0,
                "peak_excursion": 0.0,
                "excursion_joint": "",
            })
            return base
        values = [peak for _, peak, _ in self._peaks]
        end = self._peaks[-1][0]
        tail = [
            peak for elapsed, peak, _ in self._peaks
            if end - elapsed <= self.hold_time
        ]
        tail_velocity = max(tail) if tail else 0.0
        base.update({
            "sample_count": len(self._peaks),
            "peak_velocity": self.peak_velocity,
            "peak_joint": self.peak_joint,
            "peak_time": self.peak_time,
            # tail_ratio near 1.0 means the reading is flat: it is not damping
            # out, and waiting longer will not help.
            "tail_velocity": tail_velocity,
            "tail_ratio": (
                tail_velocity / self.peak_velocity
                if self.peak_velocity > 0 else 0.0),
            "fraction_below_tol": (
                sum(1 for value in values if value < self.vel_tol)
                / len(values)),
            "p50": percentile(values, 0.50),
            "p95": percentile(values, 0.95),
            # The pair that exposes the readback bias: a large peak_velocity
            # next to a near-zero excursion means nothing actually moved.
            "peak_excursion": self.peak_excursion,
            "excursion_joint": self.excursion_joint,
        })
        return base


def settle_decision(samples, vel_tol, hold_time, timeout,
                    quantile=STRICT_QUANTILE, criterion=DISPLACEMENT):
    """Replay the settle rule over a chronological trace.

    ``samples`` is ``[(t, {joint: velocity}), ...]`` or
    ``[(t, {joint: velocity}, {joint: position}), ...]``.
    Returns ``(settled, elapsed, diagnostics)``.
    """
    tracker = SettleTracker(vel_tol, hold_time, quantile, criterion)
    if not samples:
        return False, 0.0, tracker.diagnostics()
    t0 = samples[0][0]
    for sample in samples:
        timestamp, velocities = sample[0], sample[1]
        positions = sample[2] if len(sample) > 2 else None
        elapsed = timestamp - t0
        if elapsed > timeout:
            break
        if tracker.update(elapsed, velocities, positions):
            return True, tracker.settled_at, tracker.diagnostics()
    return False, tracker.elapsed, tracker.diagnostics()


def format_diagnostics(diagnostics):
    """One-line summary for a status message."""
    return (
        "criterion=%s excursion=%.6f@%s tol=%.5f | vel_peak=%.4f@%s "
        "tail_ratio=%.2f n=%d"
        % (
            diagnostics.get("criterion", "?"),
            diagnostics.get("peak_excursion", 0.0),
            diagnostics.get("excursion_joint", "?") or "-",
            diagnostics.get("displacement_tol", 0.0),
            diagnostics.get("peak_velocity", 0.0),
            diagnostics.get("peak_joint", "?") or "-",
            diagnostics.get("tail_ratio", 0.0),
            diagnostics.get("sample_count", 0),
        ))


def trace_statistics(samples):
    """Per-joint and max-over-joints distribution for a recorded trace."""
    if not samples:
        return {"sample_count": 0, "joints": {}, "max_over_joints": {}}
    per_joint = {}
    combined = []
    for sample in samples:
        velocities = sample[1]
        peak, _ = sample_peak(velocities)
        combined.append(peak)
        for name, value in velocities.items():
            per_joint.setdefault(name, []).append(abs(float(value)))
    return {
        "sample_count": len(samples),
        "joints": {
            name: _distribution(values)
            for name, values in sorted(per_joint.items())
        },
        "max_over_joints": _distribution(combined),
    }


def position_excursions(samples):
    """Per-joint peak-to-peak movement across a whole recorded trace."""
    extremes = {}
    for sample in samples:
        if len(sample) < 3 or not sample[2]:
            continue
        for name, value in sample[2].items():
            value = float(value)
            low, high = extremes.get(name, (value, value))
            extremes[name] = (min(low, value), max(high, value))
    return {name: high - low for name, (low, high) in sorted(extremes.items())}


def _distribution(values):
    return {
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p100": percentile(values, 1.0),
        "mean": sum(values) / len(values) if values else 0.0,
    }
