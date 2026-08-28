#!/usr/bin/env python3
"""The release-settle rule, replayed against real recorded traces.

EX1 blocked every placement run: the gate rejected an arm that was provably
motionless. The traces here were recorded from the running simulation
(phase 8) and pin down both directions of the rule, so it cannot regress into
either "never passes" or, far worse, "passes while the arm is moving".

No ROS: the decision is a pure function of a sampled trace.
"""

import json
import os
import sys
import unittest


from luggage_planning.settle_criterion import (  # noqa: E402
    DISPLACEMENT,
    VELOCITY,
    SettleTracker,
    position_excursions,
    settle_decision,
)

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

VEL_TOL = 0.03
HOLD_TIME = 0.25
TIMEOUT = 3.0


def load_trace(name):
    with open(os.path.join(DATA, name), "r") as stream:
        payload = json.load(stream)
    names = payload["joint_names"]
    return [
        (sample[0], dict(zip(names, sample[1])), dict(zip(names, sample[2])))
        for sample in payload["samples"]
    ]


def synthetic(duration, velocity, rate=50.0, joints=("j1", "j2")):
    """A trace where the joints really do creep at ``velocity`` rad/s."""
    samples = []
    step = 1.0 / rate
    count = int(duration * rate)
    for index in range(count):
        t = index * step
        position = velocity * t
        samples.append((
            t,
            {name: velocity for name in joints},
            {name: position for name in joints},
        ))
    return samples


class TestRecordedIdleTraces(unittest.TestCase):
    """A motionless arm must settle. Otherwise nothing can ever be placed."""

    def test_parked_arm_settles(self):
        trace = load_trace("settle_idle_observe.json")
        settled, _elapsed, diagnostics = settle_decision(
            trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=DISPLACEMENT)
        self.assertTrue(settled)
        self.assertLess(diagnostics["peak_excursion"], 1e-4)

    def test_extended_static_arm_settles_although_velocity_reads_high(self):
        """The EX1 blocker, reproduced from a recording.

        The shoulder is extended, the reported velocity exceeds the tolerance,
        and the arm is not moving at all.
        """
        trace = load_trace("settle_static_extended.json")
        excursions = position_excursions(trace)
        self.assertLess(max(excursions.values()), 1e-3)

        reported_peak = max(
            max(abs(v) for v in velocities.values())
            for _t, velocities, _p in trace)
        self.assertGreater(reported_peak, VEL_TOL)

        by_velocity, _e, _d = settle_decision(
            trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=VELOCITY)
        by_displacement, _e, _d = settle_decision(
            trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=DISPLACEMENT)
        self.assertFalse(by_velocity, "this is the blocker being reproduced")
        self.assertTrue(by_displacement)


class TestRecordedMotionTrace(unittest.TestCase):
    """A moving arm must never settle. This is the safety direction."""

    def test_mid_motion_is_rejected(self):
        trace = load_trace("settle_mid_motion.json")
        settled, _elapsed, diagnostics = settle_decision(
            trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=DISPLACEMENT)
        self.assertFalse(settled)
        self.assertGreater(diagnostics["peak_excursion"], VEL_TOL * HOLD_TIME)

    def test_the_old_velocity_rule_would_have_released_mid_motion(self):
        """Why the signal was changed, not just the estimator.

        Gazebo's kinematic position mode under-reports joint velocity by more
        than an order of magnitude, so the velocity rule calls this window
        settled while the joint is travelling at roughly 0.48 rad/s.
        """
        trace = load_trace("settle_mid_motion.json")
        span = trace[-1][0] - trace[0][0]
        travel = abs(
            trace[-1][2]["elfin_joint2"] - trace[0][2]["elfin_joint2"])
        actual_speed = travel / span
        reported_peak = max(
            abs(velocities["elfin_joint2"]) for _t, velocities, _p in trace)

        self.assertGreater(actual_speed, 0.4)
        self.assertLess(reported_peak, 0.04)
        self.assertGreater(actual_speed / reported_peak, 10.0)

        by_velocity, _e, _d = settle_decision(
            trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=VELOCITY)
        self.assertTrue(
            by_velocity,
            "recorded evidence that the old rule passed a moving arm")


class TestSyntheticDrift(unittest.TestCase):
    """Guard against an estimator that smooths real slow motion away."""

    def test_constant_creep_above_tolerance_is_rejected(self):
        trace = synthetic(duration=3.0, velocity=0.05)
        for criterion in (DISPLACEMENT, VELOCITY):
            settled, _elapsed, _d = settle_decision(
                trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=criterion)
            self.assertFalse(settled, criterion)

    def test_creep_just_below_tolerance_is_accepted(self):
        # The displacement budget is vel_tol * hold_time, so a joint moving at
        # half the tolerance uses half the budget and passes.
        trace = synthetic(duration=3.0, velocity=0.5 * VEL_TOL)
        settled, _elapsed, _d = settle_decision(
            trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=DISPLACEMENT)
        self.assertTrue(settled)

    def test_oscillation_that_returns_to_its_start_is_rejected(self):
        """Peak-to-peak, not start-to-end, or a shaking arm slips through."""
        samples = []
        for index in range(150):
            t = index / 50.0
            # 2 Hz, amplitude 0.02 rad: net displacement per period is zero.
            offset = 0.02 * ((index // 12) % 2)
            samples.append((t, {"j1": 0.0}, {"j1": offset}))
        settled, _elapsed, _d = settle_decision(
            samples, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=DISPLACEMENT)
        self.assertFalse(settled)


class TestTrackerContract(unittest.TestCase):
    def test_window_must_span_hold_time_before_any_verdict(self):
        tracker = SettleTracker(VEL_TOL, HOLD_TIME, criterion=DISPLACEMENT)
        self.assertFalse(tracker.update(0.0, {"j1": 0.0}, {"j1": 0.0}))
        self.assertFalse(tracker.update(0.1, {"j1": 0.0}, {"j1": 0.0}))
        self.assertTrue(tracker.update(0.30, {"j1": 0.0}, {"j1": 0.0}))

    def test_velocity_criterion_still_reproduces_the_original_rule(self):
        """The old behaviour stays available for comparison runs."""
        tracker = SettleTracker(VEL_TOL, HOLD_TIME, criterion=VELOCITY)
        self.assertFalse(tracker.update(0.0, {"j1": 0.001}))
        self.assertFalse(tracker.update(0.1, {"j1": 0.5}))   # resets the run
        self.assertFalse(tracker.update(0.2, {"j1": 0.001}))
        self.assertFalse(tracker.update(0.3, {"j1": 0.001}))
        self.assertTrue(tracker.update(0.46, {"j1": 0.001}))

    def test_diagnostics_expose_the_disagreement(self):
        trace = load_trace("settle_static_extended.json")
        _settled, _elapsed, diagnostics = settle_decision(
            trace, VEL_TOL, HOLD_TIME, TIMEOUT, criterion=DISPLACEMENT)
        # A large reported velocity next to a near-zero excursion is exactly
        # the fingerprint of the readback bias; both must be visible.
        self.assertGreater(diagnostics["peak_velocity"], VEL_TOL)
        self.assertLess(diagnostics["peak_excursion"], 1e-3)
        self.assertEqual(diagnostics["peak_joint"], "elfin_joint2")

    def test_empty_trace_is_not_settled(self):
        settled, elapsed, diagnostics = settle_decision(
            [], VEL_TOL, HOLD_TIME, TIMEOUT)
        self.assertFalse(settled)
        self.assertEqual(elapsed, 0.0)
        self.assertEqual(diagnostics["sample_count"], 0)


if __name__ == "__main__":
    unittest.main()
