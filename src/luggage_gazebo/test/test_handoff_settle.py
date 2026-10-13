#!/usr/bin/env python3
"""Behavioural tests for the bounded leg-to-leg handoff wait.

Run08 (2026-10-08): the controller reported success while joint 2 was still
0.298 rad short and creeping at 0.077 rad/s; the reported velocity read about
-0.0007 rad/s. The tests drive ``await_handoff`` with an injected clock and a
synthetic 100 Hz feedback feed, so every case is deterministic and offline.

The synthetic Run08 tail tests the algorithm. It is not evidence about the
historical trajectory: the previous leg's last knot was never captured.
"""

import math
import os
import sys
import unittest

REPO = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "luggage_gazebo"))

from luggage_gazebo.handoff_settle import (  # noqa: E402
    ARM_JOINTS,
    HANDOFF_ENDPOINT_MISMATCH,
    HANDOFF_SETTLE_TIMEOUT,
    HandoffPolicy,
    HandoffSettleTracker,
    START_TOLERANCE_RAD,
    await_handoff,
    check_endpoint_continuity,
    summarize_for_extras,
)

TARGET = [-2.218311, -1.212007, 1.87346, -2.237272, -1.567003, -0.64752]
NAMES = list(ARM_JOINTS)
PERIOD = 0.01


class FakeTime(object):
    """Monotonic clock whose ``sleep`` advances it."""

    def __init__(self, start=1000.0):
        self.start = start
        self.now = start
        self.sleeps = 0

    def mono(self):
        return self.now

    def sleep(self, seconds):
        self.sleeps += 1
        self.now += seconds


class Feed(object):
    """Joint-state feed at 100 Hz relative to the action result (t=0).

    ``position(t)`` and ``velocity(t)`` give the six joints at seconds ``t``
    after the result. ``alive(t)`` gates publication (False = feed stopped).
    """

    def __init__(self, clock, position, velocity=None, alive=None,
                 names=None, stamp_of=None, repeat=1):
        self.clock = clock
        self.position = position
        self.velocity = velocity or (lambda t: [0.0] * 6)
        self.alive = alive or (lambda t: True)
        self.names = names or NAMES
        self.stamp_of = stamp_of or (lambda k: 100.0 + k * PERIOD)
        self.repeat = repeat
        self.next_k = 0
        self.reads = 0

    def read(self):
        self.reads += 1
        out = []
        while True:
            t = self.next_k * PERIOD
            if self.clock.start + t > self.clock.now:
                break
            self.next_k += 1
            if not self.alive(t):
                continue
            pos = self.position(t)
            vel = self.velocity(t)
            if self.names != NAMES:
                index = {n: i for i, n in enumerate(NAMES)}
                pos = [pos[index[n]] for n in self.names]
                vel = None if vel is None else [
                    vel[index[n]] for n in self.names]
            sample = {
                "stamp": self.stamp_of(self.next_k - 1),
                "name": list(self.names),
                "position": list(pos),
                "velocity": None if vel is None else list(vel),
            }
            for _ in range(self.repeat):
                out.append((dict(sample), self.clock.start + t))
        return out


def offset(joint, value, base=None):
    pos = list(base or TARGET)
    pos[joint] += value
    return pos


def run(feed, clock, policy=None, previous=None, following=None,
        should_abort=None):
    policy = policy or HandoffPolicy()
    previous = previous or (NAMES, TARGET)
    following = following or (NAMES, TARGET)
    return await_handoff(
        policy, previous, following,
        read_samples=feed.read, now_mono=clock.mono, sleep=clock.sleep,
        started_mono=clock.start, should_abort=should_abort)


class TestGateIsNotWidened(unittest.TestCase):
    def test_default_tolerance_is_the_executor_gate(self):
        self.assertEqual(HandoffPolicy().tolerance_rad, 0.01)
        self.assertEqual(START_TOLERANCE_RAD, 0.01)

    def test_just_outside_the_gate_never_becomes_ready(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: offset(1, 0.0101))
        record = run(feed, clock)
        self.assertEqual(record["status"], "timeout")
        self.assertEqual(record["code"], HANDOFF_SETTLE_TIMEOUT)

    def test_just_inside_the_gate_is_ready(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: offset(1, 0.0099))
        record = run(feed, clock)
        self.assertEqual(record["status"], "ready")


class TestAlreadySettled(unittest.TestCase):
    def test_ready_after_the_hold_window_not_before(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET))
        record = run(feed, clock)
        self.assertEqual(record["status"], "ready")
        self.assertGreaterEqual(record["waited_sec"], 0.3)
        self.assertLess(record["waited_sec"], 0.5)
        check = record["final_start_check"]
        self.assertTrue(check["within_tolerance"])
        self.assertLessEqual(check["max_delta_next_first"], 0.01)

    def test_joint_order_is_normalised_by_name(self):
        clock = FakeTime()
        feed = Feed(
            clock, lambda t: list(TARGET),
            names=["elfin_joint1", "elfin_joint2", "elfin_joint5",
                   "elfin_joint3", "elfin_joint4", "elfin_joint6"])
        record = run(feed, clock)
        self.assertEqual(record["status"], "ready")


class TestDelayedConvergence(unittest.TestCase):
    ARRIVAL = 3.87

    def _creep(self, t):
        # Run08 shape: joint 2 0.298 rad short, +0.077 rad/s until arrival.
        deficit = max(0.0, 0.298 - 0.077 * t)
        return offset(1, -deficit)

    def test_waits_for_arrival_even_when_reported_velocity_reads_still(self):
        clock = FakeTime()
        # The reported velocity lies: -0.0007 rad/s while moving +0.077.
        feed = Feed(clock, self._creep,
                    velocity=lambda t: [0.0, -0.0007, 0, 0, 0, 0])
        record = run(feed, clock)
        self.assertEqual(record["status"], "ready")
        self.assertGreaterEqual(record["waited_sec"], self.ARRIVAL)
        self.assertLess(record["waited_sec"], 4.8)
        rows = record["samples"]
        early = [row for row in rows if row["stamp"] < 100.0 + 1.0]
        self.assertTrue(early)
        self.assertTrue(all(row["velocity_ok"] for row in early))
        self.assertFalse(any(row["in_tolerance"] for row in early))

    def test_in_tolerance_but_still_drifting_waits_for_the_plateau(self):
        # Inside +-0.009 for the whole run, drifting 0.045 rad/s, then flat.
        clock = FakeTime()

        def pos(t):
            drift = min(t, 0.4) * 0.045
            return offset(1, -0.009 + drift)

        feed = Feed(clock, pos)
        record = run(feed, clock)
        self.assertEqual(record["status"], "ready")
        # Drift stops at 0.4 s. The 0.3 s trailing window may carry at most
        # 0.01 rad/s, so ready cannot come before the window is nearly flat.
        self.assertGreaterEqual(record["waited_sec"], 0.6)
        # Mid-drift the run exists and the velocity field says "still",
        # yet the verdict is waiting because positions are moving.
        probe_clock = FakeTime()
        probe = Feed(probe_clock, pos)
        probe_clock.now = probe_clock.start + 0.36
        mid = HandoffSettleTracker(
            HandoffPolicy(), list(TARGET), list(TARGET))
        for sample, rx in probe.read():
            mid.observe(sample, rx)
        verdict = mid.verdict(probe_clock.now)
        self.assertEqual(verdict["status"], "waiting")
        self.assertIn("position_still_moving", verdict["reasons"])


class TestOscillationWithZeroNetDisplacement(unittest.TestCase):
    def test_back_and_forth_inside_tolerance_is_not_stable(self):
        # +-0.006 rad at 10 Hz: the arm is inside the 0.01 rad gate, the
        # reported velocity says still, and over each 0.3 s window (three
        # whole periods) the NET displacement is zero. Span, not net
        # displacement, must refuse it.
        clock = FakeTime()
        feed = Feed(clock, lambda t: offset(
            1, 0.006 * math.sin(2.0 * math.pi * 10.0 * t)))
        record = run(feed, clock)
        self.assertEqual(record["status"], "timeout")
        self.assertIn("position_still_moving",
                      record["last_verdict"]["reasons"])


class TestRefusals(unittest.TestCase):
    def _assert_timeout(self, record, clock):
        self.assertEqual(record["status"], "timeout")
        self.assertEqual(record["code"], HANDOFF_SETTLE_TIMEOUT)
        self.assertGreaterEqual(record["waited_sec"], 5.97)
        self.assertLessEqual(record["waited_sec"], 6.1)

    def test_never_converging_times_out_at_the_deadline(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: offset(1, -0.05))
        record = run(feed, clock)
        self._assert_timeout(record, clock)

    def test_timeout_is_measured_from_the_action_result(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: offset(1, -0.05))
        clock.now += 5.5  # the driver was slow to enter the wait
        record = run(feed, clock)
        self.assertEqual(record["status"], "timeout")
        self.assertLess(record["waited_sec"], 0.6)

    def test_oscillating_around_the_threshold_never_holds(self):
        clock = FakeTime()

        def pos(t):
            return offset(1, 0.009 if int(t / 0.1) % 2 == 0 else 0.011)

        feed = Feed(clock, pos)
        record = run(feed, clock)
        self._assert_timeout(record, clock)

    def test_in_tolerance_with_reported_velocity_above_limit_is_refused(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET),
                    velocity=lambda t: [0.0, 0.02, 0, 0, 0, 0])
        record = run(feed, clock)
        self._assert_timeout(record, clock)
        self.assertIn("velocity_above_limit",
                      record["last_verdict"]["reasons"])

    def test_missing_velocity_is_not_zero(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET), velocity=lambda t: None)
        record = run(feed, clock)
        self._assert_timeout(record, clock)
        self.assertIn("velocity_missing", record["last_verdict"]["reasons"])

    def test_partial_velocity_is_not_zero(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET),
                    velocity=lambda t: [0.0] * 5)
        record = run(feed, clock)
        self._assert_timeout(record, clock)

    def test_non_finite_velocity_is_refused(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET),
                    velocity=lambda t: [0.0, float("nan"), 0, 0, 0, 0])
        record = run(feed, clock)
        self._assert_timeout(record, clock)

    def test_feedback_that_stops_before_the_hold_never_becomes_ready(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET), alive=lambda t: t < 0.2)
        record = run(feed, clock)
        self._assert_timeout(record, clock)

    def test_stale_newest_sample_cannot_satisfy_the_verdict(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET))
        clock.now += 0.5
        tracker = HandoffSettleTracker(
            HandoffPolicy(), list(TARGET), list(TARGET))
        for sample, rx in feed.read():
            tracker.observe(sample, rx)
        self.assertEqual(tracker.verdict(clock.now)["status"], "ready")
        stalled = tracker.verdict(clock.now + 2.0)
        self.assertEqual(stalled["status"], "waiting")
        self.assertIn("feedback_stalled", stalled["reasons"])

    def test_frozen_stamp_is_never_stable(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET),
                    stamp_of=lambda k: 100.0)
        record = run(feed, clock)
        self._assert_timeout(record, clock)
        self.assertGreater(record["rejected"]["non_advancing_stamp"], 100)

    def test_duplicated_samples_are_ignored_but_do_not_block_ready(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET), repeat=3)
        record = run(feed, clock)
        self.assertEqual(record["status"], "ready")
        self.assertGreater(record["rejected"]["non_advancing_stamp"], 0)

    def test_reordered_older_sample_is_ignored(self):
        tracker = HandoffSettleTracker(
            HandoffPolicy(), list(TARGET), list(TARGET))
        base = {"name": NAMES, "position": list(TARGET),
                "velocity": [0.0] * 6}
        tracker.observe(dict(base, stamp=10.02), 1.0)
        reason = tracker.observe(dict(base, stamp=10.01), 1.01)
        self.assertEqual(reason, "non_advancing_stamp")
        self.assertEqual(tracker.rejected["non_advancing_stamp"], 1)

    def test_feedback_gap_restarts_the_hold(self):
        clock = FakeTime()
        # Alive 0..0.2, dead 0.2..0.7, alive again: hold restarts at 0.7.
        feed = Feed(clock, lambda t: list(TARGET),
                    alive=lambda t: t < 0.2 or t >= 0.7)
        record = run(feed, clock)
        self.assertEqual(record["status"], "ready")
        self.assertGreaterEqual(record["waited_sec"], 0.99)
        self.assertEqual(record["rejected"].get("feedback_gap_resets"), 1)

    def test_must_be_near_both_endpoints(self):
        following = list(TARGET)
        following[0] += 0.005
        previous = list(TARGET)
        clock = FakeTime()
        # 0.008 behind the previous knot is 0.013 from the next one.
        feed = Feed(clock, lambda t: offset(0, -0.008))
        record = run(feed, clock, previous=(NAMES, previous),
                     following=(NAMES, following))
        self.assertEqual(record["status"], "timeout")
        clock2 = FakeTime()
        feed2 = Feed(clock2, lambda t: offset(0, 0.0025))
        record2 = run(feed2, clock2, previous=(NAMES, previous),
                      following=(NAMES, following))
        self.assertEqual(record2["status"], "ready")

    def test_wrong_joint_names_are_refused_not_assumed(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET))
        original = feed.read

        def read():
            out = original()
            for sample, _rx in out:
                sample["name"] = ["a", "b", "c", "d", "e", "f"]
            return out

        feed.read = read
        record = run(feed, clock)
        self._assert_timeout(record, clock)


class TestStructuralMismatch(unittest.TestCase):
    def test_discontinuous_endpoints_are_rejected_without_waiting(self):
        following = list(TARGET)
        following[2] += 0.05
        clock = FakeTime()
        # The arm sits exactly at the previous knot: a perfect feed would
        # otherwise reach ready and then the executor would refuse.
        feed = Feed(clock, lambda t: list(TARGET))
        record = run(feed, clock, following=(NAMES, following))
        self.assertEqual(record["status"], "endpoint_mismatch")
        self.assertEqual(record["code"], HANDOFF_ENDPOINT_MISMATCH)
        self.assertNotEqual(record["code"], HANDOFF_SETTLE_TIMEOUT)
        self.assertEqual(clock.sleeps, 0)
        self.assertEqual(feed.reads, 0)
        self.assertEqual(record["continuity"]["status"], "discontinuous")
        self.assertGreater(record["continuity"]["max_delta"], 0.049)

    def test_continuity_uses_the_executor_tolerance(self):
        near = list(TARGET)
        near[1] += 0.0099
        self.assertEqual(check_endpoint_continuity(
            NAMES, TARGET, NAMES, near)["status"], "continuous")
        far = list(TARGET)
        far[1] += 0.0101
        self.assertEqual(check_endpoint_continuity(
            NAMES, TARGET, NAMES, far)["status"], "discontinuous")

    def test_wrapped_angle_is_continuous(self):
        wrapped = list(TARGET)
        wrapped[0] = TARGET[0] + 2.0 * math.pi
        self.assertEqual(check_endpoint_continuity(
            NAMES, TARGET, NAMES, wrapped)["status"], "continuous")

    def test_missing_or_misordered_endpoints_are_not_continuous(self):
        self.assertEqual(check_endpoint_continuity(
            NAMES, None, NAMES, TARGET)["status"], "missing_endpoint")
        self.assertEqual(check_endpoint_continuity(
            ["x"] * 6, TARGET, NAMES, TARGET)["status"], "joint_order")


class TestAbort(unittest.TestCase):
    def test_abort_stops_the_wait_with_the_reason(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: offset(1, -0.05))

        def should_abort():
            if clock.now - clock.start >= 0.5:
                return "PAYLOAD_LOST:vacuum_released"
            return None

        record = run(feed, clock, should_abort=should_abort)
        self.assertEqual(record["status"], "aborted")
        self.assertEqual(record["code"], "PAYLOAD_LOST:vacuum_released")
        self.assertLess(record["waited_sec"], 0.6)

    def test_cancellation_before_any_feedback(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET))
        record = run(feed, clock, should_abort=lambda: "HANDOFF_CANCELLED")
        self.assertEqual(record["status"], "aborted")
        self.assertEqual(record["code"], "HANDOFF_CANCELLED")
        self.assertEqual(feed.reads, 0)


class TestRecord(unittest.TestCase):
    def test_trace_is_bounded_and_summary_is_small(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: offset(1, -0.05))
        record = run(feed, clock)
        self.assertLessEqual(len(record["samples"]), 700)
        slim = summarize_for_extras(record, max_rows=40)
        self.assertLessEqual(len(slim["samples"]), 42)
        self.assertEqual(slim["samples_total"], len(record["samples"]))
        self.assertEqual(slim["samples"][-1], record["samples"][-1])

    def test_record_carries_policy_and_per_sample_residuals(self):
        clock = FakeTime()
        feed = Feed(clock, lambda t: list(TARGET))
        record = run(feed, clock)
        self.assertEqual(record["policy"]["timeout_sec"], 6.0)
        self.assertEqual(record["policy"]["max_abs_velocity"], 0.01)
        self.assertEqual(record["policy"]["hold_sec"], 0.3)
        row = record["samples"][0]
        for key in ("stamp", "rx_mono", "position", "velocity",
                    "max_delta_previous_last", "max_delta_next_first"):
            self.assertIn(key, row)


if __name__ == "__main__":
    unittest.main()
