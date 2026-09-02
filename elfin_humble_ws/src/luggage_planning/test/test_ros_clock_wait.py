#!/usr/bin/env python3
"""Unit tests for sim-time action waits (no ROS graph)."""

from __future__ import division

import threading
import time
import unittest

from luggage_planning.ros_clock_wait import (
    ClockTimeout,
    is_ros_time_clock,
    wait_event,
)


class _Stamp(object):
    def __init__(self, ns):
        self.nanoseconds = int(ns)

    def __sub__(self, other):
        return _Stamp(self.nanoseconds - other.nanoseconds)

    def __gt__(self, other):
        return self.nanoseconds > other.nanoseconds


class FakeRosClock(object):
    clock_type = "ROS_TIME"

    def __init__(self):
        self.ns = 0

    def now(self):
        return _Stamp(self.ns)

    def advance(self, sec):
        self.ns += int(float(sec) * 1e9)


class TestRosClockWait(unittest.TestCase):

    def test_detects_ros_time_by_name(self):
        self.assertTrue(is_ros_time_clock(FakeRosClock()))
        self.assertFalse(is_ros_time_clock(None))

    def test_wall_wait_returns_on_event(self):
        event = threading.Event()
        event.set()
        ok, reason = wait_event(event, 0.5, clock=None)
        self.assertTrue(ok)
        self.assertEqual(reason, "")

    def test_sim_timeout_ignores_short_wall_time(self):
        clock = FakeRosClock()
        timer = ClockTimeout(clock, timeout_sec=1.0, wall_scale=25.0)
        time.sleep(0.05)
        self.assertFalse(timer.done())
        clock.advance(1.01)
        self.assertTrue(timer.done())
        self.assertEqual(timer.reason, "timeout")

    def test_sim_clock_stalled(self):
        clock = FakeRosClock()
        timer = ClockTimeout(
            clock, timeout_sec=10.0, frozen_wall_sec=0.05, wall_scale=25.0)
        time.sleep(0.12)
        self.assertTrue(timer.done())
        self.assertEqual(timer.reason, "sim_clock_stalled")

    def test_wait_event_sim_completes_when_clock_advances(self):
        clock = FakeRosClock()
        event = threading.Event()

        def _run():
            time.sleep(0.02)
            event.set()

        worker = threading.Thread(target=_run)
        worker.start()
        ok, reason = wait_event(
            event, 2.0, clock=clock, poll_sec=0.01, frozen_wall_sec=2.0)
        worker.join()
        self.assertTrue(ok)
        self.assertEqual(reason, "")


if __name__ == "__main__":
    unittest.main()
