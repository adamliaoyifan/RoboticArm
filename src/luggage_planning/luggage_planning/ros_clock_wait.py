#!/usr/bin/env python3
"""Wait helpers that honor /clock when the node uses sim time.

Wall-clock ``Event.wait(timeout)`` fires while Gazebo is still integrating the
same few milliseconds of physics. Trajectory ``time_from_start`` and the
controllers run on ``/clock``, so action/FJT timeouts must too.

When ``clock`` is ROS_TIME, ``timeout_sec`` is simulation seconds. A wall
watchdog still aborts if ``/clock`` stalls or real-time factor is extreme.
No rclpy import: unit tests can pass a fake clock.
"""

from __future__ import division

import time


def is_ros_time_clock(clock):
    if clock is None:
        return False
    ctype = getattr(clock, "clock_type", None)
    if ctype is None:
        return False
    name = getattr(ctype, "name", None) or str(ctype)
    if "ROS_TIME" in str(name):
        return True
    try:
        return int(ctype) == 1
    except (TypeError, ValueError):
        return False


class ClockTimeout(object):
    """Deadline on a ROS clock, plus wall-time stall / cap guards."""

    def __init__(self, clock, timeout_sec, frozen_wall_sec=20.0,
                 wall_scale=25.0):
        self.clock = clock
        self.timeout_sec = max(0.0, float(timeout_sec))
        self.frozen_wall_sec = float(frozen_wall_sec)
        self.wall_cap = max(
            self.timeout_sec * float(wall_scale),
            self.timeout_sec + 30.0)
        self.reason = ""
        self._use_sim = is_ros_time_clock(clock)
        self._wall0 = time.monotonic()
        self._t0 = clock.now() if self._use_sim else None
        self._last_sim = self._t0
        self._last_sim_wall = self._wall0

    def elapsed(self):
        if not self._use_sim:
            return time.monotonic() - self._wall0
        return (self.clock.now() - self._t0).nanoseconds * 1e-9

    def done(self):
        wall = time.monotonic() - self._wall0
        if wall > self.wall_cap:
            self.reason = "wall_cap"
            return True
        if self._use_sim:
            now = self.clock.now()
            if now > self._last_sim:
                self._last_sim = now
                self._last_sim_wall = time.monotonic()
            elif (time.monotonic() - self._last_sim_wall
                  > self.frozen_wall_sec):
                self.reason = "sim_clock_stalled"
                return True
            if (now - self._t0).nanoseconds >= self.timeout_sec * 1e9:
                self.reason = "timeout"
                return True
            return False
        if wall >= self.timeout_sec:
            self.reason = "timeout"
            return True
        return False


def wait_event(event, timeout_sec, clock=None, poll_sec=0.05,
               frozen_wall_sec=20.0, wall_scale=25.0):
    """Block until *event* is set or the timeout expires.

    Returns ``(reached, reason)``. *reason* is empty on success, otherwise
    ``timeout``, ``sim_clock_stalled``, or ``wall_cap``.
    """
    if event.is_set():
        return True, ""
    timeout_sec = float(timeout_sec)
    if timeout_sec <= 0.0:
        return event.is_set(), "" if event.is_set() else "timeout"
    if not is_ros_time_clock(clock):
        ok = event.wait(timeout_sec)
        return ok, "" if ok else "timeout"
    timer = ClockTimeout(
        clock, timeout_sec, frozen_wall_sec=frozen_wall_sec,
        wall_scale=wall_scale)
    while not timer.done():
        if event.wait(float(poll_sec)):
            return True, ""
    return event.is_set(), timer.reason
