#!/usr/bin/env python3
"""`_home_arm` must not abandon the arm where a failed exit left it.

In motion_occ/2026-09-21_1838 the portal hop aborted with
GOAL_TOLERANCE_VIOLATED and `_home_arm` returned immediately. The arm stayed
inside the container, so trials 1 and 2 planned from a start state already in
collision and failed before they picked anything. `goto_observe` is an FJT to
a fixed joint vector and does not depend on the portal pose, so it is still
worth one attempt once the arm has stopped moving.
"""

import os
import sys
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from place_smoke_driver import PlaceSmokeDriver  # noqa: E402
from luggage_gazebo.place_metrics import PlaceTrial, place_ok  # noqa: E402


class _Driver(object):
    """Only the collaborators `_home_arm` actually touches."""

    def __init__(self, exit_ok, goto_ok, inside=True):
        self._exit_ok = exit_ok
        self._goto_ok = goto_ok
        self._inside = inside
        self._home_arm_state = {}
        self._exit_yaw = {}
        self.calls = []

    def suction_xyz(self):
        return ((1.03, -0.735, 1.177) if self._inside else (0.1, 0.0, 1.5)), ""

    def ros_now_sec(self):
        return 0.0

    def _exit_to_portal(self):
        self.calls.append("exit")
        return self._exit_ok, "ok" if self._exit_ok else "error_code=-4"

    def wait_arm_settle(self, reason="", **kwargs):
        self.calls.append("settle")
        return "still after 0.45s"

    def goto_observe(self):
        self.calls.append("goto")
        return self._goto_ok, "ok" if self._goto_ok else "abort", None

    _home_arm = PlaceSmokeDriver._home_arm
    home_arm_code = PlaceSmokeDriver.home_arm_code


class TestHomeArmContainment(unittest.TestCase):

    def test_failed_exit_settles_then_still_returns_to_observe(self):
        driver = _Driver(exit_ok=False, goto_ok=True)
        ok, notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["exit", "settle", "goto"])
        self.assertFalse(ok, "the exit failure must still be reported")
        self.assertIn("settle=", notes)
        self.assertEqual(driver.home_arm_code(), "EXIT_FAILED_RECOVERED")

    def test_failed_exit_that_cannot_recover_is_a_distinct_code(self):
        driver = _Driver(exit_ok=False, goto_ok=False)
        ok, _notes, _result = driver._home_arm()
        self.assertFalse(ok)
        self.assertEqual(driver.home_arm_code(), "EXIT_FAILED_STUCK")

    def test_exit_ok_but_observe_unreachable_stays_goto_failed(self):
        driver = _Driver(exit_ok=True, goto_ok=False)
        driver._home_arm()
        self.assertEqual(driver.calls, ["exit", "goto"])
        self.assertEqual(driver.home_arm_code(), "GOTO_FAILED")

    def test_success_has_no_reason_code(self):
        driver = _Driver(exit_ok=True, goto_ok=True)
        ok, _notes, _result = driver._home_arm()
        self.assertTrue(ok)
        self.assertEqual(driver.home_arm_code(), "")

    def test_arm_already_clear_skips_the_exit(self):
        driver = _Driver(exit_ok=False, goto_ok=True, inside=False)
        ok, _notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["goto"])
        self.assertTrue(ok)


class TestReturnFailureIsNotAPlaceFailure(unittest.TestCase):

    def test_exit_failure_after_home_still_counts_the_place(self):
        for code in ("EXIT_FAILED_RECOVERED", "EXIT_FAILED_STUCK"):
            self.assertTrue(place_ok(
                PlaceTrial(index=0, fail_code=code, place_state="HOME")), code)

    def test_exit_failure_before_home_does_not(self):
        for code in ("EXIT_FAILED_RECOVERED", "EXIT_FAILED_STUCK"):
            self.assertFalse(place_ok(
                PlaceTrial(index=0, fail_code=code, place_state="CARRY_READY")),
                code)


if __name__ == "__main__":
    unittest.main()
