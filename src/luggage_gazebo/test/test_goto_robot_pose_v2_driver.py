#!/usr/bin/env python3
"""GoToV2 driver migration surfaces (ST09-I S5.7-6).

`goto_observe` is now a caller-side adjudicator over the certified
GoToRobotPoseV2 service (current/here = stay put; a missing typed
surface is a named refusal, never a v1 detour), and the cargo
`_exit_to_portal` rides the same service while freezing a STRUCTURED
exit proof (评审点 1): request/route/cert/scene identity + the final
joint state + the unloaded phase. The proof exists only on success and
is void at the next cycle start.
"""

import os
import sys
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from cargo_map_integrate_eval_driver import (  # noqa: E402
    CargoMapIntegrateDriver,
)
from pick_retreat_eval_driver import PickRetreatEvalDriver  # noqa: E402


class _Clock(object):
    def now(self):
        import rclpy.time

        return rclpy.time.Time()


class _GotoDriver(object):
    """Shell with only what goto_observe touches."""

    def __init__(self, observe_pose="pickup_observe_sim", typed=True):
        self._args = type("_A", (), {
            "observe_pose": observe_pose, "goto_timeout": 30.0})()
        self._typed = object() if typed else None
        self.calls = []

    def _goto_named_pose_v2(self, pose_name, timeout=30.0):
        self.calls.append(pose_name)
        return True, "goto legs ok"

    goto_observe = PickRetreatEvalDriver.goto_observe


class TestGotoObserve(unittest.TestCase):
    def test_current_name_skips_motion_caller_side(self):
        driver = _GotoDriver(observe_pose="current")
        ok, message, result = driver.goto_observe()
        self.assertTrue(ok)
        self.assertIn("current", message)
        self.assertIsNone(result)
        self.assertEqual(driver.calls, [])

    def test_here_name_is_also_current(self):
        driver = _GotoDriver(observe_pose="Here")
        ok, message, _result = driver.goto_observe()
        self.assertTrue(ok)
        self.assertIn("current", message)
        self.assertEqual(driver.calls, [])

    def test_missing_typed_surface_is_named_refusal(self):
        driver = _GotoDriver(typed=False)
        ok, message, _result = driver.goto_observe()
        self.assertFalse(ok)
        self.assertEqual(message, "GOTO_V2_UNAVAILABLE")
        self.assertEqual(driver.calls, [])

    def test_named_pose_rides_the_v2_helper(self):
        driver = _GotoDriver()
        ok, message, _result = driver.goto_observe()
        self.assertTrue(ok)
        self.assertEqual(driver.calls, ["pickup_observe_sim"])


class _PortalDriver(object):
    """Shell with only what _exit_to_portal touches."""

    def __init__(self, goto_ok=True):
        self._goto_ok = goto_ok
        self._args = type("_A", (), {"goto_timeout": 30.0})()
        self._last_joint_state = {"position": [0.1, -0.2, 0.3, -0.4, 0.5,
                                               -0.6, 99.0]}
        self._portal_exited = False
        self._exit_yaw = {"source": "previous"}
        self._portal_exit_proof = None
        self._goto_calls = []

    def _goto_named_pose_v2(self, pose_name, timeout=30.0):
        self._goto_calls.append(pose_name)
        if self._goto_ok:
            self._last_goto_route = {
                "request_id": "req-exit-1",
                "route_digest": "digest-exit-1",
                "certification_id": "cert-exit-1",
                "scene_revision": "scene-md5-1",
            }
            return True, ""
        self._last_goto_route = None
        return False, "GOTO_LEG_FAILED:boom"

    def get_clock(self):
        return _Clock()

    _exit_to_portal = CargoMapIntegrateDriver._exit_to_portal


class TestExitToPortalProof(unittest.TestCase):
    def test_success_freezes_the_structured_proof(self):
        driver = _PortalDriver(goto_ok=True)
        ok, message = driver._exit_to_portal()
        self.assertTrue(ok)
        self.assertEqual(driver._goto_calls, ["portal_observe_sim"])
        self.assertTrue(driver._portal_exited)
        self.assertEqual(driver._exit_yaw, {"source":
                                            "named_portal_pose"})
        proof = driver._portal_exit_proof
        self.assertEqual(proof["request_id"], "req-exit-1")
        self.assertEqual(proof["route_digest"], "digest-exit-1")
        self.assertEqual(proof["certification_id"], "cert-exit-1")
        self.assertEqual(proof["scene_revision"], "scene-md5-1")
        # first 6 joints only, exactly the arm state at exit
        self.assertEqual(proof["final_joint_values"],
                         [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        self.assertEqual(proof["attachment_state"], "unloaded")
        self.assertIsInstance(proof["proof_ns"], int)

    def test_failure_preserves_flags_and_proofs_nothing(self):
        driver = _PortalDriver(goto_ok=False)
        ok, _message = driver._exit_to_portal()
        self.assertFalse(ok)
        self.assertFalse(driver._portal_exited)
        # the previous yaw source survives a failed exit attempt
        self.assertEqual(driver._exit_yaw, {"source": "previous"})
        self.assertIsNone(driver._portal_exit_proof)


if __name__ == "__main__":
    unittest.main()
