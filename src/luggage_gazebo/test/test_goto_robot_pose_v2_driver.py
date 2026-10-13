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
from place_smoke_driver import PlaceSmokeDriver  # noqa: E402


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

    def __init__(self, goto_ok=True, already_there=False,
                 empty_cert=False, proof=None):
        self._goto_ok = goto_ok
        self._already_there = already_there
        self._empty_cert = empty_cert
        self._args = type("_A", (), {"goto_timeout": 30.0})()
        self._last_joint_state = {"position": [0.1, -0.2, 0.3, -0.4, 0.5,
                                               -0.6, 99.0]}
        self._portal_exited = False
        self._exit_yaw = {"source": "previous"}
        self._portal_exit_proof = proof
        self._goto_calls = []

    def _goto_named_pose_v2(self, pose_name, timeout=30.0):
        self._goto_calls.append(pose_name)
        if self._already_there:
            # helper contract: zero legs executed, nothing promoted
            self._last_goto_route = None
            return True, "already_there"
        if self._goto_ok:
            route = {
                "request_id": "req-exit-1",
                "route_digest": "digest-exit-1",
                "certification_id": "cert-exit-1",
                "scene_revision": "scene-md5-1",
            }
            if self._empty_cert:
                route = {
                    "request_id": "",
                    "route_digest": "",
                    "certification_id": "",
                    "scene_revision": "scene-md5-1",
                }
            self._last_goto_route = route
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

    def test_already_there_never_mints_a_proof(self):
        # [G0-1] an already-there response executes zero legs and
        # promotes nothing — an empty record is NOT a certified exit
        # and must never be frozen as a new proof.
        driver = _PortalDriver(already_there=True)
        ok, _message = driver._exit_to_portal()
        self.assertTrue(ok)
        self.assertTrue(driver._portal_exited)
        self.assertIsNone(driver._portal_exit_proof)

    def test_already_there_inherits_the_still_valid_original_proof(self):
        # [G0-1] already-there may only INHERIT the original proof from
        # the real certified exit — never mint, never clobber.
        original = {
            "request_id": "req-exit-0",
            "route_digest": "digest-exit-0",
            "certification_id": "cert-exit-0",
            "scene_revision": "scene-md5-1",
            "final_joint_values": [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
            "attachment_state": "unloaded",
            "proof_ns": 42,
        }
        driver = _PortalDriver(already_there=True, proof=original)
        ok, _message = driver._exit_to_portal()
        self.assertTrue(ok)
        self.assertIs(driver._portal_exit_proof, original)

    def test_empty_cert_route_freezes_no_proof(self):
        # [G0-1] defensive freeze site: a "successful" route record
        # with empty request/route/cert never becomes a proof
        # (adjudication would reject it as uncertified_exit anyway).
        driver = _PortalDriver(goto_ok=True, empty_cert=True)
        ok, _message = driver._exit_to_portal()
        self.assertTrue(ok)
        self.assertIsNone(driver._portal_exit_proof)


class _StatsResponse(object):
    def __init__(self, revision):
        self.success = True
        self.map_revision = int(revision)


class _GotoResponse(object):
    def __init__(self, success=True, reason_code="", already_there=False):
        self.success = bool(success)
        self.reason_code = str(reason_code)
        self.already_there = bool(already_there)
        self.route_digest = ""
        self.certification_id = ""
        self.admission_map_revision = 0
        self.leg_digests = []
        self.leg_load_states = []
        self.motion_segments = []


class _ScriptedClient(object):
    def wait_for_service(self, timeout_sec=None):
        return True


class _RetryDriver(object):
    """Shell with only what _goto_named_pose_v2 + the boot-churn settle
    touch; call_srv answers from a scripted response queue."""

    def __init__(self, responses):
        self.responses = list(responses)
        self.goto_request_ids = []
        self._group = None
        self._last_joint_state = {"position": [0.0] * 6}
        self._frame_scene_revision = "scene-md5-1"

    def create_client(self, srv_type, name, callback_group=None):
        return _ScriptedClient()

    def call_srv(self, client, request, timeout):
        if getattr(request, "pose_name", None) is not None:
            self.goto_request_ids.append(str(request.request_id))
        return self.responses.pop(0)

    def _frame_request_id(self, prefix):
        return "%s-x" % prefix

    _goto_named_pose_v2 = PickRetreatEvalDriver._goto_named_pose_v2
    _wait_map_quiescence = PickRetreatEvalDriver._wait_map_quiescence


class TestGotoMapStaleRetry(unittest.TestCase):
    """[ST09-I G2] boot-churn MAP_STALE is retried after an OBSERVABLE
    revision-quiescence wait; other refusals are not retried; the
    fail-closed gate itself is untouched."""

    def test_map_stale_retries_after_quiescence(self):
        driver = _RetryDriver([
            _StatsResponse(5),                       # attempt 0 stats
            _GotoResponse(False, "MAP_STALE"),       # attempt 0 refused
            _StatsResponse(6), _StatsResponse(6),    # settle: advances then still
            _StatsResponse(6),                       # attempt 1 stats
            _GotoResponse(True, already_there=True),
        ])
        ok, message = driver._goto_named_pose_v2("pickup_observe_sim")
        self.assertTrue(ok)
        self.assertEqual(message, "already_there")
        self.assertEqual(len(driver.goto_request_ids), 2)
        self.assertNotEqual(driver.goto_request_ids[0],
                            driver.goto_request_ids[1])

    def test_map_stale_exhausts_retries_and_returns_the_reason(self):
        driver = _RetryDriver([
            _StatsResponse(5), _GotoResponse(False, "MAP_STALE"),
            _StatsResponse(6), _StatsResponse(6),   # settle: 2 polls
            _StatsResponse(6), _GotoResponse(False, "MAP_STALE"),
            _StatsResponse(6),                        # settle: already equal
            _StatsResponse(6), _GotoResponse(False, "MAP_STALE"),
        ])
        ok, message = driver._goto_named_pose_v2("pickup_observe_sim")
        self.assertFalse(ok)
        self.assertEqual(message, "MAP_STALE")
        self.assertEqual(len(driver.goto_request_ids), 3)

    def test_other_refusals_are_not_retried(self):
        driver = _RetryDriver([
            _StatsResponse(5),
            _GotoResponse(False, "POSE_NOT_VALID"),
        ])
        ok, message = driver._goto_named_pose_v2("pickup_observe_sim")
        self.assertFalse(ok)
        self.assertEqual(message, "POSE_NOT_VALID")
        self.assertEqual(len(driver.goto_request_ids), 1)


class _JointStateShell(object):
    """Only what _on_joint_state touches (ring kept as a plain list)."""

    def __init__(self):
        self._last_joint_state = None
        self._segment_exec_t0 = None
        self._joint_ring = []
        self._joint_ring_horizon_sec = 0.0

    _on_joint_state = PlaceSmokeDriver._on_joint_state


class TestJointStateIngestionNormalizesOrder(unittest.TestCase):
    """[ST09-I G2] the Gazebo broadcaster publishes /joint_states in
    controller-registration order (observed live: j1,j2,j5,j3,j4,j6);
    every driver consumer assumes canonical elfin_joint1..6. Without
    name-alignment at ingestion the boot pose (physically AT
    pickup_observe_sim) compares 2.77 rad off, already-there never
    fires, and the first goto hits the empty boot map's MAP_STALE."""

    def _message(self, names, positions):
        from sensor_msgs.msg import JointState
        from builtin_interfaces.msg import Time as TimeMsg

        msg = JointState()
        msg.header.stamp.sec = 100
        msg.name = list(names)
        msg.position = list(positions)
        msg.velocity = []
        msg.effort = []
        return msg

    def test_permuted_publisher_order_is_normalized_by_name(self):
        # verbatim shape observed in the g2 run2 bag
        msg = self._message(
            ["elfin_joint1", "elfin_joint2", "elfin_joint5",
             "elfin_joint3", "elfin_joint4", "elfin_joint6"],
            [-1.25, -1.7466, 1.7714, -0.9974, -1.9153, 1.861])
        driver = _JointStateShell()
        driver._on_joint_state(msg)
        record = driver._last_joint_state
        self.assertEqual(
            record["position"],
            [-1.25, -1.7466, -0.9974, -1.9153, 1.7714, 1.861])
        self.assertEqual(
            record["name"],
            ["elfin_joint%d" % i for i in range(1, 7)])

    def test_canonical_order_passes_through_unchanged(self):
        msg = self._message(
            ["elfin_joint%d" % i for i in range(1, 7)],
            [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        driver = _JointStateShell()
        driver._on_joint_state(msg)
        self.assertEqual(
            driver._last_joint_state["position"],
            [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])

    def test_nameless_message_keeps_raw_order(self):
        msg = self._message([], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        driver = _JointStateShell()
        driver._on_joint_state(msg)
        self.assertEqual(
            driver._last_joint_state["position"],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        self.assertEqual(driver._last_joint_state["name"], [])


if __name__ == "__main__":
    unittest.main()
