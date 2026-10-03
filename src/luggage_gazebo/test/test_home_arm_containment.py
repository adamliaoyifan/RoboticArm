#!/usr/bin/env python3
"""`_home_arm` is scene-state classification + sanctioned calls only.

[ST09-I S5] The portal hop used to be an ad-hoc cartesian
MotionSegment constructed driver-side; the executor answers those with
SEGMENT_GENERATION_REMOVED (the S3 generation-bypass deletion), and the
unbounded aperture prism misread the mirrored deploy pickup observe
(~1.7 m outside the face) as "inside the opening". Now: classify the
cup in container-local coordinates (inside_container / opening_corridor /
outside_container / unknown). outside_container classifies the TCP
position only — it does NOT authorize a bare motion; the named-pose
goto rides the GoToRobotPose surface, and any certified motion is
built by RouteProducer+MoveIt. Otherwise the exit is the
CERTIFIED route's own retreat/return legs — arriving still inside is a
named refusal (EXIT_REQUIRES_CERTIFIED_ROUTE), never a rebuilt
geometric escape.
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
    """Only the collaborators `_home_arm` actually touches (the ensure
    machine: the selector's verdict + the structured exit proof)."""

    def __init__(self, ready=True, reason="OK", zone="outside_container",
                 attachment="unloaded", goto_ok=True, proof_valid=False):
        self._ensure = (True, bool(ready), str(reason), str(zone),
                        str(attachment), "{}")
        self._goto_ok = goto_ok
        self._proof_valid = proof_valid
        self._home_arm_state = {}
        self._exit_yaw = {}
        self.calls = []

    def _ensure_cycle_start(self):
        return self._ensure

    def _portal_exit_proof_valid(self):
        return self._proof_valid, "valid" if self._proof_valid else "absent"

    def ros_now_sec(self):
        return 0.0

    def goto_observe(self):
        self.calls.append("goto")
        return self._goto_ok, "ok" if self._goto_ok else "abort", None

    _home_arm = PlaceSmokeDriver._home_arm
    home_arm_code = PlaceSmokeDriver.home_arm_code
    ENSURE_NAMED_STOPS = PlaceSmokeDriver.ENSURE_NAMED_STOPS


class TestHomeArmClassification(unittest.TestCase):
    """[ST09-I S5.7-6 §2] the ensure-first machine with the structured
    exit-proof exception (评审点 1)."""

    def test_already_ready_goes_straight_to_the_named_pose(self):
        driver = _Driver(ready=True, zone="outside_container")
        ok, notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["goto"])
        self.assertTrue(ok)
        self.assertIn("cycle=ALREADY_READY", notes)
        self.assertIn("zone=outside_container", notes)
        self.assertEqual(driver.home_arm_code(), "")

    def test_goto_failure_is_goto_failed(self):
        driver = _Driver(ready=True, goto_ok=False)
        ok, _notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["goto"])
        self.assertFalse(ok)
        self.assertEqual(driver.home_arm_code(), "GOTO_FAILED")

    def test_recovery_required_is_the_named_stop(self):
        for reason, zone in (
                ("EXIT_ROUTE_UNAVAILABLE", "inside_container"),
                ("EXIT_ROUTE_UNAVAILABLE", "opening_corridor"),
                ("CARRY_EXIT_UNSUPPORTED", "inside_container"),
                ("IDENTITY_UNKNOWN", "unknown"),
                ("ATTACHMENT_STATE_UNKNOWN", "outside_container")):
            driver = _Driver(ready=False, reason=reason, zone=zone)
            ok, notes, _result = driver._home_arm()
            self.assertEqual(driver.calls, [], reason)
            self.assertFalse(ok)
            self.assertIn(reason, notes)
            self.assertIn("cycle=RECOVERY_REQUIRED", notes)
            self.assertEqual(driver.home_arm_code(), reason)

    def test_portal_proof_allows_goto_from_the_corridor(self):
        driver = _Driver(ready=False, reason="EXIT_ROUTE_UNAVAILABLE",
                         zone="opening_corridor", proof_valid=True)
        ok, notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["goto"])
        self.assertTrue(ok)
        self.assertIn("exit=portal_proof", notes)
        self.assertTrue(driver._home_arm_state["portal_proof"])
        self.assertEqual(driver.home_arm_code(), "")

    def test_portal_proof_invalid_keeps_the_named_stop(self):
        driver = _Driver(ready=False, reason="EXIT_ROUTE_UNAVAILABLE",
                         zone="opening_corridor", proof_valid=False)
        ok, _notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, [])
        self.assertFalse(ok)
        self.assertEqual(
            driver.home_arm_code(), "EXIT_ROUTE_UNAVAILABLE")

    def test_proof_never_overrides_other_reasons(self):
        driver = _Driver(ready=False, reason="CARRY_EXIT_UNSUPPORTED",
                         zone="outside_container", proof_valid=True)
        ok, _notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, [])
        self.assertFalse(ok)
        self.assertEqual(
            driver.home_arm_code(), "CARRY_EXIT_UNSUPPORTED")

    def test_failure_keeps_the_gz_contact_capture(self):
        driver = _Driver(ready=False, reason="IDENTITY_UNKNOWN",
                         zone="unknown")
        driver._capture_gz_contacts = lambda: {"available": False}
        ok, _notes, _result = driver._home_arm()
        self.assertFalse(ok)
        self.assertIn("contacts", driver._home_arm_state)

    def test_service_absent_fails_closed_by_name(self):
        driver = _Driver(ready=False, reason="ENSURE_CYCLE_START_UNAVAILABLE",
                         zone="unknown", attachment="unknown")
        ok, _notes, _result = driver._home_arm()
        self.assertFalse(ok)
        self.assertEqual(driver.home_arm_code(),
                         "ENSURE_CYCLE_START_UNAVAILABLE")


class _ZoneDriver(object):
    """Real scene geometry, stubbed TF, for the zone classifier."""

    def __init__(self, scene, suction):
        self._scene_config = scene
        self._suction = suction
        self._typed = None

    def suction_xyz(self):
        return self._suction, ""

    _aperture_corridor_depth = PlaceSmokeDriver._aperture_corridor_depth
    _container_zone = PlaceSmokeDriver._container_zone


class TestMirroredDeployZoneClassification(unittest.TestCase):
    """scene_tf_deploy: the opening-side workspace must classify OUTSIDE."""

    @classmethod
    def setUpClass(cls):
        from luggage_description.scene_tf_config_utils import (
            load_scene_tf_config)
        scene_path = os.path.join(
            _SCRIPTS, "..", "..", "luggage_description", "config",
            "scene_tf_deploy.yaml")
        cls.scene = load_scene_tf_config(scene_path)

    def zone(self, suction):
        return _ZoneDriver(self.scene, suction)._container_zone()[0]

    def test_pickup_observe_tcp_is_outside(self):
        # Synthesized pickup observe TCP (world 0.945, 0.214, 1.748):
        # ~1.7 m outside the face yet inside the unbounded aperture
        # prism — the smoke03/04 misclassification.
        self.assertEqual(
            self.zone((0.945, 0.214, 1.748)), "outside_container")

    def test_opening_mouth_is_in_the_corridor(self):
        self.assertEqual(self.zone((-0.70, 0.6, 1.30)), "opening_corridor")

    def test_container_interior_is_inside(self):
        self.assertEqual(self.zone((-1.20, 0.6, 1.30)), "inside_container")

    def test_derived_corridor_depth_bounds(self):
        # wall (outer 1.57 - inner 1.49)/2 = 0.04; envelope falls back to
        # 0.30 offline; margin 0.10 -> corridor ~0.44. Just beyond the
        # derived depth classifies outside.
        from place_smoke_driver import (
            APERTURE_ESCAPE_MARGIN_M, CAMERA_ENVELOPE_FALLBACK_M)
        wall = (1.57 - 1.49) / 2.0
        depth = wall + CAMERA_ENVELOPE_FALLBACK_M + APERTURE_ESCAPE_MARGIN_M
        self.assertAlmostEqual(depth, 0.44, places=6)
        face_x, normal_x = -0.745, -1.0
        just_out = (face_x - normal_x * (depth + 0.05), 0.398, 1.30)
        self.assertEqual(self.zone(just_out), "outside_container")


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
