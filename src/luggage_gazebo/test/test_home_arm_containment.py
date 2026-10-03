#!/usr/bin/env python3
"""`_home_arm` is scene-state classification + sanctioned calls only.

[ST09-I S5] The portal hop used to be an ad-hoc cartesian
MotionSegment constructed driver-side; the executor answers those with
SEGMENT_GENERATION_REMOVED (the S3 generation-bypass deletion), and the
unbounded aperture prism misread the mirrored deploy pickup observe
(~1.7 m outside the face) as "inside the opening". Now: classify the
cup in container-local coordinates (inside / near_opening / outside);
OUTSIDE -> the allowed named-pose goto; otherwise the exit is the
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
    """Only the collaborators `_home_arm` actually touches."""

    def __init__(self, zone="outside", goto_ok=True, portal_exited=False):
        self._zone = zone
        self._goto_ok = goto_ok
        self._portal_exited = portal_exited
        self._home_arm_state = {}
        self._exit_yaw = {}
        self.calls = []

    def _container_zone(self):
        return self._zone, ""

    def ros_now_sec(self):
        return 0.0

    def goto_observe(self):
        self.calls.append("goto")
        return self._goto_ok, "ok" if self._goto_ok else "abort", None

    _home_arm = PlaceSmokeDriver._home_arm
    home_arm_code = PlaceSmokeDriver.home_arm_code


class TestHomeArmClassification(unittest.TestCase):

    def test_outside_goes_straight_to_the_named_pose(self):
        driver = _Driver(zone="outside", goto_ok=True)
        ok, notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["goto"])
        self.assertTrue(ok)
        self.assertIn("zone=outside", notes)
        self.assertEqual(driver.home_arm_code(), "")

    def test_outside_but_goto_fails_is_goto_failed(self):
        driver = _Driver(zone="outside", goto_ok=False)
        ok, _notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["goto"])
        self.assertFalse(ok)
        self.assertEqual(driver.home_arm_code(), "GOTO_FAILED")

    def test_inside_requires_the_certified_route(self):
        for zone in ("inside", "near_opening"):
            driver = _Driver(zone=zone, goto_ok=True)
            ok, notes, _result = driver._home_arm()
            self.assertEqual(driver.calls, [], zone)
            self.assertFalse(ok)
            self.assertIn("EXIT_REQUIRES_CERTIFIED_ROUTE", notes)
            self.assertEqual(
                driver.home_arm_code(), "EXIT_REQUIRES_CERTIFIED_ROUTE")

    def test_portal_exited_still_allows_the_goto(self):
        driver = _Driver(zone="near_opening", goto_ok=True,
                         portal_exited=True)
        ok, _notes, _result = driver._home_arm()
        self.assertEqual(driver.calls, ["goto"])
        self.assertTrue(ok)


class _ZoneDriver(object):
    """Real scene geometry, stubbed TF, for the zone classifier."""

    def __init__(self, scene, suction):
        self._scene_config = scene
        self._suction = suction

    def suction_xyz(self):
        return self._suction, ""

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
        self.assertEqual(self.zone((0.945, 0.214, 1.748)), "outside")

    def test_opening_mouth_is_near_opening(self):
        self.assertEqual(self.zone((-0.70, 0.6, 1.30)), "near_opening")

    def test_container_interior_is_inside(self):
        self.assertEqual(self.zone((-1.20, 0.6, 1.30)), "inside")


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
