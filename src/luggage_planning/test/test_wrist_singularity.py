#!/usr/bin/env python3
"""Replay the 45 recorded wrist tracks and the dual-quaternion IK pick.

Guard 2 exists because transit is a pose_target: it never entered
``_plan_cartesian``, so the excursion gate left ``cartesian_excursion: null``
while every flipping transit still drove joint5 through k*pi. The 32
legitimate packing hops park joint5 at 1.547-1.567 rad from k*pi; the
tracks that flipped or approached the singularity came within 0.005-0.454
rad. Dual-quaternion IK (Guard 1) is the cause-side fix for keep_tool_down
pose targets: theta and theta+pi are the same rectangular payload, so we
keep the branch nearer the seed instead of whatever KDL returns.
"""

from __future__ import division

import json
import math
import os
import unittest

from luggage_planning.motion_executor import (
    JOINTS,
    MotionExecutor,
    SegmentExecResult,
    _J6_ROTATION_LIMIT_RAD,
    _POSE_TARGET_MAX_JOINT_DELTA_RAD,
    _WRIST_SINGULARITY_MARGIN_RAD,
    equivalent_tool_down_pose,
    j6_rotation,
    pose_target_joint_delta,
    wrist_margin,
)
from luggage_planning.pose import Point, Pose, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK

_FIXTURE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "data", "wrist_singularity_segments.json")

LEGITIMATE = (
    "stage_mid", "stage_late", "traverse", "insert", "descend", "retreat")
FLIP_NAMES = ("transit", "place_exit")


class _Point(object):
    def __init__(self, positions):
        self.positions = positions


def _measure(segment, **kwargs):
    return wrist_margin(
        [_Point(row) for row in segment["positions"]], **kwargs)


def _measure_j6(segment, **kwargs):
    return j6_rotation(
        [_Point(row) for row in segment["positions"]], **kwargs)


def _yaw_of(pose):
    return 2.0 * math.atan2(
        float(pose.orientation.y), float(pose.orientation.x))


class _FakeFuture(object):
    def __init__(self, response):
        self._response = response

    def result(self):
        return self._response


class _FakeIK(object):
    """Return a flipped branch for yaw~0 and a near-seed branch for yaw~pi."""

    def __init__(self, flipped, near):
        self.flipped = list(flipped)
        self.near = list(near)
        self.yaws = []

    def wait_for_service(self, timeout_sec=2.0):
        return True

    def call_async(self, request):
        pose = request.ik_request.pose_stamped.pose
        yaw = _yaw_of(pose)
        self.yaws.append(yaw)
        wrapped = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
        joints = self.flipped if wrapped < 0.5 else self.near
        response = GetPositionIK.Response()
        response.error_code.val = MoveItErrorCodes.SUCCESS
        response.solution.joint_state.name = list(JOINTS)
        response.solution.joint_state.position = list(joints)
        return _FakeFuture(response)


def _bare_executor():
    executor = object.__new__(MotionExecutor)
    executor._group = "elfin_arm"
    executor._link = "suction_contact_frame"
    executor._frame = "world"
    executor._wrist_margin = _WRIST_SINGULARITY_MARGIN_RAD
    executor._pose_target_max_delta = _POSE_TARGET_MAX_JOINT_DELTA_RAD
    executor._j6_limit = _J6_ROTATION_LIMIT_RAD
    executor._pose_target_joint_delta = None
    executor._wrist_measurement = None
    executor._j6_measurement = None
    executor._cartesian_excursion = None
    executor._occupancy = None
    executor._occupancy_object_count = 0
    executor._planned_traj = None
    executor._start_joints = []
    executor._last_boundary = {}
    executor._wait_future = lambda future, timeout_sec: future.result()
    executor._wrist_free = set()
    return executor


class TestWristMarginFixture(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with open(_FIXTURE, encoding="utf-8") as handle:
            cls.segments = json.load(handle)["segments"]

    def test_fixture_has_the_measured_45(self):
        self.assertEqual(len(self.segments), 45)
        legit = [row for row in self.segments if row["name"] in LEGITIMATE]
        self.assertEqual(len(legit), 32)

    def test_legitimate_segments_are_accepted(self):
        for row in self.segments:
            if row["name"] not in LEGITIMATE:
                continue
            measured = _measure(row)
            self.assertTrue(
                measured["ok"],
                "%s/%s rejected at margin %.4f" % (
                    row["run"], row["name"], measured["margin_rad"]))
            self.assertGreaterEqual(measured["margin_rad"], 1.546)

    def test_near_singularity_transit_and_place_exit_are_rejected(self):
        rejected = []
        for row in self.segments:
            if row["name"] not in FLIP_NAMES:
                continue
            measured = _measure(row)
            if not measured["ok"]:
                rejected.append((row, measured))
                self.assertLess(measured["margin_rad"], 0.50)
        self.assertGreaterEqual(len(rejected), 10)
        self.assertTrue(
            all(row["name"] in FLIP_NAMES for row, _m in rejected))

    def test_nonflip_place_exit_is_accepted(self):
        accepted = [
            row for row in self.segments
            if row["name"] == "place_exit" and _measure(row)["ok"]]
        self.assertTrue(accepted)
        best = max(accepted, key=lambda row: _measure(row)["margin_rad"])
        self.assertGreater(_measure(best)["margin_rad"], 1.50)

    def test_bound_keeps_margin_on_both_sides(self):
        legit = [
            _measure(row)["margin_rad"]
            for row in self.segments if row["name"] in LEGITIMATE]
        rejected = [
            _measure(row)["margin_rad"]
            for row in self.segments
            if row["name"] in FLIP_NAMES and not _measure(row)["ok"]]
        self.assertGreater(min(legit), _WRIST_SINGULARITY_MARGIN_RAD * 3.0)
        self.assertLess(max(rejected), _WRIST_SINGULARITY_MARGIN_RAD)

    def test_seed_to_goal_flip_is_rejected_without_a_dense_path(self):
        """A pose_target IK pair parked at +/-pi/2 still crosses k*pi."""
        seed = [0.0, 0.0, 0.0, 0.0, math.pi / 2.0, 0.0]
        goal = [0.0, 0.0, 0.0, 0.0, -math.pi / 2.0, 0.0]
        measured = wrist_margin([seed, goal])
        self.assertFalse(measured["ok"])
        self.assertTrue(measured["crosses_branch"])
        self.assertGreater(measured["margin_rad"], 1.50)

    def test_same_branch_hop_at_pi_over_two_is_accepted(self):
        seed = [0.0, 0.0, 0.0, 0.0, 1.57, 0.0]
        goal = [0.1, -0.1, 0.0, 0.0, 1.56, 0.05]
        measured = wrist_margin([seed, goal])
        self.assertTrue(measured["ok"])

    def test_nothing_to_judge_returns_none(self):
        self.assertIsNone(wrist_margin([]))
        self.assertIsNone(wrist_margin([[0.0] * 6]))

    def test_limit_is_configurable(self):
        row = next(
            item for item in self.segments if item["name"] == "stage_mid")
        self.assertTrue(_measure(row)["ok"])
        self.assertFalse(_measure(row, margin_rad=1.60)["ok"])


class TestJ6RotationFixture(unittest.TestCase):
    """Dump measurement of flange travel. Not a refuse gate."""

    @classmethod
    def setUpClass(cls):
        with open(_FIXTURE, encoding="utf-8") as handle:
            cls.segments = json.load(handle)["segments"]

    def test_legitimate_hops_stay_under_the_limit(self):
        for row in self.segments:
            if row["name"] not in LEGITIMATE:
                continue
            measured = _measure_j6(row)
            self.assertTrue(
                measured["ok"],
                "%s/%s j6 travel %.3f" % (
                    row["run"], row["name"], measured["travel_rad"]))
            self.assertLessEqual(measured["travel_rad"], 0.71)

    def test_1838_transit_flip_is_accepted(self):
        """j5 1.57->4.71 with 1.90 rad of j6: PTP through the wrist is ok."""
        row = next(
            item for item in self.segments
            if item["run"] == "2026-09-21_1838" and item["name"] == "transit")
        measured = _measure_j6(row)
        self.assertTrue(measured["ok"])
        self.assertLess(measured["travel_rad"], _J6_ROTATION_LIMIT_RAD)
        self.assertFalse(_measure(row)["ok"])

    def test_unwind_place_exit_is_rejected(self):
        row = next(
            item for item in self.segments
            if item["run"] == "2026-09-21_1838"
            and item["name"] == "place_exit"
            and _measure_j6(item)["travel_rad"] > 5.0)
        measured = _measure_j6(row)
        self.assertFalse(measured["ok"])
        self.assertGreater(measured["travel_rad"], 5.0)
        self.assertLess(measured["net_rad"], 0.40)

    def test_seed_to_goal_j5_flip_with_still_flange_is_ok(self):
        seed = [0.0, 0.0, 0.0, 0.0, math.pi / 2.0, 0.30]
        goal = [0.0, 0.0, 0.0, 0.0, -math.pi / 2.0, 0.30]
        self.assertTrue(j6_rotation([seed, goal])["ok"])
        self.assertFalse(wrist_margin([seed, goal])["ok"])

    def test_nothing_to_judge_returns_none(self):
        self.assertIsNone(j6_rotation([]))
        self.assertIsNone(j6_rotation([[0.0] * 6]))


class TestPoseTargetJointDelta(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        with open(_FIXTURE, encoding="utf-8") as handle:
            cls.pose_targets = [
                row for row in json.load(handle)["segments"]
                if row["type"] == "pose_target"]

    def test_fixture_separates_flip_from_near_seed(self):
        flips = []
        near = []
        for row in self.pose_targets:
            measured = pose_target_joint_delta(
                row["positions"][0], row["positions"][-1])
            if abs(row["positions"][-1][4] - row["positions"][0][4]) > 1.5:
                flips.append(measured)
            else:
                near.append(measured)
        self.assertTrue(flips)
        self.assertTrue(near)
        # Diagnostic only: a j5 hop still shows up as a large max_rad.
        self.assertGreater(min(item["max_rad"] for item in flips), 3.0)
        self.assertLess(max(item["max_rad"] for item in near), 0.80)

    def test_nothing_to_judge_returns_none(self):
        self.assertIsNone(pose_target_joint_delta([], [0.0] * 6))
        self.assertIsNone(pose_target_joint_delta([0.0] * 6, [0.0] * 5))


class TestDualQuaternionIK(unittest.TestCase):

    def setUp(self):
        self.seed = [0.20, -1.50, -1.20, 0.10, 1.57, 0.30]
        self.flipped = list(self.seed)
        self.flipped[4] = self.seed[4] - math.pi
        self.near = [0.22, -1.48, -1.18, 0.12, 1.56, 0.32]
        self.pose = Pose(
            position=Point(x=0.4, y=0.1, z=1.2),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0))
        self.executor = _bare_executor()
        self.executor._ik = _FakeIK(self.flipped, self.near)

    def test_equivalent_pose_is_yaw_plus_pi(self):
        flipped = equivalent_tool_down_pose(self.pose)
        self.assertAlmostEqual(flipped.position.x, 0.4)
        self.assertAlmostEqual(flipped.position.y, 0.1)
        self.assertAlmostEqual(flipped.position.z, 1.2)
        self.assertAlmostEqual(_yaw_of(self.pose), 0.0, places=6)
        self.assertAlmostEqual(abs(_yaw_of(flipped)), math.pi, places=6)

    def test_namespace_pose_still_flips(self):
        from types import SimpleNamespace
        pose = SimpleNamespace(
            position=SimpleNamespace(x=0.4, y=0.1, z=1.2),
            orientation=SimpleNamespace(x=1.0, y=0.0, z=0.0, w=0.0))
        flipped = equivalent_tool_down_pose(pose)
        self.assertAlmostEqual(flipped.position.x, 0.4)
        self.assertAlmostEqual(abs(_yaw_of(flipped)), math.pi, places=6)

    def test_boundary_record_emits_wrist_delta_and_j6(self):
        self.executor._wrist_measurement = {
            "ok": True, "margin_rad": 1.56, "margin_limit_rad": 0.50}
        self.executor._pose_target_joint_delta = {
            "ok": True, "max_rad": 0.40, "limit_rad": 1.50}
        self.executor._j6_measurement = {
            "ok": True, "travel_rad": 0.71, "limit_rad": 2.50}
        result = SegmentExecResult(True, "ok")
        self.executor._stamp_boundary(result, "transit")
        record = self.executor.last_boundary()
        self.assertEqual(record["wrist_margin"]["margin_rad"], 1.56)
        self.assertEqual(record["pose_target_joint_delta"]["max_rad"], 0.40)
        self.assertEqual(record["j6_rotation"]["travel_rad"], 0.71)


if __name__ == "__main__":
    unittest.main()
