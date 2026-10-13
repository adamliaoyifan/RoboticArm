"""ST08 execution, scene and attachment acceptance.

CPU only. FRAME_MIGRATION_CASE_LOG selects an append-only JSONL file.
Oracles are local matrices, not a second call into the function under test.
"""

from __future__ import division

import copy
import itertools
import json
import math
import os
from types import SimpleNamespace
import unittest

import pytest

from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import BoundaryContext, BoundaryContractError
from luggage_planning.motion_frame_contracts import (
    CANONICAL_JOINTS,
    ExecutionContext,
    admit_motion_segment,
    compose_orientation,
    fk_budget,
    framed_query_offset,
    lift_vector,
    map_container_point,
    rotation_angle,
)
from luggage_planning.motion_executor import JOINTS, MotionExecutor


SCENE = "scene-st08"
STAMP = 1234500000000
WDH = (0.80, 0.50, 0.32)
YAWS = (0.0, math.pi / 2.0, math.pi, math.atan2(4.0, 3.0))
ORIGINS = ((0.0, 0.0, 0.0), (1.5, -0.4, 0.2))
FLOORS = (0.53, 0.71)
SUPPORTS = (0.0, 0.30)
GRASPS = ((0.0, 0.0, 0.16), (0.10, -0.05, 0.16))
SLOT_YAW = 0.3
CASES = list(enumerate(itertools.product(YAWS, ORIGINS, FLOORS, SUPPORTS, GRASPS)))


def _R(quaternion):
    x, y, z, w = quaternion
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _mul(a, b):
    return tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3) for i in range(3))


def _mm(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3))


def _apply(matrix, vector):
    return tuple(sum(matrix[i][k] * vector[k] for k in range(3)) for i in range(3))


def _yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def _context(time_basis="scene", reference="suction_contact_frame", frame="world"):
    boundary = BoundaryContext(
        interface_id="F15", producer="test", consumer="executor",
        request_id="req", scene_revision=SCENE, frame_id=frame,
        reference=reference, time_basis=time_basis)
    return ExecutionContext(boundary, "suction_contact_frame", "world")


def _pose(position, quaternion, reference, semantics, frame, basis="scene"):
    return {
        "contract_version": 2, "kind": "pose", "frame_id": frame,
        "length_unit": "m", "time_basis": basis, "scene_revision": SCENE,
        "request_id": "req", "stamp_ns": STAMP, "reference": reference,
        "orientation_semantics": semantics,
        "position": [float(v) for v in position],
        "quaternion_xyzw": [float(v) for v in quaternion],
    }


def _policy(**overrides):
    record = {
        "policy_valid": True, "keep_tool_down": True, "keep_camera_down": False,
        "lock_wrist": False, "allow_ompl_fallback": True,
        "required_cartesian_fraction": 0.0,
    }
    record.update(overrides)
    return record


def _segment(position, quaternion, segment_type="pose_target"):
    record = {
        "contract_version": 2, "request_id": "req", "scene_revision": SCENE,
        "stamp_ns": STAMP, "time_basis": "scene", "name": "transit",
        "segment_type": segment_type, "target_link": "suction_contact_frame",
        "target_pose": _pose(position, quaternion, "suction_contact_frame",
                             "tool_quaternion", "world"),
        "waypoints": [], "joint_trajectory": None,
    }
    record.update(_policy())
    return record


def _log(case_id, payload):
    path = os.environ.get("FRAME_MIGRATION_CASE_LOG")
    if not path:
        return
    row = {"case_id": case_id, "status": "pass"}
    row.update(payload)
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(row) + "\n")


def _geometry(yaw, origin, floor, support, grasp):
    slot_c = (0.2, -0.15, floor + support + WDH[2] / 2.0)
    container_q = _yaw_q(yaw)
    slot_q = _yaw_q(SLOT_YAW)
    world_slot = _add(origin, _apply(_R(container_q), slot_c))
    world_rot = _mm(_R(container_q), _R(slot_q))
    tool = _add(world_slot, _apply(world_rot, grasp))
    # Tool quaternion equals the box quaternion for an identity grasp rotation.
    return {
        "container_q": container_q,
        "origin": origin,
        "slot_c": slot_c,
        "slot_q": slot_q,
        "world_slot": world_slot,
        "world_rot": world_rot,
        "tool": tool,
        "tool_q": _yaw_q(yaw + SLOT_YAW),
    }


def _corners(center, rotation, size):
    hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    out = []
    for sx in (-hx, hx):
        for sy in (-hy, hy):
            for sz in (-hz, hz):
                out.append(_add(center, _apply(rotation, (sx, sy, sz))))
    return out


class TestMatrix(unittest.TestCase):
    def test_case_count(self):
        self.assertEqual(len(CASES), 64)

    def test_execute_chain(self):
        from luggage_planning.motion_frame_adapters import message_from_admitted, record_from_message
        for index, (yaw, origin, floor, support, grasp) in CASES:
            geo = _geometry(yaw, origin, floor, support, grasp)
            identity = _pose(geo["tool"], geo["tool_q"], "suction_contact_frame",
                             "tool_quaternion", "world")
            world_poses = [
                {"position": list(geo["tool"]), "quaternion_xyzw": list(geo["tool_q"])}]
            from luggage_planning.typed_driver_adapter import (
                pose_segment_message)
            message = pose_segment_message(
                "transit", geo["tool"], geo["tool_q"],
                str(identity["request_id"]), str(identity["scene_revision"]))
            wired = record_from_message(message)
            admitted = admit_motion_segment(wired, _context())
            got = admitted.target_pose["position"]
            for axis in range(3):
                self.assertLessEqual(abs(got[axis] - geo["tool"][axis]), 1e-6)
            self.assertLessEqual(
                rotation_angle(admitted.target_pose["quaternion_xyzw"], geo["tool_q"]), 1e-6)
            self.assertEqual(admitted.target_link, "suction_contact_frame")
            _log("execute/%02d" % index, {"yaw": yaw})

    def test_collision_corners(self):
        from luggage_description.frame_diagnostics import BoundaryContext as BC
        from luggage_planning.pickup_collision import admit_collision_box
        from luggage_planning.scene_box_v2 import admit_placed_box
        for index, (yaw, origin, floor, support, grasp) in CASES:
            del grasp
            geo = _geometry(yaw, origin, floor, support, (0.0, 0.0, 0.16))
            world_q = compose_orientation(geo["container_q"], geo["slot_q"])
            center = map_container_point(geo["slot_c"], origin, geo["container_q"])
            expected = _corners(center, _mm(_R(geo["container_q"]), _R(geo["slot_q"])), WDH)
            got_rot = _R(world_q)
            got = _corners(center, got_rot, WDH)
            for left, right in zip(expected, got):
                for axis in range(3):
                    self.assertLessEqual(abs(left[axis] - right[axis]), 1e-9)
            slot = _pose(geo["slot_c"], geo["slot_q"], "box_center", "box_axis_pi",
                         "container_link")
            # The name `_pose` above is the builder. The admission result uses another name.
            context = BC(
                interface_id="F17", producer="t", consumer="scene",
                request_id="req", scene_revision=SCENE, frame_id="container_link",
                reference="box_center", time_basis="scene")
            from luggage_planning.motion_frame_adapters import message_from_admitted
            del message_from_admitted
            from luggage_description.ros_frame_bindings.poses import pose_to_message
            slot_msg = pose_to_message(slot, "slot", context)
            _ctx, _admitted_pose, world, size, _box = admit_placed_box(
                slot_msg, WDH, "box-a", SCENE, origin, (0.0, 0.0, yaw))
            for axis in range(3):
                self.assertLessEqual(abs(world["position"][axis] - center[axis]), 1e-6)
            self.assertEqual(len(size), 3)
            detection = _pose(center, world_q, "box_center", "box_axis_pi", "world", "acquisition")
            det_context = BC(
                interface_id="F17", producer="t", consumer="pickup",
                request_id="req", scene_revision=SCENE, frame_id="world",
                reference="box_center", time_basis="acquisition")
            box = SimpleNamespace(height_source=1, height_valid=True)
            from luggage_planning.pick_authorization import pick_authorized
            if not pick_authorized(box):
                box = SimpleNamespace(
                    height_valid=True, height_source=1, width=WDH[0], depth=WDH[1], height=WDH[2])
            try:
                admitted = admit_collision_box(detection, WDH, det_context, box=box)
                self.assertEqual(len(admitted["position"]), 3)
            except ContractError as exc:
                if exc.code != "DETECT_FULL_GEOMETRY_REQUIRED":
                    raise
            _log("collision/%02d" % index, {"yaw": yaw})

    def test_attach_relation(self):
        from luggage_planning.attachment_frame_contracts import (
            admit_attachment, compose_box_world,
        )
        for index, (yaw, origin, floor, support, grasp) in CASES:
            geo = _geometry(yaw, origin, floor, support, grasp)
            box = _pose(geo["world_slot"], geo["tool_q"], "box_center", "box_axis_pi",
                        "world", "acquisition")
            # Tool pose is the box pose plus the grasp offset in box axes.
            tool_pos = geo["tool"]
            tool = _pose(tool_pos, geo["tool_q"], "suction_contact_frame",
                         "tool_quaternion", "world", "acquisition")
            measured = {
                "id": "box", "header_frame": "world", "stamp_ns": STAMP,
                "position": list(geo["world_slot"]),
                "quaternion_xyzw": list(geo["tool_q"]),
                "width": WDH[0], "depth": WDH[1], "height": WDH[2],
                "height_valid": True, "height_source": 1,
            }
            context = BoundaryContext(
                interface_id="F18", producer="t", consumer="vacuum",
                request_id="req", scene_revision=SCENE, frame_id="world",
                reference="box_center", time_basis="acquisition")
            attachment = admit_attachment(
                tool, box, measured, None, 7, 7, "box", context)
            moved_tool = _add(tool_pos, (0.2, -0.1, 0.05))
            moved_q = _yaw_q(yaw + SLOT_YAW + 0.4)
            restored = compose_box_world(attachment, moved_tool, moved_q)
            again = compose_box_world(attachment, tool_pos, geo["tool_q"])
            for axis in range(3):
                self.assertLessEqual(abs(again[0][axis] - geo["world_slot"][axis]), 1e-9)
            self.assertLessEqual(rotation_angle(again[1], geo["tool_q"]), 1e-7)
            third = compose_box_world(attachment, moved_tool, moved_q)
            for axis in range(3):
                self.assertLessEqual(abs(restored[0][axis] - third[0][axis]), 1e-9)
            _log("attach/%02d" % index, {"yaw": yaw})


class TestJointsAndFk(unittest.TestCase):
    def test_canonical_names_match_executor(self):
        self.assertEqual(tuple(JOINTS), CANONICAL_JOINTS)

    def test_joint_time_survives_permutation(self):
        from luggage_planning.motion_frame_adapters import timed_trajectory
        names = list(reversed(CANONICAL_JOINTS))
        points = []
        for step in range(1, 4):
            values = [float(step + index) for index in range(6)]
            ordered = [values[names.index(name)] for name in CANONICAL_JOINTS]
            # store in reversed name order
            stored = [ordered[CANONICAL_JOINTS.index(name)] for name in names]
            points.append({
                "positions": stored,
                "velocities": [0.1] * 6,
                "accelerations": [0.2] * 6,
                "effort": [],
                "time_from_start_ns": step * 100000000,
            })
        joint = {
            "contract_version": 2, "kind": "joint_trajectory", "request_id": "req",
            "scene_revision": SCENE, "stamp_ns": STAMP, "time_basis": "robot_state",
            "angle_unit": "rad", "joint_names": names, "points": points,
        }
        record = {
            "contract_version": 2, "request_id": "req", "scene_revision": SCENE,
            "stamp_ns": STAMP, "time_basis": "robot_state", "name": "carry",
            "segment_type": "joint_path", "target_link": "suction_contact_frame",
            "target_pose": None, "waypoints": [], "joint_trajectory": joint,
        }
        record.update(_policy())
        context = _context("robot_state")
        admitted = admit_motion_segment(record, context)
        self.assertEqual(admitted.joint_names, CANONICAL_JOINTS)
        self.assertEqual(
            [point["time_from_start_ns"] for point in admitted.joint_points],
            [100000000, 200000000, 300000000])
        self.assertEqual(list(admitted.joint_points[0]["velocities"]), [0.1] * 6)
        executor = object.__new__(MotionExecutor)
        executor._joint_path_continuity = 1.2
        captured = {}

        def hook(trajectory):
            captured["names"] = list(trajectory.joint_trajectory.joint_names)
            captured["times"] = [
                point.time_from_start.sec * 10**9 + point.time_from_start.nanosec
                for point in trajectory.joint_trajectory.points]
            captured["velocities"] = [
                list(point.velocities) for point in trajectory.joint_trajectory.points]
            return SimpleNamespace(success=True, message="timed joint path", fraction=1.0,
                                   used_ompl_fallback=False, moveit_error_code=0)

        executor._timed_submit = hook
        executor._cancel_check = None
        executor._v2_lock = __import__("threading").Lock()
        executor._frame = "world"
        executor._link = "suction_contact_frame"
        result = executor.execute_segment_v2(record, context)
        self.assertTrue(result.success)
        self.assertEqual(captured["names"], list(CANONICAL_JOINTS))
        self.assertEqual(captured["times"], [100000000, 200000000, 300000000])
        self.assertEqual(captured["velocities"][2], [0.1] * 6)
        out_names, out_points = timed_trajectory(admitted)
        self.assertEqual(out_names, list(CANONICAL_JOINTS))
        self.assertEqual(len(out_points), 3)

    def test_fk_thresholds_and_alternate(self):
        target = _pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0),
                       "suction_contact_frame", "tool_quaternion", "world")
        good = ([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
        at_pos = ([1.0 + 0.002, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
        over_pos = ([1.0 + 0.0021, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
        half = math.sin(0.01)
        at_rot = ([1.0, 2.0, 3.0], [0.0, 0.0, math.sin(0.01), math.cos(0.01)])
        over_rot = ([1.0, 2.0, 3.0], [0.0, 0.0, math.sin(0.0101), math.cos(0.0101)])
        self.assertTrue(fk_budget(lambda _j: good, [0] * 6, target["position"], target["quaternion_xyzw"])[0])
        self.assertTrue(fk_budget(lambda _j: at_pos, [0] * 6, target["position"], target["quaternion_xyzw"])[0])
        self.assertFalse(fk_budget(lambda _j: over_pos, [0] * 6, target["position"], target["quaternion_xyzw"])[0])
        self.assertTrue(fk_budget(lambda _j: at_rot, [0] * 6, target["position"], target["quaternion_xyzw"])[0])
        over_ok, over_msg = fk_budget(lambda _j: over_rot, [0] * 6, target["position"], target["quaternion_xyzw"])
        self.assertFalse(over_ok)
        self.assertTrue(over_msg.startswith("FK_VERIFY_BUDGET"))
        missing_ok, missing_msg = fk_budget(None, [0] * 6, target["position"], target["quaternion_xyzw"])
        self.assertFalse(missing_ok)
        self.assertTrue(missing_msg.startswith("FK_VERIFY_UNAVAILABLE"))
        self.assertTrue(half > 0.0)

        executor = object.__new__(MotionExecutor)
        executor._fk_verifier = lambda joints: good if joints[0] == 0.0 else over_pos
        executor.fk_submissions = []
        executor._frame = "world"
        executor._link = "suction_contact_frame"
        executor._occupancy = None
        executor._occupancy_object_count = 0
        from moveit_msgs.msg import Constraints
        executor._joint_constraints = lambda positions: Constraints()
        waits = []

        class Move:
            def wait_for_server(self, timeout_sec=0):
                waits.append(timeout_sec)
                return False

        executor._move_group = Move()
        executor._group = "elfin_arm"
        executor._attempts = 1
        executor._planning_time = 0.1
        executor._planner_id = "RRTConnect"
        from luggage_planning.motion_frame_contracts import AdmittedSegment
        admitted = AdmittedSegment(
            segment_type="pose_target", name="t", target_link="suction_contact_frame",
            request_id="req", scene_revision=SCENE, stamp_ns=STAMP, time_basis="scene",
            keep_tool_down=False, keep_camera_down=False, lock_wrist=False,
            allow_ompl_fallback=False, required_cartesian_fraction=0.0,
            target_pose=target, waypoints=(), joint_names=None, joint_points=None)
        from luggage_planning.motion_executor import _V2_ADMITTED
        token = _V2_ADMITTED.set(admitted)
        try:
            bad = executor._run_joint_goal([0.2] * 6, None, 1.0, "alt")
            good_result = executor._run_joint_goal([0.0] * 6, None, 1.0, "first")
        finally:
            _V2_ADMITTED.reset(token)
        self.assertFalse(bad.success)
        self.assertTrue(bad.message.startswith("FK_VERIFY_BUDGET"))
        self.assertEqual(waits, [5.0])
        self.assertFalse(good_result.success)
        self.assertIn([0.0] * 6, executor.fk_submissions)

    def test_keep_tool_down_fk_ignores_yaw(self):
        from luggage_description.frame_transforms import quaternion_product
        from luggage_planning.motion_frame_contracts import tool_down_tilt
        # 180 deg about X: tool +Z is world -Z.
        down = (1.0, 0.0, 0.0, 0.0)
        yaw_pi = quaternion_product(down, (0.0, 0.0, 1.0, 0.0))
        target = _pose((1.0, 2.0, 3.0), down,
                       "suction_contact_frame", "tool_quaternion", "world")
        flipped = ([1.0, 2.0, 3.0], list(yaw_pi))
        locked = fk_budget(
            lambda _j: flipped, [0] * 6, target["position"],
            target["quaternion_xyzw"], keep_tool_down=False)
        self.assertFalse(locked[0])
        free = fk_budget(
            lambda _j: flipped, [0] * 6, target["position"],
            target["quaternion_xyzw"], keep_tool_down=True)
        self.assertTrue(free[0], free)
        self.assertLess(tool_down_tilt(yaw_pi), 1e-9)
        extra = (math.sin(0.03), 0.0, 0.0, math.cos(0.03))
        tilted_q = quaternion_product(down, extra)
        tilted = ([1.0, 2.0, 3.0], list(tilted_q))
        bad = fk_budget(
            lambda _j: tilted, [0] * 6, target["position"],
            target["quaternion_xyzw"], keep_tool_down=True)
        self.assertFalse(bad[0])
        self.assertTrue(bad[1].startswith("FK_VERIFY_BUDGET"))
        self.assertGreater(tool_down_tilt(tilted_q), 0.02)


class TestNegatives(unittest.TestCase):
    def _reject(self, mutate, code):
        record = _segment((0, 0, 1), (0, 0, 0, 0, 1) if False else (0, 0, 0, 1))
        mutate(record)
        with self.assertRaises(BoundaryContractError) as caught:
            admit_motion_segment(record, _context())
        self.assertEqual(caught.exception.code, code)

    def test_codes(self):
        self._reject(lambda r: r.__setitem__("request_id", ""), "MISSING_IDENTITY")
        self._reject(lambda r: r.__setitem__("target_link", ""), "MISSING_IDENTITY")
        self._reject(lambda r: r["target_pose"].__setitem__("frame_id", "container_link"),
                     "FRAME_CONTRACT_MISMATCH")
        self._reject(lambda r: r["target_pose"].__setitem__("reference", "box_center"),
                     "POSE_REFERENCE_MISMATCH")
        self._reject(lambda r: r["target_pose"].__setitem__("orientation_semantics", "box_axis_pi"),
                     "ORIENTATION_SEMANTICS_MISMATCH")
        self._reject(lambda r: r.__setitem__("contract_version", 1), "CONTRACT_VERSION_MISMATCH")
        self._reject(lambda r: r["target_pose"].__setitem__("length_unit", "mm"), "UNIT_MISMATCH")
        self._reject(lambda r: r.__setitem__("scene_revision", "other"), "CALIBRATION_REVISION_MISMATCH")
        self._reject(lambda r: r.__setitem__("stamp_ns", None), "INVALID_STAMP")
        self._reject(lambda r: r["target_pose"].__setitem__("position", [float("nan"), 0, 0]),
                     "INVALID_NUMERIC_PAYLOAD")
        self._reject(lambda r: r["target_pose"].__setitem__("quaternion_xyzw", [0, 0, 0, 0]),
                     "INVALID_QUATERNION")
        self._reject(lambda r: r.__setitem__("segment_type", "rrt"), "UNKNOWN_SEGMENT_TYPE")
        self._reject(lambda r: r.__setitem__("target_link", "elfin_link6"), "TARGET_LINK_MISMATCH")
        self._reject(lambda r: r.__setitem__("policy_valid", False), "INVALID_NUMERIC_PAYLOAD")
        self._reject(lambda r: r.__setitem__("geometry_hash", "abc"), "UNSUPPORTED_RECORD_FIELD")
        self._reject(lambda r: r.__setitem__("map_revision", 3), "UNSUPPORTED_RECORD_FIELD")

    def test_joint_negatives(self):
        joint = {
            "contract_version": 2, "kind": "joint_trajectory", "request_id": "req",
            "scene_revision": SCENE, "stamp_ns": STAMP, "time_basis": "robot_state",
            "angle_unit": "rad", "joint_names": list(CANONICAL_JOINTS),
            "points": [{
                "positions": [0, 0, 0, 0, 0, 0], "velocities": [],
                "accelerations": [], "effort": [], "time_from_start_ns": 10,
            }],
        }
        record = {
            "contract_version": 2, "request_id": "req", "scene_revision": SCENE,
            "stamp_ns": STAMP, "time_basis": "robot_state", "name": "carry",
            "segment_type": "joint_path", "target_link": "suction_contact_frame",
            "target_pose": None, "waypoints": [], "joint_trajectory": joint,
        }
        record.update(_policy())
        bad = copy.deepcopy(record)
        bad["time_basis"] = "acquisition"
        bad["joint_trajectory"]["time_basis"] = "acquisition"
        with self.assertRaises(BoundaryContractError) as caught:
            admit_motion_segment(bad, _context("acquisition"))
        self.assertEqual(caught.exception.code, "TIME_BASIS_MISMATCH")
        named = copy.deepcopy(record)
        named["joint_trajectory"]["joint_names"] = list(CANONICAL_JOINTS[:-1])
        named["joint_trajectory"]["points"][0]["positions"] = [0, 0, 0, 0, 0]
        with self.assertRaises(BoundaryContractError) as caught:
            admit_motion_segment(named, _context("robot_state"))
        self.assertEqual(caught.exception.code, "INVALID_JOINT_NAMES")

    def test_tilt_and_query_offset(self):
        tilted = (0.0, math.sin(0.1), 0.0, math.cos(0.1))
        with self.assertRaises(ContractError) as caught:
            map_container_point((1, 0, 0), (0, 0, 0), tilted)
        self.assertEqual(caught.exception.code, "UNSUPPORTED_CONTAINER_TILT")
        level = _yaw_q(math.pi / 2)
        lifted = lift_vector(0.008, level)
        self.assertLess(abs(lifted[2] - 0.008), 1e-12)
        self.assertLess(abs(lifted[0]), 1e-12)
        same = framed_query_offset((0.1, 0.2, 0.3), "container_link", level)
        self.assertEqual(same, [0.1, 0.2, 0.3])
        with self.assertRaises(ContractError):
            framed_query_offset((0, 0, 0), "elfin_link6", level)

    def test_q_and_pi(self):
        q = (0.0, 0.0, 0.0, 1.0)
        self.assertLessEqual(rotation_angle(q, tuple(-v for v in q)), 1e-9)
        pi = _yaw_q(math.pi)
        self.assertGreater(rotation_angle(q, pi), 1.0)


class TestProvenance(unittest.TestCase):
    def test_gt_does_not_change_planning_pose(self):
        from luggage_planning.attachment_frame_contracts import admit_attachment
        from luggage_planning.vacuum_backend import SimVacuumBackend
        box = _pose((1, 2, 3), (0, 0, 0, 1), "box_center", "box_axis_pi", "world", "acquisition")
        tool = _pose((1, 2, 3.16), (0, 0, 0, 1), "suction_contact_frame",
                     "tool_quaternion", "world", "acquisition")
        measured = {
            "id": "box", "header_frame": "world", "stamp_ns": STAMP,
            "position": [1, 2, 3], "quaternion_xyzw": [0, 0, 0, 1],
            "width": 0.8, "depth": 0.5, "height": 0.32,
            "height_valid": True, "height_source": 1,
        }
        context = BoundaryContext(
            interface_id="F18", producer="t", consumer="vacuum",
            request_id="req", scene_revision=SCENE, frame_id="world",
            reference="box_center", time_basis="acquisition")
        attachment = admit_attachment(tool, box, measured, None, 3, 3, "box", context)
        physical = dict(attachment)
        physical["t_tool_box_translation"] = [9, 9, 9]
        scene = SimpleNamespace(calls=[])

        def attach_pickup_box(*args, **kwargs):
            scene.calls.append(args)
            return True, "ok"

        scene.attach_pickup_box = attach_pickup_box
        backend = SimVacuumBackend(SimpleNamespace(set_model_pose=lambda *a: (True, "")), scene)
        ok, _message = backend.attach({
            "attach_record": attachment, "physical_relation": physical,
            "model_name": "box",
        })
        self.assertTrue(ok)
        before = backend.planning_box_pose((1, 2, 3.16), (0, 0, 0, 1))
        physical["t_tool_box_translation"] = [4, 4, 4]
        after = backend.planning_box_pose((1, 2, 3.16), (0, 0, 0, 1))
        self.assertEqual(before, after)
        with self.assertRaises(BoundaryContractError) as caught:
            admit_attachment(tool, box, measured, None, 4, 3, "box", context)
        self.assertEqual(caught.exception.code, "GRASP_BINDING_MISMATCH")


if __name__ == "__main__":
    unittest.main()
