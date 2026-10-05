#!/usr/bin/env python3
"""j5 and j6 are not refuse gates. keep_tool_down is the orientation gate.

A keep_tool_down goal constrains suction roll/pitch to tool_down_abs_tol
and leaves yaw free. Joint travel on j5 and j6 is recorded only.
Excursion still refuses an arm unwind.
"""

from __future__ import division

import math
import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts")))

from luggage_planning.motion_executor import (  # noqa: E402
    JOINTS,
    TOOL_DOWN_QUAT,
    MotionExecutor,
    SegmentExecResult,
    _TOOL_DOWN_ABS_TOL,
)
from luggage_planning.pose import Point, Pose, Quaternion  # noqa: E402
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory  # noqa: E402
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # noqa: E402


_SEED = [0.20, -1.50, -1.20, 0.10, math.pi / 2.0, 0.30]
_J5_FLIP = [0.20, -1.50, -1.20, 0.10, -math.pi / 2.0, 0.30]
_J6_SPIN = [0.20, -1.50, -1.20, 0.10, math.pi / 2.0, 0.30 + 3.50]


class _Point(object):
    def __init__(self, positions):
        self.positions = list(positions)


class _FakeFuture(object):
    def __init__(self, response):
        self._response = response

    def result(self):
        return self._response


class _FakeIK(object):
    def __init__(self, joints):
        self.joints = list(joints)

    def wait_for_service(self, timeout_sec=2.0):
        return True

    def call_async(self, request):
        response = SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            solution=SimpleNamespace(
                joint_state=SimpleNamespace(
                    name=list(JOINTS), position=list(self.joints))))
        return _FakeFuture(response)


def _pose():
    return Pose(
        position=Point(x=1.0, y=-0.5, z=0.8),
        orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))


def _segment(name, typ="cartesian", keep_tool_down=False):
    return SimpleNamespace(
        name=name,
        type=typ,
        target_pose=_pose(),
        waypoints=[],
        keep_tool_down=keep_tool_down,
        keep_camera_down=False,
        lock_wrist=False,
        allow_ompl_fallback=False,
        required_cartesian_fraction=0.0,
    )


def _cartesian_response(points, fraction=1.0):
    traj = SimpleNamespace(
        joint_names=list(JOINTS),
        points=[_Point(row) for row in points])
    return SimpleNamespace(
        fraction=float(fraction),
        solution=SimpleNamespace(joint_trajectory=traj),
    )


def _robot_traj(points):
    joint = JointTrajectory()
    joint.joint_names = list(JOINTS)
    for row in points:
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in row]
        joint.points.append(pt)
    traj = RobotTrajectory()
    traj.joint_trajectory = joint
    return traj


def _move_result(points, code=MoveItErrorCodes.SUCCESS):
    return SimpleNamespace(
        error_code=SimpleNamespace(val=int(code)),
        planned_trajectory=_robot_traj(points),
    )


def _policy_executor():
    executor = object.__new__(MotionExecutor)
    executor._group = "elfin_arm"
    executor._link = "suction_contact_frame"
    executor._frame = "world"
    executor._max_step = 0.01
    executor._min_fraction = 0.95
    executor._avoid_collisions = True
    executor._excursion_limit = 3.0
    executor._wrist_margin = 0.50
    executor._wrist_free = set()
    executor._pose_target_max_delta = 1.50
    executor._j6_limit = 2.50
    executor._tool_down_abs_tol = _TOOL_DOWN_ABS_TOL
    executor._attempts = 10
    executor._planning_time = 5.0
    executor._planner_id = "RRTConnect"
    executor._vel_scale = 0.3
    executor._acc_scale = 0.3
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
    executor._notify = lambda *args, **kwargs: None
    executor._fk_xyz = lambda *args, **kwargs: None
    executor._node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(
            info=lambda *args, **kwargs: None,
            warning=lambda *args, **kwargs: None))
    return executor


class CartesianJointTravelTest(unittest.TestCase):
    def test_j5_flip_is_not_refused(self):
        executor = _policy_executor()
        response = _cartesian_response([_SEED, _J5_FLIP])
        _fraction, plan = executor._judge_cartesian_response(
            _segment("transit"), [_pose()], response, 1.0)
        self.assertFalse(getattr(plan, "j6_spin", False))
        self.assertFalse(executor._wrist_measurement["ok"])
        self.assertTrue(executor._j6_measurement["ok"])

    def test_j6_spin_is_not_refused(self):
        executor = _policy_executor()
        response = _cartesian_response([_SEED, _J6_SPIN])
        _fraction, plan = executor._judge_cartesian_response(
            _segment("place_exit"), [_pose()], response, 1.0)
        self.assertIs(plan, response)
        self.assertFalse(executor._j6_measurement["ok"])

    def test_excursion_still_rejects(self):
        executor = _policy_executor()
        start = [0.0, 0.0, 0.0, 0.0, 1.57, 0.0]
        end = [6.58, 0.0, 0.0, 0.0, 1.57, 0.0]
        response = _cartesian_response([start, end])
        executor._fk_xyz = lambda *args, **kwargs: (0.0, 0.0, 0.0)
        segment = _segment("place_exit")
        segment.target_pose = Pose(
            position=Point(x=1.39, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0))
        _fraction, plan = executor._judge_cartesian_response(
            segment, [segment.target_pose], response, 1.0)
        self.assertTrue(getattr(plan, "excursion_rejected", False))


class PoseTargetJointTravelTest(unittest.TestCase):
    def _run(self, ik_joints):
        executor = _policy_executor()
        executor._ik = _FakeIK(ik_joints)
        captured = []

        def _joint_goal(positions, feedback_cb, execute_timeout, note,
                        seed_joints=None, segment_msg=None):
            captured.append(list(positions))
            return SegmentExecResult(True, "ok", 1.0)

        executor._run_joint_goal = _joint_goal
        result = executor._run_pose_target(
            _segment("transit", typ="pose_target", keep_tool_down=True),
            None, 45.0, current_joints=_SEED)
        return captured, result

    def test_j5_flip_ik_is_kept(self):
        captured, result = self._run(_J5_FLIP)
        self.assertTrue(result.success)
        self.assertEqual(captured[0][4], _J5_FLIP[4])

    def test_j6_spin_ik_is_kept(self):
        captured, result = self._run(_J6_SPIN)
        self.assertTrue(result.success)
        self.assertEqual(len(captured), 1)
        # _wrap_near folds the 3.5 rad hop onto the nearer 2π image.
        delta = abs(captured[0][5] - _SEED[5])
        self.assertLess(delta, math.pi)
        self.assertAlmostEqual(
            (captured[0][5] - _J6_SPIN[5]) % (2.0 * math.pi), 0.0, places=5)


class FinishMoveGroupTest(unittest.TestCase):
    def test_j6_spin_is_not_refused(self):
        executor = _policy_executor()
        executor._execute_robot_trajectory = (
            lambda *args, **kwargs: SegmentExecResult(True, "ok", 1.0))
        result = executor._finish_move_group(
            _move_result([_SEED, _J6_SPIN]), "IK joint goal",
            _segment("transit"), None, 45.0, "")
        self.assertTrue(result.success)
        self.assertFalse(executor._j6_measurement["ok"])


class ToolDownConstraintTest(unittest.TestCase):
    def test_keep_tool_down_frees_yaw_and_bounds_tilt(self):
        down = _policy_executor()._tool_down_constraint()
        self.assertAlmostEqual(down.orientation.x, TOOL_DOWN_QUAT[0])
        self.assertAlmostEqual(down.orientation.w, TOOL_DOWN_QUAT[3])
        self.assertAlmostEqual(down.absolute_x_axis_tolerance, 0.05)
        self.assertAlmostEqual(down.absolute_y_axis_tolerance, 0.05)
        self.assertAlmostEqual(down.absolute_z_axis_tolerance, math.pi)

    def test_keep_tool_down_goal_uses_only_that_constraint(self):
        from geometry_msgs.msg import Pose as PoseMsg
        executor = _policy_executor()
        segment = _segment("transit", keep_tool_down=True)
        segment.target_pose = PoseMsg()
        segment.target_pose.position.x = 1.0
        segment.target_pose.orientation.y = 1.0
        constraints = executor._build_goal_constraints(segment)
        self.assertEqual(len(constraints.orientation_constraints), 1)
        down = constraints.orientation_constraints[0]
        self.assertAlmostEqual(down.absolute_z_axis_tolerance, math.pi)
        self.assertAlmostEqual(down.orientation.x, 1.0)

    def test_without_keep_tool_down_locks_the_target_quaternion(self):
        from geometry_msgs.msg import Pose as PoseMsg
        executor = _policy_executor()
        segment = _segment("pre_grasp", keep_tool_down=False)
        segment.target_pose = PoseMsg()
        segment.target_pose.orientation.y = 1.0
        constraints = executor._build_goal_constraints(segment)
        self.assertEqual(len(constraints.orientation_constraints), 1)
        orient = constraints.orientation_constraints[0]
        self.assertAlmostEqual(orient.absolute_z_axis_tolerance, 0.05)
        self.assertAlmostEqual(orient.orientation.y, 1.0)


class ToolDownWiringTest(unittest.TestCase):
    def test_node_declares_tool_down_tolerance(self):
        import motion_planner_node as mod
        source = open(mod.__file__, encoding="utf-8").read()
        self.assertIn('"tool_down_abs_tol"', source)
        self.assertIn('self.declare_parameter("tool_down_abs_tol", 0.05)', source)


if __name__ == "__main__":
    unittest.main()
