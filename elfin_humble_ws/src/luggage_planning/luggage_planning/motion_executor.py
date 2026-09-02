#!/usr/bin/env python3
"""MoveIt 2 client wrapper for one motion segment at a time.

Node-layer code (per docs/architecture/perception_architecture.md): it
imports moveit_msgs at the top level by design; algorithm modules must not
import it. Not a Node subclass - construct with an already-initialized node
(so the caller owns executors/callback groups) and call ``execute_segment``.

Segment routing (docs/plans/closed_loop_pick_retreat_nodes.md):

    pose_target  -> moveit_msgs/action/MoveGroup (OMPL)
    cartesian    -> moveit_msgs/srv/GetCartesianPath -> ExecuteTrajectory,
                    fraction >= cartesian_min_fraction or OMPL fallback
    named pose   -> handled by the node via plain FJT (see motion_planner_node)

``keep_tool_down`` adds an orientation constraint (tool Z down).
``keep_camera_down`` / ``lock_wrist`` are NOT implemented on purpose; the
result message says so instead of silently ignoring the flags.
"""

from __future__ import division

import math
import threading
from dataclasses import dataclass

from luggage_planning.ros_clock_wait import wait_event

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point as PointMsg, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    PositionIKRequest,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from shape_msgs.msg import SolidPrimitive

# Tool-Z (suction normal) pointing at world -Z: quaternion (x=1, w=0).
TOOL_DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)
JOINTS = ["elfin_joint1", "elfin_joint2", "elfin_joint3",
          "elfin_joint4", "elfin_joint5", "elfin_joint6"]
# Match S20 URDF revolute limits (±2π). Used only to pick the nearest
# equivalent wrap of an IK solution so OMPL does not command a 4 rad spin.
_JOINT_LIMIT = 6.28
_VEL_SCALE = 0.3
_ACC_SCALE = 0.3


@dataclass
class SegmentExecResult:
    success: bool
    message: str
    fraction: float = 0.0
    used_ompl_fallback: bool = False
    moveit_error_code: int = 0


def _wrap_near(current, target, lower=-_JOINT_LIMIT, upper=_JOINT_LIMIT):
    """Choose target + k·2π inside limits that is closest to current."""
    best = target
    best_err = abs(target - current)
    for k in range(-3, 4):
        cand = target + k * 2.0 * math.pi
        if cand < lower - 1e-6 or cand > upper + 1e-6:
            continue
        err = abs(cand - current)
        if err < best_err:
            best, best_err = cand, err
    return best


class MotionExecutor:

    def __init__(self, node, group_name="elfin_arm",
                 link_name="suction_contact_frame",
                 planner_id="RRTConnect",
                 allowed_planning_time=5.0,
                 num_planning_attempts=10,
                 planning_frame="world",
                 cartesian_max_step=0.01,
                 cartesian_min_fraction=0.95,
                 cartesian_avoid_collisions=True,
                 tool_down_abs_tol=0.05):
        self._node = node
        self._group = str(group_name)
        self._link = str(link_name)
        self._planner_id = str(planner_id)
        self._planning_time = float(allowed_planning_time)
        self._attempts = int(num_planning_attempts)
        self._frame = str(planning_frame)
        self._max_step = float(cartesian_max_step)
        self._min_fraction = float(cartesian_min_fraction)
        self._avoid_collisions = bool(cartesian_avoid_collisions)
        self._tool_down_abs_tol = float(tool_down_abs_tol)

        import rclpy
        self._rclpy = rclpy
        # MoveIt 2 Humble serves the MoveGroup action at /move_action
        # (node info on the move_group node); /move_group/action/move_group
        # has no server in this distro.
        self._move_group = rclpy.action.ActionClient(
            node, MoveGroup, "/move_action")
        self._execute = rclpy.action.ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory")
        self._cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self._ik = node.create_client(GetPositionIK, "/compute_ik")

    # ------------------------------------------------------------------

    def wait_ready(self, timeout_sec=60.0):
        """True when the MoveGroup/ExecuteTrajectory servers and the
        cartesian service are all available."""
        ok = self._move_group.wait_for_server(timeout_sec=timeout_sec)
        if not ok:
            return False
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return False
        if not self._cartesian.wait_for_service(timeout_sec=5.0):
            return False
        return self._ik.wait_for_service(timeout_sec=5.0)

    # ------------------------------------------------------------------

    def execute_segment(self, segment_msg, feedback_cb=None,
                        execute_timeout=45.0, current_joints=None):
        """Execute one luggage_msgs/MotionSegment.

        Returns ``SegmentExecResult``. ``fraction`` is 1.0 for pose-target
        paths and the computed Cartesian fraction otherwise.
        """
        seg_type = str(segment_msg.type)
        if seg_type == "pose_target":
            return self._run_pose_target(segment_msg, feedback_cb,
                                         execute_timeout, current_joints)
        if seg_type == "cartesian":
            fraction, plan = self._plan_cartesian(segment_msg, feedback_cb)
            if plan is None:
                return SegmentExecResult(
                    False, "compute_cartesian_path timeout", 0.0)
            if fraction < self._min_fraction:
                if segment_msg.allow_ompl_fallback:
                    self._notify(feedback_cb, "planning", segment_msg.name,
                                 fraction,
                                 "cartesian %.3f < %.3f; OMPL fallback"
                                 % (fraction, self._min_fraction))
                    fallback = self._run_pose_target(
                        segment_msg, feedback_cb, execute_timeout,
                        current_joints)
                    fallback.message = "%s (fallback from cartesian %.3f)" % (
                        fallback.message, fraction)
                    fallback.fraction = fraction
                    fallback.used_ompl_fallback = True
                    return fallback
                return SegmentExecResult(
                    False,
                    "cartesian fraction %.3f below %.3f and no OMPL fallback"
                    % (fraction, self._min_fraction),
                    fraction)
            return self._execute_trajectory(plan, segment_msg, fraction,
                                            feedback_cb, execute_timeout)
        return SegmentExecResult(
            False, "unknown segment type %r" % seg_type, 0.0)

    def probe_segment(self, segment_msg, start_joints=None):
        """IK + optional cartesian fraction, no execution (dry-run)."""
        ik = self._ik_joints(segment_msg.target_pose, start_joints)
        record = {
            "name": str(segment_msg.name),
            "type": str(segment_msg.type),
            "ik_ok": ik is not None,
            "ik_joints": ik,
            "fraction": None,
            "cartesian_ok": None,
        }
        if str(segment_msg.type) == "cartesian":
            fraction, plan = self._plan_cartesian(
                segment_msg, None, start_joints=start_joints)
            record["fraction"] = fraction if plan is not None else 0.0
            record["cartesian_ok"] = (
                plan is not None and fraction >= self._min_fraction)
        return record

    # ------------------------------------------------------------------
    # pose_target

    def _build_goal_constraints(self, segment_msg):
        target = segment_msg.target_pose
        constraints = Constraints()

        pos = PositionConstraint()
        pos.header.frame_id = self._frame
        pos.link_name = self._link
        box = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.005]  # 5 mm position tolerance
        box.primitives = [primitive]
        center = PointMsg()
        center.x = target.position.x
        center.y = target.position.y
        center.z = target.position.z
        box.primitive_poses = [target]
        box.primitive_poses[0].orientation.x = 0.0
        box.primitive_poses[0].orientation.y = 0.0
        box.primitive_poses[0].orientation.z = 0.0
        box.primitive_poses[0].orientation.w = 1.0
        pos.constraint_region = box
        pos.weight = 1.0
        constraints.position_constraints = [pos]

        orient = OrientationConstraint()
        orient.header.frame_id = self._frame
        orient.link_name = self._link
        orient.orientation = target.orientation
        orient.absolute_x_axis_tolerance = 0.05
        orient.absolute_y_axis_tolerance = 0.05
        orient.absolute_z_axis_tolerance = 0.05
        orient.weight = 1.0
        constraints.orientation_constraints = [orient]

        if segment_msg.keep_tool_down:
            down = OrientationConstraint()
            down.header.frame_id = self._frame
            down.link_name = self._link
            down.orientation.x, down.orientation.y = TOOL_DOWN_QUAT[:2]
            down.orientation.z, down.orientation.w = TOOL_DOWN_QUAT[2:]
            down.absolute_x_axis_tolerance = self._tool_down_abs_tol
            down.absolute_y_axis_tolerance = self._tool_down_abs_tol
            down.absolute_z_axis_tolerance = 3.14  # yaw free
            down.weight = 1.0
            constraints.orientation_constraints.append(down)

        return constraints

    def _run_pose_target(self, segment_msg, feedback_cb, execute_timeout,
                         current_joints=None):
        self._notify(feedback_cb, "planning", segment_msg.name, 0.0)
        goal = MoveGroup.Goal()
        goal.request.group_name = self._group
        goal.request.num_planning_attempts = self._attempts
        goal.request.allowed_planning_time = self._planning_time
        goal.request.planner_id = self._planner_id
        goal.request.start_state.is_diff = True
        goal.request.max_velocity_scaling_factor = _VEL_SCALE
        goal.request.max_acceleration_scaling_factor = _ACC_SCALE
        ik_joints = self._ik_joints(segment_msg.target_pose, current_joints)
        if ik_joints is not None:
            note = "IK joint goal"
            return self._run_joint_goal(
                ik_joints, feedback_cb, execute_timeout, note)
        goal.request.goal_constraints = [self._build_goal_constraints(
            segment_msg)]
        note = "pose constraint fallback"
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        unimplemented = self._unimplemented_flags_note(segment_msg)
        if not self._move_group.wait_for_server(timeout_sec=5.0):
            return SegmentExecResult(
                False, "move_group action server unavailable", 0.0)
        future = self._move_group.send_goal_async(
            goal, feedback_callback=self._movegroup_feedback(
                feedback_cb, segment_msg.name))
        handle = self._wait_future(future, 5.0 + self._planning_time)
        if handle is None or not handle.accepted:
            return SegmentExecResult(
                False, "MoveGroup goal rejected/timeout", 0.0)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return SegmentExecResult(
                False, "MoveGroup execution timeout", 0.0)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return SegmentExecResult(
                False, "MoveGroup error_code=%s (%s)" % (code, note),
                0.0, moveit_error_code=code)
        message = "pose_target ok (%s)" % note
        if unimplemented:
            message += "; " + unimplemented
        return SegmentExecResult(True, message, 1.0, moveit_error_code=code)

    def execute_joints(self, positions, feedback_cb=None, execute_timeout=45.0,
                       current_joints=None):
        """MoveIt joint-space plan/execute to ``positions`` (rad, JOINTS order)."""
        wrapped = list(positions)
        if current_joints and len(current_joints) == len(JOINTS):
            wrapped = [
                _wrap_near(cur, tgt) for cur, tgt in zip(current_joints, wrapped)]
        return self._run_joint_goal(
            wrapped, feedback_cb, execute_timeout, "named joint goal")

    def _run_joint_goal(self, positions, feedback_cb, execute_timeout, note):
        self._notify(feedback_cb, "planning", note, 0.0)
        goal = MoveGroup.Goal()
        goal.request.group_name = self._group
        goal.request.num_planning_attempts = self._attempts
        goal.request.allowed_planning_time = self._planning_time
        goal.request.planner_id = self._planner_id
        goal.request.start_state.is_diff = True
        goal.request.max_velocity_scaling_factor = _VEL_SCALE
        goal.request.max_acceleration_scaling_factor = _ACC_SCALE
        goal.request.goal_constraints = [self._joint_constraints(positions)]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        if not self._move_group.wait_for_server(timeout_sec=5.0):
            return SegmentExecResult(
                False, "move_group action server unavailable", 0.0)
        future = self._move_group.send_goal_async(
            goal, feedback_callback=self._movegroup_feedback(feedback_cb, note))
        handle = self._wait_future(future, 5.0 + self._planning_time)
        if handle is None or not handle.accepted:
            return SegmentExecResult(
                False, "MoveGroup goal rejected/timeout", 0.0)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return SegmentExecResult(
                False, "MoveGroup execution timeout", 0.0)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return SegmentExecResult(
                False, "MoveGroup error_code=%s (%s)" % (code, note),
                0.0, moveit_error_code=code)
        return SegmentExecResult(
            True, "joint_target ok (%s)" % note, 1.0, moveit_error_code=code)

    def _ik_joints(self, pose, current_joints):
        """IK seeded at the current arm, then wrap each joint nearest current.

        OMPL pose-constraint sampling otherwise returns a ±2π equivalent of
        joint1 (limits are ±6.28) and the controller times out 4 rad away.
        """
        if not self._ik.wait_for_service(timeout_sec=2.0):
            return None
        request = GetPositionIK.Request()
        request.ik_request.group_name = self._group
        request.ik_request.ik_link_name = self._link
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.is_diff = True
        if current_joints and len(current_joints) == len(JOINTS):
            request.ik_request.robot_state.joint_state.name = list(JOINTS)
            request.ik_request.robot_state.joint_state.position = [
                float(v) for v in current_joints]
        request.ik_request.timeout = DurationMsg(sec=1, nanosec=0)
        stamped = PoseStamped()
        stamped.header.frame_id = self._frame
        stamped.pose = pose
        request.ik_request.pose_stamped = stamped
        future = self._ik.call_async(request)
        response = self._wait_future(future, 3.0)
        if response is None:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        by_name = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position))
        if not all(name in by_name for name in JOINTS):
            return None
        raw = [float(by_name[name]) for name in JOINTS]
        if not current_joints or len(current_joints) != len(JOINTS):
            return raw
        return [
            _wrap_near(cur, tgt) for cur, tgt in zip(current_joints, raw)]

    @staticmethod
    def _joint_constraints(positions):
        constraints = Constraints()
        for name, value in zip(JOINTS, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.02
            joint.tolerance_below = 0.02
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        return constraints

    # ------------------------------------------------------------------
    # cartesian

    def _plan_cartesian(self, segment_msg, feedback_cb, start_joints=None):
        self._notify(feedback_cb, "planning", segment_msg.name, 0.0)
        request = GetCartesianPath.Request()
        request.header.frame_id = self._frame
        request.group_name = self._group
        request.link_name = self._link
        request.start_state.is_diff = True
        if start_joints is not None:
            request.start_state.joint_state.name = list(JOINTS)
            request.start_state.joint_state.position = [
                float(v) for v in start_joints]
        request.waypoints = [segment_msg.target_pose]
        request.max_step = self._max_step
        request.jump_threshold = 0.0
        request.avoid_collisions = self._avoid_collisions
        future = self._cartesian.call_async(request)
        response = self._wait_future(future, 15.0)
        if response is None:
            return 0.0, None
        return float(response.fraction), response

    def _execute_trajectory(self, plan_response, segment_msg, fraction,
                            feedback_cb, execute_timeout):
        self._notify(feedback_cb, "executing", segment_msg.name, fraction)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = plan_response.solution
        if not self._execute.wait_for_server(timeout_sec=5.0):
            return SegmentExecResult(
                False, "execute_trajectory server unavailable", fraction)
        future = self._execute.send_goal_async(goal)
        handle = self._wait_future(future, 10.0)
        if handle is None or not handle.accepted:
            return SegmentExecResult(
                False, "ExecuteTrajectory goal rejected", fraction)
        wrapped = self._wait_goal(handle, execute_timeout)
        if wrapped is None:
            handle.cancel_goal()
            return SegmentExecResult(
                False, "ExecuteTrajectory timeout", fraction)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return SegmentExecResult(
                False, "ExecuteTrajectory error_code=%s" % code,
                fraction, moveit_error_code=code)
        message = "cartesian ok (fraction %.3f)" % fraction
        unimplemented = self._unimplemented_flags_note(segment_msg)
        if unimplemented:
            message += "; " + unimplemented
        return SegmentExecResult(
            True, message, fraction, moveit_error_code=code)

    # ------------------------------------------------------------------
    # helpers

    @staticmethod
    def _unimplemented_flags_note(segment_msg):
        missing = []
        if segment_msg.keep_camera_down:
            missing.append("keep_camera_down")
        if segment_msg.lock_wrist:
            missing.append("lock_wrist")
        if not missing:
            return ""
        return "NOT_IMPLEMENTED: %s" % ", ".join(missing)

    def _notify(self, feedback_cb, stage, segment_name, fraction, note=""):
        if feedback_cb is None:
            return
        try:
            feedback_cb(stage, segment_name, fraction, note)
        except Exception:  # noqa: BLE001 - feedback must never break execution
            pass

    def _movegroup_feedback(self, feedback_cb, segment_name):
        # MoveGroup feedback carries only a state string (MONITORING_PLANNING,
        # PLANNING, ...), no numeric fraction; map planning states to the
        # "planning" stage and everything else to "executing".
        def _cb(_feedback_msg):
            # rclpy hands the callback a wrapper whose .feedback is the
            # MoveGroup.Feedback carrying the state string.
            raw = getattr(_feedback_msg, "feedback", _feedback_msg)
            state = str(getattr(raw, "state", ""))
            stage = "planning" if "PLAN" in state.upper() else "executing"
            self._notify(feedback_cb, stage, segment_name, 0.0)
        return _cb

    def _wait_future(self, future, timeout_sec):
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout_sec):
            return None
        return future.result()

    def _wait_goal(self, handle, timeout_sec):
        """Wait for an accepted goal. Timeout is sim seconds when use_sim_time."""
        event = threading.Event()
        future = handle.get_result_async()
        future.add_done_callback(lambda _f: event.set())
        reached, _reason = wait_event(
            event, timeout_sec, clock=self._node.get_clock())
        if not reached:
            return None
        return future.result()
