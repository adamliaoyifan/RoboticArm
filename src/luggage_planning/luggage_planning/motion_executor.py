#!/usr/bin/env python3
"""MoveIt 2 client wrapper for one motion segment at a time.

Node-layer code (per docs/architecture/perception_architecture.md): it
imports moveit_msgs at the top level by design; algorithm modules must not
import it. Not a Node subclass - construct with an already-initialized node
(so the caller owns executors/callback groups) and call ``execute_segment``.

Segment routing (docs/plans/closed_loop_pick_retreat_nodes.md):

    pose_target  -> compute_ik then MoveGroup plan_only -> rest-to-rest
                    ExecuteTrajectory (no MoveIt TOTG execute)
    cartesian    -> GetCartesianPath geometry -> rest-to-rest
                    ExecuteTrajectory; fraction >= cartesian_min_fraction
                    or OMPL/IK fallback (also rest-to-rest)
    named pose   -> handled by the node via plain FJT (see motion_planner_node)

Pose-constraint OMPL is not used: it cannot sample KDL-unreachable
goals (MoveIt 99999) and can wrap joint1 by ±2π. ``keep_tool_down`` is
baked into waypoint target_pose; extra orientation constraints are not
applied on the path. ``keep_camera_down`` / ``lock_wrist`` are NOT
implemented on purpose; the result message says so instead of silently
ignoring the flags.
"""

from __future__ import division

import math
import threading
from dataclasses import dataclass

from luggage_planning.ros_clock_wait import wait_event
from luggage_planning.trajectory_timing import time_parameterize_cartesian

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK

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


def _sec_to_duration(sec):
    nsec_total = int(round(float(sec) * 1e9))
    if nsec_total < 0:
        nsec_total = 0
    return DurationMsg(
        sec=nsec_total // 1000000000,
        nanosec=nsec_total % 1000000000,
    )


def apply_rest_to_rest_to_joint_trajectory(joint_trajectory, vel_scale, acc_scale):
    """Replace TOTG/linear times with the site rest-to-rest trapezoid.

    Keeps the geometric knots. Writes ``time_from_start``, velocities, and
    accelerations capped to 60 deg/s and 60 deg/s^2.
    """
    points = list(joint_trajectory.points)
    if len(points) < 2:
        return None
    timed = time_parameterize_cartesian(
        [list(pt.positions) for pt in points], vel_scale, acc_scale)
    for point, t_s, vel, acc in zip(
            points, timed.times_s, timed.velocities_rad,
            timed.accelerations_rad):
        point.time_from_start = _sec_to_duration(t_s)
        point.velocities = list(vel)
        point.accelerations = list(acc)
    return timed


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
                 tool_down_abs_tol=0.05,
                 velocity_scaling=_VEL_SCALE,
                 acceleration_scaling=_ACC_SCALE):
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
        self._vel_scale = float(velocity_scaling)
        self._acc_scale = float(acceleration_scaling)
        self._tool_down_abs_tol = float(tool_down_abs_tol)
        # Set per execute_segment call (goals are serialized: the stack
        # never runs two motion segments at once, move_group would refuse
        # the second). When the outer action goal is cancelled, inner
        # action handles are cancelled so the controller stops instead of
        # grinding on past the caller's timeout.
        self._cancel_check = None
        self._last_ik_code = 0

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
                        execute_timeout=45.0, current_joints=None,
                        cancel_check=None):
        """Execute one luggage_msgs/MotionSegment.

        Returns ``SegmentExecResult``. ``fraction`` is 1.0 for pose-target
        paths and the computed Cartesian fraction otherwise.
        ``cancel_check`` is polled while inner action goals run; a True
        return cancels the inner goal so the controller stops.
        """
        self._cancel_check = cancel_check
        try:
            return self._execute_segment_inner(
                segment_msg, feedback_cb, execute_timeout, current_joints)
        finally:
            self._cancel_check = None

    def _execute_segment_inner(self, segment_msg, feedback_cb,
                               execute_timeout, current_joints):
        seg_type = str(segment_msg.type)
        if seg_type == "pose_target":
            return self._run_pose_target(segment_msg, feedback_cb,
                                         execute_timeout, current_joints)
        if seg_type == "cartesian":
            fraction, plan = self._plan_cartesian(segment_msg, feedback_cb)
            if plan is None:
                return SegmentExecResult(
                    False, "compute_cartesian_path timeout", 0.0)
            # A segment may carry its own stricter fraction requirement
            # (suction approach/attach/retry_reverse require 1.0): enforce
            # it BEFORE execution so a partial path is never dispatched.
            required = max(
                self._min_fraction,
                float(getattr(segment_msg,
                              "required_cartesian_fraction", 0.0) or 0.0))
            if fraction < required:
                if (segment_msg.allow_ompl_fallback
                        and required <= self._min_fraction):
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
                    % (fraction, required),
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

    def _run_pose_target(self, segment_msg, feedback_cb, execute_timeout,
                         current_joints=None):
        self._notify(feedback_cb, "planning", segment_msg.name, 0.0)
        ik_joints = self._ik_joints(segment_msg.target_pose, current_joints)
        if ik_joints is not None:
            result = self._run_joint_goal(
                ik_joints, feedback_cb, execute_timeout,
                "joint-space execute after IK")
            unimplemented = self._unimplemented_flags_note(segment_msg)
            if result.success and unimplemented:
                result.message += "; " + unimplemented
            return result
        # Pose-constraint OMPL cannot sample a pose KDL already rejected
        # (RRTConnect "Unable to sample any valid states for goal tree"
        # → MoveIt 99999) and can wrap joint1 by ±2π.
        return SegmentExecResult(
            False,
            "IK failed error_code=%s (no pose-constraint fallback)"
            % self._last_ik_code,
            0.0, moveit_error_code=int(self._last_ik_code))

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
        goal.request.goal_constraints = [self._joint_constraints(positions)]
        goal.planning_options.plan_only = True
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
        wrapped = self._wait_goal(handle, 5.0 + self._planning_time)
        if wrapped is None:
            handle.cancel_goal()
            return SegmentExecResult(
                False, "MoveGroup plan timeout", 0.0)
        result = getattr(wrapped, "result", wrapped)
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            return SegmentExecResult(
                False, "MoveGroup error_code=%s (%s)" % (code, note),
                0.0, moveit_error_code=code)
        planned = getattr(result, "planned_trajectory", None)
        if planned is None or not planned.joint_trajectory.points:
            return SegmentExecResult(
                False, "MoveGroup plan_only returned no trajectory (%s)" % note,
                0.0, moveit_error_code=code)
        executed = self._execute_robot_trajectory(
            planned, fraction=1.0, feedback_cb=feedback_cb,
            execute_timeout=execute_timeout, segment_name=note)
        if executed.success:
            executed.message = "joint_target ok (%s; rest-to-rest)" % note
        elif executed.moveit_error_code == MoveItErrorCodes.CONTROL_FAILED:
            executed.message = (
                "%s; CONTROL_FAILED=-4 is execution "
                "(controller/ServoJ/waypoint), not IK" % executed.message
            )
        return executed

    def _ik_joints(self, pose, current_joints):
        """IK seeded at the current arm, then wrap each joint nearest current.

        OMPL pose-constraint sampling otherwise returns a ±2π equivalent of
        joint1 (limits are ±6.28) and the controller times out 4 rad away.
        """
        self._last_ik_code = 0
        if not self._ik.wait_for_service(timeout_sec=2.0):
            self._last_ik_code = -1
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
            self._last_ik_code = -1
            return None
        self._last_ik_code = int(response.error_code.val)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        by_name = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position))
        if not all(name in by_name for name in JOINTS):
            self._last_ik_code = -2
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
        # Geometry only. Do not let GetCartesianPath TOTG (0.0→1.0 or the
        # launch 0.6×108 deg/s^2 profile) time the path. Rest-to-rest
        # overwrites times/vel/acc before ExecuteTrajectory.
        future = self._cartesian.call_async(request)
        response = self._wait_future(future, 15.0)
        if response is None:
            return 0.0, None
        return float(response.fraction), response

    def _retime_robot_trajectory(self, robot_trajectory):
        """Drop MoveIt TOTG times; write rest-to-rest vel/acc/time."""
        timed = apply_rest_to_rest_to_joint_trajectory(
            robot_trajectory.joint_trajectory,
            self._vel_scale, self._acc_scale)
        if timed is not None:
            self._node.get_logger().info(
                "rest-to-rest duration=%.3fs path=%.4f rad "
                "v_peak=%.3f rad/s a_max=%.3f rad/s^2 (site cap 60 deg/s^2)"
                % (timed.duration, timed.path_length, timed.v_peak, timed.a_max)
            )
        return robot_trajectory

    def _execute_trajectory(self, plan_response, segment_msg, fraction,
                            feedback_cb, execute_timeout):
        executed = self._execute_robot_trajectory(
            plan_response.solution, fraction, feedback_cb, execute_timeout,
            segment_name=segment_msg.name)
        if executed.success:
            message = "cartesian ok (fraction %.3f; rest-to-rest)" % fraction
            unimplemented = self._unimplemented_flags_note(segment_msg)
            if unimplemented:
                message += "; " + unimplemented
            executed.message = message
        return executed

    def _execute_robot_trajectory(
            self, robot_trajectory, fraction, feedback_cb, execute_timeout,
            segment_name=""):
        self._notify(feedback_cb, "executing", segment_name, fraction)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self._retime_robot_trajectory(robot_trajectory)
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
        return SegmentExecResult(
            True, "rest-to-rest execute ok", fraction, moveit_error_code=code)

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
        watcher = None
        if self._cancel_check is not None:
            def _watch():
                while not event.wait(0.2):
                    try:
                        if self._cancel_check():
                            handle.cancel_goal()
                            return
                    except Exception:  # noqa: BLE001 - watcher must not leak
                        return
            watcher = threading.Thread(target=_watch, daemon=True)
            watcher.start()
        reached, _reason = wait_event(
            event, timeout_sec, clock=self._node.get_clock())
        if not reached:
            if watcher is not None and watcher.is_alive():
                try:
                    handle.cancel_goal()
                except Exception:  # noqa: BLE001 - best-effort cancel
                    pass
            return None
        return future.result()
