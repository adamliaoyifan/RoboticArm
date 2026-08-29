#!/usr/bin/env python3
"""PlanMotion action server + GoToRobotPose (ROS 2 Humble port).

One segment per PlanMotion goal; the eval driver loops the four pick
segments itself. Feedback walks planning -> executing -> settling
(``settle_criterion.SettleTracker`` on /joint_states) so the caller can
wait for geometry_ok to recover before the next segment.

GoToRobotPose resolves a named pose from robot_poses YAML to joint angles
and sends them via plain FJT (no MoveIt; that is what observe_pose_hold
does, and named poses are joint-space anyway).

Keeps the original node name per docs/plans/closed_loop_pick_retreat_nodes.md.
"""

from __future__ import division

import os
import threading
import time

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from luggage_msgs.action import GoToRobotPose, PlanMotion

from luggage_planning.motion_executor import MotionExecutor
from luggage_planning.settle_criterion import SettleTracker

JOINTS = ["elfin_joint1", "elfin_joint2", "elfin_joint3",
          "elfin_joint4", "elfin_joint5", "elfin_joint6"]


class MotionPlannerNode(Node):

    def __init__(self):
        super().__init__("motion_planner")
        self._group = ReentrantCallbackGroup()

        self.declare_parameter("robot_poses_config", "")
        self.declare_parameter("execute_timeout", 90.0)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("planner_id", "RRTConnect")
        self.declare_parameter("cartesian_max_step", 0.01)
        self.declare_parameter("cartesian_min_fraction", 0.95)
        # Settle gate between segments.
        self.declare_parameter("settle_vel_tol", 0.02)
        self.declare_parameter("settle_hold_time", 0.5)
        self.declare_parameter("settle_timeout", 8.0)
        self.declare_parameter("named_pose_duration", 8.0)

        self._joint_state = None
        self._joint_lock = threading.Lock()
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self._group)

        self._executor_client = MotionExecutor(
            self,
            allowed_planning_time=float(
                self.get_parameter("planning_time").value),
            num_planning_attempts=int(
                self.get_parameter("num_planning_attempts").value),
            planner_id=str(self.get_parameter("planner_id").value),
            cartesian_max_step=float(
                self.get_parameter("cartesian_max_step").value),
            cartesian_min_fraction=float(
                self.get_parameter("cartesian_min_fraction").value),
        )
        self._fjt = rclpy.action.ActionClient(
            self, FollowJointTrajectory,
            "/elfin_arm_controller/follow_joint_trajectory",
            callback_group=self._group)

        self._plan_action = ActionServer(
            self, PlanMotion, "/motion_planner/plan_motion",
            execute_callback=self._execute_plan_motion,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._group)
        self._pose_action = ActionServer(
            self, GoToRobotPose, "/motion_planner/go_to_robot_pose",
            execute_callback=self._execute_goto_pose,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._group)

        # Do not wait_for_server here: __init__ runs before the executor
        # spins, so discovery of /move_action always times out and the
        # node sits 60 s before advertising PlanMotion. execute_callback
        # already calls wait_ready(15s) per goal.
        self.get_logger().info(
            "motion_planner up (move_group probed on first PlanMotion)")

    # ------------------------------------------------------------------
    # shared plumbing

    def _accept_goal(self, goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def _on_joint_state(self, msg):
        if set(JOINTS) <= set(msg.name):
            with self._joint_lock:
                self._joint_state = msg

    def _joint_positions(self):
        with self._joint_lock:
            if self._joint_state is None:
                return None, None
            by_name = dict(zip(self._joint_state.name,
                               self._joint_state.position))
            by_vel = dict(zip(self._joint_state.name,
                              self._joint_state.velocity
                              if len(self._joint_state.velocity) else []))
            return ([float(by_name.get(j, 0.0)) for j in JOINTS],
                    [float(by_vel.get(j, 0.0)) for j in JOINTS])

    def _named_pose(self, pose_name):
        path = str(self.get_parameter("robot_poses_config").value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(
                get_package_share_directory("luggage_description"),
                "config", "robot_poses.yaml.example")
        with open(path, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
        try:
            pose = config["poses"][pose_name]
            return [float(v) for v in pose["values"]]
        except KeyError as exc:
            raise RuntimeError("pose %r not found in %s (%s)"
                               % (pose_name, path, exc))

    # ------------------------------------------------------------------
    # PlanMotion

    def _execute_plan_motion(self, goal_handle):
        segment = goal_handle.request.segment
        result = PlanMotion.Result()
        name = str(segment.name)

        def feedback(stage, segment_name, fraction, note=""):
            fb = PlanMotion.Feedback()
            fb.stage = str(stage)
            fb.segment_name = str(segment_name)
            fb.fraction = float(fraction)
            goal_handle.publish_feedback(fb)
            if note:
                self.get_logger().info("segment %s: %s" % (segment_name, note))

        # Generous: first graph discovery inside this process can take
        # seconds even when move_group has been up the whole time.
        if not self._executor_client.wait_ready(timeout_sec=15.0):
            goal_handle.abort()
            result.success = False
            result.message = "move_group unavailable"
            return result

        try:
            ok, message, fraction = self._executor_client.execute_segment(
                segment, feedback_cb=feedback,
                execute_timeout=float(
                    self.get_parameter("execute_timeout").value),
                current_joints=self._joint_positions()[0])
        except Exception as exc:  # noqa: BLE001 - action boundary
            self.get_logger().error("segment %s raised: %s" % (name, exc))
            goal_handle.abort()
            result.success = False
            result.message = "executor raised: %s" % exc
            return result

        if not ok:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            result.success = False
            result.message = message
            return result

        settled = self._wait_settled(
            lambda: feedback("settling", name, fraction))
        if not settled:
            goal_handle.abort()
            result.success = False
            result.message = "%s; settle timeout" % message
            return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "%s; canceled after settle" % message
            return result

        goal_handle.succeed()
        result.success = True
        result.message = message
        return result

    def _wait_settled(self, pulse, timeout=None):
        timeout = timeout or float(self.get_parameter("settle_timeout").value)
        tracker = SettleTracker(
            float(self.get_parameter("settle_vel_tol").value),
            float(self.get_parameter("settle_hold_time").value))
        deadline = time.monotonic() + timeout
        t0 = time.monotonic()
        while time.monotonic() < deadline:
            time.sleep(0.05)
            positions, velocities = self._joint_positions()
            if positions is None:
                continue
            now = time.monotonic()
            pos_d = dict(zip(JOINTS, positions))
            vel_d = dict(zip(JOINTS, velocities or [0.0] * len(JOINTS)))
            tracker.update(now - t0, vel_d, pos_d)
            if tracker.settled_at is not None:
                return True
            pulse()
        return tracker.settled_at is not None

    # ------------------------------------------------------------------
    # GoToRobotPose (named joint pose via FJT)

    def _execute_goto_pose(self, goal_handle):
        result = GoToRobotPose.Result()
        pose_name = str(goal_handle.request.pose_name)

        def feedback(stage, remaining_error=0.0):
            fb = GoToRobotPose.Feedback()
            fb.stage = str(stage)
            fb.remaining_error = float(remaining_error)
            goal_handle.publish_feedback(fb)

        try:
            target = self._named_pose(pose_name)
        except Exception as exc:  # noqa: BLE001 - action boundary
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

        positions, _ = self._joint_positions()
        if positions is not None and max(
                abs(a - b) for a, b in zip(positions, target)) < 0.02:
            goal_handle.succeed()
            result.success = True
            result.already_there = True
            result.message = "already at %s" % pose_name
            return result

        if not self._fjt.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = "FJT action server unavailable"
            return result

        feedback("executing")
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(JOINTS)
        duration = float(self.get_parameter("named_pose_duration").value)
        sec = int(duration)
        point.time_from_start = Duration(
            sec=sec, nanosec=int((duration - sec) * 1e9))
        traj.points = [point]
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        event = threading.Event()
        future = self._fjt.send_goal_async(goal)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(10.0):
            goal_handle.abort()
            result.success = False
            result.message = "FJT goal send timeout"
            return result
        handle = future.result()
        if not handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = "FJT goal rejected"
            return result

        result_event = threading.Event()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _f: result_event.set())
        if not result_event.wait(duration + 60.0):
            handle.cancel_goal()
            goal_handle.abort()
            result.success = False
            result.message = "FJT execution timeout"
            return result
        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            goal_handle.abort()
            result.success = False
            result.message = "FJT status=%s error_code=%s" % (
                wrapped.status, wrapped.result.error_code)
            return result

        settled = self._wait_settled(
            lambda: feedback("settling"))
        if not settled:
            goal_handle.abort()
            result.success = False
            result.message = "reached %s but settle timeout" % pose_name
            return result

        goal_handle.succeed()
        result.success = True
        result.already_there = False
        result.message = "reached %s" % pose_name
        return result


def main(argv=None):
    rclpy.init(args=argv)
    node = MotionPlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
