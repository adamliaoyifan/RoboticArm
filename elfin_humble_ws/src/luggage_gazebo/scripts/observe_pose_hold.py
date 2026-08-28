#!/usr/bin/env python3
"""Hold the named observe pose after elfin_arm_controller is active.

World starts unpaused so gz_ros2_control can SwitchController. Gravity is
off on the moving links so JointPositionReset survives until this FJT
claims the position command (Noetic hold_observe_on_start).
"""

from __future__ import division

import sys
import time

import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]
_FALLBACK = [3.5702, -1.3263, -1.0965, 3.9564, 1.6234, 0.4522]


def _load_pose(path, name):
    with open(path, "r", encoding="utf-8") as handle:
        pose = yaml.safe_load(handle)["poses"][name]
    values = [float(v) for v in pose["values"]]
    if len(values) != 6:
        raise ValueError("pose '%s' must have 6 values" % name)
    return values


class ObservePoseHold(Node):
    def __init__(self):
        super().__init__("observe_pose_hold")
        self.declare_parameter("spawn_at_observe", True)
        self.declare_parameter("robot_poses_config", "")
        self.declare_parameter("observe_pose_name", "observe")
        self.declare_parameter("hold_duration", 0.5)
        self.declare_parameter(
            "action", "/elfin_arm_controller/follow_joint_trajectory")
        self._client = ActionClient(
            self, FollowJointTrajectory, self.get_parameter("action").value)
        self._joint_state = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, msg):
        self._joint_state = msg

    def _wait(self, seconds):
        deadline = time.time() + seconds
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        if not bool(self.get_parameter("spawn_at_observe").value):
            return 0

        poses_path = self.get_parameter("robot_poses_config").value
        pose_name = self.get_parameter("observe_pose_name").value
        try:
            target = _load_pose(poses_path, pose_name)
        except Exception as exc:
            self.get_logger().warn(
                "Could not load pose '%s' (%s); using fallback observe" % (
                    pose_name, exc))
            target = list(_FALLBACK)

        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("FollowJointTrajectory server not available")
            return 1
        deadline = time.time() + 10.0
        while self._joint_state is None and time.time() < deadline and rclpy.ok():
            self._wait(0.05)
        if self._joint_state is None:
            self.get_logger().error("no /joint_states yet; skip hold trajectory")
            return 1

        by_name = dict(zip(self._joint_state.name, self._joint_state.position))
        start = [float(by_name.get(name, target[i])) for i, name in enumerate(JOINTS)]
        duration = float(self.get_parameter("hold_duration").value)
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        p0 = JointTrajectoryPoint()
        p0.positions = start
        p0.velocities = [0.0] * 6
        p0.time_from_start = Duration(sec=0, nanosec=0)
        p1 = JointTrajectoryPoint()
        p1.positions = [float(v) for v in target]
        p1.velocities = [0.0] * 6
        sec = int(duration)
        nsec = int(round((duration - sec) * 1e9))
        p1.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [p0, p1]
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        send_future = self._client.send_goal_async(goal)
        wait_until = time.time() + 10.0
        while not send_future.done() and time.time() < wait_until and rclpy.ok():
            self._wait(0.05)
        handle = send_future.result() if send_future.done() else None
        if handle is None or not handle.accepted:
            self.get_logger().error("observe hold goal rejected")
            return 1
        result_future = handle.get_result_async()
        wait_until = time.time() + duration + 15.0
        while not result_future.done() and time.time() < wait_until and rclpy.ok():
            self._wait(0.05)
        if not result_future.done():
            self.get_logger().error("observe hold timed out")
            return 1
        wrapped = result_future.result()
        err = wrapped.result.error_code
        self.get_logger().info(
            "Held pose '%s' error_code=%s" % (pose_name, err))
        return 0 if err == 0 else 1


def main():
    rclpy.init()
    node = ObservePoseHold()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 0
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
