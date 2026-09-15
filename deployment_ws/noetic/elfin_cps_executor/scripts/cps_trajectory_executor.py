#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
import threading

import actionlib
import rospkg
import rospy
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def _import_huayan_backend():
    scripts_dir = os.path.join(rospkg.RosPack().get_path("elfin_cps_executor"), "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    from huayan_backend import HuayanBackend  # noqa: PLC0415

    return HuayanBackend


HuayanBackend = _import_huayan_backend()


DEFAULT_JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]


class CpsTrajectoryExecutor:
    def __init__(self):
        self._joint_names = rospy.get_param("~joint_names", DEFAULT_JOINT_NAMES)
        self._positions = [0.0] * len(self._joint_names)
        self._lock = threading.Lock()

        self._backend = HuayanBackend(
            logger=self._log,
            robot_ip=rospy.get_param("~robot_ip", "192.168.0.10"),
            robot_port=rospy.get_param("~robot_port", 10003),
            default_velocity_deg=rospy.get_param("~default_velocity_deg", 30.0),
            max_velocity_deg=rospy.get_param("~max_velocity_deg", 60.0),
            poll_interval_s=rospy.get_param("~poll_interval_s", 0.05),
            blend_radius_mm=rospy.get_param("~blend_radius_mm", 5.0),
            final_blend_radius_mm=rospy.get_param("~final_blend_radius_mm", 0.0),
            controller_start_timeout_s=rospy.get_param("~controller_start_timeout_s", 30.0),
            power_off_on_disconnect=rospy.get_param("~power_off_on_disconnect", False),
        )

        self._joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self._joint_timer = rospy.Timer(rospy.Duration(0.01), self._publish_joint_states)

        self._server = actionlib.SimpleActionServer(
            "follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self._server.start()

        rospy.on_shutdown(self._backend.disconnect)
        rospy.loginfo("Python CPS trajectory executor ready")

    def connect(self):
        return self._backend.connect()

    def _execute(self, goal):
        trajectory = goal.trajectory
        error = self._normalize_trajectory(trajectory)
        result = FollowJointTrajectoryResult()
        if error:
            result.error_code = FollowJointTrajectoryResult.INVALID_JOINTS
            result.error_string = error
            self._server.set_aborted(result, error)
            return

        ok, message = self._backend.execute_trajectory(
            trajectory,
            should_cancel=lambda: self._server.is_preempt_requested() or rospy.is_shutdown(),
            feedback_cb=self._publish_feedback,
        )

        if self._server.is_preempt_requested():
            result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            result.error_string = "Trajectory preempted"
            self._server.set_preempted(result, result.error_string)
        elif ok:
            result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
            self._server.set_succeeded(result, "Trajectory execution complete")
        else:
            result.error_code = FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
            result.error_string = message
            self._server.set_aborted(result, message)

    def _normalize_trajectory(self, trajectory):
        if not trajectory.points:
            return None
        if sorted(trajectory.joint_names) != sorted(self._joint_names):
            return "Expected joints %s, got %s" % (self._joint_names, trajectory.joint_names)

        order = [trajectory.joint_names.index(name) for name in self._joint_names]
        if order == list(range(len(self._joint_names))):
            return None

        for point in trajectory.points:
            point.positions = [point.positions[i] for i in order]
            if point.velocities:
                point.velocities = [point.velocities[i] for i in order]
            if point.accelerations:
                point.accelerations = [point.accelerations[i] for i in order]
        trajectory.joint_names = list(self._joint_names)
        return None

    def _publish_feedback(self, positions):
        with self._lock:
            self._positions = list(positions)

        feedback = FollowJointTrajectoryFeedback()
        feedback.header.stamp = rospy.Time.now()
        feedback.joint_names = list(self._joint_names)
        feedback.actual.positions = list(positions)
        self._server.publish_feedback(feedback)

    def _publish_joint_states(self, _event):
        self._backend.refresh_positions()
        with self._lock:
            positions = self._backend.current_positions or self._positions
            self._positions = list(positions)

        msg = JointState()
        msg.header = Header(stamp=rospy.Time.now())
        msg.name = list(self._joint_names)
        msg.position = list(self._positions)
        self._joint_pub.publish(msg)

    @staticmethod
    def _log(level, message):
        if level == "error":
            rospy.logerr(message)
        elif level == "warn":
            rospy.logwarn(message)
        else:
            rospy.loginfo(message)


def main():
    rospy.init_node("cps_trajectory_executor")
    executor = CpsTrajectoryExecutor()
    if rospy.get_param("~connect_on_start", True):
        executor.connect()
    rospy.spin()


if __name__ == "__main__":
    main()
