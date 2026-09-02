#!/usr/bin/env python3
"""Convert planning-side trajectory topics into FollowJointTrajectory goals.

This node is the only coupling point between an independent planner and the
Huayan SDK executor. New messages preempt the in-flight goal.
"""
from __future__ import annotations

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectory

try:
    from moveit_msgs.msg import RobotTrajectory
except ImportError:  # pragma: no cover - optional at runtime
    RobotTrajectory = None


_STATUS_NAMES = {
    GoalStatus.PENDING: "pending",
    GoalStatus.ACTIVE: "executing",
    GoalStatus.PREEMPTED: "preempted",
    GoalStatus.SUCCEEDED: "succeeded",
    GoalStatus.ABORTED: "aborted",
    GoalStatus.REJECTED: "rejected",
    GoalStatus.PREEMPTING: "preempting",
    GoalStatus.RECALLING: "recalling",
    GoalStatus.RECALLED: "recalled",
    GoalStatus.LOST: "lost",
}


class TrajectoryCommandBridge:
    def __init__(self):
        action_ns = rospy.get_param(
            "~action_ns", "/elfin_arm_controller/follow_joint_trajectory"
        )
        wait_timeout_s = float(rospy.get_param("~wait_timeout_s", 30.0))

        self._status_pub = rospy.Publisher("execute_status", String, queue_size=1, latch=True)
        self._publish_status("waiting_for_executor")

        self._client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for executor action %s", action_ns)
        if not self._client.wait_for_server(rospy.Duration(wait_timeout_s)):
            rospy.logerr("Executor action %s not available", action_ns)
            self._publish_status("executor_unavailable")
            raise rospy.ROSException("follow_joint_trajectory server not available")

        self._publish_status("idle")
        rospy.loginfo("Trajectory command bridge ready")

        rospy.Subscriber("execute_trajectory", JointTrajectory, self._on_joint_traj, queue_size=1)
        rospy.Subscriber("cancel_execution", Empty, self._on_cancel, queue_size=1)
        if RobotTrajectory is not None:
            rospy.Subscriber(
                "execute_robot_trajectory",
                RobotTrajectory,
                self._on_robot_traj,
                queue_size=1,
            )
        else:
            rospy.logwarn("moveit_msgs not found; /execute_robot_trajectory is disabled")

    def _on_joint_traj(self, trajectory):
        self._send(trajectory)

    def _on_robot_traj(self, robot_trajectory):
        self._send(robot_trajectory.joint_trajectory)

    def _on_cancel(self, _msg):
        rospy.logwarn("Cancel requested")
        self._client.cancel_all_goals()
        self._publish_status("canceled")

    def _send(self, trajectory):
        if not trajectory.points:
            rospy.logwarn("Ignoring empty trajectory")
            return

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        rospy.loginfo(
            "Sending trajectory (%d points, joints=%s)",
            len(trajectory.points),
            list(trajectory.joint_names),
        )
        self._publish_status("executing")
        self._client.send_goal(goal, done_cb=self._done)

    def _done(self, status, result):
        name = _STATUS_NAMES.get(status, "status_%s" % status)
        detail = result.error_string if result is not None else ""
        if detail:
            rospy.loginfo("Execution %s: %s", name, detail)
        else:
            rospy.loginfo("Execution %s", name)
        self._publish_status(name)

    def _publish_status(self, status):
        self._status_pub.publish(String(data=status))


def main():
    rospy.init_node("trajectory_command_bridge")
    TrajectoryCommandBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
