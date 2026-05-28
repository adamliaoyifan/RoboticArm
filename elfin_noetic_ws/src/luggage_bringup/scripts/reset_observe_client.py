#!/usr/bin/env python3
"""One-shot client: wait for motion_planner and call go_to_robot_pose(observe)."""

import sys

import rospy
from luggage_msgs.srv import GoToRobotPose


def main():
    rospy.init_node("reset_observe_client")
    pose_name = rospy.get_param("~pose_name", "observe")
    service_timeout = rospy.get_param("~service_timeout", 120.0)
    move_group_delay = rospy.get_param("~move_group_delay", 10.0)

    service_name = "/motion_planner/go_to_robot_pose"
    rospy.loginfo("Waiting for %s ...", service_name)
    try:
        rospy.wait_for_service(service_name, timeout=service_timeout)
    except rospy.ROSException:
        rospy.logerr("Timed out waiting for %s", service_name)
        sys.exit(1)

    if move_group_delay > 0:
        rospy.loginfo("Waiting %.1fs for move_group ...", move_group_delay)
        rospy.sleep(move_group_delay)

    go_to = rospy.ServiceProxy(service_name, GoToRobotPose)
    try:
        resp = go_to(pose_name)
    except rospy.ServiceException as exc:
        rospy.logerr("go_to_robot_pose call failed: %s", exc)
        sys.exit(1)

    if resp.success:
        rospy.loginfo(
            "Observe reset OK: %s (already_there=%s)", resp.message, resp.already_there
        )
    else:
        rospy.logerr("Observe reset failed: %s", resp.message)
        sys.exit(1)


if __name__ == "__main__":
    main()
