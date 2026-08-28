#!/usr/bin/env python3
"""Print / save current arm joint angles (e.g. after MoveIt Execute in RViz)."""

import os
import sys

import rospy
from sensor_msgs.msg import JointState

JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]


def format_yaml(values):
    formatted = ", ".join("%.4f" % v for v in values)
    return "    values: [%s]" % formatted


def main():
    rospy.init_node("export_observe_joints")
    timeout = rospy.get_param("~timeout", 10.0)
    output_path = rospy.get_param("~output", os.path.expanduser("~/observe_pose_tuned.yaml"))

    try:
        msg = rospy.wait_for_message("/joint_states", JointState, timeout=timeout)
    except rospy.ROSException:
        rospy.logerr("No /joint_states within %.1fs", timeout)
        sys.exit(1)

    positions = dict(zip(msg.name, msg.position))
    missing = [name for name in JOINT_NAMES if name not in positions]
    if missing:
        rospy.logerr("Missing joints in joint_states: %s", ", ".join(missing))
        sys.exit(1)

    values = [positions[name] for name in JOINT_NAMES]
    yaml_line = format_yaml(values)

    print("# Current arm joints (rad)")
    print(yaml_line)
    for name, value in zip(JOINT_NAMES, values):
        print("#   %s: %.4f" % (name, value))

    content = (
        "# Tuned observe pose — paste values into robot_poses.yaml\n"
        "joints:\n"
        + "\n".join("  - %s" % n for n in JOINT_NAMES)
        + "\n"
        + yaml_line
        + "\n"
    )
    with open(output_path, "w") as handle:
        handle.write(content)
    rospy.loginfo("Saved to %s", output_path)


if __name__ == "__main__":
    main()
