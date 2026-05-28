#!/usr/bin/env python3
"""Phase 0 stub — logs configured YAML paths."""

import rospy


def main():
    rospy.init_node("description_params")
    rospy.loginfo(
        "luggage_description stub — vacuum: %s",
        rospy.get_param("~luggage/vacuum_config", rospy.get_param("luggage/vacuum_config", "")),
    )
    rospy.logwarn("TODO: xacro load vacuum_gripper.yaml and attach to elfin_end_link")
    rospy.spin()


if __name__ == "__main__":
    main()
