#!/usr/bin/env python3
"""Delete known RobotArm Gazebo models before a fresh spawn."""

import rospy
from gazebo_msgs.srv import DeleteModel, GetWorldProperties


DEFAULT_MODELS = [
    "S20",
    "airport_container",
    "suitcase_standard",
    "suitcase_large",
    "suitcase_carryon",
]
DEFAULT_PREFIXES = ["pickup_box_"]


def main():
    rospy.init_node("clear_gazebo_models")
    models = rospy.get_param("~models", DEFAULT_MODELS)
    prefixes = rospy.get_param("~prefixes", DEFAULT_PREFIXES)
    timeout = float(rospy.get_param("~timeout", 30.0))

    rospy.loginfo("Waiting for Gazebo model services before cleanup ...")
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=timeout)
    rospy.wait_for_service("/gazebo/delete_model", timeout=timeout)

    get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    existing = set(get_world().model_names)

    deleted = []
    targets = set(models)
    for name in existing:
        if any(name.startswith(prefix) for prefix in prefixes):
            targets.add(name)

    for name in sorted(targets):
        if name not in existing:
            continue
        resp = delete_model(name)
        if resp.success:
            deleted.append(name)
            rospy.loginfo("Deleted existing Gazebo model '%s'", name)
        else:
            rospy.logwarn("Could not delete Gazebo model '%s': %s", name, resp.status_message)

    if deleted:
        rospy.sleep(0.5)
        rospy.loginfo("Gazebo cleanup removed: %s", ", ".join(deleted))
    else:
        rospy.loginfo("Gazebo cleanup found no stale RobotArm models")


if __name__ == "__main__":
    main()
