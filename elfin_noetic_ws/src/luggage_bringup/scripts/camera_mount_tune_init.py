#!/usr/bin/env python3
"""Set arm to observe pose (+ mount joints) after Gazebo model and controllers are ready."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import actionlib
import rospy
import rospkg
import yaml
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelConfiguration
from mount_config_utils import mount_dict_to_tune_joints
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINT_NAMES = [
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
]

MOUNT_JOINT_NAMES = [
    "cam_mount_tx",
    "cam_mount_ty",
    "cam_mount_tz",
    "cam_mount_rx",
    "cam_mount_ry",
    "cam_mount_rz",
]

ARM_CONTROLLER_ACTION = "/elfin_arm_controller/follow_joint_trajectory"

DEFAULT_MOUNT_JOINTS = [
    -0.017202,
    0.129806,
    0.101650,
    -1.57079632679,
    -1.57079632679,
    1.57079632679,
]


def _pkg_config():
    return os.path.join(rospkg.RosPack().get_path("luggage_description"), "config")


def load_observe_arm_pose():
    path = rospy.get_param(
        "~robot_poses_config",
        os.path.join(_pkg_config(), "robot_poses.yaml.example"),
    )
    pose_name = rospy.get_param("~observe_pose_name", "observe")
    with open(path, "r") as handle:
        config = yaml.safe_load(handle)
    pose = config["poses"][pose_name]
    joints = list(pose["joints"])
    values = [float(v) for v in pose["values"]]
    if len(joints) != len(values):
        raise ValueError("observe pose joints/values length mismatch")
    return joints, values


def load_mount_pose():
    path = rospy.get_param(
        "~mount_config",
        os.path.join(_pkg_config(), "realsense_d435_mount.yaml.example"),
    )
    with open(path, "r") as handle:
        mount = yaml.safe_load(handle).get("mount", {})
    try:
        return mount_dict_to_tune_joints(mount)
    except (KeyError, TypeError, ValueError):
        return list(DEFAULT_MOUNT_JOINTS)


def wait_for_gazebo_model(model_name, timeout_sec):
    deadline = rospy.Time.now() + rospy.Duration(timeout_sec)
    rate = rospy.Rate(5)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        try:
            msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=2.0)
            if model_name in msg.name:
                rospy.loginfo("Gazebo model %s is spawned", model_name)
                return True
        except rospy.ROSException:
            pass
        rate.sleep()
    return False


def wait_for_arm_controller(timeout_sec):
    client = actionlib.SimpleActionClient(ARM_CONTROLLER_ACTION, FollowJointTrajectoryAction)
    if client.wait_for_server(rospy.Duration(timeout_sec)):
        rospy.loginfo("Arm controller action server ready")
        return client
    return None


def move_arm_via_controller(client, joint_names, joint_positions, duration_sec=2.0):
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = list(joint_names)
    point = JointTrajectoryPoint()
    point.positions = list(joint_positions)
    point.time_from_start = rospy.Duration(duration_sec)
    goal.trajectory.points = [point]
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(duration_sec + 10.0))
    if not finished:
        client.cancel_goal()
        return False, "arm trajectory timed out"
    state = client.get_state()
    if state != actionlib.GoalStatus.SUCCEEDED:
        return False, "arm trajectory state=%s" % state
    return True, "ok"


def set_gazebo_joints(set_model_config, model_name, joint_names, joint_positions):
    resp = set_model_config(
        model_name=model_name,
        urdf_param_name="robot_description",
        joint_names=joint_names,
        joint_positions=joint_positions,
    )
    return resp.success, resp.status_message


def publish_mount_joint_states(positions):
    pub = rospy.Publisher("/cam_mount_tune/joint_states", JointState, queue_size=1, latch=True)
    rospy.sleep(0.2)
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = list(MOUNT_JOINT_NAMES)
    msg.position = list(positions)
    pub.publish(msg)


def apply_observe_pose(
    model_name="S20",
    apply_mount=True,
    gazebo_timeout=120.0,
    controller_timeout=120.0,
):
    """Wait for sim, move arm via ros_control, set mount joints in Gazebo."""
    if not wait_for_gazebo_model(model_name, gazebo_timeout):
        return False, "Gazebo model %s not found" % model_name

    arm_joints, arm_values = load_observe_arm_pose()
    mount_values = load_mount_pose() if apply_mount else None

    client = wait_for_arm_controller(controller_timeout)
    if client is None:
        return False, "elfin_arm_controller action server not available"

    ok, msg = move_arm_via_controller(client, arm_joints, arm_values)
    if not ok:
        return False, "arm move failed: %s" % msg

    if not apply_mount:
        return True, "arm at observe pose"

    srv_name = rospy.get_param(
        "~gazebo_set_model_configuration_service", "/gazebo/set_model_configuration"
    )
    try:
        rospy.wait_for_service(srv_name, timeout=10.0)
        set_model_config = rospy.ServiceProxy(srv_name, SetModelConfiguration)
    except rospy.ROSException:
        return False, "SetModelConfiguration unavailable for mount joints"

    ok, status = set_gazebo_joints(
        set_model_config, model_name, MOUNT_JOINT_NAMES, mount_values
    )
    if not ok:
        return False, "mount SetModelConfiguration rejected: %s" % status

    publish_mount_joint_states(mount_values)
    return True, "arm at observe pose with mount joints applied"


def main():
    try:
        rospy.init_node("camera_mount_tune_init")
        if not rospy.get_param("~set_observe_on_start", True):
            rospy.loginfo("set_observe_on_start=false — skipping observe pose")
            return

        model_name = rospy.get_param("~gazebo_model_name", "S20")
        apply_mount = bool(rospy.get_param("~apply_mount_on_start", True))
        gazebo_timeout = float(rospy.get_param("~gazebo_wait_timeout", 120.0))
        controller_timeout = float(rospy.get_param("~controller_wait_timeout", 120.0))

        rospy.loginfo(
            "Waiting for Gazebo model + arm controller before moving to observe ..."
        )
        ok, message = apply_observe_pose(
            model_name=model_name,
            apply_mount=apply_mount,
            gazebo_timeout=gazebo_timeout,
            controller_timeout=controller_timeout,
        )
        if ok:
            rospy.loginfo("%s", message)
        else:
            rospy.logerr("%s", message)
            sys.exit(1)
    except Exception as exc:
        try:
            rospy.logerr("camera_mount_tune_init failed: %s", exc)
        except Exception:
            print("camera_mount_tune_init failed:", exc, file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
