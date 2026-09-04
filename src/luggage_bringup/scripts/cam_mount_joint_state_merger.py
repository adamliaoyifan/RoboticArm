#!/usr/bin/env python3
"""Merge arm /joint_states with suction_flange + adapter_mount + cam_mount tune joints for robot_state_publisher."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import rospy
import rospkg
import yaml
from mount_config_utils import mount_dict_to_tune_joints
from sensor_msgs.msg import JointState

MOUNT_JOINT_NAMES = [
    "cam_mount_tx",
    "cam_mount_ty",
    "cam_mount_tz",
    "cam_mount_rx",
    "cam_mount_ry",
    "cam_mount_rz",
]

ADAPTER_JOINT_NAMES = [
    "adapter_mount_tx",
    "adapter_mount_ty",
    "adapter_mount_tz",
    "adapter_mount_rx",
    "adapter_mount_ry",
    "adapter_mount_rz",
]

FLANGE_JOINT_NAMES = [
    "suction_flange_tx",
    "suction_flange_ty",
    "suction_flange_tz",
    "suction_flange_rx",
    "suction_flange_ry",
    "suction_flange_rz",
]

ALL_TUNE_JOINT_NAMES = MOUNT_JOINT_NAMES + ADAPTER_JOINT_NAMES + FLANGE_JOINT_NAMES

DEFAULT_MOUNT_JOINTS = [
    -0.017202,
    0.129806,
    0.101650,
    -1.57079632679,
    -1.57079632679,
    1.57079632679,
]

DEFAULT_ADAPTER_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

DEFAULT_FLANGE_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _load_default_mount_positions():
    config_dir = os.path.join(
        rospkg.RosPack().get_path("luggage_description"), "config"
    )
    positions = {}

    cam_path = os.path.join(config_dir, "realsense_d435_mount.yaml.example")
    try:
        with open(cam_path, "r") as handle:
            mount = yaml.safe_load(handle).get("mount", {})
        positions.update(dict(zip(MOUNT_JOINT_NAMES, mount_dict_to_tune_joints(mount))))
    except (IOError, OSError, TypeError, ValueError):
        positions.update(dict(zip(MOUNT_JOINT_NAMES, DEFAULT_MOUNT_JOINTS)))

    adp_path = os.path.join(config_dir, "eef_mount_adapter.yaml.example")
    try:
        with open(adp_path, "r") as handle:
            data = yaml.safe_load(handle) or {}
        mount = data.get("mount", {})
        fixed = mount.get("fixed", {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})
        xyz = [float(v) for v in fixed.get("xyz", [0, 0, 0])]
        rpy = [float(v) for v in fixed.get("rpy", [0, 0, 0])]
        positions.update(dict(zip(ADAPTER_JOINT_NAMES, xyz + rpy)))
    except (IOError, OSError, TypeError, ValueError):
        positions.update(dict(zip(ADAPTER_JOINT_NAMES, DEFAULT_ADAPTER_JOINTS)))

    flange_path = os.path.join(config_dir, "suction_flange.yaml.example")
    try:
        with open(flange_path, "r") as handle:
            data = yaml.safe_load(handle) or {}
        mount = data.get("mount", {})
        fixed = mount.get("fixed", {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})
        xyz = [float(v) for v in fixed.get("xyz", [0, 0, 0])]
        rpy = [float(v) for v in fixed.get("rpy", [0, 0, 0])]
        positions.update(dict(zip(FLANGE_JOINT_NAMES, xyz + rpy)))
    except (IOError, OSError, TypeError, ValueError):
        positions.update(dict(zip(FLANGE_JOINT_NAMES, DEFAULT_FLANGE_JOINTS)))

    return positions


class CamMountJointStateMerger:
    def __init__(self):
        self._arm_msg = None
        self._mount_positions = _load_default_mount_positions()

        arm_topic = rospy.get_param("~arm_joint_states_topic", "/joint_states")
        mount_topic = rospy.get_param("~mount_joint_states_topic", "/cam_mount_tune/joint_states")
        adapter_topic = rospy.get_param("~adapter_joint_states_topic", "/adapter_mount_tune/joint_states")
        flange_topic = rospy.get_param("~flange_joint_states_topic", "/suction_flange_tune/joint_states")
        out_topic = rospy.get_param("~output_joint_states_topic", "/joint_states_merged")

        rospy.Subscriber(arm_topic, JointState, self._on_arm, queue_size=1)
        rospy.Subscriber(mount_topic, JointState, self._on_mount, queue_size=1)
        rospy.Subscriber(adapter_topic, JointState, self._on_mount, queue_size=1)
        rospy.Subscriber(flange_topic, JointState, self._on_mount, queue_size=1)
        self._pub = rospy.Publisher(out_topic, JointState, queue_size=10, latch=True)
        self._publish()

    def _on_mount(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._mount_positions:
                self._mount_positions[name] = pos
        self._publish()

    def _on_arm(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self._mount_positions:
                self._mount_positions[name] = pos
        self._arm_msg = msg
        self._publish()

    def _publish(self):
        if self._arm_msg is None:
            return
        out = JointState()
        out.header = self._arm_msg.header
        out.name = list(self._arm_msg.name)
        out.position = list(self._arm_msg.position)
        out.velocity = list(self._arm_msg.velocity) if self._arm_msg.velocity else []
        out.effort = list(self._arm_msg.effort) if self._arm_msg.effort else []

        for name in ALL_TUNE_JOINT_NAMES:
            if name not in out.name:
                out.name.append(name)
                out.position.append(self._mount_positions[name])
                if out.velocity:
                    out.velocity.append(0.0)
                if out.effort:
                    out.effort.append(0.0)

        self._pub.publish(out)


def main():
    rospy.init_node("cam_mount_joint_state_merger")
    CamMountJointStateMerger()
    rospy.spin()


if __name__ == "__main__":
    main()
