#!/usr/bin/env python3
"""Publish static TF: world -> container_link -> container_opening_frame."""

import os
import sys

import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import TransformStamped

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from container_config_utils import (  # noqa: E402
    base_in_world,
    default_config_path,
    load_container_config,
    opening_in_container,
    origin_in_world,
)


def _broadcast(static_broadcaster, parent, child, xyz, rpy):
    import math

    cr, sr = math.cos(rpy[0] * 0.5), math.sin(rpy[0] * 0.5)
    cp, sp = math.cos(rpy[1] * 0.5), math.sin(rpy[1] * 0.5)
    cy, sy = math.cos(rpy[2] * 0.5), math.sin(rpy[2] * 0.5)
    msg = TransformStamped()
    msg.header.stamp = rospy.Time(0)
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = xyz[0]
    msg.transform.translation.y = xyz[1]
    msg.transform.translation.z = xyz[2]
    msg.transform.rotation.w = cr * cp * cy + sr * sp * sy
    msg.transform.rotation.x = sr * cp * cy - cr * sp * sy
    msg.transform.rotation.y = cr * sp * cy + sr * cp * sy
    msg.transform.rotation.z = cr * cp * sy - sr * sp * cy
    static_broadcaster.sendTransform(msg)


class ContainerTfPublisher:
    def __init__(self):
        config_path = rospy.get_param("~container_config", default_config_path())
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._container_frame = rospy.get_param("~container_frame", "container_link")
        self._opening_frame = rospy.get_param("~opening_frame", "container_opening_frame")
        self._republish_period = float(rospy.get_param("~republish_period", 30.0))

        config = load_container_config(config_path)
        self._base_xyz, self._base_rpy = base_in_world(config)
        self._container_xyz, self._container_rpy = origin_in_world(config)
        self._opening_xyz, self._opening_rpy = opening_in_container(config)
        self._config_path = config_path

        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_all()
        if self._republish_period > 0.0:
            rospy.Timer(rospy.Duration(self._republish_period), self._on_timer, oneshot=False)

    def _publish_all(self):
        _broadcast(
            self._broadcaster,
            self._world_frame,
            self._base_frame,
            self._base_xyz,
            self._base_rpy,
        )
        _broadcast(
            self._broadcaster,
            self._world_frame,
            self._container_frame,
            self._container_xyz,
            self._container_rpy,
        )
        _broadcast(
            self._broadcaster,
            self._container_frame,
            self._opening_frame,
            self._opening_xyz,
            self._opening_rpy,
        )
        rospy.loginfo(
            "Static TF: %s -> %s (base), %s -> %s -> %s (container, config=%s)",
            self._world_frame,
            self._base_frame,
            self._world_frame,
            self._container_frame,
            self._opening_frame,
            self._config_path,
        )

    def _on_timer(self, _event):
        self._publish_all()


def main():
    rospy.init_node("container_tf_publisher")
    ContainerTfPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
