#!/usr/bin/env python3
"""Publish static TF transforms from scene_tf.yaml."""

import os
import sys

import rospy
import rospkg
import tf2_ros
from geometry_msgs.msg import TransformStamped

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from scene_tf_config_utils import (  # noqa: E402
    default_scene_tf_config_path,
    load_scene_tf_config,
    static_transforms,
)


def _make_transform(parent, child, xyz, rpy):
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
    return msg


def _resolve_config_path():
    private = rospy.get_param("~scene_tf_config", "")
    if private:
        return private
    return rospy.get_param("/luggage/scene_tf_config", default_scene_tf_config_path())


class StaticTfPublisher:
    def __init__(self):
        self._config_path = _resolve_config_path()
        self._republish_period = float(rospy.get_param("~republish_period", 30.0))
        self._config = load_scene_tf_config(self._config_path)
        self._transforms = static_transforms(self._config)
        self._broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_all()
        if self._republish_period > 0.0:
            rospy.Timer(rospy.Duration(self._republish_period), self._on_timer, oneshot=False)

    def _publish_all(self):
        stamped = [
            _make_transform(
                item["parent"],
                item["child"],
                item["translation"],
                item["rotation_rpy"],
            )
            for item in self._transforms
        ]
        self._broadcaster.sendTransform(stamped)
        rospy.loginfo(
            "Static TF from %s (%d transforms): %s",
            self._config_path,
            len(self._transforms),
            ", ".join("%s->%s" % (t["parent"], t["child"]) for t in self._transforms),
        )

    def _on_timer(self, _event):
        self._publish_all()


def main():
    rospy.init_node("container_tf_publisher")
    StaticTfPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
