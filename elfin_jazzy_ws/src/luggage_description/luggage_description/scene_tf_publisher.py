#!/usr/bin/env python3
"""Publish static TF from scene_tf.yaml (ROS 2).

Quaternion math is independent of rclpy so unit tests can check it against
the ROS 1 ``container_tf_publisher._make_transform`` formula.
"""

from __future__ import division

import math

from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    resolve_scene_tf_config_path,
    static_transforms,
    task_roi_from_scene,
)


def rpy_to_quaternion(rpy):
    """Return (x, y, z, w) for RPY, matching the ROS 1 static TF publisher."""
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def transform_payload(parent, child, xyz, rpy):
    qx, qy, qz, qw = rpy_to_quaternion(rpy)
    return {
        "parent": str(parent),
        "child": str(child),
        "translation": [float(v) for v in xyz],
        "rotation_xyzw": [qx, qy, qz, qw],
    }


def static_transform_payloads(config):
    return [
        transform_payload(
            item["parent"],
            item["child"],
            item["translation"],
            item["rotation_rpy"],
        )
        for item in static_transforms(config)
    ]


def to_transform_stamped(payload, stamp=None):
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped

    msg = TransformStamped()
    msg.header.stamp = stamp if stamp is not None else Time(sec=0, nanosec=0)
    msg.header.frame_id = payload["parent"]
    msg.child_frame_id = payload["child"]
    msg.transform.translation.x = payload["translation"][0]
    msg.transform.translation.y = payload["translation"][1]
    msg.transform.translation.z = payload["translation"][2]
    msg.transform.rotation.x = payload["rotation_xyzw"][0]
    msg.transform.rotation.y = payload["rotation_xyzw"][1]
    msg.transform.rotation.z = payload["rotation_xyzw"][2]
    msg.transform.rotation.w = payload["rotation_xyzw"][3]
    return msg


class ContainerTfPublisher:
    """rclpy node body. Instantiated after ``rclpy.create_node`` or as Node."""

    def __init__(self, node):
        self._node = node
        node.declare_parameter("scene_tf_config", "")
        node.declare_parameter("republish_period", 0.0)
        raw_path = node.get_parameter("scene_tf_config").value
        self._config_path = resolve_scene_tf_config_path(raw_path or None)
        self._republish_period = float(node.get_parameter("republish_period").value)
        self._config = load_scene_tf_config(self._config_path)
        self._payloads = static_transform_payloads(self._config)
        self._roi = task_roi_from_scene(self._config)
        self._declare_roi_params(self._roi)

        from tf2_ros import StaticTransformBroadcaster

        self._broadcaster = StaticTransformBroadcaster(node)
        self._publish_all()
        if self._republish_period > 0.0:
            self._timer = node.create_timer(self._republish_period, self._publish_all)

    def _declare_roi_params(self, roi):
        for key, value in roi.items():
            name = "task_roi.%s" % key
            self._node.declare_parameter(name, value)

    def _publish_all(self):
        stamped = [to_transform_stamped(item) for item in self._payloads]
        self._broadcaster.sendTransform(stamped)
        self._node.get_logger().info(
            "Static TF from %s (%d transforms): %s"
            % (
                self._config_path,
                len(self._payloads),
                ", ".join(
                    "%s->%s" % (item["parent"], item["child"])
                    for item in self._payloads
                ),
            )
        )
        self._node.get_logger().info(
            "task ROI (this node only): center=%s dims=%s yaw=%.4f "
            "opening=%s aperture=%.3fx%.3f"
            % (
                [round(v, 4) for v in self._roi["container_center"]],
                [round(v, 4) for v in self._roi["container_dims"]],
                self._roi["container_yaw"],
                [round(v, 4) for v in self._roi["opening_center"]],
                self._roi["aperture_width"],
                self._roi["aperture_height"],
            )
        )


def main(args=None):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("container_tf_publisher")
    ContainerTfPublisher(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
