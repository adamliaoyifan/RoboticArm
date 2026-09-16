#!/usr/bin/env python3
"""Launch-owned CameraInfo alias for the single Gazebo pinhole.

Gazebo rgbd_camera publishes one CameraInfo. This node republishes it onto the
colour CameraInfo topic so both canonical products exist on the ROS graph.
"""

from __future__ import division

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class CameraInfoAlias(Node):
    def __init__(self):
        super().__init__("camera_info_alias")
        self.declare_parameter("input_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_topic", "/camera/color/camera_info")
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(
            CameraInfo, self.get_parameter("output_topic").value, qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("input_topic").value,
            self._on_info, qos)

    def _on_info(self, msg):
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = CameraInfoAlias()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
