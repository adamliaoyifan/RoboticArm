#!/usr/bin/env python3
"""Convert Fortress rgbd_camera depth (32FC1 metres) to 16UC1 millimetres.

Launch-owned single-stream adapter onto /camera/depth/image_raw. Truncated or
unsupported payloads are dropped (fail closed).
"""

from __future__ import division

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from luggage_gazebo.depth_image_adapter import convert_depth_metres_to_mm


class DepthImageRepublisher(Node):
    def __init__(self):
        super().__init__("depth_image_republisher")
        self.declare_parameter("input_topic", "/camera/depth/image_meters")
        self.declare_parameter("output_topic", "/camera/depth/image_raw")
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("near_depth_m", 0.26)

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._near_m = float(self.get_parameter("near_depth_m").value)
        self._max_m = float(self.get_parameter("max_depth_m").value)
        self._drop_counts = {
            "unsupported_encoding": 0,
            "malformed_header": 0,
            "malformed_step": 0,
            "truncated_payload": 0,
            "invalid_range": 0,
        }
        self._pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value, qos)
        self.create_subscription(
            Image, self.get_parameter("input_topic").value, self._on_depth, qos)

    def _on_depth(self, msg):
        mm_bytes, reason = convert_depth_metres_to_mm(
            msg.encoding, msg.width, msg.height, msg.step, msg.data,
            near_m=self._near_m, max_m=self._max_m)
        if reason is not None:
            self._drop_counts[reason] = self._drop_counts.get(reason, 0) + 1
            return
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = msg.width * 2
        out.data = mm_bytes
        self._pub.publish(out)


def main():
    rclpy.init()
    node = DepthImageRepublisher()
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
