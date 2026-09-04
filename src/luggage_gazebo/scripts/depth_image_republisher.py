#!/usr/bin/env python3
"""Convert Fortress rgbd_camera depth (32FC1 metres, inf misses) to D435-like 16UC1 mm.

RViz Image treats float metres as ~0–1 and breaks on inf, so the raw gz depth
looks solid black even when the camera is publishing. Real D435 depth is
16UC1 millimetres; this node is the sim-side adapter onto /camera/depth/image_raw.
"""

from __future__ import division

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class DepthImageRepublisher(Node):
    def __init__(self):
        super().__init__("depth_image_republisher")
        self.declare_parameter("input_topic", "/camera/depth/image_meters")
        self.declare_parameter("output_topic", "/camera/depth/image_raw")
        self.declare_parameter("max_depth_m", 3.0)

        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._max_mm = int(round(float(self.get_parameter("max_depth_m").value) * 1000.0))
        self._pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value, qos)
        self.create_subscription(
            Image, self.get_parameter("input_topic").value, self._on_depth, qos)

    def _on_depth(self, msg):
        if msg.encoding not in ("32FC1", "TYPE_32FC1"):
            self._pub.publish(msg)
            return
        meters = np.frombuffer(msg.data, dtype=np.float32)
        if meters.size != msg.width * msg.height:
            meters = meters[: msg.width * msg.height]
        meters = meters.reshape(msg.height, msg.width)
        finite = np.isfinite(meters) & (meters > 0.0)
        mm = np.zeros(meters.shape, dtype=np.uint16)
        scaled = np.clip(np.round(meters * 1000.0), 0, 65535)
        mm[finite] = scaled[finite].astype(np.uint16)
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = "16UC1"
        out.is_bigendian = 0
        out.step = msg.width * 2
        out.data = mm.tobytes()
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
