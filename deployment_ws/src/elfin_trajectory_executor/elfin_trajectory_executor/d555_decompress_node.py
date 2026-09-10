"""Replay helper: JPEG/PNG D555 topics → Image on the canonical names."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from .rgbd_codec import jpeg_to_color, png_to_depth

_SENSOR = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class D555DecompressNode(Node):
    def __init__(self) -> None:
        super().__init__("d555_decompress")
        self.declare_parameter("camera_namespace", "camera")
        self.declare_parameter("camera_name", "d555")
        ns = str(self.get_parameter("camera_namespace").value).strip("/")
        name = str(self.get_parameter("camera_name").value).strip("/")
        base = f"/{ns}/{name}"
        self._color_pub = self.create_publisher(Image, f"{base}/color/image_raw", _SENSOR)
        self._depth_pub = self.create_publisher(
            Image, f"{base}/aligned_depth_to_color/image_raw", _SENSOR
        )
        self.create_subscription(
            CompressedImage,
            f"{base}/color/image_raw/compressed",
            self._on_color,
            _SENSOR,
        )
        self.create_subscription(
            CompressedImage,
            f"{base}/aligned_depth_to_color/image_raw/compressed",
            self._on_depth,
            _SENSOR,
        )
        self.get_logger().info("[d555_decompress] jpeg/png → image_raw at %s" % base)

    def _on_color(self, msg: CompressedImage) -> None:
        try:
            self._color_pub.publish(jpeg_to_color(msg))
        except Exception as exc:
            self.get_logger().error("color decode: %s" % exc)

    def _on_depth(self, msg: CompressedImage) -> None:
        try:
            self._depth_pub.publish(png_to_depth(msg))
        except Exception as exc:
            self.get_logger().error("depth decode: %s" % exc)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = D555DecompressNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
