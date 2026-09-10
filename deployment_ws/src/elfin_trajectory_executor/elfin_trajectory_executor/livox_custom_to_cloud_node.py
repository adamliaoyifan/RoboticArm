"""Replay helper: Livox CustomMsg → PointCloud2 XYZRTLT."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .livox_codec import custom_points_to_cloud

_SENSOR = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class LivoxCustomToCloudNode(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_cloud")
        self.declare_parameter("input_topic", "/livox/lidar_custom")
        self.declare_parameter("output_topic", "/livox/lidar")
        try:
            from livox_ros_driver2.msg import CustomMsg
        except ImportError as exc:
            raise RuntimeError(
                "livox_ros_driver2 is not on PYTHONPATH. "
                "source deployment_ws/livox_ws/env.sh"
            ) from exc
        inn = str(self.get_parameter("input_topic").value)
        out = str(self.get_parameter("output_topic").value)
        self._pub = self.create_publisher(PointCloud2, out, _SENSOR)
        self.create_subscription(CustomMsg, inn, self._on_msg, _SENSOR)
        self.get_logger().info("[livox_custom_to_cloud] %s → PointCloud2 %s" % (inn, out))

    def _on_msg(self, msg) -> None:
        cloud = custom_points_to_cloud(msg.header, msg.points, msg.timebase)
        self._pub.publish(cloud)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = LivoxCustomToCloudNode()
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
