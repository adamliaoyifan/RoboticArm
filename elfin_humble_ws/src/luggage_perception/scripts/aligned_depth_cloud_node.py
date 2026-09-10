#!/usr/bin/env python3
"""Unproject colour-aligned depth to PointCloud2 in the depth optical frame.

Used on the real D555 so the preprocessor can keep its /camera/depth/points
input without the driver's depth-native /depth/color/points.
"""

from __future__ import division

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header


def unproject_z16(depth, fx, fy, cx, cy, scale=0.001):
    """Return Nx3 xyz in the image optical frame. depth is HxW uint16 mm."""
    height, width = depth.shape
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    z = depth.astype(np.float32) * float(scale)
    valid = np.isfinite(z) & (z > 0.0)
    x = (uu - float(cx)) / float(fx) * z
    y = (vv - float(cy)) / float(fy) * z
    pts = np.stack((x[valid], y[valid], z[valid]), axis=1)
    return pts


def xyz_to_cloud(header, points):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.data = np.asarray(points, dtype=np.float32).tobytes()
    return msg


class AlignedDepthCloudNode(Node):

    def __init__(self):
        super().__init__("aligned_depth_cloud")
        self.declare_parameter(
            "depth_topic", "/camera/d555/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "info_topic", "/camera/d555/aligned_depth_to_color/camera_info")
        self.declare_parameter("points_topic", "/camera/depth/points")
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        be = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self._info = None
        self._pub = self.create_publisher(
            PointCloud2, self.get_parameter("points_topic").value, qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value,
            self._on_info, qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("info_topic").value,
            self._on_info, be)
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self._on_depth, qos)
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self._on_depth, be)
        self.get_logger().info(
            "aligned_depth_cloud %s + info -> %s"
            % (self.get_parameter("depth_topic").value,
               self.get_parameter("points_topic").value))

    def _on_info(self, msg):
        self._info = msg

    def _on_depth(self, msg):
        if self._info is None:
            return
        k = self._info.k
        fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return
        if msg.encoding in ("16UC1", "mono16"):
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                msg.height, msg.width)
            points = unproject_z16(depth, fx, fy, cx, cy, 0.001)
        elif msg.encoding in ("32FC1",):
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width)
            height, width = depth.shape
            u = np.arange(width, dtype=np.float32)
            v = np.arange(height, dtype=np.float32)
            uu, vv = np.meshgrid(u, v)
            valid = np.isfinite(depth) & (depth > 0.0)
            x = (uu - cx) / fx * depth
            y = (vv - cy) / fy * depth
            points = np.stack((x[valid], y[valid], depth[valid]), axis=1)
        else:
            return
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
        self._pub.publish(xyz_to_cloud(header, points))


def main(argv=None):
    rclpy.init(args=argv)
    node = AlignedDepthCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
