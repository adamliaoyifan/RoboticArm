"""Rewrite D555 RGB-D stamps onto the host ROS system clock.

D555 PoE/DDS frames arrive as HARDWARE_CLOCK. The RealSense wrapper maps
that clock onto ROS time, but it also resets the mapping whenever the HW
counter goes backwards (logged as "Hardware clock reset"). Those jumps
break replay / TF / color-depth association.

This node takes the hardware-synced pair (identical driver stamps when
``enable_sync`` is on), assigns **one** host ``rclpy`` time to color,
aligned depth, and both CameraInfos, and republishes the canonical topics.

Livox and CPS already stamp with host time and are not rewritten here.
"""

from __future__ import annotations

import copy
from typing import Iterable

import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu
from std_msgs.msg import String

from .rgbd_codec import (
    canonicalize_color_compressed,
    canonicalize_depth_compressed,
    color_to_jpeg,
    depth_to_png,
    jpeg_to_color,
    png_to_depth,
)

CLOCK_MASTER_JSON = (
    '{"master":"host_ros_system_time",'
    '"d555":"receive_sync_group",'
    '"livox":"driver_host_stamp",'
    '"cps":"host_ros_system_time"}'
)

# CameraInfo: subscribe Best Effort/Volatile (compatible with the D555
# driver whether it offers SensorData or Reliable/Volatile). Also keep a
# Reliable/Volatile reader: FastDDS sometimes starves Best Effort readers
# of Reliable writers. Publish Reliable + Transient Local so RViz and a
# late-joining preprocessor both see a valid info message.
# Do not subscribe Transient Local: the driver is Volatile and that pair
# logs DURABILITY and delivers nothing.
_INFO = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_INFO_STREAM = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_SENSOR = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
_IMU_RELIABLE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def apply_common_stamp(msgs: Iterable, stamp) -> None:
    """Write the same header.stamp onto every message (in place)."""
    for msg in msgs:
        msg.header.stamp = stamp


class D555HostStampNode(Node):
    def __init__(self) -> None:
        super().__init__("d555_host_stamp")
        self.declare_parameter("camera_namespace", "camera")
        self.declare_parameter("camera_name", "d555")
        self.declare_parameter("slop_s", 0.05)
        self.declare_parameter("enable_imu", True)
        self.declare_parameter("compress", True)
        self.declare_parameter("publish_raw", False)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("subscribe_compressed", False)
        ns = str(self.get_parameter("camera_namespace").value).strip("/")
        name = str(self.get_parameter("camera_name").value).strip("/")
        base = f"/{ns}/{name}"
        slop = float(self.get_parameter("slop_s").value)
        self._compress = bool(self.get_parameter("compress").value)
        self._publish_raw = bool(self.get_parameter("publish_raw").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._subscribe_compressed = bool(
            self.get_parameter("subscribe_compressed").value
        )
        if not self._compress:
            self._publish_raw = True
            self._subscribe_compressed = False

        color_hw = f"{base}/color/image_hw"
        depth_hw = f"{base}/aligned_depth_to_color/image_hw"
        if self._subscribe_compressed:
            color_in = color_hw + "/compressed"
            depth_in = depth_hw + "/compressed"
            color_type = CompressedImage
            depth_type = CompressedImage
            on_pair = self._on_rgbd_compressed
        else:
            color_in = color_hw
            depth_in = depth_hw
            color_type = Image
            depth_type = Image
            on_pair = self._on_rgbd_raw
        self._color_in = color_in

        self._cinfo = None
        self._dinfo = None
        self.create_subscription(
            CameraInfo, f"{base}/color/camera_info_hw", self._on_cinfo, _SENSOR
        )
        self.create_subscription(
            CameraInfo, f"{base}/color/camera_info_hw", self._on_cinfo, _INFO_STREAM
        )
        self.create_subscription(
            CameraInfo,
            f"{base}/aligned_depth_to_color/camera_info_hw",
            self._on_dinfo,
            _SENSOR,
        )
        self.create_subscription(
            CameraInfo,
            f"{base}/aligned_depth_to_color/camera_info_hw",
            self._on_dinfo,
            _INFO_STREAM,
        )

        self._color_sub = Subscriber(self, color_type, color_in, qos_profile=_SENSOR)
        self._depth_sub = Subscriber(self, depth_type, depth_in, qos_profile=_SENSOR)
        self._sync = ApproximateTimeSynchronizer(
            [self._color_sub, self._depth_sub],
            queue_size=20,
            slop=slop,
        )
        self._sync.registerCallback(on_pair)

        self._color_pub = None
        self._depth_pub = None
        self._color_jpg_pub = None
        self._depth_png_pub = None
        if self._publish_raw:
            self._color_pub = self.create_publisher(
                Image, f"{base}/color/image_raw", _SENSOR
            )
            self._depth_pub = self.create_publisher(
                Image, f"{base}/aligned_depth_to_color/image_raw", _SENSOR
            )
        if self._compress:
            self._color_jpg_pub = self.create_publisher(
                CompressedImage, f"{base}/color/image_raw/compressed", _SENSOR
            )
            self._depth_png_pub = self.create_publisher(
                CompressedImage,
                f"{base}/aligned_depth_to_color/image_raw/compressed",
                _SENSOR,
            )
        self._cinfo_pub = self.create_publisher(
            CameraInfo, f"{base}/color/camera_info", _INFO
        )
        self._dinfo_pub = self.create_publisher(
            CameraInfo, f"{base}/aligned_depth_to_color/camera_info", _INFO
        )
        self._status_pub = self.create_publisher(String, "/clock_sync/master", 10)
        self._logged = False
        self._n = 0
        self._warned = False

        self._imu_n = 0
        if bool(self.get_parameter("enable_imu").value):
            self._imu_pub = self.create_publisher(Imu, f"{base}/imu", _SENSOR)
            self._gyro_pub = self.create_publisher(Imu, f"{base}/gyro/sample", _SENSOR)
            self._accel_pub = self.create_publisher(
                Imu, f"{base}/accel/sample", _SENSOR
            )
            # D555 DDS publishes combined BMI088 on motion/sample (Best Effort).
            # Subscribe *_hw after remap, and the unmapped name if remap missed.
            self._subscribe_imu(f"{base}/motion/sample_hw", self._on_motion)
            self._subscribe_imu(f"{base}/motion/sample", self._on_motion)
            self._subscribe_imu(f"{base}/imu_hw", self._on_imu)
            self._subscribe_imu(f"{base}/gyro/sample_hw", self._on_gyro)
            self._subscribe_imu(f"{base}/accel/sample_hw", self._on_accel)
        else:
            self._imu_pub = None
            self._gyro_pub = None
            self._accel_pub = None

        self.create_timer(1.0, self._publish_status)
        self.create_timer(10.0, self._warn_if_silent)
        self._publish_status()
        self.get_logger().info(
            f"[d555_host_stamp] master=host ROS system time. "
            f"RGB-D sync slop={slop:.3f}s in={color_in} "
            f"subscribe_compressed={self._subscribe_compressed} "
            f"compress={self._compress} jpeg_q={self._jpeg_quality} "
            f"publish_raw={self._publish_raw}"
        )

    def _host_stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_status(self) -> None:
        self._status_pub.publish(String(data=CLOCK_MASTER_JSON))

    def _warn_if_silent(self) -> None:
        if self._n > 0 or self._warned:
            return
        self._warned = True
        self.get_logger().error(
            "[d555_host_stamp] no color+aligned_depth pair in 10s. "
            "Check realsense2_camera_node is still up and publishing %s. "
            "Canonical /camera/d555/* topics stay silent until the HW pair arrives."
            % self._color_in
        )

    def _on_cinfo(self, msg: CameraInfo) -> None:
        self._cinfo = msg

    def _on_dinfo(self, msg: CameraInfo) -> None:
        self._dinfo = msg

    def _finish_group(self, cinfo, dinfo, stamp) -> None:
        cinfo = copy.deepcopy(cinfo)
        dinfo = copy.deepcopy(dinfo)
        apply_common_stamp((cinfo, dinfo), stamp)
        self._cinfo_pub.publish(cinfo)
        self._dinfo_pub.publish(dinfo)
        self._n += 1
        if not self._logged:
            self._logged = True
            self.get_logger().info(
                "[d555_host_stamp] first RGB-D group stamped with host ROS time"
            )

    def _on_rgbd_raw(self, color: Image, depth: Image) -> None:
        if self._cinfo is None or self._dinfo is None:
            return
        stamp = self._host_stamp()
        apply_common_stamp((color, depth), stamp)
        if self._color_pub is not None:
            self._color_pub.publish(color)
            self._depth_pub.publish(depth)
        if self._color_jpg_pub is not None:
            try:
                self._color_jpg_pub.publish(color_to_jpeg(color, self._jpeg_quality))
                self._depth_png_pub.publish(depth_to_png(depth))
            except Exception as exc:
                self.get_logger().error(f"[d555_host_stamp] compress failed: {exc}")
                return
        self._finish_group(self._cinfo, self._dinfo, stamp)

    def _on_rgbd_compressed(
        self, color: CompressedImage, depth: CompressedImage
    ) -> None:
        if self._cinfo is None or self._dinfo is None:
            return
        stamp = self._host_stamp()
        try:
            color = canonicalize_color_compressed(color)
            depth = canonicalize_depth_compressed(depth)
        except ValueError as exc:
            self.get_logger().error(f"[d555_host_stamp] compressed format: {exc}")
            return
        apply_common_stamp((color, depth), stamp)
        if self._color_pub is not None:
            try:
                self._color_pub.publish(jpeg_to_color(color))
                self._depth_pub.publish(png_to_depth(depth))
            except Exception as exc:
                self.get_logger().error(f"[d555_host_stamp] decompress failed: {exc}")
                return
        if self._color_jpg_pub is not None:
            self._color_jpg_pub.publish(color)
            self._depth_png_pub.publish(depth)
        self._finish_group(self._cinfo, self._dinfo, stamp)

    def _subscribe_imu(self, topic: str, callback) -> None:
        self.create_subscription(Imu, topic, callback, _SENSOR)
        self.create_subscription(Imu, topic, callback, _IMU_RELIABLE)

    def _note_imu(self, source: str) -> None:
        self._imu_n += 1
        if self._imu_n == 1:
            self.get_logger().info(
                "[d555_host_stamp] first IMU restamped from %s" % source
            )

    def _on_motion(self, msg: Imu) -> None:
        """DDS combined sample → canonical imu + gyro + accel bag names."""
        msg.header.stamp = self._host_stamp()
        self._imu_pub.publish(msg)
        self._gyro_pub.publish(msg)
        self._accel_pub.publish(msg)
        self._note_imu("motion/sample_hw")

    def _on_imu(self, msg: Imu) -> None:
        msg.header.stamp = self._host_stamp()
        self._imu_pub.publish(msg)
        self._note_imu("imu_hw")

    def _on_gyro(self, msg: Imu) -> None:
        msg.header.stamp = self._host_stamp()
        self._gyro_pub.publish(msg)
        self._note_imu("gyro/sample_hw")

    def _on_accel(self, msg: Imu) -> None:
        msg.header.stamp = self._host_stamp()
        self._accel_pub.publish(msg)
        self._note_imu("accel/sample_hw")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = D555HostStampNode()
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
