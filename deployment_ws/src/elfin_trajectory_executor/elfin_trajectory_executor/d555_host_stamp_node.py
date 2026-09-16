"""
实机代码 (site / real-cell). Not part of the Gazebo simulation stack.
Rewrite D555 RGB-D stamps onto the host ROS system clock.

D555 PoE/DDS frames arrive as HARDWARE_CLOCK. The RealSense wrapper maps
that clock onto ROS time, but it also resets the mapping whenever the HW
counter goes backwards (logged as "Hardware clock reset"). Those jumps
break replay / TF / color-depth association.

This node groups color and aligned depth by their **exact** device stamp
(identical driver stamps when ``enable_sync`` is on), assigns **one** host
``rclpy`` time per device stamp, and republishes the canonical topics. Two
frames that do not share a device stamp are not one exposure, so they are
never fused into a pair: the unmatched side expires and is counted.

Livox and CPS already stamp with host time and are not rewritten here.
"""

from __future__ import annotations

import copy
import json
from collections import OrderedDict
from typing import Iterable

import rclpy
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
# D555 image_transport raw publishers on this cell are RELIABLE +
# TRANSIENT_LOCAL. A BEST_EFFORT/VOLATILE reader never sees frames, so
# canonical /camera/d555/color/image_raw stays silent while IMU still works.
_IMAGE_HW = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def apply_common_stamp(msgs: Iterable, stamp) -> None:
    """Write the same header.stamp onto every message (in place)."""
    for msg in msgs:
        msg.header.stamp = stamp


def stamp_key(stamp) -> tuple:
    """Exact integer identity of a header stamp."""
    return (int(stamp.sec), int(stamp.nanosec))


def _key_ns(key: tuple) -> int:
    return int(key[0]) * 1000000000 + int(key[1])


class ExactStampGrouper:
    """Group colour and aligned depth by identical device stamp.

    No slop window: a tolerance match would fuse two exposures, and the
    downstream preprocessor could then never tell a real synchronisation
    fault from a genuine acquisition. One host stamp is allocated per device
    stamp, so whichever side arrives second reuses the first allocation.

    Duplicate deliveries (the same message seen through two QoS readers)
    replace their slot and never emit twice. An incomplete stamp evicted by
    capacity, horizon, or a device-clock rollback is counted by side.
    """

    def __init__(self, maxlen: int = 15, horizon_ns: int = 1000000000,
                 rollback_ns: int = 500000000) -> None:
        self.maxlen = max(1, int(maxlen))
        self.horizon_ns = max(1, int(horizon_ns))
        self.rollback_ns = max(1, int(rollback_ns))
        self.unpaired_color = 0
        self.unpaired_depth = 0
        self.groups = 0
        self.duplicates = 0
        self.resets = 0
        self._pending: OrderedDict = OrderedDict()
        self._emitted: OrderedDict = OrderedDict()
        self._newest_ns = None

    def counters(self) -> dict:
        return {
            "groups": self.groups,
            "unpaired_color": self.unpaired_color,
            "unpaired_depth": self.unpaired_depth,
            "duplicates": self.duplicates,
            "clock_resets": self.resets,
            "pending": len(self._pending),
        }

    def add(self, stream: str, key: tuple, msg, host_stamp_factory):
        """Insert one side; return ``(color, depth, host_stamp)`` or None."""
        if stream not in ("color", "depth"):
            raise ValueError("unknown stream %r" % stream)
        key = (int(key[0]), int(key[1]))
        key_ns = _key_ns(key)
        if self._newest_ns is not None and key_ns < self._newest_ns - self.rollback_ns:
            # Driver logged a hardware clock reset: the old timeline can
            # never complete, so drop it rather than pairing across the jump.
            self._flush("rollback")
            self.resets += 1
        if key in self._emitted:
            self.duplicates += 1
            return None
        entry = self._pending.get(key)
        if entry is None:
            entry = {"host": host_stamp_factory()}
            self._pending[key] = entry
        elif stream in entry:
            self.duplicates += 1
            return None
        entry[stream] = msg
        if self._newest_ns is None or key_ns > self._newest_ns:
            self._newest_ns = key_ns
        self._prune()
        if "color" in entry and "depth" in entry:
            self._pending.pop(key, None)
            self._note_emitted(key)
            self.groups += 1
            return entry["color"], entry["depth"], entry["host"]
        return None

    def _note_emitted(self, key: tuple) -> None:
        self._emitted[key] = True
        while len(self._emitted) > 4 * self.maxlen:
            self._emitted.popitem(last=False)

    def _count_incomplete(self, entry: dict) -> None:
        if "color" in entry and "depth" not in entry:
            self.unpaired_color += 1
        elif "depth" in entry and "color" not in entry:
            self.unpaired_depth += 1

    def _flush(self, _reason: str) -> None:
        for entry in self._pending.values():
            self._count_incomplete(entry)
        self._pending.clear()
        self._emitted.clear()
        self._newest_ns = None

    def _prune(self) -> None:
        horizon = (self._newest_ns - self.horizon_ns
                   if self._newest_ns is not None else None)
        for key in list(self._pending.keys()):
            if horizon is not None and _key_ns(key) < horizon:
                self._count_incomplete(self._pending.pop(key))
        while len(self._pending) > self.maxlen:
            _key, entry = self._pending.popitem(last=False)
            self._count_incomplete(entry)


class D555HostStampNode(Node):
    def __init__(self) -> None:
        super().__init__("d555_host_stamp")
        self.declare_parameter("camera_namespace", "camera")
        self.declare_parameter("camera_name", "d555")
        # Retired: pairing is exact-device-stamp only. Kept so existing
        # launch overrides do not fail; a non-zero value logs a warning.
        self.declare_parameter("slop_s", 0.0)
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
        self._n_color_hw = 0
        self._n_depth_hw = 0
        self._last_pair_key = None
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

        self._on_pair = on_pair
        self._grouper = ExactStampGrouper()
        for qos in (_IMAGE_HW, _SENSOR):
            self.create_subscription(
                color_type, color_in, self._on_color_hw, qos)
            self.create_subscription(
                depth_type, depth_in, self._on_depth_hw, qos)

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
            f"RGB-D grouped by exact device stamp in={color_in} "
            f"subscribe_compressed={self._subscribe_compressed} "
            f"compress={self._compress} jpeg_q={self._jpeg_quality} "
            f"publish_raw={self._publish_raw}"
        )
        if slop > 0.0:
            self.get_logger().warning(
                "[d555_host_stamp] slop_s=%.3f is retired: colour and aligned "
                "depth are grouped by exact device stamp only" % slop)

    def _host_stamp(self):
        return self.get_clock().now().to_msg()

    def _publish_status(self) -> None:
        payload = json.loads(CLOCK_MASTER_JSON)
        payload["pairing"] = "exact_device_stamp"
        payload.update(self._grouper.counters())
        payload["color_hw"] = self._n_color_hw
        payload["depth_hw"] = self._n_depth_hw
        self._status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _warn_if_silent(self) -> None:
        if self._n > 0 or self._warned:
            return
        self._warned = True
        self.get_logger().error(
            "[d555_host_stamp] no color+aligned_depth pair in 10s. "
            "in=%s color_hw=%d aligned_hw=%d cinfo=%s dinfo=%s pairs=%d. "
            "D555 image_transport is RELIABLE+TRANSIENT_LOCAL; "
            "canonical /camera/d555/* stay silent until a matched pair arrives."
            % (
                self._color_in,
                self._n_color_hw,
                self._n_depth_hw,
                self._cinfo is not None,
                self._dinfo is not None,
                self._n,
            )
        )

    def _on_cinfo(self, msg: CameraInfo) -> None:
        self._cinfo = msg

    def _on_dinfo(self, msg: CameraInfo) -> None:
        self._dinfo = msg

    def _on_color_hw(self, msg) -> None:
        self._n_color_hw += 1
        self._group("color", msg)

    def _on_depth_hw(self, msg) -> None:
        self._n_depth_hw += 1
        self._group("depth", msg)

    def _group(self, stream: str, msg) -> None:
        if self._cinfo is None or self._dinfo is None:
            return
        grouped = self._grouper.add(
            stream, stamp_key(msg.header.stamp), msg, self._host_stamp)
        if grouped is None:
            return
        color, depth, stamp = grouped
        if not self._should_emit_pair(stamp_key(color.header.stamp)):
            return
        self._on_pair(color, depth, stamp)

    def _should_emit_pair(self, key: tuple) -> bool:
        if key == self._last_pair_key:
            return False
        self._last_pair_key = key
        return True

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

    def _on_rgbd_raw(self, color: Image, depth: Image, stamp) -> None:
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
        self, color: CompressedImage, depth: CompressedImage, stamp
    ) -> None:
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
