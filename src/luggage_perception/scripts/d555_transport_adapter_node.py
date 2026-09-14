#!/usr/bin/env python3
"""D555 raw-driver edge: acquisition-clock mapping plus JPEG/PNG transport.

The node deliberately does not pair colour and depth.  The D555 driver emits
hardware-synchronised equal source stamps; this adapter maps equal source
stamps identically and the shared SensorPreprocessor remains the sole pairing
owner.
"""

from __future__ import annotations

import copy
import json

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from luggage_perception.device_clock import DeviceClockMapper
from luggage_perception.rgbd_codec import color_to_jpeg, depth_to_png


_IMAGE_HW = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=20,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
_SENSOR = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=20,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE)
_INFO_HW_RELIABLE = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)
_INFO_OUT = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def _time_msg(ns):
    out = Time()
    out.sec = int(ns // 1000000000)
    out.nanosec = int(ns % 1000000000)
    return out


class D555TransportAdapter(Node):
    def __init__(self):
        super().__init__("d555_transport_adapter")
        defaults = {
            "camera_namespace": "camera",
            "camera_name": "d555",
            "jpeg_quality": 80,
            "publish_raw": False,
            "require_clock_lock": True,
            "clock_min_samples": 5,
            "clock_rollback_sec": 0.5,
            "clock_max_offset_slew_ms": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        ns = str(self.get_parameter("camera_namespace").value).strip("/")
        name = str(self.get_parameter("camera_name").value).strip("/")
        self._base = "/%s/%s" % (ns, name)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self._publish_raw = bool(self.get_parameter("publish_raw").value)
        self._require_lock = bool(
            self.get_parameter("require_clock_lock").value)
        self._clock = DeviceClockMapper(
            min_samples=int(self.get_parameter("clock_min_samples").value),
            rollback_ns=int(float(
                self.get_parameter("clock_rollback_sec").value) * 1e9),
            max_offset_slew_ns=int(float(self.get_parameter(
                "clock_max_offset_slew_ms").value) * 1e6),
        )
        self._color_info = None
        self._depth_info = None
        self._counts = {
            "color_in": 0, "depth_in": 0, "color_out": 0,
            "depth_out": 0, "warmup_drop": 0, "codec_error": 0,
        }

        self._color_pub = self.create_publisher(
            CompressedImage, self._base + "/color/image_raw/compressed", _SENSOR)
        self._depth_pub = self.create_publisher(
            CompressedImage,
            self._base + "/aligned_depth_to_color/image_raw/compressed", _SENSOR)
        self._color_info_pub = self.create_publisher(
            CameraInfo, self._base + "/color/camera_info", _INFO_OUT)
        self._depth_info_pub = self.create_publisher(
            CameraInfo, self._base + "/aligned_depth_to_color/camera_info", _INFO_OUT)
        self._raw_color_pub = None
        self._raw_depth_pub = None
        if self._publish_raw:
            self._raw_color_pub = self.create_publisher(
                Image, self._base + "/color/image_raw", _SENSOR)
            self._raw_depth_pub = self.create_publisher(
                Image, self._base + "/aligned_depth_to_color/image_raw", _SENSOR)

        self._mapping_pub = self.create_publisher(
            String, "/clock_sync/d555", _SENSOR)
        self._master_pub = self.create_publisher(
            String, "/clock_sync/master", _INFO_OUT)
        self.create_subscription(
            CameraInfo, self._base + "/color/camera_info_hw",
            self._on_color_info, _SENSOR)
        self.create_subscription(
            CameraInfo, self._base + "/color/camera_info_hw",
            self._on_color_info, _INFO_HW_RELIABLE)
        self.create_subscription(
            CameraInfo, self._base + "/aligned_depth_to_color/camera_info_hw",
            self._on_depth_info, _SENSOR)
        self.create_subscription(
            CameraInfo, self._base + "/aligned_depth_to_color/camera_info_hw",
            self._on_depth_info, _INFO_HW_RELIABLE)
        self.create_subscription(
            Image, self._base + "/color/image_hw", self._on_color, _IMAGE_HW)
        self.create_subscription(
            Image, self._base + "/aligned_depth_to_color/image_hw",
            self._on_depth, _IMAGE_HW)
        self.create_timer(1.0, self._publish_master)
        self._publish_master()

    def _on_color_info(self, msg):
        self._color_info = msg

    def _on_depth_info(self, msg):
        self._depth_info = msg

    def _mapping(self, msg, stream):
        receipt_ns = int(self.get_clock().now().nanoseconds)
        mapped = self._clock.map(_stamp_ns(msg.header.stamp), receipt_ns)
        event = {
            "stream": stream,
            "device_stamp_ns": mapped.device_ns,
            "mapped_host_stamp_ns": mapped.mapped_host_ns,
            "receipt_host_stamp_ns": mapped.receipt_host_ns,
            "estimated_delay_ns": mapped.estimated_delay_ns,
            "epoch": mapped.epoch,
            "quality": mapped.quality,
        }
        self._mapping_pub.publish(String(data=json.dumps(event, sort_keys=True)))
        if self._require_lock and mapped.quality != "locked":
            self._counts["warmup_drop"] += 1
            return None
        return _time_msg(mapped.mapped_host_ns)

    @staticmethod
    def _publish_info(cached, publisher, stamp):
        if cached is None:
            return
        info = copy.deepcopy(cached)
        info.header.stamp = stamp
        publisher.publish(info)

    def _on_color(self, msg):
        self._counts["color_in"] += 1
        stamp = self._mapping(msg, "color")
        if stamp is None:
            return
        try:
            out = color_to_jpeg(msg, self._jpeg_quality)
        except Exception as exc:  # codec failures are observable, not fatal
            self._counts["codec_error"] += 1
            self.get_logger().error("D555 color compression failed: %s" % exc)
            return
        out.header.stamp = stamp
        self._color_pub.publish(out)
        if self._raw_color_pub is not None:
            msg.header.stamp = stamp
            self._raw_color_pub.publish(msg)
        self._publish_info(self._color_info, self._color_info_pub, stamp)
        self._counts["color_out"] += 1

    def _on_depth(self, msg):
        self._counts["depth_in"] += 1
        stamp = self._mapping(msg, "depth")
        if stamp is None:
            return
        try:
            out = depth_to_png(msg)
        except Exception as exc:  # codec failures are observable, not fatal
            self._counts["codec_error"] += 1
            self.get_logger().error("D555 depth compression failed: %s" % exc)
            return
        out.header.stamp = stamp
        self._depth_pub.publish(out)
        if self._raw_depth_pub is not None:
            msg.header.stamp = stamp
            self._raw_depth_pub.publish(msg)
        self._publish_info(self._depth_info, self._depth_info_pub, stamp)
        self._counts["depth_out"] += 1

    def _publish_master(self):
        payload = {
            "master": "host_ros_system_time",
            "d555": "mapped_device_acquisition_time",
            "mapping": self._clock.diagnostics(),
            "counts": dict(self._counts),
        }
        self._master_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


def main():
    rclpy.init()
    node = D555TransportAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
