#!/usr/bin/env python3
"""Semantic point filter node (ROS 2 Humble port).

Routes the preprocessed depth cloud through the semantic label mask:
pixels labelled cargo go to ``/luggage/semantic/cargo_points`` (the
detector's input), obstacle labels to ``/luggage/semantic/obstacle_points``.

This is deliberately much thinner than the ROS 1 node: frame conversion,
inf filtering and slop-based pairing all live in the preprocessor now. The
only alignment logic here is an **exact** ``(sec, nanosec)`` stamp join
between the cloud and the mask - both inherit the preprocessor's RGB
primary stamp, so the keys match by construction. Approximate-time pairing
is not allowed downstream of the preprocessor
(docs/architecture/sensor_data_pipeline.md).

``fallback_to_raw`` from the ROS 1 node is deliberately gone: an
unfiltered cloud must never be published as cargo geometry (acceptance
criterion). No mask -> no cargo cloud, reason recorded in stats.

Intrinsics come from the live ``camera_info`` (preferred; in simulation the
preprocessor feeds both slots from the single gz camera_info, which is
self-consistent with ``extrinsics_source: identity``). A realsense yaml is
only an offline fallback for ``extrinsics_source: config`` on real
hardware.
"""

from __future__ import division

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String

from luggage_perception import ros_message_adapters as adapters
from luggage_perception.semantic_point_filter import (
    CameraIntrinsics,
    DepthToColorExtrinsics,
    SemanticPointFilter,
)


def _stamp_key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


class SemanticPointFilterNode(Node):

    def __init__(self):
        super().__init__("semantic_point_filter")
        defaults = {
            "input.cloud": "/luggage/preprocessed/camera/depth/points",
            "input.mask": "/luggage/semantic/mask",
            "input.instance_mask": "/luggage/semantic/instance_mask",
            "input.camera_info": "/luggage/preprocessed/camera/color/camera_info",
            "output.cargo_points": "/luggage/semantic/cargo_points",
            "output.obstacle_points": "/luggage/semantic/obstacle_points",
            "output.stats": "~/stats_json",
            "cargo_labels": [2],
            "obstacle_labels": [2, 4],
            # "identity" (gz: color and depth are one sensor) | "config"
            # (real D435: depth_to_color from the realsense yaml).
            "extrinsics_source": "identity",
            "realsense_extrinsics_config": "",
            "buffer_maxlen": 10,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._cargo_labels = [
            int(v) for v in self.get_parameter("cargo_labels").value]
        self._obstacle_labels = [
            int(v) for v in self.get_parameter("obstacle_labels").value]
        self._buffer_maxlen = max(2, int(self.get_parameter("buffer_maxlen").value))

        extrinsics = self._load_extrinsics()
        self._intrinsics = None          # from live camera_info
        self._extrinsics = extrinsics
        self._filter = None              # built once intrinsics arrive

        # Exact-stamp join buffers (bounded, oldest evicted first).
        self._clouds = {}
        self._masks = {}
        self._instances = {}
        self._counts = {"cloud": 0, "mask": 0, "instance": 0,
                        "joined": 0, "processed": 0, "no_intrinsics": 0,
                        "mask_decode_fail": 0, "cloud_decode_fail": 0}

        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        # camera_info from the preprocessor is also best-effort sensor data.
        stats_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._cargo_pub = self.create_publisher(
            PointCloud2, self.get_parameter("output.cargo_points").value,
            sensor_qos)
        self._obstacle_pub = self.create_publisher(
            PointCloud2, self.get_parameter("output.obstacle_points").value,
            sensor_qos)
        self._stats_pub = self.create_publisher(
            String, self.get_parameter("output.stats").value, stats_qos)

        self.create_subscription(
            PointCloud2, self.get_parameter("input.cloud").value,
            self._on_cloud, sensor_qos)
        self.create_subscription(
            Image, self.get_parameter("input.mask").value, self._on_mask,
            sensor_qos)
        self.create_subscription(
            Image, self.get_parameter("input.instance_mask").value,
            self._on_instance, sensor_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter("input.camera_info").value,
            self._on_camera_info, sensor_qos)

        self.get_logger().info(
            "semantic_point_filter ready (extrinsics=%s, cargo_labels=%s)"
            % (self.get_parameter("extrinsics_source").value, self._cargo_labels))

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------

    def _load_extrinsics(self):
        source = str(self.get_parameter("extrinsics_source").value)
        if source == "identity":
            return DepthToColorExtrinsics.identity()
        if source == "config":
            path = str(self.get_parameter("realsense_extrinsics_config").value)
            if not path:
                raise RuntimeError(
                    "extrinsics_source=config requires realsense_extrinsics_config")
            import yaml
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle)
            section = data.get("camera", {}).get("extrinsics", {})
            return DepthToColorExtrinsics.from_dict(
                section.get("depth_to_color", {}))
        raise RuntimeError("unknown extrinsics_source %r" % source)

    # ------------------------------------------------------------------
    # Subscriptions + exact-stamp join
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg):
        frame = adapters.camera_info_frame_from_msg(msg)
        intr = CameraIntrinsics(
            frame.fx, frame.fy, frame.cx, frame.cy, frame.width, frame.height,
            distortion_coeffs=list(frame.distortion_coeffs),
            distortion_model=frame.distortion_model,
        )
        if self._intrinsics is None or (
                intr.fx, intr.fy, intr.cx, intr.cy) != (
                self._intrinsics.fx, self._intrinsics.fy,
                self._intrinsics.cx, self._intrinsics.cy):
            self._filter = SemanticPointFilter(
                intr, intr, self._extrinsics,
                self._cargo_labels, self._obstacle_labels)
        self._intrinsics = intr

    def _on_cloud(self, msg):
        self._counts["cloud"] += 1
        self._store(self._clouds, _stamp_key(msg), msg)
        self._try_process(_stamp_key(msg))

    def _on_mask(self, msg):
        self._counts["mask"] += 1
        self._store(self._masks, _stamp_key(msg), msg)
        self._try_process(_stamp_key(msg))

    def _on_instance(self, msg):
        self._counts["instance"] += 1
        self._store(self._instances, _stamp_key(msg), msg)

    def _store(self, buffer_, key, msg):
        buffer_[key] = msg
        while len(buffer_) > self._buffer_maxlen:
            oldest = min(buffer_)
            del buffer_[oldest]
            if oldest == key:
                break

    def _try_process(self, key):
        if key not in self._clouds or key not in self._masks:
            return
        self._counts["joined"] += 1
        cloud_msg = self._clouds.pop(key)
        mask_msg = self._masks.pop(key)
        instance_msg = self._instances.pop(key, None)

        label_map = adapters.image_array_from_msg(mask_msg)
        if label_map is None:
            self._counts["mask_decode_fail"] += 1
            self._warn_throttled(
                "dropping mask with encoding %s" % mask_msg.encoding)
            return
        points = adapters.cloud_points_from_msg(cloud_msg)
        if points is None:
            self._counts["cloud_decode_fail"] += 1
            self._warn_throttled("dropping cloud with unsupported layout")
            return
        if self._filter is None:
            self._counts["no_intrinsics"] += 1
            self._warn_throttled("no camera_info yet; dropping joined pair")
            return

        instance_map = (adapters.mono16_array_from_msg(instance_msg)
                        if instance_msg is not None else None)
        cargo, obstacle = self._filter.filter_points(
            points, label_map, instance_map)
        self._counts["processed"] += 1

        stamp = cloud_msg.header.stamp
        frame_id = cloud_msg.header.frame_id
        if cargo:
            self._cargo_pub.publish(adapters.cloud_msg_from_points(
                [p[:3] for p in cargo], stamp, frame_id))
        if obstacle:
            self._obstacle_pub.publish(adapters.cloud_msg_from_points(
                [p[:3] for p in obstacle], stamp, frame_id))
        self._publish_stats()

    def _publish_stats(self):
        record = dict(self._filter.last_stats)
        record.update(self._counts)
        self._stats_pub.publish(String(data=json.dumps(record, sort_keys=True)))

    def _warn_throttled(self, text):
        now = self.get_clock().now().nanoseconds
        last = getattr(self, "_warn_ns", 0)
        if now - last > 5e9:
            self.get_logger().warning(text)
            self._warn_ns = now


def main(argv=None):
    rclpy.init(args=argv)
    node = SemanticPointFilterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
