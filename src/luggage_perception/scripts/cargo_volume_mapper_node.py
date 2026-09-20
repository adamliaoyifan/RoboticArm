#!/usr/bin/env python3
"""Cargo occupancy map node (ROS 2 Humble).

Thin shell around the pure ``CargoVolumeMapper``: geometry commits and
settled cargo-view integrates in, surface map + stats out.

Services (absolute names, ROS 2 has no ``~`` semantics):
  /cargo_map/add_placed_box          AddPlacedBox   -> mark_placed_box
  /cargo_map/remove_placed_box       RemovePlacedBox-> unmark_placed_box
  /cargo_map/reset                   ResetCargoMap  -> reset()
  /cargo_map/get_stats               GetCargoMapStats
  /cargo_map/integrate_cargo_view    IntegrateCargoView

Topics:
  /luggage/cargo_map/surface_2d  std_msgs/String (JSON, transient-local)
  /luggage/cargo_map/committed   std_msgs/String (JSON, transient-local)

``IntegrateCargoView`` consumes ``/luggage/semantic/cargo_points_untracked``
(label-filtered, pre-tracker). Latest-cloud integrate is forbidden: the
request stamp, geometry hash, and expected map revision must match.
SOURCE_GEOMETRY commits remain a lock against free-space misses (RS-2).
"""

from __future__ import division

import json
import threading
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from geometry_msgs.msg import Pose as PoseMsg
from luggage_msgs.srv import (
    AddPlacedBox,
    GetCargoMapStats,
    IntegrateCargoView,
    RemovePlacedBox,
    ResetCargoMap,
)

from luggage_description.scene_tf_config_utils import (
    container_hull_local_inside_fn,
    container_inner_ceiling_z,
    container_inner_dimensions,
    container_inner_floor_z,
    container_inner_geometry_descriptor,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
    static_transforms,
)
from luggage_perception import ros_message_adapters as adapters
from luggage_perception.cargo_instance_tracker import (
    rotation_from_xyzw,
    transform_points,
)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_view_integration import (
    REASON_TF_MISSING,
    REASON_VIEW_EMPTY,
    SCHEMA_VERSION,
    apply_integrate,
    select_fresh_record,
    stamp_key,
    stamp_to_sec,
    validate_integrate_request,
)


def _stamp_to_tf_time(stamp):
    return rclpy.time.Time(
        seconds=int(stamp.sec), nanoseconds=int(stamp.nanosec))


class CargoVolumeMapperNode(Node):

    def __init__(self):
        super().__init__("cargo_volume_mapper")
        group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("occupancy_params_yaml", "")
        self.declare_parameter(
            "cargo_cloud_topic", "/luggage/semantic/cargo_points_untracked")
        self.declare_parameter("base_frame", "container_link")
        self.declare_parameter("cloud_buffer_size", 12)
        self.declare_parameter("freshness_window_sec", 0.20)
        self.declare_parameter("max_raycast_points", 2000)
        self.declare_parameter("integrate_min_z_above_floor", 0.05)

        config_path = str(self.get_parameter("scene_tf_config").value)
        if not config_path:
            config_path = resolve_scene_tf_config_path()
        scene = load_scene_tf_config(config_path)
        inner = container_inner_dimensions(scene)
        inner_l = float(inner[0])
        inner_w = float(inner[1])
        floor_z = container_inner_floor_z(scene)
        ceiling_z = container_inner_ceiling_z(scene)
        inner_h = ceiling_z - floor_z
        volume_center_z = floor_z + 0.5 * inner_h

        descriptor = container_inner_geometry_descriptor(scene)
        self._geometry_hash = str(descriptor["geometry_hash"])
        self._mapper = CargoVolumeMapper(
            (inner_l, inner_w, inner_h),
            (0.0, 0.0, volume_center_z),
            0.0,
            resolution=float(self.get_parameter("resolution").value),
            hull_local_inside=container_hull_local_inside_fn(scene),
            geometry_descriptor=descriptor,
            max_raycast_points=int(
                self.get_parameter("max_raycast_points").value),
        )
        self._frame = str(self.get_parameter("base_frame").value) or "container_link"
        self._floor_z = float(floor_z)
        self._min_z_above_floor = float(
            self.get_parameter("integrate_min_z_above_floor").value)
        self._world_from_container = self._load_container_pose(scene)
        self._buffer_size = max(
            1, int(self.get_parameter("cloud_buffer_size").value))
        self._freshness_window_sec = float(
            self.get_parameter("freshness_window_sec").value)
        self._lock = threading.Lock()
        self._clouds = OrderedDict()
        self._integrated = set()

        self._tf_buffer = Buffer()
        self._tf_node = Node(
            "cargo_volume_mapper_tf", use_global_arguments=False)
        self._tf_listener = TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True)

        transient = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._surface_pub = self.create_publisher(
            String, "/luggage/cargo_map/surface_2d", transient)
        self._ledger_pub = self.create_publisher(
            String, "/luggage/cargo_map/committed", transient)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("cargo_cloud_topic").value),
            self._on_cloud, sensor_qos, callback_group=group)

        self.create_service(
            AddPlacedBox, "/cargo_map/add_placed_box",
            self._on_add, callback_group=group)
        self.create_service(
            RemovePlacedBox, "/cargo_map/remove_placed_box",
            self._on_remove, callback_group=group)
        self.create_service(
            ResetCargoMap, "/cargo_map/reset",
            self._on_reset, callback_group=group)
        self.create_service(
            GetCargoMapStats, "/cargo_map/get_stats",
            self._on_stats, callback_group=group)
        self.create_service(
            IntegrateCargoView, "/cargo_map/integrate_cargo_view",
            self._on_integrate, callback_group=group)

        self._publish_all()
        self.get_logger().info(
            "cargo_volume_mapper ready (inner %.2fx%.2fx%.2f m, res %.2f, "
            "geometry_hash %s, cloud=%s)"
            % (inner_l, inner_w, ceiling_z - floor_z,
               float(self.get_parameter("resolution").value),
               self._geometry_hash,
               self.get_parameter("cargo_cloud_topic").value))

    def shutdown_tf(self):
        listener = getattr(self, "_tf_listener", None)
        executor = getattr(listener, "executor", None) if listener else None
        if executor is not None:
            executor.shutdown()
            thread = getattr(listener, "dedicated_listener_thread", None)
            if thread is not None:
                thread.join(timeout=2.0)
        tf_node = getattr(self, "_tf_node", None)
        if tf_node is not None:
            tf_node.destroy_node()
            self._tf_node = None

    @staticmethod
    def _load_container_pose(scene):
        for entry in static_transforms(scene):
            if entry.get("child") == "container_link":
                return {
                    "translation": list(entry.get("translation",
                                                  [0.0, 0.0, 0.0])),
                    "rotation_rpy": list(entry.get("rotation_rpy",
                                                   [0.0, 0.0, 0.0])),
                }
        return {"translation": [0.0, 0.0, 0.0],
                "rotation_rpy": [0.0, 0.0, 0.0]}

    def _to_container(self, pose_world):
        """world pose -> container frame (yaw-only container assumed)."""
        import math
        tx, ty, tz = self._world_from_container["translation"]
        yaw = self._world_from_container["rotation_rpy"][2]
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        dx = pose_world.position.x - tx
        dy = pose_world.position.y - ty
        out = PoseMsg()
        out.position.x = dx * cos_y - dy * sin_y
        out.position.y = dx * sin_y + dy * cos_y
        out.position.z = pose_world.position.z - tz
        out.orientation = pose_world.orientation
        return out

    def _on_cloud(self, msg):
        points = adapters.cloud_points_from_msg(msg)
        if points is None:
            return
        finite = points[np.isfinite(points).all(axis=1)] if points.shape[0] else points
        key = stamp_key(msg.header.stamp)
        record = {
            "stamp_key": key,
            "stamp_sec": stamp_to_sec(msg.header.stamp),
            "stamp": msg.header.stamp,
            "frame_id": str(msg.header.frame_id or ""),
            "points": finite,
            "n_points": int(finite.shape[0]),
        }
        with self._lock:
            self._clouds[key] = record
            while len(self._clouds) > self._buffer_size:
                self._clouds.popitem(last=False)

    def _lookup_rt(self, target, source, stamp):
        if not source:
            return None
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                target, source, _stamp_to_tf_time(stamp),
                timeout=Duration(seconds=0.05))
        except (TransformException, Exception):  # noqa: BLE001 - TF boundary
            return None
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        rot = rotation_from_xyzw(r.x, r.y, r.z, r.w)
        trans = (float(t.x), float(t.y), float(t.z))
        return rot, trans

    def _fail_integrate(self, response, reason, message, stats=None):
        stats = stats or self._mapper.stats()
        response.schema_version = SCHEMA_VERSION
        response.success = False
        response.reason_code = str(reason)
        response.message = message
        response.unknown_ratio = float(stats["unknown_ratio"])
        response.occupancy_ratio = float(stats["occupancy_ratio"])
        response.resulting_map_revision = int(stats["map_revision"])
        return response

    def _on_integrate(self, request, response):
        stats = self._mapper.stats()
        current_rev = int(stats["map_revision"])
        reason = validate_integrate_request(
            request.settled, request.schema_version, request.geometry_hash,
            request.expected_map_revision, self._geometry_hash, current_rev)
        if reason:
            return self._fail_integrate(
                response, reason, reason, stats)

        with self._lock:
            records = list(self._clouds.values())
            integrated = set(self._integrated)
        record, reason = select_fresh_record(
            records, stamp_to_sec(request.source_acquisition_stamp),
            self._freshness_window_sec, integrated)
        if reason:
            return self._fail_integrate(response, reason, reason, stats)

        rt = self._lookup_rt(
            self._frame, record["frame_id"], record["stamp"])
        if rt is None:
            return self._fail_integrate(
                response, REASON_TF_MISSING,
                "tf %s -> %s at stamp missing" % (
                    record["frame_id"], self._frame),
                stats)
        rot, trans = rt
        points_base = transform_points(record["points"], rot, trans)
        zmin = self._floor_z + max(0.0, self._min_z_above_floor)
        if points_base is not None and getattr(points_base, "shape", (0,))[0]:
            points_base = points_base[points_base[:, 2] >= zmin]
        result = apply_integrate(
            self._mapper, points_base, origin=trans,
            voxel_size=self._mapper.resolution)
        if not result["ok"]:
            return self._fail_integrate(
                response, result["reason_code"] or REASON_VIEW_EMPTY,
                "n_finite=%d n_kept=%d" % (
                    result["n_finite"], result["n_kept"]),
                self._mapper.stats())

        with self._lock:
            self._integrated.add(tuple(record["stamp_key"]))
        self._publish_all()
        stats = self._mapper.stats()
        response.schema_version = SCHEMA_VERSION
        response.success = True
        response.reason_code = ""
        response.message = (
            "integrated n_finite=%d n_kept=%d latency_ms=%.1f view=%s"
            % (result["n_finite"], result["n_kept"],
               1000.0 * result["latency_sec"],
               request.view_request_id))
        response.unknown_ratio = float(stats["unknown_ratio"])
        response.occupancy_ratio = float(stats["occupancy_ratio"])
        response.resulting_map_revision = int(stats["map_revision"])
        response.integrated_acquisition_stamp = record["stamp"]
        return response

    def _on_add(self, request, response):
        slot = request.slot
        center_c = self._to_container(slot.place_pose)
        size = [max(0.0, float(slot.width)),
                max(0.0, float(slot.depth)),
                max(0.0, float(slot.height))]
        yaw = (
            self._yaw_from_pose(slot.place_pose)
            - float(self._world_from_container["rotation_rpy"][2])
        )
        try:
            added = self._mapper.mark_placed_box(
                [center_c.position.x, center_c.position.y,
                 center_c.position.z],
                size, yaw=yaw)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "mark failed: %s" % exc
            return response
        self._publish_all()
        response.success = True
        response.message = "%s %d boxes (rev %d)" % (
            "committed" if added else "already committed",
            len(self._mapper.commit_ledger()), self._mapper._revision)
        return response

    def _on_remove(self, request, response):
        slot = request.slot
        center_c = self._to_container(slot.place_pose)
        size = [max(0.0, float(slot.width)),
                max(0.0, float(slot.depth)),
                max(0.0, float(slot.height))]
        yaw = (
            self._yaw_from_pose(slot.place_pose)
            - float(self._world_from_container["rotation_rpy"][2])
        )
        removed = self._mapper.unmark_placed_box(
            [center_c.position.x, center_c.position.y,
             center_c.position.z],
            size, yaw=yaw)
        self._publish_all()
        response.success = bool(removed)
        response.message = (
            "removed" if removed else "no matching committed box")
        return response

    def _on_reset(self, _request, response):
        self._mapper.reset(preserve_placed=False)
        with self._lock:
            self._clouds.clear()
            self._integrated.clear()
        self._publish_all()
        response.success = True
        response.message = "reset"
        return response

    def _on_stats(self, _request, response):
        stats = self._mapper.stats()
        response.success = True
        response.unknown_ratio = float(stats["unknown_ratio"])
        response.occupancy_ratio = float(stats["occupancy_ratio"])
        response.occupied_volume = float(stats["occupied_volume"])
        response.free_volume = float(stats["free_volume"])
        response.unknown_count = int(stats["unknown_count"])
        response.free_count = int(stats["free_count"])
        response.occupied_count = int(stats["occupied_count"])
        response.frontier_count = int(stats["frontier_count"])
        response.total_voxels = int(stats["total_voxels"])
        response.map_revision = int(stats["map_revision"])
        response.geometry_hash = self._geometry_hash
        response.message = "committed=%d" % stats["committed_box_count"]
        return response

    @staticmethod
    def _yaw_from_pose(pose):
        import math
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _publish_all(self):
        surface = self._mapper.surface_map_2d()
        self._surface_pub.publish(String(data=json.dumps(
            surface, sort_keys=True)))
        self._ledger_pub.publish(String(data=json.dumps({
            "frame": self._frame,
            "container_in_world": self._world_from_container,
            "boxes": self._mapper.commit_ledger(),
            "map_revision": self._mapper._revision,
            "geometry_hash": self._geometry_hash,
        }, sort_keys=True)))


def main(argv=None):
    rclpy.init(args=argv)
    node = CargoVolumeMapperNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        node.shutdown_tf()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
