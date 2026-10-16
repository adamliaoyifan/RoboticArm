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
import math
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
from luggage_msgs.msg import SurfaceContractV2
from luggage_msgs.srv import (
    AddPlacedBox,
    AddPlacedBoxV2,
    GetCargoMapStats,
    IntegrateCargoView,
    RemovePlacedBox,
    RemovePlacedBoxV2,
    ResetCargoMap,
)

from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import (
    BoundaryContext, BoundaryContractError,
)
from luggage_description.ros_frame_bindings.poses import pose_from_message
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
from luggage_perception.cargo_volume_mapper import (
    CargoVolumeMapper, normalize_ledger_slot,
)
from luggage_perception.frame_message_adapters import measured_surface_to_message
from luggage_perception.cargo_view_integration import (
    MAP_MEASURED,
    MAP_RELEASE_INTEGRATED,
    MAP_STATIC_EMPTY,
    REASON_TF_MISSING,
    REASON_VIEW_EMPTY,
    SCHEMA_VERSION,
    apply_integrate,
    measured_identity,
    release_identity,
    select_fresh_record,
    stamp_key,
    stamp_to_sec,
    validate_integrate_request,
)
from luggage_perception.occupancy_confirmation import PendingConfirmation


#: Declared measured-commit provenance vocabulary for AddPlacedBoxV2 (F19).
#: A v2 ledger commit is a *measured* statement — the box was placed and
#: measured back. ``verify_placed_box`` is the VerifyPlacedBox readback
#: producer; anything else (notably ``planned``) is an intent, not a
#: measurement, and must never enter the ledger as verified geometry.
V2_MEASURED_SOURCES = ("verify_placed_box",)

#: Correlation identity the mapper stamps on every v2 surface publication
#: (one per map revision); v2 subscribers can verify it without latest-TF.
SURFACE_V2_REQUEST_TEMPLATE = "map-rev-%d"


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
        self.declare_parameter("container_depth_topic", "/luggage/perception/container_depth_points")
        self.declare_parameter("base_frame", "container_link")
        self.declare_parameter("cloud_buffer_size", 12)
        self.declare_parameter("freshness_window_sec", 0.20)
        self.declare_parameter("max_raycast_points", 2000)
        self.declare_parameter("integrate_min_z_above_floor", 0.05)
        # Opt-in: a release/removal/pose change leaves the map pending until
        # an IntegrateCargoView confirms it. Off keeps the earlier commit
        # behaviour for flows that never integrate a view.
        self.declare_parameter("require_confirmed_map", False)
        self._require_confirmed_map = bool(
            self.get_parameter("require_confirmed_map").value)
        # ST04 F06/F19: v2 surface publication and ledger services are
        # enabled only by an explicit scene revision; unset leaves the node
        # byte-identical to its v1 behavior.
        self.declare_parameter("frame_scene_revision", "")
        self.declare_parameter("surface_v2_topic", "/luggage/cargo_map/surface_v2")

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
        self._depth_clouds = OrderedDict()
        self._view_integration_lock = threading.Lock()
        self._integrated = set()
        #: Empty-container identity until a settled view or a placed-box
        #: commit. Stamp 0 is legal only while this stays static_empty,
        #: placed_count is 0, and no view has been integrated.
        self._last_acquisition_ns = 0
        self._map_source = MAP_STATIC_EMPTY
        self._cargo_observed = False
        self._pending = PendingConfirmation(self._require_confirmed_map)
        self._pose_at_confirm = None

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

        self.create_subscription(
            PointCloud2, str(self.get_parameter("container_depth_topic").value),
            lambda msg: self._on_cloud(msg, depth_view=True), sensor_qos, callback_group=group)
        self.create_service(
            IntegrateCargoView, "/cargo_map/integrate_container_depth_view",
            self._on_integrate_depth, callback_group=group)

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

        self._frame_scene_revision = str(
            self.get_parameter("frame_scene_revision").value or "")
        self._surface_v2_pub = None
        if self._frame_scene_revision:
            self._surface_v2_pub = self.create_publisher(
                SurfaceContractV2,
                str(self.get_parameter("surface_v2_topic").value), transient)
            self.create_service(
                AddPlacedBoxV2, "/cargo_map/add_placed_box_v2",
                self._on_add_v2, callback_group=group)
            self.create_service(
                RemovePlacedBoxV2, "/cargo_map/remove_placed_box_v2",
                self._on_remove_v2, callback_group=group)

        # ST09 F21: identity heartbeat + peer readiness. Map commits (v1
        # and v2) and surface publications are gated; off = legacy.
        self.declare_parameter("frame_runtime", False)
        self.declare_parameter("frame_identity_json", "")
        self.declare_parameter("frame_peers", "")
        self._frame_runtime = None
        if bool(self.get_parameter("frame_runtime").value):
            import sys as _sys
            from luggage_description.ros_frame_bindings.runtime import (
                node_runtime_from_params)
            self._frame_runtime = node_runtime_from_params(
                self,
                resolved_config={
                    "frame_scene_revision": self._frame_scene_revision,
                    "cargo_cloud_topic": str(
                        self.get_parameter("cargo_cloud_topic").value or ""),
                    "base_frame": str(
                        self.get_parameter("base_frame").value or ""),
                },
                modules=[(_sys.modules["__main__"], "module", "application")],
                package_names=["luggage_perception"])

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

    def _on_cloud(self, msg, *, depth_view=False):
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
            buffer = self._depth_clouds if depth_view else self._clouds
            buffer[key] = record
            while len(buffer) > self._buffer_size:
                buffer.popitem(last=False)

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

    def _gate_mutation(self, response, request_id="", endpoint=None):
        """Admission for a map commit; False fills the refusal."""
        runtime = getattr(self, "_frame_runtime", None)
        if runtime is None:
            return True
        from luggage_description.frame_contracts import ContractError
        try:
            if endpoint is None:
                runtime.authorize_legacy_effect()
            else:
                admission = runtime.admit_request(
                    request_id, endpoint=endpoint)
                runtime.authorize_effect(admission, request_id)
        except ContractError as error:
            response.success = False
            response.message = "%s: %s" % (error.code, error.field)
            return False
        return True

    def _on_integrate(self, request, response):
        with self._view_integration_lock:
            return self._integrate_view(request, response, depth_view=False)

    def _on_integrate_depth(self, request, response):
        with self._view_integration_lock:
            return self._integrate_view(request, response, depth_view=True)

    def _integrate_view(self, request, response, *, depth_view):
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(response):
            return response
        stats = self._mapper.stats()
        current_rev = int(stats["map_revision"])
        reason = validate_integrate_request(
            request.settled, request.schema_version, request.geometry_hash,
            request.expected_map_revision, self._geometry_hash, current_rev)
        if reason:
            return self._fail_integrate(
                response, reason, reason, stats)

        # The integration answers only for changes it began after: a box
        # placed while this view is being integrated stays pending.
        token_at_start = self._pending.begin()
        with self._lock:
            records = list((self._depth_clouds if depth_view else self._clouds).values())
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
        if depth_view:
            from luggage_perception.depth_view_rays import integrate_depth_acquisition
            result = integrate_depth_acquisition(self._mapper, points_base, trans)
        else:
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

        sensor_ns = (
            int(record["stamp"].sec) * 1_000_000_000
            + int(record["stamp"].nanosec))
        stamp_ns, source = measured_identity(sensor_ns)
        with self._lock:
            self._integrated.add(tuple(record["stamp_key"]))
            self._last_acquisition_ns = int(stamp_ns)
            self._map_source = source
            self._cargo_observed = True
            self._pending.settle(
                token_at_start, stamp_ns, result["observed_points"],
                bool(request.settled))
            self._pose_at_confirm = dict(self._world_from_container)
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
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(response):
            return response
        slot = request.slot
        size = [max(0.0, float(slot.width)),
                max(0.0, float(slot.depth)),
                max(0.0, float(slot.height))]
        try:
            # F19: the v1 world-pose shape normalizes through the single
            # shared ledger entry (declared v1 shim) — one conversion, never
            # _to_container plus a second normalization.
            center, yaw_c, size = normalize_ledger_slot(
                [slot.place_pose.position.x, slot.place_pose.position.y,
                 slot.place_pose.position.z],
                self._yaw_from_pose(slot.place_pose), size,
                source_frame="world",
                container_yaw=float(
                    self._world_from_container["rotation_rpy"][2]),
                container_translation=self._world_from_container["translation"])
            added = self._mapper.mark_placed_box(center, size, yaw=yaw_c)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "mark failed: %s" % exc
            return response
        self._adopt_release(
            committed_now=bool(added), footprint=(center, size, yaw_c))
        self._publish_all()
        response.success = True
        response.message = "%s %d boxes (rev %d)" % (
            "committed" if added else "already committed",
            len(self._mapper.commit_ledger()), self._mapper._revision)
        return response

    def _on_remove(self, request, response):
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(response):
            return response
        slot = request.slot
        size = [max(0.0, float(slot.width)),
                max(0.0, float(slot.depth)),
                max(0.0, float(slot.height))]
        try:
            center, yaw_c, size = normalize_ledger_slot(
                [slot.place_pose.position.x, slot.place_pose.position.y,
                 slot.place_pose.position.z],
                self._yaw_from_pose(slot.place_pose), size,
                source_frame="world",
                container_yaw=float(
                    self._world_from_container["rotation_rpy"][2]),
                container_translation=self._world_from_container["translation"])
            removed = self._mapper.unmark_placed_box(center, size, yaw=yaw_c)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "unmark failed: %s" % exc
            return response
        if removed:
            self._adopt_after_removal()
        self._publish_all()
        response.success = bool(removed)
        response.message = (
            "removed" if removed else "no matching committed box")
        return response

    def _on_reset(self, _request, response):
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(response):
            return response
        self._mapper.reset(preserve_placed=False)
        with self._lock:
            self._clouds.clear()
            self._depth_clouds.clear()
            self._integrated.clear()
            self._last_acquisition_ns = 0
            self._map_source = MAP_STATIC_EMPTY
            self._cargo_observed = False
            self._pending.clear()
            self._pose_at_confirm = None
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
        response.last_acquisition_stamp_ns = int(self._last_acquisition_ns)
        response.map_source = str(self._map_source)
        response.placed_count = int(stats["committed_box_count"])
        response.cargo_observed = bool(self._cargo_observed)
        response.message = "committed=%d" % stats["committed_box_count"]
        return response

    @staticmethod
    def _yaw_from_pose(pose):
        import math
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _node_clock_ns(self):
        stamp = self.get_clock().now().to_msg()
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _adopt_release(self, committed_now, footprint):
        """A placed box leaves the static-empty identity.

        ``require_confirmed_map`` on: a new commit records a pending change
        whose ``stamp_ns`` is the commit clock. It does not write that clock
        into the acquisition stamp. A map with no sensor acquisition yet
        keeps stamp <= 0; placement stays closed, and an unloaded observation
        move is admitted separately without a forged acquisition time. Off:
        a new commit stamps the node clock and a duplicate commit keeps the
        existing release stamp.
        """
        if len(self._mapper.commit_ledger()) <= 0:
            return
        previous_source = self._map_source
        now_ns = self._node_clock_ns()
        stamp, source = release_identity(now_ns)
        self._map_source = source
        if self._require_confirmed_map:
            if committed_now:
                self._pending.record("box_placed", now_ns, footprint)
            return
        if committed_now or int(self._last_acquisition_ns) <= 0 \
                or previous_source != MAP_RELEASE_INTEGRATED:
            self._last_acquisition_ns = int(stamp)

    def _adopt_after_removal(self):
        placed = len(self._mapper.commit_ledger())
        self._pending.record("box_removed", self._node_clock_ns())
        if placed > 0:
            self._map_source = MAP_RELEASE_INTEGRATED
            return
        if self._cargo_observed:
            self._map_source = MAP_MEASURED
            return
        self._map_source = MAP_STATIC_EMPTY
        self._last_acquisition_ns = 0
        self._pending.clear()

    def update_container_pose(self, pose):
        """Invalidate when the container itself has moved since confirmation."""
        from luggage_perception.cargo_view_integration import (
            container_pose_moved,
        )

        if container_pose_moved(self._pose_at_confirm, pose):
            self._pending.record(
                "container_pose_changed", self._node_clock_ns())
        self._world_from_container = {
            "translation": list(pose.get("translation") or []),
            "rotation_rpy": list(pose.get("rotation_rpy") or []),
        }

    def note_occupancy_event(self, event):
        """Record a cargo change that the mapper did not observe itself.

        ``box_disturbed`` and ``cargo_edited`` have no automatic detector.
        Callers that see them use this. Return, pickup spawn, grasp, and
        planning are ignored.
        """
        self._pending.record(event, self._node_clock_ns())

    def _publish_all(self):
        surface = self._mapper.surface_map_2d()
        surface["acquisition_stamp_ns"] = int(self._last_acquisition_ns)
        surface["map_source"] = str(self._map_source)
        surface["placed_count"] = len(self._mapper.commit_ledger())
        surface["cargo_observed"] = bool(self._cargo_observed)
        surface.update(self._pending.surface_fields())
        self._surface_pub.publish(String(data=json.dumps(
            surface, sort_keys=True)))
        self._ledger_pub.publish(String(data=json.dumps({
            "frame": self._frame,
            "container_in_world": self._world_from_container,
            "boxes": self._mapper.commit_ledger(),
            "map_revision": self._mapper._revision,
            "geometry_hash": self._geometry_hash,
        }, sort_keys=True)))
        # ST04 F06: scene-bound v2 surface beside the byte-stable v1 JSON.
        if self._surface_v2_pub is not None:
            try:
                record = self._mapper.surface_contract_v2(
                    self._frame_scene_revision,
                    SURFACE_V2_REQUEST_TEMPLATE % int(self._mapper._revision))
                self._surface_v2_pub.publish(
                    measured_surface_to_message(record, self._surface_context()))
            except BoundaryContractError as error:
                self.get_logger().warning(
                    "surface v2 rejected (%s %s)" % (error.code, error.field))
            except Exception as error:  # noqa: BLE001 - never break v1
                self.get_logger().warning(
                    "surface v2 emit failed: %s" % error)

    def _surface_context(self):
        return BoundaryContext(
            "F06", "cargo_volume_mapper_node", "surface_v2_subscribers",
            SURFACE_V2_REQUEST_TEMPLATE % int(self._mapper._revision),
            self._frame_scene_revision, "container_link", "",
            "scene", geometry_hash=self._geometry_hash,
            map_revision=int(self._mapper._revision))

    def _decode_slot_v2(self, slot_pose, request_id):
        """Decode one v2 slot record (F19) with explicit consumer identity."""
        context = BoundaryContext(
            "F19", "v2_client", "cargo_volume_mapper_node",
            str(request_id), self._frame_scene_revision,
            "container_link", "box_center", "scene",
            geometry_hash=self._geometry_hash,
            map_revision=None)
        return pose_from_message(slot_pose, "slot", context)

    def _ledger_guard(self, expected_map_revision, geometry_hash,
                      response=None):
        """Reject stale revision / absent or foreign hull before any mutation.

        The v2 boundary is measured-commit-only: the hull identity is a
        required, nonempty field — an absent ``geometry_hash`` is a missing
        identity, not an unconstrained commit. (v1 callbacks do not pass
        through this guard; their legacy semantics are unchanged.)
        """
        stats = self._mapper.stats()
        if int(expected_map_revision) != int(stats["map_revision"]):
            if response is not None:
                response.success = False
                response.message = "stale map revision %s != %s" % (
                    expected_map_revision, stats["map_revision"])
            return False
        incoming = str(geometry_hash or "")
        if not incoming:
            if response is not None:
                response.success = False
                response.message = "geometry_hash required for v2 commits"
            return False
        if incoming != self._geometry_hash:
            if response is not None:
                response.success = False
                response.message = "geometry hash mismatch"
            return False
        return True

    def _on_add_v2(self, request, response):
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(
                response,
                request_id=str(request.slot_pose.identity.request_id),
                endpoint="/cargo_map/add_placed_box_v2"):
            return response
        try:
            slot = self._decode_slot_v2(
                request.slot_pose, request.slot_pose.identity.request_id)
        except ContractError as error:
            response.success = False
            response.message = "slot rejected (%s %s)" % (
                error.code, error.field)
            return response
        if not self._ledger_guard(
                request.expected_map_revision, request.geometry_hash,
                response):
            return response
        measured_source = str(request.measured_source or "")
        if measured_source not in V2_MEASURED_SOURCES:
            # The v2 ledger records measured commits only: an unmeasured
            # intent ("planned") must not become verified geometry.
            response.success = False
            response.message = (
                "measured_source %r is not a declared measured provenance "
                "(one of %s)" % (measured_source, list(V2_MEASURED_SOURCES)))
            return response
        yaw = 2.0 * math.atan2(slot["quaternion_xyzw"][2],
                                slot["quaternion_xyzw"][3])
        try:
            # v2 slot records are already container_link: the shared
            # normalization applies no second yaw subtraction.
            center, yaw_c, size = normalize_ledger_slot(
                slot["position"], yaw, list(request.size_wdh),
                source_frame="container_link")
            added = self._mapper.mark_placed_box(
                center, size, yaw=yaw_c, measured_source=measured_source)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "mark failed: %s" % exc
            return response
        self._adopt_release(
            committed_now=bool(added), footprint=(center, size, yaw_c))
        self._publish_all()
        response.success = True
        response.message = "%s %d boxes (rev %d)" % (
            "committed" if added else "already committed",
            len(self._mapper.commit_ledger()), self._mapper._revision)
        response.resulting_map_revision = int(self._mapper._revision)
        return response

    def _on_remove_v2(self, request, response):
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(
                response,
                request_id=str(request.slot_pose.identity.request_id),
                endpoint="/cargo_map/remove_placed_box_v2"):
            return response
        try:
            slot = self._decode_slot_v2(
                request.slot_pose, request.slot_pose.identity.request_id)
        except ContractError as error:
            response.success = False
            response.message = "slot rejected (%s %s)" % (
                error.code, error.field)
            return response
        if not self._ledger_guard(
                request.expected_map_revision, request.geometry_hash,
                response):
            return response
        yaw = 2.0 * math.atan2(slot["quaternion_xyzw"][2],
                                slot["quaternion_xyzw"][3])
        try:
            center, yaw_c, size = normalize_ledger_slot(
                slot["position"], yaw, list(request.size_wdh),
                source_frame="container_link")
            removed = self._mapper.unmark_placed_box(center, size, yaw=yaw_c)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "unmark failed: %s" % exc
            return response
        if removed:
            self._adopt_after_removal()
        self._publish_all()
        response.success = bool(removed)
        response.message = (
            "removed" if removed else "no matching committed box")
        response.resulting_map_revision = int(self._mapper._revision)
        return response


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
