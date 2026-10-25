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
import os
import sys
import threading
import time
from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
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
    GetCargoVolumeV2,
    IntegrateCargoView,
    NotifyOccupancyEvent,
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
    REASON_TF_MISSING,
    SCHEMA_VERSION,
    measured_identity,
    select_fresh_record,
    stamp_key,
    stamp_to_sec,
    validate_integrate_request,
)
from luggage_perception.occupancy_map_store import (
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
    UpdateStatus,
)


#: [B1] These two constants moved into occupancy_map_store (the server-side
#: gate); re-exported here so existing references resolve unchanged.
from luggage_perception.occupancy_map_store import (  # noqa: E402,F401
    MAP_STATIC_EMPTY,
    V2_MEASURED_SOURCES,
    VERIFY_ANCHOR_TOLERANCE_MAX_M,
)

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
        # All map services serialize mutations and snapshot export together.
        map_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        from luggage_description.cargo_grid_parameters import declare_resolution
        grid_resolution = declare_resolution(self)
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
            resolution=grid_resolution,
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
        # [B1] The store is the sole transactional owner of the map: the
        # service handlers below are adapters that build MapUpdate records
        # and call apply_update. ``_pending`` bookkeeping, the integrated
        # stamp set, and the acquisition identity moved under its
        # transaction lock (they used to be split across this node).
        self._store = OccupancyMapStore(
            self._mapper,
            require_confirmation=self._require_confirmed_map,
            clock_ns=self._node_clock_ns)
        self._pending = self._store.pending
        # [integration-prior-g1 merge] Node-side I/O state the merged
        # body still uses: view serialization + the acquisition identity
        # mirrors (the STORE stays the sole map-state authority).
        self._view_integration_lock = threading.Lock()
        self._map_source = MAP_STATIC_EMPTY
        self._last_acquisition_ns = 0

        def _sync_acquisition_identity(_snapshot=None):
            stats = self._store.v1_surface_dict()
            self._map_source = str(stats.get("map_source")
                                   or MAP_STATIC_EMPTY)
            self._last_acquisition_ns = int(
                stats.get("acquisition_stamp_ns") or 0)
        self._sync_acquisition_identity = _sync_acquisition_identity
        _sync_acquisition_identity()

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

        # [integration-prior-g1 merge] The prior branch's depth-view
        # acquisition channel (/cargo_map/integrate_container_depth_view
        # + container_depth_topic buffer) is PARKED: it wrote the mapper
        # directly (integrate_depth_acquisition), bypassing the
        # OccupancyMapStore. Re-entering requires an apply_update
        # INTEGRATE wrapper; until then single-authority wins.

        self.create_service(
            AddPlacedBox, "/cargo_map/add_placed_box",
            self._on_add, callback_group=map_group)
        self.create_service(
            RemovePlacedBox, "/cargo_map/remove_placed_box",
            self._on_remove, callback_group=map_group)
        self.create_service(
            ResetCargoMap, "/cargo_map/reset",
            self._on_reset, callback_group=map_group)
        self.create_service(
            GetCargoMapStats, "/cargo_map/get_stats",
            self._on_stats, callback_group=map_group)
        self.create_service(
            IntegrateCargoView, "/cargo_map/integrate_cargo_view",
            self._on_integrate, callback_group=group)
        # [release-failure contract] World-side occupancy facts (the
        # place driver's vacuum-off box_released) enter the map here —
        # before any acceptance decision.
        self.create_service(
            NotifyOccupancyEvent, "/cargo_map/note_occupancy_event",
            self._on_note_event, callback_group=group)

        self._frame_scene_revision = str(
            self.get_parameter("frame_scene_revision").value or "")
        self._surface_v2_pub = None
        if self._frame_scene_revision:
            self._surface_v2_pub = self.create_publisher(
                SurfaceContractV2,
                str(self.get_parameter("surface_v2_topic").value), transient)
            self.create_service(
                AddPlacedBoxV2, "/cargo_map/add_placed_box_v2",
                self._on_add_v2, callback_group=map_group)
            self.create_service(
                RemovePlacedBoxV2, "/cargo_map/remove_placed_box_v2",
                self._on_remove_v2, callback_group=map_group)

        self.create_service(
            GetCargoVolumeV2, "/cargo_map/get_volume_v2",
            self._on_volume, callback_group=map_group)

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
            buffer = self._clouds  # depth_view channel parked (see note above)
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
                    request_id, endpoint=endpoint,
                    scope=("occupancy_commit" if endpoint == "/cargo_map/add_placed_box_v2"
                           else None))
                runtime.authorize_effect(admission, request_id)
        except ContractError as error:
            response.success = False
            response.message = "%s: %s" % (error.code, error.field)
            return False
        return True

    def _on_integrate(self, request, response):
        with self._view_integration_lock:
            return self._integrate_view(request, response)

    def _integrate_view(self, request, response):
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(response):
            return response
        stats = self._mapper.occupancy_summary()
        current_rev = int(stats["map_revision"])
        # Early-out on doomed requests (the store re-validates
        # authoritatively inside the transaction).
        reason = validate_integrate_request(
            request.settled, request.schema_version, request.geometry_hash,
            request.expected_map_revision, self._geometry_hash, current_rev)
        if reason:
            return self._fail_integrate(
                response, reason, reason, stats)

        with self._lock:
            records = list(self._clouds.values())
        record, reason = select_fresh_record(
            records, stamp_to_sec(request.source_acquisition_stamp),
            self._freshness_window_sec, set())
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
        sensor_ns = (
            int(record["stamp"].sec) * 1_000_000_000
            + int(record["stamp"].nanosec))
        acquisition_ns, _source = measured_identity(sensor_ns)
        result = self._store.apply_update(MapUpdate(
            operation_id=str(request.operation_id or "")
            or "integrate:%s:%r" % (request.view_request_id,
                                    tuple(record["stamp_key"])),
            kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=SnapshotIdentity(
                str(request.expected_epoch or ""),
                int(request.expected_map_revision),
                str(request.expected_content_digest or "")),
            geometry_hash=str(request.geometry_hash or ""),
            points=points_base, origin=trans, minimum_z=zmin,
            stamp_key=tuple(record["stamp_key"]),
            acquisition_stamp_ns=int(acquisition_ns),
            settled=bool(request.settled),
            schema_version=int(request.schema_version),
            pose_at_confirm=dict(self._world_from_container),
        ))
        response.status = result.status.value
        response.epoch = result.identity.epoch
        response.content_digest = result.identity.content_digest
        if not result.accepted:
            reason = result.reason or (
                "ALREADY_PROCESSED"
                if result.status is UpdateStatus.ALREADY_PROCESSED
                else result.status.value)
            return self._fail_integrate(
                response, reason,
                str(result.details.get("message") or reason),
                self._mapper.occupancy_summary())
        details = result.details
        snapshot = result.snapshot
        self._publish_all(snapshot)
        stats = self._mapper.occupancy_summary()
        response.schema_version = SCHEMA_VERSION
        response.success = True
        response.reason_code = ""
        response.message = (
            "integrated n_finite=%s n_kept=%s latency_ms=%.1f view=%s"
            % (details.get("n_finite"), details.get("n_kept"),
               1000.0 * float(details.get("latency_sec") or 0.0),
               request.view_request_id))
        self.get_logger().info("OCC_PREPROCESS " + json.dumps({
            "view": request.view_request_id,
            "counts": details.get("preprocess_counts", {}),
            "timings": details.get("preprocess_timings", {}),
        }, sort_keys=True))
        response.unknown_ratio = float(stats["unknown_ratio"])
        response.occupancy_ratio = float(stats["occupancy_ratio"])
        response.resulting_map_revision = int(snapshot.identity.revision)
        response.integrated_acquisition_stamp = record["stamp"]
        return response

    def _on_note_event(self, request, response):
        """[release-failure contract] Adapter for world-side occupancy
        facts: build a NOTE_EVENT MapUpdate (footprint-carrying when the
        caller supplies box geometry) and run it through the one
        transaction. ``box_released`` at vacuum-off is the primary
        producer."""
        event = str(request.event or "")
        center = [float(v) for v in request.box_center_container]
        size = [float(v) for v in request.box_size_wdh]
        has_box = bool(event) and all(
            v == v and abs(v) != float("inf") for v in center + size) \
            and all(v > 0.0 for v in size)
        result = self._store.apply_update(MapUpdate(
            operation_id="event:%s:%s:%d" % (
                event, str(request.source or "external"), self._node_clock_ns()),
            kind=UpdateKind.NOTE_EVENT,
            event=event,
            box_id=str(getattr(request, "box_id", "") or ""),
            box=((center, size, float(request.box_yaw))
                 if has_box else None)))
        response.status = result.status.value
        response.reason = result.reason or (
            "" if result.accepted else result.status.value)
        response.epoch = result.identity.epoch
        response.content_digest = result.identity.content_digest
        response.occupancy_confirmed = bool(
            result.snapshot.occupancy_confirmed)
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
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "mark failed: %s" % exc
            return response
        result = self._store.apply_update(MapUpdate(
            operation_id=self._v1_operation_id(
                "add_placed_box", center, size, yaw_c),
            kind=UpdateKind.ADD_BOX_V1,
            box=(list(center), list(size), float(yaw_c))))
        self._publish_all(result.snapshot)
        # v1 semantics: a retried add reports success while the box is in
        # the ledger (the map, not the call count, is the contract).
        response.success = (result.accepted or result.status
                            is UpdateStatus.ALREADY_PROCESSED)
        response.message = "%s %d boxes (rev %d)" % (
            "committed" if result.details.get("committed_now")
            else "already committed",
            len(self._mapper.commit_ledger()),
            result.identity.revision)
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
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "unmark failed: %s" % exc
            return response
        result = self._store.apply_update(MapUpdate(
            operation_id=self._v1_operation_id(
                "remove_placed_box", center, size, yaw_c),
            kind=UpdateKind.REMOVE_BOX_V1,
            box=(list(center), list(size), float(yaw_c))))
        self._publish_all(result.snapshot)
        response.success = result.accepted
        response.message = (
            "removed" if result.details.get("removed")
            else "no matching committed box")
        return response

    @staticmethod
    def _v1_operation_id(endpoint, center, size, yaw):
        """v1 calls carry no idempotency key: derive one from the
        normalized content so a literal retry maps to one transaction."""
        from st09_contracts._base import digest_of
        return "v1:%s:%s" % (endpoint, digest_of(
            tuple(float(v) for v in center),
            tuple(float(v) for v in size), float(yaw)))

    def _on_reset(self, _request, response):
        _gate = getattr(self, "_gate_mutation", None)
        if _gate is not None and not _gate(response):
            return response
        result = self._store.apply_update(
            MapUpdate(operation_id="reset:%d" % self._node_clock_ns(),
                      kind=UpdateKind.RESET))
        with self._lock:
            self._clouds.clear()
        self._publish_all(result.snapshot)
        response.success = result.accepted
        response.message = "reset"
        response.resulting_map_revision = int(result.identity.revision)
        response.status = result.status.value
        response.epoch = result.identity.epoch
        response.content_digest = result.identity.content_digest
        return response

    def _on_volume(self, request, response):
        """Read only; same callback group as every owner map mutation.

        [integration-prior-g1 2026-10-19] The snapshot identity is the
        STORE's (epoch + revision + content_digest) — the same pair
        Select's admission pins. Raw mapper revision reads and init-only
        acquisition mirrors are gone: the identity and the acquisition
        provenance come from the store's own projection on EVERY read,
        so a volume can never be served across a snapshot boundary.
        """
        from luggage_perception.cargo_volume_snapshot import copy_volume
        response.success = False
        snapshot = self._store.snapshot()
        identity = snapshot.identity
        self._sync_acquisition_identity()
        if (not self._frame_scene_revision or
                request.scene_revision != self._frame_scene_revision or
                request.geometry_hash != self._geometry_hash or
                request.expected_map_revision != identity.revision):
            response.reason = "SNAPSHOT_IDENTITY_MISMATCH"
            return response
        if ((request.expected_epoch and
                request.expected_epoch != identity.epoch) or
                (request.expected_content_digest and
                 request.expected_content_digest != identity.content_digest)):
            response.reason = "SNAPSHOT_IDENTITY_MISMATCH"
            return response
        confirmation = self._pending.surface_fields()
        if (self._frame != "container_link" or self._map_source != "measured" or
                self._last_acquisition_ns <= 0 or
                not confirmation["occupancy_confirmed"] or
                confirmation["pending_changes"] or
                (self._mapper.commit_ledger() and not self._require_confirmed_map)):
            response.reason = "NO_CONFIRMED_MEASURED_VOLUME"
            return response
        try:
            volume = copy_volume(self._mapper)
            for name, value in volume.items():
                setattr(response, name, list(value) if name in (
                    "states", "active", "dimensions_xyz", "origin_container") else value)
        except ValueError as error:
            response.reason = str(error)
            return response
        response.scene_revision = self._frame_scene_revision
        response.geometry_hash = self._geometry_hash
        response.map_revision = identity.revision
        response.epoch = identity.epoch
        response.content_digest = identity.content_digest
        response.frame_id = self._frame
        response.acquisition_stamp_ns = self._last_acquisition_ns
        response.map_source = self._map_source
        response.occupancy_confirmed = True
        response.has_pending_changes = False
        response.success = True
        response.reason = "OK"
        return response

    def _on_stats(self, _request, response):
        # [ADMISSION-DIAG 2026-10-25, env-gated] recv/compute-end times on
        # this process's monotonic clock. Zero effect unless
        # ELFIN_MAP_STATS_TRACE is set.
        _trace = bool(os.environ.get("ELFIN_MAP_STATS_TRACE"))
        _t_recv = time.monotonic() if _trace else None
        stats = self._mapper.stats()
        if _trace:
            print("[map-stats-trace] recv=%.6f compute_end=%.6f "
                  "compute_s=%.6f" % (
                      _t_recv, time.monotonic(),
                      time.monotonic() - _t_recv), file=sys.stderr,
                  flush=True)
        snapshot = self._store.snapshot()
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
        response.last_acquisition_stamp_ns = int(
            snapshot.acquisition_stamp_ns)
        response.map_source = str(snapshot.map_source)
        response.placed_count = int(stats["committed_box_count"])
        response.cargo_observed = bool(snapshot.cargo_observed)
        response.epoch = snapshot.identity.epoch
        response.content_digest = snapshot.identity.content_digest
        response.occupancy_confirmed = bool(snapshot.occupancy_confirmed)
        response.change_event = str(snapshot.change_event)
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

    def update_container_pose(self, pose):
        """Invalidate when the container itself has moved since confirmation."""
        from luggage_perception.cargo_view_integration import (
            container_pose_moved,
        )

        if container_pose_moved(self._store.pose_at_confirm, pose):
            self._store.apply_update(MapUpdate(
                operation_id="pose:%d" % self._node_clock_ns(),
                kind=UpdateKind.CONTAINER_POSE))
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
        self._store.apply_update(MapUpdate(
            operation_id="note:%s:%d" % (event, self._node_clock_ns()),
            kind=UpdateKind.NOTE_EVENT, event=str(event)))

    def _publish_all(self, snapshot=None):
        # [B1] Single-snapshot publish: v1 JSON, committed ledger, and the
        # v2 contract are deterministic projections of ONE immutable
        # snapshot handed in by the transaction that just ran (or the
        # current one on read-only paths) — the SAME object is passed to
        # every projector, so a concurrent transaction between two
        # projections cannot produce a mixed-revision publication set.
        snapshot = snapshot if snapshot is not None \
            else self._store.snapshot()
        self._surface_pub.publish(String(data=json.dumps(
            self._store.v1_surface_dict(snapshot), sort_keys=True)))
        self._ledger_pub.publish(String(data=json.dumps(
            self._store.committed_dict(
                self._frame, self._world_from_container, snapshot),
            sort_keys=True)))
        # ST04 F06: scene-bound v2 surface beside the byte-stable v1 JSON.
        if self._surface_v2_pub is not None:
            try:
                record = self._store.v2_record(
                    self._frame_scene_revision,
                    SURFACE_V2_REQUEST_TEMPLATE % snapshot.identity.revision,
                    snapshot)
                self._surface_v2_pub.publish(
                    measured_surface_to_message(record, self._surface_context()))
            except BoundaryContractError as error:
                self.get_logger().warning(
                    "surface v2 rejected (%s %s)" % (error.code, error.field))
            except Exception as error:  # noqa: BLE001 - never break v1
                self.get_logger().warning(
                    "surface v2 emit failed: %s" % error)

    def _surface_context(self):
        revision = self._store.map_revision
        return BoundaryContext(
            "F06", "cargo_volume_mapper_node", "surface_v2_subscribers",
            SURFACE_V2_REQUEST_TEMPLATE % int(revision),
            self._frame_scene_revision, "container_link", "",
            "scene", geometry_hash=self._geometry_hash,
            map_revision=int(revision))

    def _decode_slot_v2(self, slot_pose, request_id):
        """Decode one v2 slot record (F19) with explicit consumer identity."""
        context = BoundaryContext(
            "F19", "v2_client", "cargo_volume_mapper_node",
            str(request_id), self._frame_scene_revision,
            "container_link", "box_center", "scene",
            geometry_hash=self._geometry_hash,
            map_revision=None)
        return pose_from_message(slot_pose, "slot", context)

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
        if not str(request.geometry_hash or ""):
            # The v2 boundary is measured-commit-only: the hull identity is
            # a required, nonempty field (equality is checked inside the
            # transaction against the store's premise).
            response.success = False
            response.message = "geometry_hash required for v2 commits"
            return response
        yaw = 2.0 * math.atan2(slot["quaternion_xyzw"][2],
                                slot["quaternion_xyzw"][3])
        try:
            # v2 slot records are already container_link: the shared
            # normalization applies no second yaw subtraction.
            center, yaw_c, size = normalize_ledger_slot(
                slot["position"], yaw, list(request.size_wdh),
                source_frame="container_link")
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "mark failed: %s" % exc
            return response
        # [B1 adapter] Everything below — premise (revision/epoch/digest/
        # hull), the verify-anchor evidence gate (finite/non-negative/
        # ceiling tolerance, slot-vs-measured delta, measured provenance),
        # the commit itself, and the pending-change adoption — runs inside
        # the store's one transaction.
        result = self._store.apply_update(MapUpdate(
            operation_id=str(request.operation_id or "")
            or "addv2:%s" % request.slot_pose.identity.request_id,
            kind=UpdateKind.ADD_BOX_V2,
            expected_snapshot=SnapshotIdentity(
                str(request.expected_epoch or ""),
                int(request.expected_map_revision),
                str(request.expected_content_digest or "")),
            geometry_hash=str(request.geometry_hash or ""),
            box=(list(center), list(size), float(yaw_c)),
            box_id=str(getattr(request, "box_id", "") or ""),
            measured_source=str(request.measured_source or ""),
            verified_center=(
                None if float(request.verify_tolerance_m or 0.0) <= 0.0
                else [float(v) for v in
                      request.verified_center_container]),
            verify_tolerance_m=float(request.verify_tolerance_m or 0.0),
            verify_source=str(getattr(request, "verify_source", "")
                              or "")))
        self._finish_v2(response, result, expect="committed")
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
        if not str(request.geometry_hash or ""):
            response.success = False
            response.message = "geometry_hash required for v2 commits"
            return response
        yaw = 2.0 * math.atan2(slot["quaternion_xyzw"][2],
                                slot["quaternion_xyzw"][3])
        try:
            center, yaw_c, size = normalize_ledger_slot(
                slot["position"], yaw, list(request.size_wdh),
                source_frame="container_link")
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "unmark failed: %s" % exc
            return response
        result = self._store.apply_update(MapUpdate(
            operation_id=str(request.operation_id or "")
            or "removev2:%s" % request.slot_pose.identity.request_id,
            kind=UpdateKind.REMOVE_BOX_V2,
            expected_snapshot=SnapshotIdentity(
                str(request.expected_epoch or ""),
                int(request.expected_map_revision),
                str(request.expected_content_digest or "")),
            geometry_hash=str(request.geometry_hash or ""),
            box=(list(center), list(size), float(yaw_c))))
        self._finish_v2(response, result, expect="removed")
        return response

    def _finish_v2(self, response, result, expect):
        """Map one transaction result onto a v2 service response."""
        response.status = result.status.value
        response.reason = result.reason
        response.epoch = result.identity.epoch
        response.content_digest = result.identity.content_digest
        response.resulting_map_revision = int(result.identity.revision)
        if result.status is UpdateStatus.RECORDED_PENDING:
            response.success = False
            response.message = (
                "late fact recorded pending (%s); re-read identity %s"
                % (result.reason, result.identity))
            self._publish_all(result.snapshot)
            return
        if not result.accepted and result.status is not (
                UpdateStatus.ALREADY_PROCESSED):
            response.success = False
            response.message = str(
                result.details.get("message") or result.reason)
            return
        if result.status is UpdateStatus.ALREADY_PROCESSED:
            # Literal retry: the map already reflects this write; report
            # the same headline the original call produced.
            response.success = True
            response.message = "already %s (replayed)" % expect
            self._publish_all(result.snapshot)
            return
        response.success = True
        if expect == "committed":
            response.message = "%s %d boxes (rev %d)" % (
                "committed" if result.details.get("committed_now")
                else "already committed",
                len(self._mapper.commit_ledger()),
                result.identity.revision)
        else:
            response.message = (
                "removed" if result.details.get("removed")
                else "no matching committed box")
            response.success = bool(result.details.get("removed"))
        self._publish_all(result.snapshot)


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
