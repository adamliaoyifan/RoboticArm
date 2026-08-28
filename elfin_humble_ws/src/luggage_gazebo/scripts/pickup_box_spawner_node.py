#!/usr/bin/env python3
"""Runtime spawner for the single fixed-source pickup box (ROS 2).

Port of the noetic pickup_box_spawner_node: samples box size/mass/yaw from
the catalog, spawns (or deletes) the Gazebo model through the ros_gz_bridge
service bridge (``/world/<world>/create`` and ``/world/<world>/remove``),
and serves SpawnNextBox / ClearCurrentBox / FinalizeCurrentBox /
GetCurrentBox.

State that the noetic node kept on the ROS 1 parameter server
(``/luggage/current_box``, ``/luggage/finalized_models``,
``/luggage/perception/size_eval/spawned``) is published as transient-local
JSON topics instead (migration plan section 9).
"""

from __future__ import division

import json
import math
import os
import random
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Pose, Quaternion
from luggage_msgs.msg import DetectedLuggage
from luggage_msgs.srv import (
    ClearCurrentBox,
    FinalizeCurrentBox,
    GetCurrentBox,
    SpawnNextBox,
)
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from std_msgs.msg import String

from luggage_description.box_catalog_utils import (
    box_catalog_entries,
    box_mass_model,
    box_size_range,
    default_box_catalog_path,
    load_box_catalog,
    sample_box_mass,
    sample_box_size,
)
from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    pickup_source_in_world,
    resolve_scene_tf_config_path,
)
from ament_index_python.packages import get_package_share_directory

from luggage_description.suitcase_visual import (
    VISUAL_IDS,
    load_sized_suitcases_manifest,
    pickup_box_pose,
    pickup_visual_sdf,
    size_tier_name,
    sized_model_name,
    visual_id_for_entry,
)

WORLD_NAME = "airport_loading"


def _quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * sy,
    )


class PickupBoxSpawner(Node):

    def __init__(self):
        super().__init__("pickup_box_spawner")
        self._group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("box_catalog_config", "")
        self.declare_parameter("model_prefix", "pickup_box")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("world", WORLD_NAME)
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("yaw_mode", "")
        self.declare_parameter("yaw_range", [0.0, 0.0])
        self.declare_parameter("xy_jitter_range", [0.0, 0.0])
        # catalog: pick small/medium/large. continuous: envelope sample (box
        # visual only; mesh ignores it).
        self.declare_parameter("size_mode", "catalog")
        self.declare_parameter("sequence_ids", [""],
                               descriptor=None)
        # Box visual: ogre2 can lag create. Mesh uses preloaded URIs (0).
        self.declare_parameter("visual_settle_sec", 2.0)
        # box: primitive AABB. mesh: pre-scaled model:// visual=collision.
        self.declare_parameter("visual_kind", "box")
        self.declare_parameter("models_root", "")

        scene_cfg = self.get_parameter("scene_tf_config").value
        scene_cfg_path = scene_cfg or resolve_scene_tf_config_path()
        box_catalog_path = (
            self.get_parameter("box_catalog_config").value
            or default_box_catalog_path()
        )
        self._model_prefix = self.get_parameter("model_prefix").value
        self._world_frame = self.get_parameter("world_frame").value
        seed = self.get_parameter("random_seed").value
        self._rng = random.Random(seed if seed >= 0 else None)
        self._yaw_mode_override = self.get_parameter("yaw_mode").value
        yaw_range = self.get_parameter("yaw_range").value
        self._yaw_range_override = list(yaw_range) if yaw_range else []
        xy_jitter = self.get_parameter("xy_jitter_range").value
        self._xy_jitter = (
            (float(xy_jitter[0]), float(xy_jitter[1]))
            if len(xy_jitter) >= 2 else (0.0, 0.0))
        self._size_mode = str(
            self.get_parameter("size_mode").value).strip().lower() or "catalog"
        self._visual_kind = str(
            self.get_parameter("visual_kind").value).strip().lower() or "box"
        if self._visual_kind == "mesh" and self._size_mode == "continuous":
            self.get_logger().warning(
                "visual_kind=mesh ignores size_mode=continuous; using catalog")
            self._size_mode = "catalog"
        settle_raw = float(self.get_parameter("visual_settle_sec").value)
        self._visual_settle_sec = max(0.0, settle_raw)
        if self._visual_kind == "mesh" and abs(settle_raw - 2.0) < 1e-9:
            # Node default is for box; mesh assets are preloaded.
            self._visual_settle_sec = 0.0
        models_root = str(self.get_parameter("models_root").value).strip()
        if not models_root:
            models_root = os.path.join(
                get_package_share_directory("luggage_gazebo"), "models")
        self._models_root = models_root
        self._sized_manifest = load_sized_suitcases_manifest(self._models_root)
        self._current_box = None
        self._current_model = None
        self._finalized_models = []
        self._sequence = 0

        scene_config = load_scene_tf_config(scene_cfg_path)
        self._catalog_config = load_box_catalog(box_catalog_path, scene_config)
        self._source_xyz, self._source_rpy = pickup_source_in_world(scene_config)
        self._entries = box_catalog_entries(self._catalog_config)
        self._size_range = box_size_range(self._catalog_config)
        self._mass_model = box_mass_model(self._catalog_config)
        self._sequence_ids = [
            str(v) for v in self.get_parameter("sequence_ids").value if str(v)
        ]
        self._entries_by_id = {entry["id"]: entry for entry in self._entries}
        if not self._entries:
            raise RuntimeError("box catalog is empty")

        world = self.get_parameter("world").value
        transient = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._box_pub = self.create_publisher(
            String, "/luggage/current_box", transient)
        self._finalized_pub = self.create_publisher(
            String, "/luggage/finalized_models", transient)
        self._size_eval_pub = self.create_publisher(
            String, "/luggage/perception/size_eval/spawned", transient)

        self._create_cli = self.create_client(
            SpawnEntity, "/world/%s/create" % world, callback_group=self._group)
        self._delete_cli = self.create_client(
            DeleteEntity, "/world/%s/remove" % world, callback_group=self._group)
        self._set_pose_cli = self.create_client(
            SetEntityPose, "/world/%s/set_pose" % world,
            callback_group=self._group)

        self.create_service(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box", self.handle_spawn_next,
            callback_group=self._group)
        self.create_service(
            ClearCurrentBox, "/pickup_box_spawner/clear_current_box", self.handle_clear,
            callback_group=self._group)
        self.create_service(
            FinalizeCurrentBox, "/pickup_box_spawner/finalize_current_box", self.handle_finalize,
            callback_group=self._group)
        self.create_service(
            GetCurrentBox, "/pickup_box_spawner/get_current_box", self.handle_get_current,
            callback_group=self._group)

        self._publish_box_state()
        self.get_logger().info(
            "pickup_box_spawner ready (visual_kind=%s size_mode=%s)"
            % (self._visual_kind, self._size_mode))

    # ------------------------------------------------------------------
    # gz service helpers (blocking on a reentrant group)
    # ------------------------------------------------------------------

    def _call(self, client, request, timeout=15.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout):
            return None
        return future.result()

    def _spawn_model(self, model_name, pose, sdf=None, sdf_path=None):
        req = SpawnEntity.Request()
        req.entity_factory.name = model_name
        if sdf is not None:
            req.entity_factory.sdf = sdf
        else:
            req.entity_factory.sdf_filename = sdf_path
        req.entity_factory.pose = pose
        resp = self._call(self._create_cli, req)
        if resp is None or not resp.success:
            return "create failed for '%s'" % model_name
        return None

    def _delete_model(self, model_name):
        req = DeleteEntity.Request()
        req.entity.name = model_name
        # Default type is NONE=0; Fortress then looks up "name of type 0"
        # and logs "Entity named [...] of type [0] not found" while the
        # ROS service can still return success. The model stays in the
        # world, the next spawn stacks on it, and RTF collapses.
        req.entity.type = Entity.MODEL
        resp = self._call(self._delete_cli, req)
        if resp is None or not resp.success:
            return "delete failed for '%s'" % model_name
        return None

    # ------------------------------------------------------------------
    # State topics (replace the ROS 1 param server)
    # ------------------------------------------------------------------

    def _box_to_record(self, box, yaw=0.0, mass_kg=0.0):
        return {
            "id": box.id,
            "width": box.width,
            "depth": box.depth,
            "height": box.height,
            "yaw": float(yaw),
            "mass_kg": float(mass_kg),
            "size_mode": self._size_mode,
            "visual_kind": self._visual_kind,
            "model_name": self._current_model or "",
            "pose": {
                "position": {
                    "x": box.pose.position.x,
                    "y": box.pose.position.y,
                    "z": box.pose.position.z,
                },
                "orientation": {
                    "x": box.pose.orientation.x,
                    "y": box.pose.orientation.y,
                    "z": box.pose.orientation.z,
                    "w": box.pose.orientation.w,
                },
            },
        }

    def _publish_box_state(self):
        payload = {}
        if self._current_box is not None:
            payload = self._box_to_record(
                self._current_box,
                yaw=getattr(self, "_current_yaw", 0.0),
                mass_kg=getattr(self, "_current_mass", 0.0),
            )
        self._box_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    # ------------------------------------------------------------------
    # Box sampling (same logic as noetic)
    # ------------------------------------------------------------------

    def _choose_entry(self):
        if self._sequence_ids:
            entry_id = self._sequence_ids[
                self._sequence % len(self._sequence_ids)]
            if entry_id not in self._entries_by_id:
                raise RuntimeError(
                    "unknown sequence box id '%s'" % entry_id)
            return self._entries_by_id[entry_id]
        total = sum(max(0.0, entry["weight"]) for entry in self._entries)
        if total <= 0.0:
            return self._rng.choice(self._entries)
        pick = self._rng.uniform(0.0, total)
        running = 0.0
        for entry in self._entries:
            running += max(0.0, entry["weight"])
            if pick <= running:
                return entry
        return self._entries[-1]

    def _sample_yaw_offset(self, entry):
        mode = (
            self._yaw_mode_override or entry.get("yaw_mode", "discrete")
        ).strip().lower()
        if mode == "continuous":
            if self._yaw_range_override and len(self._yaw_range_override) == 2:
                lo, hi = (float(self._yaw_range_override[0]),
                          float(self._yaw_range_override[1]))
            else:
                lo, hi = entry.get("yaw_range", [-3.14159265, 3.14159265])
            return self._rng.uniform(float(lo), float(hi))
        allowed = entry.get("allowed_yaws") or [0.0]
        return self._rng.choice(allowed)

    def _entry_pose(self, entry, size=None):
        """Return (pose, world_yaw) for a lying box on the pickup platform.

        Link origin is the AABB center, so z is platform-top + height/2.
        """
        yaw_offset = self._sample_yaw_offset(entry)
        size = list(size) if size is not None else entry["size"]
        dx = (
            self._rng.uniform(-self._xy_jitter[0], self._xy_jitter[0])
            if self._xy_jitter[0] else 0.0)
        dy = (
            self._rng.uniform(-self._xy_jitter[1], self._xy_jitter[1])
            if self._xy_jitter[1] else 0.0)
        xyz, rpy = pickup_box_pose(
            self._source_xyz, self._source_rpy, size, yaw_offset, (dx, dy))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation = _quaternion_from_rpy(*rpy)
        return pose, rpy[2]

    def _sample_box(self):
        """Return (entry, size, mass_kg, generated, id_suffix)."""
        entry = self._choose_entry()
        if self._size_mode != "continuous":
            size = list(entry["size"])
            mass = float(entry.get("mass_kg", 0.0))
            return entry, size, mass, False, entry["id"]
        size = sample_box_size(self._size_range, self._rng)
        mass = sample_box_mass(size, self._mass_model, self._rng)
        return entry, size, mass, True, "gen"

    def _box_sdf(self, model_name, size, mass_kg, visual_id):
        """Inline SDF. Mesh references a pre-scaled model:// URI."""
        return pickup_visual_sdf(
            model_name, size, mass_kg, visual_id,
            visual_kind=self._visual_kind,
        )

    def _gt_size(self, size, visual_id):
        """Size written to GetCurrentBox.

        Box visual: catalog AABB. Mesh: baked top-face footprint of that STL.
        Pose still uses the catalog AABB height so the mesh sits on the platform.
        """
        if self._visual_kind != "mesh":
            return [float(v) for v in size]
        tier = size_tier_name(size)
        if not tier:
            return [float(v) for v in size]
        rec = self._sized_manifest.get(sized_model_name(visual_id, tier)) or {}
        measure = rec.get("measure_size") or []
        if len(measure) != 3:
            return [float(v) for v in size]
        return [float(v) for v in measure]

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def handle_clear(self, _req, _response):
        response = ClearCurrentBox.Response()
        if not self._current_model:
            self._current_box = None
            self._publish_box_state()
            response.success = True
            response.message = "no current pickup box"
            return response
        err = self._delete_model(self._current_model)
        if err:
            response.success = False
            response.message = err
            return response
        self.get_logger().info(
            "Deleted current pickup box '%s'" % self._current_model)
        self._current_model = None
        self._current_box = None
        self._publish_box_state()
        response.success = True
        response.message = "cleared current pickup box"
        return response

    def handle_finalize(self, _req, _response):
        """Clear the pickup role without deleting the placed Gazebo model."""
        response = FinalizeCurrentBox.Response()
        if not self._current_model:
            response.success = False
            response.message = "no current pickup box to finalize"
            response.model_name = ""
            return response
        model_name = self._current_model
        record = {
            "model_name": model_name,
            "id": self._current_box.id if self._current_box else model_name,
            "size": (
                [self._current_box.width, self._current_box.depth,
                 self._current_box.height] if self._current_box else []),
            "mass_kg": getattr(self, "_current_mass", 0.0),
        }
        self._finalized_models.append(record)
        self._finalized_pub.publish(
            String(data=json.dumps(self._finalized_models, sort_keys=True)))
        self._current_model = None
        self._current_box = None
        self._publish_box_state()
        self.get_logger().info(
            "Finalized Gazebo model '%s' without deletion (total=%d)"
            % (model_name, len(self._finalized_models)))
        response.success = True
        response.message = "finalized %s without deletion" % model_name
        response.model_name = model_name
        return response

    def handle_get_current(self, _req, _response):
        response = GetCurrentBox.Response()
        if self._current_box is None:
            response.box = DetectedLuggage()
            response.success = False
            response.message = "no current pickup box"
            return response
        response.box = self._current_box
        response.success = True
        response.message = "current pickup box"
        return response

    def handle_spawn_next(self, _req, _response):
        clear = self.handle_clear(None, None)
        response = SpawnNextBox.Response()
        if not clear.success:
            response.success = False
            response.message = clear.message
            return response

        entry, size, mass_kg, _generated, id_suffix = self._sample_box()
        self._sequence += 1
        model_name = "%s_%04d_%s" % (
            self._model_prefix, self._sequence, id_suffix)
        pose, yaw = self._entry_pose(entry, size)
        visual_id = visual_id_for_entry(entry)
        if self._visual_kind == "mesh":
            visual_id = self._rng.choice(list(VISUAL_IDS))
        gt_size = self._gt_size(size, visual_id)
        try:
            err = self._spawn_model(
                model_name, pose,
                sdf=self._box_sdf(model_name, size, mass_kg, visual_id))
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = str(exc)
            return response
        if err:
            response.success = False
            response.message = err
            return response

        if self._visual_settle_sec > 0.0:
            time.sleep(self._visual_settle_sec)

        box = DetectedLuggage()
        box.id = model_name
        box.pose = pose
        box.width = gt_size[0]
        box.depth = gt_size[1]
        box.height = gt_size[2]
        box.yaw_valid = True
        short = min(abs(gt_size[0]), abs(gt_size[1]))
        box.aspect_ratio = (
            max(abs(gt_size[0]), abs(gt_size[1])) / short
            if short > 1e-12 else 1.0)
        self._current_model = model_name
        self._current_box = box
        self._current_yaw = yaw
        self._current_mass = mass_kg
        self._publish_box_state()
        # Ground truth for evaluating perception, on a separate topic so
        # nothing in the control chain can read the spawned size by accident.
        self._size_eval_pub.publish(String(data=json.dumps({
            "model_name": model_name,
            "catalog_id": entry["id"],
            "visual_id": visual_id,
            "visual_kind": self._visual_kind,
            "gz_model": (
                sized_model_name(visual_id, size_tier_name(size) or "")
                if self._visual_kind == "mesh" else ""),
            "command_size": [float(v) for v in size],
            "width": box.width,
            "depth": box.depth,
            "height": box.height,
            "mass_kg": float(mass_kg),
            "yaw": float(yaw),
        }, sort_keys=True)))
        self.get_logger().info(
            "Spawned %s visual=%s/%s size=%.3fx%.3fx%.3f gt=%.3fx%.3fx%.3f "
            "mass=%.2fkg yaw=%.3f rad mode=%s"
            % (model_name, self._visual_kind, visual_id,
               size[0], size[1], size[2],
               box.width, box.depth, box.height,
               mass_kg, yaw, self._size_mode))
        response.box = box
        response.success = True
        response.message = "spawned %s" % model_name
        return response


def main(argv=None):
    rclpy.init(args=argv)
    node = PickupBoxSpawner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
