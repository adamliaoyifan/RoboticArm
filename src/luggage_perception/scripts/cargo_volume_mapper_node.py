#!/usr/bin/env python3
"""Cargo occupancy map node (ROS 2 Humble, Todo 5 slice B).

Thin shell around the pure ``CargoVolumeMapper``: geometry commits in,
surface map + stats out.

Services (absolute names, ROS 2 has no ``~`` semantics):
  /cargo_map/add_placed_box     AddPlacedBox   -> mark_placed_box
  /cargo_map/remove_placed_box  RemovePlacedBox-> unmark_placed_box
  /cargo_map/reset              ResetCargoMap  -> reset()
  /cargo_map/get_stats          GetCargoMapStats

Topics:
  /luggage/cargo_map/surface_2d  std_msgs/String (JSON, transient-local)
      published after every commit/reset; contract mirrors
      ``CargoVolumeMapper.surface_map_2d()``.
  /luggage/cargo_map/committed   std_msgs/String (JSON, transient-local)
      the commit ledger (center/size/yaw per box) - what ComputePlacement
      and the eval driver cross-check against their own bookkeeping (A4).

SOURCE_GEOMETRY only (fail-closed): this node never integrates sensor
points; depth integration is deliberately deferred (slice B2 / Todo 6).
"""

from __future__ import division

import json

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from geometry_msgs.msg import Pose as PoseMsg
from luggage_msgs.srv import (
    AddPlacedBox,
    GetCargoMapStats,
    RemovePlacedBox,
    ResetCargoMap,
)

from ament_index_python.packages import get_package_share_directory

from luggage_description.scene_tf_config_utils import (
    container_hull_local_inside_fn,
    container_inner_ceiling_z,
    container_inner_dimensions,
    container_inner_floor_z,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
    static_transforms,
)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper


class CargoVolumeMapperNode(Node):

    def __init__(self):
        super().__init__("cargo_volume_mapper")
        group = ReentrantCallbackGroup()

        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("occupancy_params_yaml", "")

        config_path = str(self.get_parameter("scene_tf_config").value)
        if not config_path:
            config_path = resolve_scene_tf_config_path()
        scene = load_scene_tf_config(config_path)
        inner = container_inner_dimensions(scene)
        # inner = (length_x, width_y, ceiling_z_legacy)
        inner_l = float(inner[0])
        inner_w = float(inner[1])
        floor_z = container_inner_floor_z(scene)
        ceiling_z = container_inner_ceiling_z(scene)
        inner_h = ceiling_z - floor_z
        # CargoVolumeMapper.center is the usable-volume center in
        # container_link (same convention as generate_candidates
        # center_base). Floor origin would shift every slot by -inner_h/2.
        volume_center_z = floor_z + 0.5 * inner_h

        self._mapper = CargoVolumeMapper(
            (inner_l, inner_w, inner_h),
            (0.0, 0.0, volume_center_z),
            0.0,
            resolution=float(self.get_parameter("resolution").value),
            hull_local_inside=container_hull_local_inside_fn(scene),
        )
        self._frame = "container_link"
        self._world_from_container = self._load_container_pose(scene)

        transient = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._surface_pub = self.create_publisher(
            String, "/luggage/cargo_map/surface_2d", transient)
        self._ledger_pub = self.create_publisher(
            String, "/luggage/cargo_map/committed", transient)

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

        self._publish_all()
        self.get_logger().info(
            "cargo_volume_mapper ready (inner %.2fx%.2fx%.2f m, res %.2f)"
            % (inner_l, inner_w, ceiling_z - floor_z,
               float(self.get_parameter("resolution").value)))

    # ------------------------------------------------------------------
    # Container pose (container_link in world) for world-frame commits.

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
        tx, ty, _tz = self._world_from_container["translation"]
        yaw = self._world_from_container["rotation_rpy"][2]
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        dx = pose_world.position.x - tx
        dy = pose_world.position.y - ty
        out = PoseMsg()
        out.position.x = dx * cos_y - dy * sin_y
        out.position.y = dx * sin_y + dy * cos_y
        out.position.z = pose_world.position.z
        out.orientation = pose_world.orientation
        return out

    # ------------------------------------------------------------------
    # Services

    def _on_add(self, request, response):
        slot = request.slot
        center_c = self._to_container(slot.place_pose)
        size = [max(0.0, float(slot.width)),
                max(0.0, float(slot.depth)),
                max(0.0, float(slot.height))]
        yaw = self._yaw_from_pose(slot.place_pose)
        try:
            self._mapper.mark_placed_box(
                [center_c.position.x, center_c.position.y,
                 center_c.position.z],
                size, yaw=yaw)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.message = "mark failed: %s" % exc
            return response
        self._publish_all()
        response.success = True
        response.message = "committed %d boxes (rev %d)" % (
            len(self._mapper._placed_boxes), self._mapper._revision)
        return response

    def _on_remove(self, request, response):
        slot = request.slot
        center_c = self._to_container(slot.place_pose)
        size = [max(0.0, float(slot.width)),
                max(0.0, float(slot.depth)),
                max(0.0, float(slot.height))]
        removed = self._mapper.unmark_placed_box(
            [center_c.position.x, center_c.position.y,
             center_c.position.z],
            size)
        self._publish_all()
        response.success = bool(removed)
        response.message = (
            "removed" if removed else "no matching committed box")
        return response

    def _on_reset(self, _request, response):
        self._mapper.reset(preserve_placed=False)
        self._publish_all()
        response.success = True
        response.message = "reset"
        return response

    def _on_stats(self, _request, response):
        stats = self._mapper.stats()
        response.success = True
        response.unknown_ratio = float(stats["unknown_ratio"])
        response.occupancy_ratio = float(stats["occupancy_ratio"])
        response.free_volume = float(stats["free_volume"])
        response.unknown_count = int(stats["unknown_count"])
        response.free_count = int(stats["free_count"])
        response.occupied_count = int(stats["occupied_count"])
        response.frontier_count = int(stats["frontier_count"])
        response.total_voxels = int(stats["total_voxels"])
        response.map_revision = int(stats["map_revision"])
        response.message = "committed=%d" % stats["committed_box_count"]
        return response

    # ------------------------------------------------------------------

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
            "boxes": self._mapper._placed_boxes,
            "map_revision": self._mapper._revision,
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
