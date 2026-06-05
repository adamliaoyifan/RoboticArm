#!/usr/bin/env python3
"""Inspect container interior occupancy (Gazebo GT + optional depth point cloud)."""

from __future__ import division

import os
import sys

import rospy
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates

from luggage_msgs.msg import SlotSpec
from luggage_msgs.srv import InspectContainer, InspectContainerResponse

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from container_config_utils import (  # noqa: E402
    box_catalog_entries,
    container_in_base_link,
    default_box_catalog_path,
    default_config_path,
    inner_dimensions,
    load_box_catalog,
    load_container_config,
)

try:
    import sensor_msgs.point_cloud2 as pc2
except ImportError:
    pc2 = None


class ContainerInspector:
    SUITCASE_MODELS = (
        "suitcase_standard",
        "suitcase_large",
        "suitcase_carryon",
    )

    def __init__(self):
        self._config_path = rospy.get_param("~container_config", default_config_path())
        self._catalog_path = rospy.get_param("~box_catalog_config", default_box_catalog_path())
        self._config = load_container_config(self._config_path)
        self._catalog = load_box_catalog(self._catalog_path, self._config)
        self._model_sizes = {}
        for entry in box_catalog_entries(self._catalog):
            self._model_sizes[entry["model"]] = entry["size"]
            self._model_sizes[entry["id"]] = entry["size"]
        self._grid_res = float(rospy.get_param("~grid_resolution", 0.10))
        self._last_model_states = None
        self._depth_points = None
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states, queue_size=1)
        depth_topic = rospy.get_param("~depth_points_topic", "/camera/depth/points")
        if pc2 is not None:
            rospy.Subscriber(depth_topic, rospy.AnyMsg, self._on_depth_points, queue_size=1)

    def _on_model_states(self, msg):
        self._last_model_states = msg

    def _on_depth_points(self, msg):
        self._depth_points = msg

    def _inner_bounds_in_base(self):
        inner_l, inner_w, inner_h = inner_dimensions(self._config)
        base_xyz, base_rpy = container_in_base_link(self._config)
        # Approximate inner volume as axis-aligned box in base frame (container yaw only).
        import math

        yaw = base_rpy[2]
        cx = base_xyz[0] + (inner_l * 0.5) * math.cos(yaw)
        cy = base_xyz[1] + (inner_l * 0.5) * math.sin(yaw)
        cz = base_xyz[2] + inner_h * 0.5
        return {
            "center": [cx, cy, cz],
            "size": [inner_l, inner_w, inner_h],
            "yaw": yaw,
        }

    def _occupancy_from_gazebo(self):
        bounds = self._inner_bounds_in_base()
        inner_l, inner_w, inner_h = bounds["size"]
        volume = inner_l * inner_w * inner_h
        occupied = 0.0
        occupied = 0.0
        occupied_boxes = []

        for slot in self._placed_from_param():
            size = [slot.width, slot.depth, slot.height]
            occupied += size[0] * size[1] * size[2]
            occupied_boxes.append(
                {
                    "center": [
                        slot.place_pose.position.x,
                        slot.place_pose.position.y,
                        slot.place_pose.position.z,
                    ],
                    "size": size,
                    "source": "placed_param",
                }
            )

        if self._last_model_states is None:
            return volume, occupied, occupied_boxes

        names = self._last_model_states.name
        poses = self._last_model_states.pose
        for name, pose in zip(names, poses):
            size = self._size_for_model(name)
            if size is None:
                continue
            px = pose.position.x
            py = pose.position.y
            pz = pose.position.z
            if self._point_in_inner_volume(px, py, pz, bounds):
                occupied += size[0] * size[1] * size[2]
                occupied_boxes.append(
                    {"center": [px, py, pz], "size": size, "source": name}
                )

        return volume, occupied, occupied_boxes

    def _size_for_model(self, model_name):
        if model_name in self._model_sizes:
            return self._model_sizes[model_name]
        for key, size in self._model_sizes.items():
            if model_name.endswith("_%s" % key) or ("_%s_" % key) in model_name:
                return size
        return None

    @staticmethod
    def _placed_from_param():
        placed = []
        for item in rospy.get_param("/luggage/container_inspection/placed_boxes", []):
            pose_data = item.get("place_pose", {})
            pos = pose_data.get("position", {})
            ori = pose_data.get("orientation", {})
            placed.append(
                SlotSpec(
                    layer=int(item.get("layer", 0)),
                    row=int(item.get("row", 0)),
                    col=int(item.get("col", 0)),
                    width=float(item.get("width", 0.70)),
                    height=float(item.get("height", 0.28)),
                    depth=float(item.get("depth", 0.45)),
                    place_pose=Pose(
                        position=Point(
                            x=float(pos.get("x", 0.0)),
                            y=float(pos.get("y", 0.0)),
                            z=float(pos.get("z", 0.0)),
                        ),
                        orientation=Quaternion(
                            x=float(ori.get("x", 0.0)),
                            y=float(ori.get("y", 0.0)),
                            z=float(ori.get("z", 0.0)),
                            w=float(ori.get("w", 1.0)),
                        ),
                    ),
                )
            )
        return placed

    @staticmethod
    def _point_in_inner_volume(x, y, z, bounds):
        cx, cy, cz = bounds["center"]
        inner_l, inner_w, inner_h = bounds["size"]
        import math

        yaw = bounds["yaw"]
        dx = x - cx
        dy = y - cy
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        local_z = z - cz
        return (
            abs(local_x) <= inner_l * 0.5
            and abs(local_y) <= inner_w * 0.5
            and abs(local_z) <= inner_h * 0.5
        )

    def _free_slots_from_gazebo(self, bounds, occupied_boxes):
        slots = []
        inner_l, inner_w, inner_h = bounds["size"]
        cx, cy, cz = bounds["center"]
        import math

        yaw = bounds["yaw"]
        for col, offset in enumerate([-inner_l * 0.25, 0.0, inner_l * 0.25]):
            local_x = offset
            world_x = cx + math.cos(yaw) * local_x
            world_y = cy + math.sin(yaw) * local_x
            world_z = cz - inner_h * 0.5 + 0.14
            if any(
                abs(world_x - occ["center"][0]) < max(0.35, occ["size"][0] * 0.5)
                and abs(world_y - occ["center"][1]) < max(0.35, occ["size"][1] * 0.5)
                for occ in occupied_boxes
            ):
                continue
            slots.append(
                SlotSpec(
                    layer=0,
                    row=0,
                    col=col,
                    width=0.70,
                    height=0.28,
                    depth=0.45,
                    place_pose=Pose(
                        position=Point(x=world_x, y=world_y, z=world_z),
                        orientation=Quaternion(w=1.0),
                    ),
                )
            )
        return slots

    def _occupancy_from_mapper(self, bounds):
        stats = rospy.get_param("/luggage/cargo_map/stats", {})
        occupied_boxes = rospy.get_param("/luggage/cargo_map/occupied_boxes", [])
        inner_l, inner_w, inner_h = bounds["size"]
        volume = inner_l * inner_w * inner_h
        occupied = float(stats.get("occupied_count", 0)) * (self._grid_res ** 3)
        if occupied_boxes:
            occupied = max(
                occupied,
                sum(
                    occ["size"][0] * occ["size"][1] * occ["size"][2]
                    for occ in occupied_boxes
                ),
            )
        free_volume = max(0.0, float(stats.get("free_volume", volume - occupied)))
        occupancy_ratio = float(stats.get("occupancy_ratio", 0.0))
        if occupancy_ratio <= 0.0 and volume > 0.0:
            occupancy_ratio = min(1.0, occupied / volume)
        return volume, occupied, occupied_boxes, free_volume, occupancy_ratio

    def handle(self, req):
        mode = (req.mode or "gazebo_gt").strip().lower()
        bounds = self._inner_bounds_in_base()
        volume, occupied, occupied_boxes = self._occupancy_from_gazebo()
        free_volume = max(0.0, volume - occupied)
        occupancy_ratio = occupied / volume if volume > 0.0 else 0.0
        free_slots = self._free_slots_from_gazebo(bounds, occupied_boxes)

        if mode in ("depth", "fused"):
            if rospy.has_param("/luggage/cargo_map/stats"):
                (
                    volume,
                    occupied,
                    occupied_boxes,
                    free_volume,
                    occupancy_ratio,
                ) = self._occupancy_from_mapper(bounds)
                free_slots = self._free_slots_from_gazebo(bounds, occupied_boxes)
                rospy.loginfo(
                    "Inspect using fused cargo map (unknown_ratio=%.2f)",
                    float(rospy.get_param("/luggage/cargo_map/stats", {}).get("unknown_ratio", 1.0)),
                )
            elif mode == "depth" and self._depth_points is not None and pc2 is not None:
                rospy.logwarn("Depth/fused requested but cargo map missing; GT fallback")
            else:
                rospy.logwarn("Fused inspect requested but cargo map missing; GT fallback")

        if not free_slots:
            return InspectContainerResponse(
                success=False,
                message="No free slots detected in container",
                free_volume=free_volume,
                occupancy_ratio=occupancy_ratio,
                free_slots=[],
            )

        rospy.set_param("/luggage/container_inspection/free_slots", [self._slot_to_dict(s) for s in free_slots])
        rospy.set_param("/luggage/container_inspection/occupied_boxes", occupied_boxes)
        rospy.set_param("/luggage/container_inspection/free_volume", free_volume)
        rospy.set_param("/luggage/container_inspection/occupancy_ratio", occupancy_ratio)

        return InspectContainerResponse(
            success=True,
            message="mode=%s free_slots=%d occupancy=%.1f%%" % (mode, len(free_slots), occupancy_ratio * 100.0),
            free_volume=free_volume,
            occupancy_ratio=occupancy_ratio,
            free_slots=free_slots,
        )

    @staticmethod
    def _slot_to_dict(slot):
        return {
            "layer": slot.layer,
            "row": slot.row,
            "col": slot.col,
            "width": slot.width,
            "height": slot.height,
            "depth": slot.depth,
            "place_pose": {
                "position": {
                    "x": slot.place_pose.position.x,
                    "y": slot.place_pose.position.y,
                    "z": slot.place_pose.position.z,
                }
            },
        }


def main():
    rospy.init_node("container_inspector")
    inspector = ContainerInspector()
    rospy.Service("~inspect_container", InspectContainer, inspector.handle)
    rospy.loginfo("container_inspector ready")
    rospy.spin()


if __name__ == "__main__":
    main()
