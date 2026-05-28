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
    container_in_base_link,
    default_config_path,
    inner_dimensions,
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
        self._config = load_container_config(self._config_path)
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
        occupied_centers = []

        if self._last_model_states is None:
            return volume, occupied, occupied_centers

        names = self._last_model_states.name
        poses = self._last_model_states.pose
        for name, pose in zip(names, poses):
            if name not in self.SUITCASE_MODELS:
                continue
            px = pose.position.x
            py = pose.position.y
            pz = pose.position.z
            if self._point_in_inner_volume(px, py, pz, bounds):
                occupied += 0.70 * 0.45 * 0.28
                occupied_centers.append([px, py, pz])

        return volume, occupied, occupied_centers

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

    def _free_slots_from_gazebo(self, bounds, occupied_centers):
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
                abs(world_x - occ[0]) < 0.35 and abs(world_y - occ[1]) < 0.35
                for occ in occupied_centers
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

    def handle(self, req):
        mode = (req.mode or "gazebo_gt").strip().lower()
        bounds = self._inner_bounds_in_base()
        volume, occupied, occupied_centers = self._occupancy_from_gazebo()
        free_volume = max(0.0, volume - occupied)
        occupancy_ratio = occupied / volume if volume > 0.0 else 0.0
        free_slots = self._free_slots_from_gazebo(bounds, occupied_centers)

        if mode == "depth" and self._depth_points is not None and pc2 is not None:
            rospy.loginfo("Depth mode requested; using Gazebo GT fallback until TF crop is wired")

        if not free_slots:
            return InspectContainerResponse(
                success=False,
                message="No free slots detected in container",
                free_volume=free_volume,
                occupancy_ratio=occupancy_ratio,
                free_slots=[],
            )

        rospy.set_param("/luggage/container_inspection/free_slots", [self._slot_to_dict(s) for s in free_slots])
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
