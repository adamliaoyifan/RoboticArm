#!/usr/bin/env python3
"""Size-aware container placement with a legacy GetNextSlot wrapper."""

import math
import os
import sys

import rospy
import rospkg
from geometry_msgs.msg import Pose, Point, Quaternion
from luggage_msgs.msg import DetectedLuggage, SlotSpec
from luggage_msgs.srv import (
    ComputePlacement,
    ComputePlacementResponse,
    GetNextSlot,
    GetNextSlotResponse,
)

DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from container_config_utils import (  # noqa: E402
    container_in_base_link,
    default_config_path,
    inner_dimensions,
    load_container_config,
)


class BinPacker:
    def __init__(self):
        self._slot_index = 0
        self._container_config_path = rospy.get_param("~container_config", default_config_path())
        self._clearance = float(rospy.get_param("~clearance", 0.02))
        self._support_overlap_ratio = float(rospy.get_param("~support_overlap_ratio", 0.35))

    def _load_inspected_slots(self):
        data = rospy.get_param("/luggage/container_inspection/free_slots", [])
        slots = []
        for item in data:
            pose_data = item.get("place_pose", {}).get("position", {})
            slots.append(
                SlotSpec(
                    layer=int(item.get("layer", 0)),
                    row=int(item.get("row", 0)),
                    col=int(item.get("col", 0)),
                    width=float(item.get("width", 0.70)),
                    height=float(item.get("height", 0.28)),
                    depth=float(item.get("depth", 0.45)),
                    place_pose=Pose(
                        position=Point(
                            x=float(pose_data.get("x", 2.2)),
                            y=float(pose_data.get("y", 0.0)),
                            z=float(pose_data.get("z", 0.14)),
                        ),
                        orientation=Quaternion(w=1.0),
                    ),
                )
            )
        return slots

    @staticmethod
    def _slot_to_dict(slot):
        return {
            "layer": slot.layer,
            "row": slot.row,
            "col": slot.col,
            "width": slot.width,
            "depth": slot.depth,
            "height": slot.height,
            "place_pose": {
                "position": {
                    "x": slot.place_pose.position.x,
                    "y": slot.place_pose.position.y,
                    "z": slot.place_pose.position.z,
                },
                "orientation": {
                    "x": slot.place_pose.orientation.x,
                    "y": slot.place_pose.orientation.y,
                    "z": slot.place_pose.orientation.z,
                    "w": slot.place_pose.orientation.w,
                },
            },
        }

    @staticmethod
    def _slot_from_dict(item):
        pose_data = item.get("place_pose", {})
        pos = pose_data.get("position", {})
        ori = pose_data.get("orientation", {})
        return SlotSpec(
            layer=int(item.get("layer", 0)),
            row=int(item.get("row", 0)),
            col=int(item.get("col", 0)),
            width=float(item.get("width", 0.70)),
            height=float(item.get("height", 0.28)),
            depth=float(item.get("depth", 0.45)),
            place_pose=Pose(
                position=Point(
                    x=float(pos.get("x", 2.2)),
                    y=float(pos.get("y", 0.0)),
                    z=float(pos.get("z", 0.14)),
                ),
                orientation=Quaternion(
                    x=float(ori.get("x", 0.0)),
                    y=float(ori.get("y", 0.0)),
                    z=float(ori.get("z", 0.0)),
                    w=float(ori.get("w", 1.0)),
                ),
            ),
        )

    def _param_placed(self):
        return [
            self._slot_from_dict(item)
            for item in rospy.get_param("/luggage/container_inspection/placed_boxes", [])
        ]

    @staticmethod
    def _yaw_quaternion(yaw):
        return Quaternion(z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))

    @staticmethod
    def _rotate(yaw, x, y):
        return (
            math.cos(yaw) * x - math.sin(yaw) * y,
            math.sin(yaw) * x + math.cos(yaw) * y,
        )

    @staticmethod
    def _local_center(slot, base_xyz, yaw):
        dx = slot.place_pose.position.x - base_xyz[0]
        dy = slot.place_pose.position.y - base_xyz[1]
        lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        lz = slot.place_pose.position.z - base_xyz[2]
        return lx, ly, lz

    def _local_box(self, slot, base_xyz, yaw):
        cx, cy, cz = self._local_center(slot, base_xyz, yaw)
        return {
            "min_x": cx - slot.width * 0.5,
            "max_x": cx + slot.width * 0.5,
            "min_y": cy - slot.depth * 0.5,
            "max_y": cy + slot.depth * 0.5,
            "min_z": cz - slot.height * 0.5,
            "max_z": cz + slot.height * 0.5,
        }

    def _intersects(self, candidate, occupied):
        c = self._clearance
        for occ in occupied:
            if (
                candidate["min_x"] < occ["max_x"] + c
                and candidate["max_x"] > occ["min_x"] - c
                and candidate["min_y"] < occ["max_y"] + c
                and candidate["max_y"] > occ["min_y"] - c
                and candidate["min_z"] < occ["max_z"] + c
                and candidate["max_z"] > occ["min_z"] - c
            ):
                return True
        return False

    def _has_support(self, candidate, occupied):
        if candidate["min_z"] <= self._clearance:
            return True
        bottom_area = (
            (candidate["max_x"] - candidate["min_x"])
            * (candidate["max_y"] - candidate["min_y"])
        )
        if bottom_area <= 0.0:
            return False
        support_area = 0.0
        for occ in occupied:
            if abs(candidate["min_z"] - occ["max_z"]) > self._clearance:
                continue
            overlap_x = max(
                0.0,
                min(candidate["max_x"], occ["max_x"])
                - max(candidate["min_x"], occ["min_x"]),
            )
            overlap_y = max(
                0.0,
                min(candidate["max_y"], occ["max_y"])
                - max(candidate["min_y"], occ["min_y"]),
            )
            support_area += overlap_x * overlap_y
        return support_area / bottom_area >= self._support_overlap_ratio

    @staticmethod
    def _unique_sorted(values, min_value=None):
        rounded = sorted(set(round(v, 4) for v in values))
        if min_value is None:
            return rounded
        return [v for v in rounded if v >= min_value - 1e-6]

    def _candidate_mins(self, occupied, inner_l, inner_w, inner_h, box):
        width, depth, height = box.width, box.depth, box.height
        xs = [0.0] + [occ["max_x"] + self._clearance for occ in occupied]
        ys = [-inner_w * 0.5] + [occ["max_y"] + self._clearance for occ in occupied]
        zs = [0.0] + [occ["max_z"] + self._clearance for occ in occupied]
        for z in self._unique_sorted(zs, 0.0):
            if z + height > inner_h + 1e-6:
                continue
            for y in self._unique_sorted(ys, -inner_w * 0.5):
                if y < -inner_w * 0.5 - 1e-6 or y + depth > inner_w * 0.5 + 1e-6:
                    continue
                for x in self._unique_sorted(xs, 0.0):
                    if x + width > inner_l + 1e-6:
                        continue
                    yield x, y, z

    def _make_slot(self, min_x, min_y, min_z, box, base_xyz, yaw, index):
        cx = min_x + box.width * 0.5
        cy = min_y + box.depth * 0.5
        cz = min_z + box.height * 0.5
        rx, ry = self._rotate(yaw, cx, cy)
        layer = int(round(min_z * 1000.0))
        row = int(round((min_y + 10.0) * 1000.0))
        col = int(round(min_x * 1000.0))
        return SlotSpec(
            layer=layer,
            row=row,
            col=col if index == 0 else col + index,
            width=box.width,
            height=box.height,
            depth=box.depth,
            place_pose=Pose(
                position=Point(x=base_xyz[0] + rx, y=base_xyz[1] + ry, z=base_xyz[2] + cz),
                orientation=self._yaw_quaternion(yaw),
            ),
        )

    def _current_box_from_param(self):
        data = rospy.get_param("/luggage/current_box", {})
        if not data:
            return None
        pos = data.get("pose", {}).get("position", {})
        ori = data.get("pose", {}).get("orientation", {})
        return DetectedLuggage(
            id=data.get("id", "current_box"),
            width=float(data.get("width", 0.70)),
            depth=float(data.get("depth", 0.45)),
            height=float(data.get("height", 0.28)),
            pose=Pose(
                position=Point(
                    x=float(pos.get("x", 0.3)),
                    y=float(pos.get("y", -0.8)),
                    z=float(pos.get("z", 0.14)),
                ),
                orientation=Quaternion(
                    x=float(ori.get("x", 0.0)),
                    y=float(ori.get("y", 0.0)),
                    z=float(ori.get("z", 0.0)),
                    w=float(ori.get("w", 1.0)),
                ),
            ),
        )

    def handle_compute(self, req):
        box = req.box
        if box.width <= 0.0 or box.depth <= 0.0 or box.height <= 0.0:
            current = self._current_box_from_param()
            if current is not None:
                box = current
        if box.width <= 0.0 or box.depth <= 0.0 or box.height <= 0.0:
            return ComputePlacementResponse(
                slot=SlotSpec(),
                success=False,
                message="current box dimensions are missing",
            )

        config = load_container_config(self._container_config_path)
        inner_l, inner_w, inner_h = inner_dimensions(config)
        base_xyz, base_rpy = container_in_base_link(config)
        yaw = base_rpy[2]
        placed = list(req.placed) if req.placed else self._param_placed()
        occupied = [self._local_box(slot, base_xyz, yaw) for slot in placed]

        for index, (x, y, z) in enumerate(
            self._candidate_mins(occupied, inner_l, inner_w, inner_h, box)
        ):
            candidate = {
                "min_x": x,
                "max_x": x + box.width,
                "min_y": y,
                "max_y": y + box.depth,
                "min_z": z,
                "max_z": z + box.height,
            }
            if self._intersects(candidate, occupied) or not self._has_support(candidate, occupied):
                continue
            slot = self._make_slot(x, y, z, box, base_xyz, yaw, index)
            return ComputePlacementResponse(
                slot=slot,
                success=True,
                message="placement for %s at layer=%d row=%d col=%d"
                % (box.id or "box", slot.layer, slot.row, slot.col),
            )

        return ComputePlacementResponse(
            slot=SlotSpec(),
            success=False,
            message="container full for %.2fx%.2fx%.2f box" % (box.width, box.depth, box.height),
        )

    def handle(self, req):
        inspected = self._load_inspected_slots()
        if inspected:
            index = min(len(req.placed), len(inspected) - 1)
            if index < 0:
                index = 0
            slot = inspected[index]
            return GetNextSlotResponse(
                slot=slot,
                success=True,
                message="slot from container inspection col=%d" % slot.col,
            )

        current = self._current_box_from_param() or DetectedLuggage(
            id="legacy_standard", width=0.70, depth=0.45, height=0.28
        )
        resp = self.handle_compute(type("Request", (), {"box": current, "placed": req.placed})())
        return GetNextSlotResponse(slot=resp.slot, success=resp.success, message=resp.message)


def main():
    rospy.init_node("bin_packer")
    packer = BinPacker()
    rospy.Service("~get_next_slot", GetNextSlot, packer.handle)
    rospy.Service("~compute_placement", ComputePlacement, packer.handle_compute)
    rospy.loginfo("bin_packer ready")
    rospy.spin()


if __name__ == "__main__":
    main()
