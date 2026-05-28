#!/usr/bin/env python3
"""Bin packing — uses container inspection free slots when available."""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from luggage_msgs.msg import SlotSpec
from luggage_msgs.srv import GetNextSlot, GetNextSlotResponse


class BinPacker:
    def __init__(self):
        self._slot_index = 0

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

        rospy.logwarn("BinPacker: no inspection data — returning fixed slot")
        slot = SlotSpec(
            layer=0,
            row=0,
            col=len(req.placed),
            width=0.70,
            height=0.28,
            depth=0.45,
            place_pose=Pose(
                position=Point(x=2.2, y=0.0, z=0.14),
                orientation=Quaternion(w=1.0),
            ),
        )
        return GetNextSlotResponse(slot=slot, success=True, message="fixed fallback slot")


def main():
    rospy.init_node("bin_packer")
    packer = BinPacker()
    rospy.Service("~get_next_slot", GetNextSlot, packer.handle)
    rospy.loginfo("bin_packer ready")
    rospy.spin()


if __name__ == "__main__":
    main()
