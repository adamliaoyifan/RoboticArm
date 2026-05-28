#!/usr/bin/env python3
"""Phase 0 stub — build motion sequences and expose as service."""

import os
import sys

import rospy

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from luggage_msgs.msg import MotionSegment
from luggage_msgs.srv import BuildMotionSequence, BuildMotionSequenceResponse

from waypoint_generator import build_sequence, segment_names_for_phase


class WaypointGeneratorNode:
    def handle(self, req):
        rospy.logwarn("WaypointGeneratorNode.handle not implemented — returning named empty segments")
        segments = build_sequence(req.pick, req.place_slot, req.phase)
        if not segments:
            for name in segment_names_for_phase(req.phase):
                seg = MotionSegment(name=name, type="pose_target")
                segments.append(seg)
        return BuildMotionSequenceResponse(segments=segments, success=True, message="stub")


def main():
    rospy.init_node("waypoint_generator")
    node = WaypointGeneratorNode()
    rospy.Service("~build_motion_sequence", BuildMotionSequence, node.handle)
    rospy.loginfo("waypoint_generator ready (stub)")
    rospy.spin()


if __name__ == "__main__":
    main()
