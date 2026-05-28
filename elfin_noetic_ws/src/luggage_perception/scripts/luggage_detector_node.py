#!/usr/bin/env python3
"""Phase 0 stub — returns one fake detected suitcase."""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from luggage_msgs.msg import DetectedLuggage
from luggage_msgs.srv import DetectLuggage, DetectLuggageResponse


class LuggageDetector:
    def handle(self, _req):
        # TODO: ground truth from Gazebo or depth-based detection
        rospy.logwarn("LuggageDetector.handle not implemented — returning fake luggage")
        item = DetectedLuggage(
            id="fake_0",
            width=0.70,
            height=0.28,
            depth=0.45,
            pose=Pose(
                position=Point(x=1.5, y=-0.8, z=0.14),
                orientation=Quaternion(w=1.0),
            ),
        )
        return DetectLuggageResponse(luggage=[item], success=True, message="stub")


def main():
    rospy.init_node("luggage_detector")
    detector = LuggageDetector()
    rospy.Service("~detect_luggage", DetectLuggage, detector.handle)
    rospy.loginfo("luggage_detector ready (stub)")
    rospy.spin()


if __name__ == "__main__":
    main()
