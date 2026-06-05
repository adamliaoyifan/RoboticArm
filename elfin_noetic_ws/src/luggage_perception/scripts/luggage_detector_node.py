#!/usr/bin/env python3
"""Luggage detector with fixed-source current-box fallback for active loading."""

import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from luggage_msgs.msg import DetectedLuggage
from luggage_msgs.srv import DetectLuggage, DetectLuggageResponse, GetCurrentBox


class LuggageDetector:
    def __init__(self):
        self._current_box_service = rospy.get_param(
            "~current_box_service", "/pickup_box_spawner/get_current_box"
        )

    @staticmethod
    def _box_from_param():
        data = rospy.get_param("/luggage/current_box", {})
        if not data:
            return None
        pos = data.get("pose", {}).get("position", {})
        ori = data.get("pose", {}).get("orientation", {})
        return DetectedLuggage(
            id=data.get("id", "current_box"),
            width=float(data.get("width", 0.70)),
            height=float(data.get("height", 0.28)),
            depth=float(data.get("depth", 0.45)),
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

    def _current_box_from_spawner(self):
        try:
            rospy.wait_for_service(self._current_box_service, timeout=0.2)
            resp = rospy.ServiceProxy(self._current_box_service, GetCurrentBox)()
            if resp.success:
                return resp.box
        except Exception:
            return None
        return None

    def handle(self, _req):
        current = self._current_box_from_spawner() or self._box_from_param()
        if current is not None:
            return DetectLuggageResponse(
                luggage=[current], success=True, message="current fixed-source box"
            )

        rospy.logwarn("No current pickup box available — returning fake luggage")
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
