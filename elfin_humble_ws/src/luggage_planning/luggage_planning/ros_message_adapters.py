#!/usr/bin/env python3
"""ROS message <-> pose dataclass adapters for luggage_planning.

Node-layer code: it imports message types at the top level by design, and
algorithm modules (`waypoint_generator.py`, `pose.py`) must never import it.
Keeps the conversions mechanical so the dataclass <-> msg round trip is
unit-testable without a ROS graph (guarded by pytest.importorskip).
"""

from __future__ import division

from geometry_msgs.msg import Pose as PoseMsg
from luggage_msgs.msg import MotionSegment as MotionSegmentMsg
from luggage_msgs.msg import DetectedLuggage as DetectedLuggageMsg
from shape_msgs.msg import SolidPrimitive

from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion


def pose_to_msg(pose):
    """luggage_planning.pose.Pose -> geometry_msgs/Pose."""
    out = PoseMsg()
    out.position.x = float(pose.position.x)
    out.position.y = float(pose.position.y)
    out.position.z = float(pose.position.z)
    out.orientation.x = float(pose.orientation.x)
    out.orientation.y = float(pose.orientation.y)
    out.orientation.z = float(pose.orientation.z)
    out.orientation.w = float(pose.orientation.w)
    return out


def pose_from_msg(msg):
    """geometry_msgs/Pose -> luggage_planning.pose.Pose."""
    return Pose(
        position=Point(x=float(msg.position.x), y=float(msg.position.y),
                       z=float(msg.position.z)),
        orientation=Quaternion(x=float(msg.orientation.x),
                               y=float(msg.orientation.y),
                               z=float(msg.orientation.z),
                               w=float(msg.orientation.w)),
    )


def segment_to_msg(segment):
    """luggage_planning.pose.MotionSegment -> luggage_msgs/MotionSegment."""
    out = MotionSegmentMsg()
    out.name = str(segment.name)
    out.type = str(segment.type)
    out.target_pose = pose_to_msg(segment.target_pose)
    out.waypoints = [pose_to_msg(p) for p in segment.waypoints]
    out.keep_tool_down = bool(segment.keep_tool_down)
    out.keep_camera_down = bool(segment.keep_camera_down)
    out.lock_wrist = bool(segment.lock_wrist)
    out.allow_ompl_fallback = bool(segment.allow_ompl_fallback)
    return out


def segment_from_msg(msg):
    """luggage_msgs/MotionSegment -> luggage_planning.pose.MotionSegment."""
    return MotionSegment(
        name=str(msg.name),
        type=str(msg.type),
        target_pose=pose_from_msg(msg.target_pose),
        waypoints=[pose_from_msg(p) for p in msg.waypoints],
        keep_tool_down=bool(msg.keep_tool_down),
        keep_camera_down=bool(msg.keep_camera_down),
        lock_wrist=bool(msg.lock_wrist),
        allow_ompl_fallback=bool(msg.allow_ompl_fallback),
    )


def pick_from_detected(msg):
    """luggage_msgs/DetectedLuggage -> namespace for ``build_sequence``.

    ``build_sequence`` accesses ``pick.pose`` / ``pick.height`` /
    ``pick.yaw`` / ``pick.yaw_valid`` by attribute.
    """
    import math
    from types import SimpleNamespace

    orientation = msg.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y
                      + orientation.z * orientation.z))
    return SimpleNamespace(
        pose=pose_from_msg(msg.pose),
        width=float(msg.width),
        depth=float(msg.depth),
        height=float(msg.height),
        yaw=yaw,
        yaw_valid=bool(getattr(msg, "yaw_valid", True)),
        detection_id=str(msg.id),
    )


__all__ = [
    "pose_to_msg", "pose_from_msg", "segment_to_msg", "segment_from_msg",
    "pick_from_detected", "SolidPrimitive",
]
