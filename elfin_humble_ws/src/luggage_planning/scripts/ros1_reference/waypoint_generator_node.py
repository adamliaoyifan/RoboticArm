#!/usr/bin/env python3
"""Build motion sequences and expose as a ROS service with debug visualization."""

import math
import os
import sys

import rospy
import rospkg
import tf2_ros

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from luggage_msgs.msg import MotionSegment
from luggage_msgs.srv import BuildMotionSequence, BuildMotionSequenceResponse

from waypoint_generator import (
    DEFAULT_PICK_CLEARANCES,
    DEFAULT_PLACE_CLEARANCE_Z,
    build_sequence,
    segment_names_for_phase,
)

_DESC_SCRIPTS = os.path.join(
    rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, _DESC_SCRIPTS)
from scene_tf_config_utils import (  # noqa: E402
    container_opening_normal_in_base_link,
    container_opening_target_point,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)

_SEGMENT_COLORS = {
    "pre_grasp": ColorRGBA(r=0.2, g=0.8, b=0.2, a=1.0),
    "approach": ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0),
    "attach": ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
    "pick_retreat": ColorRGBA(r=0.4, g=0.4, b=1.0, a=1.0),
    "retreat": ColorRGBA(r=0.4, g=0.4, b=1.0, a=1.0),
    "transit": ColorRGBA(r=0.2, g=0.8, b=0.2, a=1.0),
    "traverse": ColorRGBA(r=0.2, g=0.7, b=1.0, a=1.0),
    "insert": ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0),
    "descend": ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
}


class WaypointGeneratorNode:
    def __init__(self):
        self._debug_pub = rospy.Publisher(
            "/luggage/debug/pick_targets", MarkerArray, queue_size=1, latch=True
        )
        self._marker_id = 0
        # Defaults come from waypoint_generator so a node started without the
        # launch file behaves identically to one started with it.
        self._pick_clearances = {
            "pre_grasp": float(rospy.get_param(
                "~pick_pre_grasp_clearance",
                DEFAULT_PICK_CLEARANCES["pre_grasp"])),
            "approach": float(rospy.get_param(
                "~pick_approach_clearance",
                DEFAULT_PICK_CLEARANCES["approach"])),
            "attach": float(rospy.get_param(
                "~pick_attach_clearance", DEFAULT_PICK_CLEARANCES["attach"])),
            "pick_retreat": float(rospy.get_param(
                "~pick_retreat_clearance",
                DEFAULT_PICK_CLEARANCES["pick_retreat"])),
            "pre_grasp_min": float(rospy.get_param(
                "~pick_pre_grasp_min",
                DEFAULT_PICK_CLEARANCES["pre_grasp_min"])),
            "approach_min": float(rospy.get_param(
                "~pick_approach_min",
                DEFAULT_PICK_CLEARANCES["approach_min"])),
        }
        self._place_clearance_z = float(rospy.get_param(
            "~place_clearance_z", DEFAULT_PLACE_CLEARANCE_Z))
        scene_path = rospy.get_param(
            "~scene_tf_config",
            rospy.get_param(
                "/luggage/scene_tf_config",
                resolve_scene_tf_config_path()))
        scene_config = load_scene_tf_config(scene_path)
        self._opening_info = {
            "point": container_opening_target_point(scene_config),
            "normal": container_opening_normal_in_base_link(scene_config),
            "outward_clearance": float(rospy.get_param(
                "~place_opening_outward_clearance", 0.15)),
            "min_height_above_opening": float(rospy.get_param(
                "~place_opening_height_clearance", 0.35)),
        }
        self._use_perception_approach = rospy.get_param("~use_perception_approach", True)
        self._suction_frame = rospy.get_param("~suction_frame", "suction_contact_frame")
        self._base_frame = rospy.get_param("~base_frame", "elfin_base_link")
        self._world_frame = rospy.get_param("~world_frame", "world")
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        rospy.loginfo(
            "waypoint clearances: pick=%s place=%.3f perception_approach=%s",
            {k: round(v, 3) for k, v in self._pick_clearances.items()},
            self._place_clearance_z,
            self._use_perception_approach,
        )

    def _next_id(self):
        self._marker_id += 1
        return self._marker_id

    def _publish_pick_markers(self, segments, phase):
        """Publish sphere + label + line-strip markers for each segment target."""
        ma = MarkerArray()
        stamp = rospy.Time.now()
        # Pick targets are world-frame (spawner); place slots are base_link.
        frame = "elfin_base_link" if phase == "place" else self._world_frame
        line_points = []

        for seg in segments:
            if not seg.target_pose or not seg.target_pose.position:
                continue
            p = seg.target_pose.position
            color = _SEGMENT_COLORS.get(seg.name, ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0))

            sphere = Marker()
            sphere.header.frame_id = frame
            sphere.header.stamp = stamp
            sphere.ns = "pick_sphere"
            sphere.id = self._next_id()
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=p.x, y=p.y, z=p.z)
            sphere.pose.orientation = Quaternion(w=1.0)
            sphere.scale.x = 0.08
            sphere.scale.y = 0.08
            sphere.scale.z = 0.08
            sphere.color = color
            sphere.lifetime = rospy.Duration(0)
            ma.markers.append(sphere)

            label = Marker()
            label.header.frame_id = frame
            label.header.stamp = stamp
            label.ns = "pick_label"
            label.id = self._next_id()
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=p.x, y=p.y, z=p.z + 0.12)
            label.pose.orientation = Quaternion(w=1.0)
            label.scale.z = 0.07
            label.text = "%s z=%.3f" % (seg.name, p.z)
            label.color = color
            label.lifetime = rospy.Duration(0)
            ma.markers.append(label)

            line_points.append(Point(x=p.x, y=p.y, z=p.z))

        if len(line_points) > 1:
            line = Marker()
            line.header.frame_id = frame
            line.header.stamp = stamp
            line.ns = "pick_path"
            line.id = self._next_id()
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.points = line_points
            line.scale.x = 0.025
            line.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.7)
            line.pose.orientation = Quaternion(w=1.0)
            line.lifetime = rospy.Duration(0)
            ma.markers.append(line)

        if ma.markers:
            self._debug_pub.publish(ma)

    def _build_perception_info(self, pick):
        """Look up suction-contact Z and compute perception_info for adaptive approach."""
        if not self._use_perception_approach:
            return None
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._world_frame, self._suction_frame, rospy.Time(0), rospy.Duration(0.3),
            )
            suction_z = tf_msg.transform.translation.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

        box_top_z = pick.pose.position.z + max(0.0, pick.height) * 0.5
        return {"box_top_z": box_top_z, "suction_z": suction_z}

    def _current_tool_yaw(self, frame):
        """Z yaw of the suction frame in *frame*, or 0 if TF is missing."""
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                frame, self._suction_frame,
                rospy.Time(0), rospy.Duration(0.3),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return 0.0
        q = tf_msg.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def handle(self, req):
        perception_info = None
        fallback_yaw = 0.0
        if req.phase == "pick":
            perception_info = self._build_perception_info(req.pick)
            fallback_yaw = self._current_tool_yaw(self._world_frame)

        opening_info = None
        if req.phase == "place":
            fallback_yaw = self._current_tool_yaw(self._base_frame)
            opening_info = dict(self._opening_info)
            try:
                current = self._tf_buffer.lookup_transform(
                    self._base_frame, self._suction_frame,
                    rospy.Time(0), rospy.Duration(0.3))
                opening_info["start_point"] = [
                    current.transform.translation.x,
                    current.transform.translation.y,
                    current.transform.translation.z,
                ]
            except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass
        segments = build_sequence(
            req.pick,
            req.place_slot,
            req.phase,
            pick_clearances=self._pick_clearances,
            place_clearance_z=self._place_clearance_z,
            perception_info=perception_info,
            opening_info=opening_info,
            fallback_yaw=fallback_yaw,
        )
        if not segments:
            for name in segment_names_for_phase(req.phase):
                seg = MotionSegment(name=name, type="pose_target")
                segments.append(seg)

        self._publish_pick_markers(segments, req.phase)
        return BuildMotionSequenceResponse(segments=segments, success=True, message="ok")



# Log level must be chosen before init_node, so it cannot come from a private
# param; log_level_utils reads the LUGGAGE_LOG_LEVEL environment variable.
import os as _os
import sys as _sys
import rospkg as _rospkg
_DESC = _os.path.join(
    _rospkg.RosPack().get_path("luggage_description"), "scripts")
if _DESC not in _sys.path:
    _sys.path.insert(0, _DESC)
from log_level_utils import resolve_log_level  # noqa: E402

def main():
    rospy.init_node("waypoint_generator", log_level=resolve_log_level())
    node = WaypointGeneratorNode()
    rospy.Service("~build_motion_sequence", BuildMotionSequence, node.handle)
    rospy.loginfo("waypoint_generator ready")
    rospy.spin()


if __name__ == "__main__":
    main()
