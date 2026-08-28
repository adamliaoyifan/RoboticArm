#!/usr/bin/env python3
"""BuildMotionSequence service shell (ROS 2 Humble port of the rospy node).

Thin node: converts the request to plain dataclasses, calls the pure
``build_sequence`` (unchanged algorithm module), converts back. Publishes
the ROS 1 ``/luggage/debug/pick_targets`` MarkerArray (pre_grasp through
pick_retreat) so RViz can show the waypoint path without Gazebo GUI.
"""

from __future__ import division

import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from luggage_msgs.srv import BuildMotionSequence

from luggage_planning import ros_message_adapters as adapters
from luggage_planning.waypoint_generator import (
    DEFAULT_PICK_CLEARANCES,
    build_sequence,
)

_SEGMENT_COLORS = {
    "pre_grasp": ColorRGBA(r=0.2, g=0.8, b=0.2, a=1.0),
    "approach": ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0),
    "attach": ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
    "pick_retreat": ColorRGBA(r=0.4, g=0.4, b=1.0, a=1.0),
    "retreat": ColorRGBA(r=0.4, g=0.4, b=1.0, a=1.0),
}


class WaypointGeneratorNode(Node):

    def __init__(self):
        super().__init__("waypoint_generator")
        group = ReentrantCallbackGroup()

        self.declare_parameter("pick_pre_grasp_clearance",
                               float(DEFAULT_PICK_CLEARANCES["pre_grasp"]))
        self.declare_parameter("pick_approach_clearance",
                               float(DEFAULT_PICK_CLEARANCES["approach"]))
        self.declare_parameter("pick_attach_clearance",
                               float(DEFAULT_PICK_CLEARANCES["attach"]))
        self.declare_parameter("pick_retreat_clearance",
                               float(DEFAULT_PICK_CLEARANCES["pick_retreat"]))
        self.declare_parameter("pick_pre_grasp_min",
                               float(DEFAULT_PICK_CLEARANCES["pre_grasp_min"]))
        self.declare_parameter("pick_approach_min",
                               float(DEFAULT_PICK_CLEARANCES["approach_min"]))
        # Fixed-clearance mode this round; the perception-adaptive branch
        # (_perception_clearances) adds a TF dependency deliberately deferred.
        self.declare_parameter("use_perception_approach", False)
        self.declare_parameter("world_frame", "world")

        latch = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._debug_pub = self.create_publisher(
            MarkerArray, "/luggage/debug/pick_targets", latch)

        self.create_service(
            BuildMotionSequence, "/waypoint_generator/build_motion_sequence",
            self._handle, callback_group=group)
        self.get_logger().info(
            "waypoint_generator ready (clearances %s)"
            % DEFAULT_PICK_CLEARANCES)

    def _clearances(self):
        # Start from the module defaults (all keys, including the *_min
        # guards) and override the four user-facing knobs.
        clearances = dict(DEFAULT_PICK_CLEARANCES)
        clearances["pre_grasp"] = float(
            self.get_parameter("pick_pre_grasp_clearance").value)
        clearances["approach"] = float(
            self.get_parameter("pick_approach_clearance").value)
        clearances["attach"] = float(
            self.get_parameter("pick_attach_clearance").value)
        clearances["pick_retreat"] = float(
            self.get_parameter("pick_retreat_clearance").value)
        clearances["pre_grasp_min"] = float(
            self.get_parameter("pick_pre_grasp_min").value)
        clearances["approach_min"] = float(
            self.get_parameter("pick_approach_min").value)
        return clearances

    def _handle(self, request, response):
        try:
            phase = str(request.phase) or "pick"
            pick = adapters.pick_from_detected(request.pick)
            segments = build_sequence(
                pick, None, phase, pick_clearances=self._clearances())
            response.segments = [adapters.segment_to_msg(s) for s in segments]
            response.success = bool(segments)
            response.message = (
                "%d segments for phase=%s" % (len(segments), phase)
                if segments else "empty sequence for phase=%s" % phase)
            self._publish_pick_markers(response.segments, phase)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.segments = []
            response.success = False
            response.message = "build failed: %s" % exc
            self.get_logger().error(response.message)
        return response

    def _publish_pick_markers(self, segments, phase):
        stamp = self.get_clock().now().to_msg()
        frame = (
            "elfin_base_link" if phase == "place"
            else str(self.get_parameter("world_frame").value))
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        line_points = []
        for index, seg in enumerate(segments):
            p = seg.target_pose.position
            color = _SEGMENT_COLORS.get(
                seg.name, ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0))
            sphere = Marker()
            sphere.header.frame_id = frame
            sphere.header.stamp = stamp
            sphere.ns = "pick_sphere"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position = Point(x=p.x, y=p.y, z=p.z)
            sphere.pose.orientation = Quaternion(w=1.0)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.08
            sphere.color = color
            array.markers.append(sphere)
            label = Marker()
            label.header.frame_id = frame
            label.header.stamp = stamp
            label.ns = "pick_label"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=p.x, y=p.y, z=p.z + 0.12)
            label.pose.orientation = Quaternion(w=1.0)
            label.scale.z = 0.07
            label.text = "%s z=%.3f" % (seg.name, p.z)
            label.color = color
            array.markers.append(label)
            line_points.append(Point(x=p.x, y=p.y, z=p.z))
        if len(line_points) > 1:
            line = Marker()
            line.header.frame_id = frame
            line.header.stamp = stamp
            line.ns = "pick_path"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.points = line_points
            line.scale.x = 0.025
            line.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.7)
            line.pose.orientation = Quaternion(w=1.0)
            array.markers.append(line)
        self._debug_pub.publish(array)


def main(argv=None):
    rclpy.init(args=argv)
    node = WaypointGeneratorNode()
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
