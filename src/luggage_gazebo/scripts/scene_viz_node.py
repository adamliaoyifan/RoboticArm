#!/usr/bin/env python3
"""RViz markers for scene assets, spawn GT box, and DetectLuggage result.

Gazebo models (container / platform / suitcase) are not on /robot_description,
so a late RViz window only shows the arm unless something republishes them as
markers. This node is display-only: it does not feed planning or detection.

Topics (RELIABLE + TRANSIENT_LOCAL, same contract as ROS 1 latched markers):

  /luggage/debug/scene_assets     container mesh, pedestal, platform, opening
  /luggage/debug/pickup_box_scene spawn GT wireframe (green)
  /luggage/debug/detected_box     DetectLuggage centre + OBB (cyan)
"""

from __future__ import division

import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from luggage_description.scene_tf_config_utils import (
    container_opening_aperture_corners_in_container,
    gazebo_container_model,
    load_scene_tf_config,
    pedestal_config,
    pedestal_enabled,
    pickup_platform_config,
    pickup_platform_enabled,
)
from luggage_perception.detect_overlay import (
    COLOR_GT_FALLBACK_BGR,
    COLOR_PERCEPTION_BGR,
    OBB_EDGES,
    obb_corners_world,
)


def _latch_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _rgba_bgr(bgr, alpha=1.0):
    b, g, r = bgr
    return ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=float(alpha))


def _stamp_now(node):
    return node.get_clock().now().to_msg()


def _delete_all(frame, stamp, ns=""):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = 0
    marker.action = Marker.DELETEALL
    return marker


def _obb_wireframe(frame, stamp, ns, marker_id, position, quat, size, color,
                   line_width=0.015):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = int(marker_id)
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = float(line_width)
    marker.color = color
    corners = obb_corners_world(position, quat, size)
    for a, b in OBB_EDGES:
        for index in (a, b):
            marker.points.append(Point(
                x=float(corners[index][0]),
                y=float(corners[index][1]),
                z=float(corners[index][2])))
    return marker


class SceneVizNode(Node):

    def __init__(self):
        super().__init__("scene_viz")
        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("container_mesh_package", "luggage_gazebo")
        self.declare_parameter("republish_period_sec", 2.0)

        path = str(self.get_parameter("scene_tf_config").value)
        if not path:
            path = os.path.join(
                get_package_share_directory("luggage_description"),
                "config", "scene_tf.yaml.example")
        self._scene = load_scene_tf_config(path)
        self._world = str(self.get_parameter("world_frame").value)
        self._mesh_pkg = str(self.get_parameter("container_mesh_package").value)

        self._scene_pub = self.create_publisher(
            MarkerArray, "/luggage/debug/scene_assets", _latch_qos())
        self._box_pub = self.create_publisher(
            MarkerArray, "/luggage/debug/pickup_box_scene", _latch_qos())
        self._detect_pub = self.create_publisher(
            MarkerArray, "/luggage/debug/detected_box", _latch_qos())

        self.create_subscription(
            String, "/luggage/current_box", self._on_current_box, _latch_qos())
        self.create_subscription(
            String, "/luggage/perception/detection/latest",
            self._on_detection, _latch_qos())

        period = float(self.get_parameter("republish_period_sec").value)
        self.create_timer(max(0.5, period), self._publish_scene)
        self._publish_scene()
        self.get_logger().info(
            "scene_viz ready (container=%s)" % gazebo_container_model(self._scene))

    def _publish_scene(self):
        stamp = _stamp_now(self)
        array = MarkerArray()
        array.markers.extend(self._container_markers(stamp))
        array.markers.extend(self._opening_markers(stamp))
        array.markers.extend(self._platform_markers(stamp))
        array.markers.extend(self._pedestal_markers(stamp))
        self._scene_pub.publish(array)

    def _container_markers(self, stamp):
        model = gazebo_container_model(self._scene)
        marker = Marker()
        marker.header.frame_id = "container_link"
        marker.header.stamp = stamp
        marker.ns = "container"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color = ColorRGBA(r=0.35, g=0.45, b=0.55, a=0.45)
        marker.mesh_resource = (
            "package://%s/models/%s/meshes/container_visual.stl"
            % (self._mesh_pkg, model))
        marker.mesh_use_embedded_materials = False
        return [marker]

    def _opening_markers(self, stamp):
        corners = container_opening_aperture_corners_in_container(self._scene)
        if not corners:
            return []
        line = Marker()
        line.header.frame_id = "container_link"
        line.header.stamp = stamp
        line.ns = "opening"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.03
        line.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        pts = list(corners) + [corners[0]]
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                       for p in pts]
        label = Marker()
        label.header.frame_id = "container_opening_frame"
        label.header.stamp = stamp
        label.ns = "opening"
        label.id = 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.orientation.w = 1.0
        label.pose.position.z = 0.08
        label.scale.z = 0.08
        label.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        label.text = "opening"
        return [line, label]

    def _box_link_cube(self, stamp, enabled, cfg, frame, ns, color):
        if not enabled:
            return []
        sx, sy, sz = cfg["size"]
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.z = float(sz) * 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(sx)
        marker.scale.y = float(sy)
        marker.scale.z = float(sz)
        marker.color = color
        return [marker]

    def _platform_markers(self, stamp):
        return self._box_link_cube(
            stamp, pickup_platform_enabled(self._scene),
            pickup_platform_config(self._scene),
            "pickup_platform_link", "platform",
            ColorRGBA(r=0.35, g=0.40, b=0.45, a=0.55))

    def _pedestal_markers(self, stamp):
        return self._box_link_cube(
            stamp, pedestal_enabled(self._scene),
            pedestal_config(self._scene),
            "pedestal_link", "pedestal",
            ColorRGBA(r=0.45, g=0.45, b=0.48, a=0.55))

    def _on_current_box(self, msg):
        stamp = _stamp_now(self)
        array = MarkerArray()
        array.markers.append(_delete_all(self._world, stamp, "pickup_box"))
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except ValueError:
            payload = {}
        if not payload or "pose" not in payload:
            self._box_pub.publish(array)
            return
        pos = payload["pose"]["position"]
        ori = payload["pose"]["orientation"]
        position = (pos["x"], pos["y"], pos["z"])
        quat = (ori["x"], ori["y"], ori["z"], ori["w"])
        size = (payload["width"], payload["depth"], payload["height"])
        color = _rgba_bgr(COLOR_GT_FALLBACK_BGR, 0.95)
        array.markers.append(_obb_wireframe(
            self._world, stamp, "pickup_box", 0, position, quat, size, color))
        sphere = Marker()
        sphere.header.frame_id = self._world
        sphere.header.stamp = stamp
        sphere.ns = "pickup_box"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(x=float(position[0]), y=float(position[1]),
                                     z=float(position[2]))
        sphere.pose.orientation = Quaternion(w=1.0)
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05
        sphere.color = color
        array.markers.append(sphere)
        label = Marker()
        label.header.frame_id = self._world
        label.header.stamp = stamp
        label.ns = "pickup_box"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            x=float(position[0]), y=float(position[1]),
            z=float(position[2] + size[2] * 0.5 + 0.08))
        label.pose.orientation.w = 1.0
        label.scale.z = 0.06
        label.color = color
        label.text = "GT %s" % payload.get("id", "box")
        array.markers.append(label)
        self._box_pub.publish(array)

    def _on_detection(self, msg):
        stamp = _stamp_now(self)
        array = MarkerArray()
        array.markers.append(_delete_all(self._world, stamp, "detected_box"))
        try:
            raw = json.loads(msg.data) if msg.data else {}
        except ValueError:
            self._detect_pub.publish(array)
            return
        detected = raw.get("detected") or {}
        position = detected.get("position")
        size = detected.get("size")
        if not raw.get("success") or not position or not size:
            self._detect_pub.publish(array)
            return
        quat = detected.get("orientation") or [0.0, 0.0, 0.0, 1.0]
        color = _rgba_bgr(COLOR_PERCEPTION_BGR, 1.0)
        array.markers.append(_obb_wireframe(
            self._world, stamp, "detected_box", 1,
            position, quat, size, color, line_width=0.02))
        sphere = Marker()
        sphere.header.frame_id = self._world
        sphere.header.stamp = stamp
        sphere.ns = "detected_box"
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = Point(
            x=float(position[0]), y=float(position[1]),
            z=float(position[2]))
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.04
        sphere.color = color
        array.markers.append(sphere)
        label = Marker()
        label.header.frame_id = self._world
        label.header.stamp = stamp
        label.ns = "detected_box"
        label.id = 2
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = Point(
            x=float(position[0]), y=float(position[1]),
            z=float(position[2] + float(size[2]) * 0.5 + 0.08))
        label.pose.orientation.w = 1.0
        label.scale.z = 0.05
        label.color = color
        label.text = "detect"
        array.markers.append(label)
        self._detect_pub.publish(array)


def main(argv=None):
    rclpy.init(args=argv)
    node = SceneVizNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
