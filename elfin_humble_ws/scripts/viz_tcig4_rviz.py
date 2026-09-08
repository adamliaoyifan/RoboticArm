#!/usr/bin/env python3
"""Publish TCIG-4 review findings as RViz markers (display only).

Does not feed planning. Fixed Frame in RViz: container_link.

  /luggage/debug/tcig4_explain

Namespaces (toggle in RViz):
  mesh     container STL (what the arm can hit)
  aabb     inner cuboid the old code treated as usable
  hull     true seven-face inner hull
  opening  door rectangle
  bug1     too-wide first box at the chamfer floor (empty Y interval)
  bug2     same center, yaw 0 (fits) vs yaw 90 (hits slant)
  bug3     ghost AABB in the removed wedge that used to raise carry height
"""

from __future__ import division

import math
import os

import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from luggage_description.scene_tf_config_utils import (
    container_aperture_edges_in_container,
    container_inner_dimensions,
    container_inner_floor_z,
    container_inner_hull_edges_in_container,
    container_inner_ceiling_z,
    gazebo_container_model,
    load_scene_tf_config,
    resolve_scene_tf_config_path,
)


def _latch():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _q_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def _pt(xyz):
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


class Tcig4Viz(Node):
    def __init__(self):
        super().__init__("tcig4_rviz_explain")
        path = resolve_scene_tf_config_path(None)
        self._scene = load_scene_tf_config(path)
        self._pub = self.create_publisher(
            MarkerArray, "/luggage/debug/tcig4_explain", _latch()
        )
        self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info(
            "TCIG-4 RViz explain on /luggage/debug/tcig4_explain (frame=container_link)"
        )

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "container_link"
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        array.markers.extend(self._mesh(stamp))
        array.markers.extend(self._wires(stamp))
        array.markers.extend(self._bug1(stamp))
        array.markers.extend(self._bug2(stamp))
        array.markers.extend(self._bug3(stamp))
        self._pub.publish(array)

    def _mesh(self, stamp):
        marker = Marker()
        marker.header.frame_id = "container_link"
        marker.header.stamp = stamp
        marker.ns = "mesh"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color = ColorRGBA(r=0.55, g=0.62, b=0.70, a=0.22)
        model = gazebo_container_model(self._scene)
        marker.mesh_resource = (
            "package://luggage_gazebo/models/%s/meshes/container_visual.stl"
            % model
        )
        return [marker]

    def _line_list(self, stamp, ns, mid, edges, color, width=0.02):
        marker = Marker()
        marker.header.frame_id = "container_link"
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(width)
        marker.color = color
        for start, end in edges:
            marker.points.append(_pt(start))
            marker.points.append(_pt(end))
        return marker

    def _cube(self, stamp, ns, mid, xyz, size, yaw, color):
        marker = Marker()
        marker.header.frame_id = "container_link"
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = _pt(xyz)
        marker.pose.orientation = _q_yaw(yaw)
        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])
        marker.color = color
        return marker

    def _label(self, stamp, ns, mid, xyz, text, color, scale=0.08):
        marker = Marker()
        marker.header.frame_id = "container_link"
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = mid
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = _pt(xyz)
        marker.pose.orientation.w = 1.0
        marker.scale.z = float(scale)
        marker.color = color
        marker.text = text
        return marker

    def _aabb_edges(self):
        length, width, _h = container_inner_dimensions(self._scene)
        z0 = container_inner_floor_z(self._scene)
        z1 = container_inner_ceiling_z(self._scene)
        hx, hy = length * 0.5, width * 0.5
        corners = [
            (-hx, -hy, z0), (hx, -hy, z0), (hx, hy, z0), (-hx, hy, z0),
            (-hx, -hy, z1), (hx, -hy, z1), (hx, hy, z1), (-hx, hy, z1),
        ]
        edges_i = (
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        )
        return [(corners[a], corners[b]) for a, b in edges_i]

    def _wires(self, stamp):
        items = [
            self._line_list(
                stamp, "aabb", 0, self._aabb_edges(),
                ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0), 0.012,
            ),
            self._line_list(
                stamp, "hull", 0,
                container_inner_hull_edges_in_container(self._scene),
                ColorRGBA(r=0.0, g=0.95, b=0.95, a=1.0), 0.025,
            ),
            self._line_list(
                stamp, "opening", 0,
                container_aperture_edges_in_container(self._scene),
                ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0), 0.03,
            ),
            self._label(
                stamp, "labels", 0, (-0.90, 0.0, 1.35),
                "DOOR -X", ColorRGBA(r=1.0, g=0.9, b=0.1, a=1.0),
            ),
            self._label(
                stamp, "labels", 1, (0.0, 0.95, 0.70),
                "CHAMFER +Y (not real space)",
                ColorRGBA(r=1.0, g=0.4, b=0.2, a=1.0),
            ),
        ]
        return items

    def _bug1(self, stamp):
        # Payload wider than the eroded floor Y span: center has nowhere to go.
        size = (0.40, 1.60, 0.25)
        z0 = container_inner_floor_z(self._scene)
        center = (0.0, 0.10, z0 + size[2] * 0.5)
        return [
            self._cube(
                stamp, "bug1", 0, center, size, 0.0,
                ColorRGBA(r=1.0, g=0.05, b=0.05, a=0.55),
            ),
            self._label(
                stamp, "bug1", 1, (0.15, 0.10, z0 + 0.45),
                "BUG1 first-box clips slant\n(empty ledger still said FREE)",
                ColorRGBA(r=1.0, g=0.3, b=0.3, a=1.0),
            ),
        ]

    def _bug2(self, stamp):
        size = (0.50, 0.16, 0.20)
        center = (-0.05, 0.55, 0.70)
        return [
            self._cube(
                stamp, "bug2", 0, center, size, 0.0,
                ColorRGBA(r=0.1, g=0.85, b=0.2, a=0.55),
            ),
            self._cube(
                stamp, "bug2", 1, (center[0], center[1], 0.95), size, math.pi / 2.0,
                ColorRGBA(r=1.0, g=0.15, b=0.7, a=0.55),
            ),
            self._label(
                stamp, "bug2", 2, (0.20, 0.55, 1.15),
                "BUG2 green yaw=0 fits\npink world-yaw=90 hits slant",
                ColorRGBA(r=1.0, g=0.4, b=0.8, a=1.0),
            ),
        ]

    def _bug3(self, stamp):
        ghost = (0.0, 0.82, 0.72)
        size = (0.20, 0.20, 0.20)
        length, _w, _h = container_inner_dimensions(self._scene)
        corridor = Marker()
        corridor.header.frame_id = "container_link"
        corridor.header.stamp = stamp
        corridor.ns = "bug3"
        corridor.id = 0
        corridor.type = Marker.CUBE
        corridor.action = Marker.ADD
        corridor.pose.position = _pt((-0.35, 0.20, 0.72))
        corridor.pose.orientation.w = 1.0
        corridor.scale.x = 0.80
        corridor.scale.y = 1.20
        corridor.scale.z = 0.30
        corridor.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.12)
        return [
            corridor,
            self._cube(
                stamp, "bug3", 1, ghost, size, 0.0,
                ColorRGBA(r=1.0, g=0.45, b=0.05, a=0.7),
            ),
            self._label(
                stamp, "bug3", 2, (0.05, 0.82, 1.00),
                "BUG3 ghost AABB in wedge\nused to raise carry height",
                ColorRGBA(r=1.0, g=0.55, b=0.1, a=1.0),
            ),
        ]


def main():
    os.environ.setdefault(
        "AMENT_PREFIX_PATH", os.environ.get("AMENT_PREFIX_PATH", "")
    )
    rclpy.init()
    node = Tcig4Viz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
