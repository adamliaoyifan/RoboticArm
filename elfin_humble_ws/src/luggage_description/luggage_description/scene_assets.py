"""RViz markers for the hardware scene (container, opening, pedestal, platform).

Gazebo models are not on /robot_description. These markers let a live RViz
window show the cell next to the CPS-driven arm. Display only.
"""

from __future__ import division

import os

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from luggage_description._share import package_share
from luggage_description.scene_mesh_utils import container_visual_mesh_path
from luggage_description.scene_tf_config_utils import (
    container_inner_hull_edges_in_container,
    container_opening_aperture_corners_in_container,
    gazebo_container_model,
    pedestal_config,
    pedestal_enabled,
    pickup_platform_config,
    pickup_platform_enabled,
)


def container_visual_mesh_resource(scene_config):
    """package:// URI if installed, else file:// from the Gazebo source tree."""
    model = gazebo_container_model(scene_config)
    try:
        installed = os.path.join(
            package_share("luggage_description"),
            "meshes",
            model,
            "container_visual.stl",
        )
        if os.path.isfile(installed):
            return (
                "package://luggage_description/meshes/%s/container_visual.stl"
                % model
            )
    except Exception:
        pass
    path = container_visual_mesh_path(scene_config)
    if os.path.isfile(path):
        return "file://" + os.path.abspath(path)
    return ""


def _pt(xyz):
    return Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))


def _box_link_cube(stamp, enabled, cfg, frame, ns, color):
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


def build_scene_asset_markers(scene_config, stamp):
    """Markers in container_link / pedestal_link / pickup_platform_link."""
    markers = []
    mesh_uri = container_visual_mesh_resource(scene_config)
    if mesh_uri:
        mesh = Marker()
        mesh.header.frame_id = "container_link"
        mesh.header.stamp = stamp
        mesh.ns = "container"
        mesh.id = 0
        mesh.type = Marker.MESH_RESOURCE
        mesh.action = Marker.ADD
        mesh.pose.orientation.w = 1.0
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
        mesh.color = ColorRGBA(r=0.35, g=0.45, b=0.55, a=0.45)
        mesh.mesh_resource = mesh_uri
        mesh.mesh_use_embedded_materials = False
        markers.append(mesh)
    hull = Marker()
    hull.header.frame_id = "container_link"
    hull.header.stamp = stamp
    hull.ns = "hull"
    hull.id = 0
    hull.type = Marker.LINE_LIST
    hull.action = Marker.ADD
    hull.pose.orientation.w = 1.0
    hull.scale.x = 0.02
    hull.color = ColorRGBA(r=0.0, g=0.85, b=0.9, a=1.0)
    for start, end in container_inner_hull_edges_in_container(scene_config):
        hull.points.append(_pt(start))
        hull.points.append(_pt(end))
    markers.append(hull)
    corners = container_opening_aperture_corners_in_container(scene_config)
    if corners:
        opening = Marker()
        opening.header.frame_id = "container_link"
        opening.header.stamp = stamp
        opening.ns = "opening"
        opening.id = 0
        opening.type = Marker.LINE_STRIP
        opening.action = Marker.ADD
        opening.pose.orientation.w = 1.0
        opening.scale.x = 0.03
        opening.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        pts = list(corners) + [corners[0]]
        opening.points = [_pt(p) for p in pts]
        markers.append(opening)
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
        markers.append(label)
    markers.extend(
        _box_link_cube(
            stamp,
            pedestal_enabled(scene_config),
            pedestal_config(scene_config),
            "pedestal_link",
            "pedestal",
            ColorRGBA(r=0.45, g=0.45, b=0.48, a=0.55),
        )
    )
    markers.extend(
        _box_link_cube(
            stamp,
            pickup_platform_enabled(scene_config),
            pickup_platform_config(scene_config),
            "pickup_platform_link",
            "platform",
            ColorRGBA(r=0.35, g=0.40, b=0.45, a=0.55),
        )
    )
    return markers


def build_scene_asset_array(scene_config, stamp):
    array = MarkerArray()
    array.markers = build_scene_asset_markers(scene_config, stamp)
    return array


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config,
        resolve_scene_tf_config_path,
    )

    class SceneAssetsNode(Node):
        def __init__(self):
            super().__init__("scene_assets")
            self.declare_parameter("scene_tf_config", "")
            self.declare_parameter("republish_period_sec", 2.0)
            path = resolve_scene_tf_config_path(
                str(self.get_parameter("scene_tf_config").value) or None
            )
            self._scene = load_scene_tf_config(path)
            qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._pub = self.create_publisher(
                MarkerArray, "/luggage/debug/scene_assets", qos
            )
            period = float(self.get_parameter("republish_period_sec").value)
            self.create_timer(max(0.5, period), self._publish)
            self._publish()
            self.get_logger().info(
                "scene_assets on /luggage/debug/scene_assets (container=%s mesh=%s)"
                % (
                    gazebo_container_model(self._scene),
                    bool(container_visual_mesh_resource(self._scene)),
                )
            )

        def _publish(self):
            self._pub.publish(
                build_scene_asset_array(
                    self._scene, self.get_clock().now().to_msg()
                )
            )

    rclpy.init(args=argv)
    node = SceneAssetsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
