#!/usr/bin/env python3
"""BuildMotionSequence service shell (ROS 2 Humble port of the rospy node).

Thin node: converts the request to plain dataclasses, calls the pure
``build_sequence`` (unchanged algorithm module), converts back. Publishes
the ROS 1 ``/luggage/debug/pick_targets`` MarkerArray so RViz can show
the waypoint path without Gazebo GUI.

Place slots are converted into ``world`` here (the unique frame boundary).
Producers may emit ``elfin_base_link``; ``build_sequence`` and the executor
always see world poses.
"""

from __future__ import division

import json
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

from luggage_description.box_catalog_utils import (
    box_catalog_path_from_scene,
    box_size_range,
    load_box_catalog,
)
from luggage_description.scene_tf_config_utils import (
    _point_in_container_link,
    container_inner_ceiling_z,
    container_inner_dimensions,
    container_inner_floor_z,
    container_opening_normal_in_world,
    container_opening_target_point_in_world,
    load_scene_tf_config,
    origin_in_world,
    resolve_scene_tf_config_path,
    xyz_world_to_base_link,
)
from luggage_perception.corridor_audit import (
    corridor_aabb,
    corridor_surface_max,
)
from luggage_msgs.msg import DetectionFrame
from luggage_msgs.srv import BuildMotionSequence

from luggage_perception.locked_stamp_window import LockedStampWindow
from luggage_planning import ros_message_adapters as adapters
from luggage_planning.waypoint_generator import (
    DEFAULT_PICK_CLEARANCES,
    DEFAULT_PLACE_CLEARANCE_Z,
    build_sequence,
    staging_offset,
)

try:
    import tf2_geometry_msgs  # noqa: F401
except ImportError:
    tf2_geometry_msgs = None

_SEGMENT_COLORS = {
    "pre_grasp": ColorRGBA(r=0.2, g=0.8, b=0.2, a=1.0),
    "approach": ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0),
    "attach": ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
    "pick_retreat": ColorRGBA(r=0.4, g=0.4, b=1.0, a=1.0),
    "retreat": ColorRGBA(r=0.4, g=0.4, b=1.0, a=1.0),
    "stage_mid": ColorRGBA(r=0.6, g=0.6, b=0.2, a=1.0),
    "stage_late": ColorRGBA(r=0.5, g=0.7, b=0.2, a=1.0),
    "stage": ColorRGBA(r=0.3, g=0.8, b=0.4, a=1.0),
    "transit": ColorRGBA(r=0.2, g=0.8, b=0.2, a=1.0),
    "traverse": ColorRGBA(r=0.2, g=0.7, b=1.0, a=1.0),
    "insert": ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0),
    "descend": ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0),
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
        self.declare_parameter("place_clearance_z",
                               float(DEFAULT_PLACE_CLEARANCE_Z))
        self.declare_parameter("place_opening_outward_clearance", 0.15)
        self.declare_parameter("place_opening_height_clearance", 0.35)
        self.declare_parameter("place_stage_outward_clearance", 0.65)
        self.declare_parameter("place_stage_height_above_opening", 0.65)
        self.declare_parameter("use_perception_approach", False)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("place_slot_frame", "elfin_base_link")
        self.declare_parameter("suction_frame", "suction_contact_frame")
        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter(
            "detection_frame_topic", "/luggage/perception/detection_frame")
        self.declare_parameter("detection_window_maxlen", 10)
        self.declare_parameter("detection_lookup_max_dt", 0.1)

        self._world_frame = str(self.get_parameter("world_frame").value)
        self._place_slot_frame = str(
            self.get_parameter("place_slot_frame").value)
        self._suction_frame = str(self.get_parameter("suction_frame").value)
        scene_path = str(self.get_parameter("scene_tf_config").value or "")
        self._scene_config = load_scene_tf_config(
            resolve_scene_tf_config_path(scene_path or None))
        inner = container_inner_dimensions(self._scene_config)
        floor_z = container_inner_floor_z(self._scene_config)
        ceiling_z = container_inner_ceiling_z(self._scene_config)
        self._inner_size = [
            float(inner[0]), float(inner[1]), float(ceiling_z - floor_z)]
        self._smallest_box = self._smallest_box_size()
        self._committed_boxes = []

        self._frame_window = LockedStampWindow(
            maxlen=max(1, int(self.get_parameter("detection_window_maxlen").value)))
        self._lookup_max_dt = float(
            self.get_parameter("detection_lookup_max_dt").value)

        latch = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._debug_pub = self.create_publisher(
            MarkerArray, "/luggage/debug/pick_targets", latch)

        stream_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            DetectionFrame,
            self.get_parameter("detection_frame_topic").value,
            self._on_detection_frame, stream_qos)
        self.create_subscription(
            String, "/luggage/cargo_map/committed",
            self._on_committed, latch, callback_group=group)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_node = Node("waypoint_generator_tf")
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True)

        self.create_service(
            BuildMotionSequence, "/waypoint_generator/build_motion_sequence",
            self._handle, callback_group=group)
        self.get_logger().info(
            "waypoint_generator ready (clearances %s, place_slot_frame=%s)"
            % (DEFAULT_PICK_CLEARANCES, self._place_slot_frame))

    def _smallest_box_size(self):
        try:
            catalog = load_box_catalog(
                box_catalog_path_from_scene(self._scene_config))
            return [low for low, _high in box_size_range(catalog)]
        except Exception:  # noqa: BLE001 - corridor probe fallback
            return [0.55, 0.40, 0.25]

    def _on_committed(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return
        boxes = payload.get("boxes") or []
        ledger = []
        for box in boxes:
            center = box.get("center")
            size = box.get("size")
            if center is None or size is None:
                continue
            ledger.append((list(center), list(size)))
        self._committed_boxes = ledger

    def _corridor_surface_max_world(self, slot):
        """Highest committed top along the opening corridor, world Z.

        Committed centers are container_link; slot pose is already world.
        """
        if not self._committed_boxes:
            return None
        pos = slot.place_pose.position
        base = xyz_world_to_base_link(
            self._scene_config, [pos.x, pos.y, pos.z])
        local = _point_in_container_link(base, self._scene_config)
        aabb = corridor_aabb(
            local, [slot.width, slot.depth, slot.height],
            self._inner_size, self._smallest_box)
        surface_local = corridor_surface_max(self._committed_boxes, aabb)
        if surface_local is None:
            return None
        origin, _rpy = origin_in_world(self._scene_config)
        return float(surface_local) + float(origin[2])

    def _on_detection_frame(self, msg):
        stamp = (
            float(msg.header.stamp.sec)
            + 1e-9 * float(msg.header.stamp.nanosec))
        self._frame_window.push(stamp, msg)

    def lookup_detection_frame(self, stamp, max_dt=None):
        """Return the nearest in-memory DetectionFrame, or None."""
        dt = self._lookup_max_dt if max_dt is None else float(max_dt)
        hit = self._frame_window.nearest(stamp, dt)
        if hit is None:
            return None
        return hit[1]

    def _clearances(self):
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

    def _opening_info(self):
        info = {
            "point": container_opening_target_point_in_world(self._scene_config),
            "normal": container_opening_normal_in_world(self._scene_config),
            "outward_clearance": float(
                self.get_parameter("place_opening_outward_clearance").value),
            "min_height_above_opening": float(
                self.get_parameter("place_opening_height_clearance").value),
            "stage_outward_clearance": float(
                self.get_parameter("place_stage_outward_clearance").value),
            "stage_height_above_opening": float(
                self.get_parameter("place_stage_height_above_opening").value),
        }
        try:
            transform = self._tf_buffer.lookup_transform(
                self._world_frame, self._suction_frame,
                rclpy.time.Time())
            t = transform.transform.translation
            info["start_point"] = [float(t.x), float(t.y), float(t.z)]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass
        return info

    def _pose_to_world(self, pose, source_frame):
        if not source_frame or source_frame == self._world_frame:
            return pose, None
        stamped = PoseStamped()
        stamped.header.frame_id = source_frame
        stamped.pose = pose
        try:
            out = self._tf_buffer.transform(
                stamped, self._world_frame, rclpy.duration.Duration(seconds=0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, AttributeError) as exc:
            return None, str(exc)
        return out.pose, None

    def _slot_in_world(self, slot):
        pose, err = self._pose_to_world(slot.place_pose, self._place_slot_frame)
        if err:
            return None, err
        slot.place_pose = pose
        return slot, None

    def _handle(self, request, response):
        try:
            phase = str(request.phase) or "pick"
            pick = adapters.pick_from_detected(request.pick)
            place_slot = request.place_slot
            opening = None
            fallback_yaw = 0.0
            notes = []
            if phase == "place":
                place_slot, err = self._slot_in_world(place_slot)
                if err:
                    response.segments = []
                    response.success = False
                    response.message = "place_slot TF %s -> %s failed: %s" % (
                        self._place_slot_frame, self._world_frame, err)
                    return response
                opening = self._opening_info()
                offset = staging_offset(
                    opening["normal"],
                    opening["stage_outward_clearance"])
                opening["stage_offset"] = offset
                mag = math.sqrt(sum(v * v for v in offset))
                if mag < 0.1:
                    notes.append("staging_degenerate mag=%.3f" % mag)
                try:
                    suction = self._tf_buffer.lookup_transform(
                        self._world_frame, self._suction_frame,
                        rclpy.time.Time())
                    q = suction.transform.rotation
                    fallback_yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    fallback_yaw = 0.0
                surface_max = self._corridor_surface_max_world(place_slot)
                if surface_max is not None:
                    notes.append("corridor_surface_max=%.3f" % surface_max)
            else:
                surface_max = None
            segments = build_sequence(
                pick, place_slot, phase,
                pick_clearances=self._clearances(),
                place_clearance_z=float(
                    self.get_parameter("place_clearance_z").value),
                opening_info=opening,
                fallback_yaw=fallback_yaw,
                corridor_surface_max=surface_max)
            response.segments = [adapters.segment_to_msg(s) for s in segments]
            response.success = bool(segments)
            message = (
                "%d segments for phase=%s" % (len(segments), phase)
                if segments else "empty sequence for phase=%s" % phase)
            if notes:
                message = "%s; %s" % (message, "; ".join(notes))
            response.message = message
            self._publish_pick_markers(response.segments, phase)
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.segments = []
            response.success = False
            response.message = "build failed: %s" % exc
            self.get_logger().error(response.message)
        return response

    def _publish_pick_markers(self, segments, phase):
        del phase  # place and pick both live in world after D-1
        stamp = self.get_clock().now().to_msg()
        frame = self._world_frame
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
        listener = getattr(node, "_tf_listener", None)
        tf_exec = getattr(listener, "executor", None) if listener else None
        if tf_exec is not None:
            tf_exec.shutdown()
            thread = getattr(listener, "dedicated_listener_thread", None)
            if thread is not None:
                thread.join(timeout=2.0)
        tf_node = getattr(node, "_tf_node", None)
        node.destroy_node()
        if tf_node is not None:
            tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
