#!/usr/bin/env python3
"""Vacuum controller node: the one VacuumCommand contract for sim + hardware.

/vacuum/command (luggage_msgs/VacuumCommand) is the same signal the real
robot will use - the backend decides what it means:
  sim     -> kinematic follow in gz + PlanningScene attach
  stub    -> state only
  hardware-> (future) GPIO + pressure sensor; node code unchanged.

Publishes /vacuum/state (luggage_msgs/VacuumState, transient-local) at the
follow rate; ``attached`` is the flag the user asked for (sim: box bound to
panel; hardware: suction sensed).

Subscribes /luggage/current_box (spawner JSON: model_name/size/mass/pose)
and reads panel pose from TF (suction_contact_frame in world).
"""

from __future__ import division

import json
import math
import threading

import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String

from luggage_msgs.msg import VacuumState
from luggage_msgs.srv import VacuumCommand

from luggage_planning.current_box_payload import box_from_current_box_payload
from luggage_planning.planning_scene_client import PlanningSceneClient
from luggage_planning.vacuum_backend import SimVacuumBackend, StubVacuumBackend
from luggage_planning.vacuum_gate import VacuumGate

WORLD = "world"


def _pose_to_lists(pose):
    return ([pose.position.x, pose.position.y, pose.position.z],
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w])


class GzSetPoseClient(object):
    """Adapts the bridged /world/<w>/set_pose service to the backend's
    minimal gz_client interface."""

    def __init__(self, node, group, world):
        self._client = node.create_client(
            SetEntityPose, "/world/%s/set_pose" % world,
            callback_group=group)
        self._ready = False

    def set_model_pose(self, model_name, xyz, quat, timeout_sec=0.05):
        if not self._ready:
            if not self._client.wait_for_service(timeout_sec=5.0):
                return False, "set_pose service unavailable"
            self._ready = True
        elif not self._client.service_is_ready():
            return False, "set_pose service unavailable"
        event = threading.Event()
        request = SetEntityPose.Request()
        request.entity.name = model_name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(xyz[0])
        request.pose.position.y = float(xyz[1])
        request.pose.position.z = float(xyz[2])
        request.pose.orientation.x = float(quat[0])
        request.pose.orientation.y = float(quat[1])
        request.pose.orientation.z = float(quat[2])
        request.pose.orientation.w = float(quat[3])
        future = self._client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(float(timeout_sec)):
            return False, "set_pose timeout"
        response = future.result()
        if response is None or not response.success:
            return False, "set_pose rejected"
        return True, ""


class VacuumControllerNode(Node):

    def __init__(self):
        super().__init__("vacuum_controller")
        group = ReentrantCallbackGroup()

        self.declare_parameter("backend", "sim")  # sim | stub
        self.declare_parameter("world", "airport_loading")
        self.declare_parameter("panel_frame", "suction_contact_frame")
        self.declare_parameter("follow_rate_hz", 30.0)
        # Gate parameters (defaults mirror the ROS 1 simulator).
        self.declare_parameter("pressure_kpa", 70.0)
        self.declare_parameter("effective_area_m2", 0.012)
        self.declare_parameter("seal_efficiency", 0.80)
        self.declare_parameter("friction_coefficient", 0.60)
        self.declare_parameter("minimum_retention_margin", 2.0)
        self.declare_parameter("max_suction_tilt_deg", 5.0)
        self.declare_parameter("contact_margin", 0.05)
        self.declare_parameter("contact_xy_margin", 0.05)
        # Spawn AABB vs visible lid: attach often sits ~1 cm into the cube.
        self.declare_parameter("contact_gap_min", -0.02)

        self._panel_frame = str(self.get_parameter("panel_frame").value)
        self._gate = VacuumGate(
            pressure_kpa=float(self.get_parameter("pressure_kpa").value),
            effective_area_m2=float(
                self.get_parameter("effective_area_m2").value),
            seal_efficiency=float(
                self.get_parameter("seal_efficiency").value),
            friction_coefficient=float(
                self.get_parameter("friction_coefficient").value),
            minimum_retention_margin=float(
                self.get_parameter("minimum_retention_margin").value),
            max_suction_tilt_deg=float(
                self.get_parameter("max_suction_tilt_deg").value),
            contact_margin=float(
                self.get_parameter("contact_margin").value),
            contact_xy_margin=float(
                self.get_parameter("contact_xy_margin").value),
            contact_gap_min=float(
                self.get_parameter("contact_gap_min").value))

        self._tf_buffer = tf2_ros.Buffer()
        # Humble TransformListener(spin_thread=True) add_node()s this node.
        # A sidecar keeps /tf off the command/follow executor.
        self._tf_node = Node("vacuum_controller_tf")
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True)
        self._follow_lock = threading.Lock()
        self._follow_skipped = 0

        # Current box from the spawner (transient-local JSON).
        self._box = None
        box_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/luggage/current_box", self._on_current_box, box_qos,
            callback_group=group)

        state_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            VacuumState, "/vacuum/state", state_qos)
        self._diag_pub = self.create_publisher(
            String, "/vacuum/events_json", state_qos)

        scene = PlanningSceneClient(self, callback_group=group)
        backend_name = str(self.get_parameter("backend").value)
        if backend_name == "stub":
            self._backend = StubVacuumBackend()
        else:
            gz_client = GzSetPoseClient(
                self, group, str(self.get_parameter("world").value))
            self._backend = SimVacuumBackend(
                gz_client, scene,
                follow_rate_hz=float(
                    self.get_parameter("follow_rate_hz").value))

        self._vacuum_on = False
        self._last_fail_reason = ""
        self._last_gate = None

        self.create_service(
            VacuumCommand, "/vacuum/command", self._handle_command,
            callback_group=group)

        follow_period = 1.0 / max(1.0, float(
            self.get_parameter("follow_rate_hz").value))
        self.create_timer(follow_period, self._follow_tick,
                          callback_group=group)

        self.get_logger().info(
            "vacuum_controller ready (backend=%s, panel=%s)"
            % (backend_name, self._panel_frame))

    # ------------------------------------------------------------------

    def _on_current_box(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        parsed = box_from_current_box_payload(data)
        self._box = parsed
        if parsed:
            size = parsed["size"]
            self.get_logger().info(
                "current_box model=%s size=%.3fx%.3fx%.3f gen=%s"
                % (parsed["model_name"], size[0], size[1], size[2],
                   parsed["generation"]))
        else:
            self.get_logger().info("current_box cleared")

    def _panel_pose(self):
        try:
            stamp = rclpy.time.Time()
            transform = self._tf_buffer.lookup_transform(
                WORLD, self._panel_frame, stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = transform.transform
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    def _box_pose_lists(self):
        if not self._box:
            return None, None
        return list(self._box["xyz"]), list(self._box["quat"])

    # ------------------------------------------------------------------

    def _handle_command(self, request, response):
        enable = bool(request.enable)
        if not enable:
            ok, message = self._backend.detach({})
            self._vacuum_on = False
            self._last_fail_reason = "" if ok else message
            self._publish_state()
            self._publish_event("detach", ok, message)
            response.success = ok
            response.message = message
            return response

        panel = self._panel_pose()
        box_xyz, box_quat = self._box_pose_lists()
        if panel is None:
            response.success = False
            response.message = "VACUUM_BACKEND_ERROR: panel TF unavailable"
            self._last_fail_reason = response.message
            self._publish_state()
            return response

        size = list(self._box["size"]) if self._box else None
        mass_kg = self._box["mass_kg"] if self._box else 0.0

        radius = 0.0
        if size:
            radius = 0.5 * math.sqrt(sum(float(v) ** 2 for v in size))

        gate = self._gate.evaluate(
            panel[0], panel[1], box_xyz, size, mass_kg, radius)
        self._last_gate = gate
        if not gate.ok:
            self._vacuum_on = False
            self._last_fail_reason = gate.reason
            self._publish_state()
            self._publish_event("attach_gate", False, gate.reason)
            response.success = False
            response.message = gate.reason
            return response

        context = {
            "model_name": self._box.get("model_name", ""),
            "panel_xyz": panel[0], "panel_quat": panel[1],
            "box_xyz": box_xyz, "box_quat": box_quat,
            "box_size": size,
        }
        ok, message = self._backend.attach(context)
        self._vacuum_on = ok
        self._last_fail_reason = "" if ok else message
        self._publish_state()
        self._publish_event("attach", ok, message)
        response.success = ok
        response.message = message
        return response

    # ------------------------------------------------------------------

    def _follow_tick(self):
        if not self._backend.is_attached():
            self._publish_state()
            return
        if not self._follow_lock.acquire(blocking=False):
            self._follow_skipped += 1
            self._publish_event("follow_skip", True, "busy")
            return
        try:
            panel = self._panel_pose()
            if panel is None:
                return
            ok, message = self._backend.follow_step(panel[0], panel[1])
            if not ok:
                self._last_fail_reason = message
            self._publish_state()
        finally:
            self._follow_lock.release()

    def _publish_state(self):
        state = VacuumState()
        state.attached = bool(self._backend.is_attached())
        state.vacuum_on = bool(self._vacuum_on)
        state.fail_reason = self._last_fail_reason or ""
        if self._last_gate is not None:
            state.contact_distance = float(self._last_gate.contact_distance)
            state.retention_margin = float(self._last_gate.retention_margin)
            state.tilt_deg = float(self._last_gate.tilt_deg)
        self._state_pub.publish(state)

    def _publish_event(self, event, ok, message):
        record = {
            "event": event,
            "ok": bool(ok),
            "message": message,
            "follow_skipped": int(self._follow_skipped),
        }
        if self._last_gate is not None:
            record["contact_distance"] = self._last_gate.contact_distance
            record["retention_margin"] = self._last_gate.retention_margin
            record["tilt_deg"] = self._last_gate.tilt_deg
        self._diag_pub.publish(String(data=json.dumps(
            record, sort_keys=True)))


def main(argv=None):
    rclpy.init(args=argv)
    node = VacuumControllerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
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
