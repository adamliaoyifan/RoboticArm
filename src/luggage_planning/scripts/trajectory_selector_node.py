#!/usr/bin/env python3
"""SelectPlacePath service. The algorithm lives in trajectory_selector.

This node does not execute motion. Callers apply the returned carry
segments and send them to PlanMotion.
"""

from __future__ import division

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from geometry_msgs.msg import Pose
from luggage_msgs.msg import MotionSegment
from luggage_msgs.srv import SelectPlacePath, SelectPlacePathV2
from luggage_description.scene_tf_config_utils import (
    load_scene_tf_config,
    origin_in_world,
    resolve_scene_tf_config_path,
)
from luggage_planning.atlas_runtime import (
    certification_block_reason,
    compute_scene_tf_hash,
    load_atlas_runtime_for_scene,
    load_runtime_atlas,
)
from luggage_planning.urdf_identity import UrdfIdentity
from luggage_planning.occupancy_place_paths import dump_rows
from luggage_planning.place_selection_client import write_joint_knots
from luggage_planning.place_service import handle_select_v2
from luggage_planning.trajectory_selector import TrajectorySelector


class TrajectorySelectorNode(Node):
    def __init__(self):
        super().__init__("trajectory_selector")
        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("atlas_runtime_config", "")
        self._selector = TrajectorySelector.from_yaml()
        self._surface = None
        self._joints = None
        self._atlas = None
        self._atlas_diag = {"loaded": False, "reason": "disabled"}
        self._atlas_key = None
        scene_path = str(self.get_parameter("scene_tf_config").value or "")
        if not scene_path:
            scene_path = resolve_scene_tf_config_path()
        self._scene_tf_path = scene_path
        self._scene_tf_hash = compute_scene_tf_hash(scene_path)
        self._scene = load_scene_tf_config(scene_path)
        cfg_path = str(self.get_parameter("atlas_runtime_config").value or "")
        self._atlas_cfg = load_atlas_runtime_for_scene(scene_path, cfg_path or None)
        if str(self._atlas_cfg.get("hash_policy") or "") == "strict":
            self._selector.config["hash_policy"] = "strict"
        self._urdf_identity = UrdfIdentity(self)
        if self._atlas_cfg.get("enabled"):
            self._urdf_identity.start()
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d", self._on_surface, qos)
        self.create_subscription(
            JointState, "/joint_states", self._on_joints, 10)
        self.create_service(
            SelectPlacePath, "/trajectory_selector/select_place_path",
            self._select)
        self.create_service(
            SelectPlacePathV2, "/trajectory_selector/select_place_path_v2",
            self._select_v2)
        # ST09 F21: identity heartbeat + peer readiness (off = legacy).
        self.declare_parameter("frame_runtime", False)
        self.declare_parameter("frame_identity_json", "")
        self.declare_parameter("frame_peers", "")
        self._frame_runtime = None
        if bool(self.get_parameter("frame_runtime").value):
            import sys as _sys
            from luggage_description.ros_frame_bindings.runtime import (
                node_runtime_from_params)
            self._frame_runtime = node_runtime_from_params(
                self,
                resolved_config={
                    "scene_tf_config": str(
                        self.get_parameter("scene_tf_config").value or ""),
                    "atlas_enabled": bool(self._atlas_cfg.get("enabled")),
                },
                modules=[(_sys.modules["__main__"], "module", "application")],
                package_names=["luggage_planning"])
        self.get_logger().info("trajectory_selector up")

    def _runtime_atlas(self):
        cfg = self._atlas_cfg
        if not cfg.get("enabled"):
            return None, {"loaded": False, "reason": "disabled"}
        self._urdf_identity.poll()
        urdf_hash = (
            self._urdf_identity.hash if self._urdf_identity.ready else "")
        key = (self._scene_tf_hash, urdf_hash)
        if key != self._atlas_key:
            self._atlas, self._atlas_diag = load_runtime_atlas(
                cfg, scene_tf_hash=self._scene_tf_hash, urdf_hash=urdf_hash)
            self._atlas_key = key
            self.get_logger().info(
                "atlas load: %s" % (
                    self._atlas_diag.get("reason")
                    or self._atlas_diag.get("hash_message")))
        return self._atlas, self._atlas_diag

    def _on_surface(self, msg):
        try:
            surface = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(surface, dict) and "height" in surface:
            self._surface = surface

    def _on_joints(self, msg):
        names = list(msg.name)
        wanted = ["elfin_joint%d" % i for i in range(1, 7)]
        if msg.position and all(name in names for name in wanted):
            self._joints = [
                float(msg.position[names.index(name)]) for name in wanted]

    def _map_to_world(self, xyz):
        """Map(container)-frame point -> world frame (scene yaw + origin)."""
        origin, rpy = origin_in_world(self._scene)
        yaw = float(rpy[2])
        c, s = math.cos(yaw), math.sin(yaw)
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        return [
            origin[0] + c * x - s * y,
            origin[1] + s * x + c * y,
            origin[2] + z,
        ]

    def _select(self, request, response):
        if getattr(self, "_frame_runtime", None) is not None:
            # v1 cannot bypass profile admission.
            from luggage_description.frame_contracts import ContractError
            try:
                self._frame_runtime.authorize_legacy_effect()
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.rows_json = json.dumps({"error": error.code})
                return response
        surface = self._surface
        if surface is None:
            response.success = False
            response.reason_code = "PLACE_PATH_INFEASIBLE"
            response.rows_json = json.dumps({"error": "surface_2d missing"})
            return response
        slots = []
        scores = list(request.candidate_scores)
        for index, slot in enumerate(request.candidates):
            score = float(scores[index]) if index < len(scores) else 0.0
            pos = slot.place_pose.position
            ori = slot.place_pose.orientation
            slots.append({
                "index": index,
                "score": score,
                "target": (float(pos.x), float(pos.y), float(pos.z)),
                "yaw": math.atan2(
                    2.0 * (float(ori.w) * float(ori.z)
                           + float(ori.x) * float(ori.y)),
                    1.0 - 2.0 * (float(ori.y) ** 2 + float(ori.z) ** 2)),
            })
        if not slots and request.box.width > 0.0:
            response.success = False
            response.reason_code = "PLACE_PATH_INFEASIBLE"
            response.rows_json = json.dumps({"error": "no slots"})
            return response
        start = request.start_pose.position
        portal = (float(start.x), float(start.y), float(start.z))
        payload = (
            float(request.box.width), float(request.box.depth),
            float(request.box.height))
        joints = list(request.start_joints) or self._joints
        atlas, diag = self._runtime_atlas()
        block = certification_block_reason(self._atlas_cfg, diag)
        if block:
            response.success = False
            response.reason_code = block
            response.rows_json = json.dumps({
                "error": block,
                "atlas": diag,
            })
            return response
        urdf_hash = (
            self._urdf_identity.hash if self._urdf_identity.ready else "")
        try:
            winner, rows, reason, _snap = self._selector.select(
                surface, slots, portal, payload, atlas=atlas,
                expected_hash=str(request.geometry_hash or "") or None,
                expected_revision=int(request.map_revision),
                start_joints=joints,
                excluded_ids=list(request.excluded_candidate_ids),
                scene_tf_hash=self._scene_tf_hash,
                urdf_hash=urdf_hash,
            )
        except Exception as exc:  # noqa: BLE001 - service boundary
            response.success = False
            response.reason_code = "PLACE_PATH_INFEASIBLE"
            response.rows_json = json.dumps({"error": str(exc)})
            return response
        response.rows_json = json.dumps(dump_rows(rows))
        response.reason_code = reason or ""
        if winner is None:
            response.success = False
            return response
        try:
            response.carry_segments = _segments(
                winner, map_to_world=self._map_to_world)
        except Exception as exc:  # noqa: BLE001 - service boundary
            # A crash inside response assembly would surface to the client
            # as a timeout/future error instead of a diagnosable failure.
            response.success = False
            response.reason_code = "PLACE_PATH_INFEASIBLE"
            response.rows_json = json.dumps(
                {"error": "segment assembly: %s" % exc})
            return response
        response.success = True
        response.slot_index = int(winner["slot_index"])
        response.selection_id = str(winner.get("selection_id") or "")
        response.path_source = str(winner.get("path_source") or "")
        return response

    def _select_v2(self, request, response):
        # World poses come from transform_pose. v1 keeps _segments.
        # This callback does not execute motion.
        if getattr(self, "_frame_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                admission = self._frame_runtime.admit_request(
                    str(getattr(request, "request_id", "") or ""),
                    endpoint="/trajectory_selector/select_place_path_v2")
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.rows_json = json.dumps({"error": error.code})
                return response
        atlas, diag = self._runtime_atlas()
        block = certification_block_reason(self._atlas_cfg, diag)
        if block:
            response.success = False
            response.reason_code = block
            response.rows_json = json.dumps({"error": block, "atlas": diag})
            return response
        response = handle_select_v2(request, response, atlas=atlas)
        if getattr(self, "_frame_runtime", None) is not None and bool(
                getattr(response, "success", False)):
            from luggage_description.frame_contracts import ContractError
            try:
                self._frame_runtime.authorize_effect(
                    admission, str(getattr(request, "request_id", "") or ""))
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.rows_json = json.dumps({"error": error.code})
        return response


def _segments(winner, map_to_world=None):
    """MotionSegment[] for the winner's carry legs, in the WORLD frame.

    The request's slots are map(container)-frame; the executor plans in
    world, so every waypoint and the target_pose are converted here. A
    cartesian response without waypoints/target_pose would leave the
    executor's default zero pose as the goal -- an invalid-target hazard,
    so the payload is complete by construction. ``map_to_world`` is
    injectable for tests (None = identity).
    """
    import math as _math

    convert = map_to_world or (lambda xyz: [float(v) for v in xyz])
    out = []
    for name in ("transit", "traverse"):
        spec = (winner.get("segments") or {}).get(name) or {}
        waypoints = list(spec.get("waypoints") or [])
        yaw = float(winner.get("yaw") or 0.0)
        half = 0.5 * yaw
        quat = (_math.cos(half), _math.sin(half), 0.0, 0.0)
        segment = MotionSegment()
        segment.name = name
        segment.type = str(spec.get("type") or "cartesian")
        segment.allow_ompl_fallback = bool(
            spec.get("allow_ompl_fallback", False))
        segment.keep_tool_down = True

        def _fill(pose, xyz):
            pose.position.x, pose.position.y, pose.position.z = (
                float(xyz[0]), float(xyz[1]), float(xyz[2]))
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

        for xyz in waypoints[:-1]:
            pose = Pose()
            _fill(pose, convert(xyz))
            segment.waypoints.append(pose)
        if waypoints:
            _fill(segment.target_pose, convert(waypoints[-1]))
        write_joint_knots(segment, spec.get("joint_knots") or [])
        out.append(segment)
    return out


def main():
    rclpy.init()
    node = TrajectorySelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
