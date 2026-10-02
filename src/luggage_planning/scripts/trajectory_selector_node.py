#!/usr/bin/env python3
"""SelectPlacePath services: the unified route producer (ST09-I).

SelectPlacePathV2 plans, certifies, and returns complete typed joint
route legs; this node never executes motion. The legacy in-process
selection is deleted — v1 keeps its frozen surface as a named refusal.
"""

from __future__ import division

import json
import os
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


class TrajectorySelectorNode(Node):
    def __init__(self):
        super().__init__("trajectory_selector")
        self.declare_parameter("scene_tf_config", "")
        self.declare_parameter("atlas_runtime_config", "")
        # Launch sets this only when the caller overrides the yaml. The
        # default True leaves atlas_runtime.yaml in charge.
        self.declare_parameter("atlas_enabled", True)
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
        if not bool(self.get_parameter("atlas_enabled").value):
            self._atlas_cfg = dict(self._atlas_cfg)
            self._atlas_cfg["enabled"] = False
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
        # Dedicated group: the route producer blocks inside these
        # callbacks waiting on move_group/planning-scene responses whose
        # clients live in the default group — sharing one mutually
        # exclusive group would deadlock the responses out.
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        self._service_group = MutuallyExclusiveCallbackGroup()
        self.create_service(
            SelectPlacePath, "/trajectory_selector/select_place_path",
            self._select, callback_group=self._service_group)
        self.create_service(
            SelectPlacePathV2, "/trajectory_selector/select_place_path_v2",
            self._select_v2, callback_group=self._service_group)
        # ST09-I: the unified route producer (off = legacy selection).
        self.declare_parameter("route_planning", True)
        self.declare_parameter("frame_scene_revision", "")
        self._route_producer = None
        self._route_planning = bool(
            self.get_parameter("route_planning").value)
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
        """v1 surface preserved as a named refusal (ST09-I).

        The unified route produces complete typed legs on the v2
        endpoint; the legacy in-process selection was a geometric bypass.
        """
        if getattr(self, "_frame_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                self._frame_runtime.authorize_legacy_effect()
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.rows_json = json.dumps({"error": error.code})
                return response
        response.success = False
        response.reason_code = "PLANNING_ROUTE_REQUIRED"
        response.rows_json = json.dumps(
            {"error": "v1 selection removed; use select_place_path_v2"})
        return response

    def _route_producer_or_none(self):
        """Build the unified route producer once, on first use.

        All collaborators are the accepted components: B's graph/edge
        builder, C's certifier/revalidator via the real MoveIt validity
        adapter, D's measured provider, and the offline URDF/IK stack.
        """
        if self._route_producer is not None:
            return self._route_producer
        from luggage_description.scene_tf_config_utils import (
            container_opening_normal_in_world,
            container_opening_target_point_in_world,
        )
        from luggage_planning.attachment_provider import (
            MeasuredAttachmentProvider,
        )
        from luggage_planning.map_revalidation import MapRevalidator
        from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
        from luggage_planning.moveit_state_validity import (
            MoveItStateValidityAdapter,
        )
        from luggage_planning.offline_ik import load_urdf, refine_to_pose
        from luggage_planning.planning_scene_client import PlanningSceneClient
        from luggage_planning.reachable_graph import (
            RouteGraph,
            make_offline_fk,
        )
        from luggage_planning.route_producer import RouteProducer
        from luggage_planning.trajectory_certifier import TrajectoryCertifier
        from luggage_planning.route_producer import (
            certification_policy_from_contract)
        import json as _json

        try:
            from ament_index_python.packages import get_package_share_directory

            contract_path = os.path.join(
                get_package_share_directory("st09_contracts"),
                "resolved_planning_contract.json")
            with open(contract_path, encoding="utf-8") as handle:
                cert_policy = certification_policy_from_contract(
                    _json.load(handle))
        except Exception:  # noqa: BLE001 - fail closed to C defaults
            self.get_logger().error(
                "route: resolved contract unavailable; refusing to plan")
            return None

        urdf = load_urdf(scene_tf_config=self._scene_tf_path)
        # path_to returns the JOINT sequence to the TCP link (tcp() FK
        # walks joints; the link name itself is not a joint).
        tcp_path = urdf.path_to("suction_contact_frame")
        fk = make_offline_fk(urdf, tcp_path)
        scene_client = PlanningSceneClient(self)

        def refine(position, quaternion, seed):
            return refine_to_pose(urdf, tcp_path, position, quaternion,
                                  seed, pos_tol=2e-3, rot_tol=2e-2)

        def validity_factory(*, scene_states, edges):
            adapter = MoveItStateValidityAdapter(
                self, fk=fk, scene_client=scene_client,
                log=lambda message: self.get_logger().info(message))
            adapter.load_edges(edges, scene_states=scene_states)
            return adapter

        self._route_producer = RouteProducer(
            graph=RouteGraph(static_identity="st09-i-route:%s" %
                             self._scene_tf_hash),
            edge_builder=MoveItEdgeAdapter(self, scene_client=scene_client),
            certifier=TrajectoryCertifier(),
            cert_policy=cert_policy,
            validity_adapter_factory=validity_factory,
            attachment_provider=MeasuredAttachmentProvider(
                str(self.get_parameter("frame_scene_revision").value
                    or "scene-runtime"), "route-selector",
                tcp_frame="suction_contact_frame", box_frame="box"),
            revalidator=MapRevalidator(),
            fk=fk, refine=refine, urdf=urdf, tcp_path=tcp_path,
            clock_ns=lambda: self.get_clock().now().nanoseconds,
            log=lambda message: self.get_logger().info(message),
            opening_info={
                "point": container_opening_target_point_in_world(self._scene),
                "normal": container_opening_normal_in_world(self._scene),
            })
        return self._route_producer

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
        self._runtime_atlas()  # prior diagnostics only; never a gate
        if not self._route_planning:
            response.success = False
            response.reason_code = "PLANNING_ROUTE_REQUIRED"
            response.rows_json = json.dumps(
                {"error": "route_planning is the only production mode"})
            return response
        producer = self._route_producer_or_none()
        if producer is None:
            response.success = False
            response.reason_code = "ROUTE_PRODUCER_UNAVAILABLE"
            return response
        from luggage_planning.place_service import fill_select_response_route
        response = fill_select_response_route(
            request, response, producer=producer)
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


def main():
    rclpy.init()
    from rclpy.executors import MultiThreadedExecutor

    node = TrajectorySelectorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
