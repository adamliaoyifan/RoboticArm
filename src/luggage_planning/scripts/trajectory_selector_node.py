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
        # [ST09-I S5.7] pre-execution re-verification of the selected
        # certified route (certify A, execute A — never a second path).
        from luggage_msgs.srv import ValidateSelectedRoute
        self.create_service(
            ValidateSelectedRoute,
            "/trajectory_selector/validate_selected_route",
            self._verify_selected, callback_group=self._service_group)
        # [ST09-I S5.7-4] certified observation motion (the driver +
        # ObservationPolicy declare WHERE; this node builds + certifies).
        from luggage_msgs.srv import PlanObservationRouteV2
        self.create_service(
            PlanObservationRouteV2,
            "/trajectory_selector/plan_observation_route",
            self._plan_observation, callback_group=self._service_group)
        # [ST09-I S5.7-3b] execution-admission liveness (the pure
        # executor checks here between legs; certify A, execute A, and
        # the certification still holds when A executes).
        from luggage_msgs.srv import CheckExecutionAdmission
        self.create_service(
            CheckExecutionAdmission,
            "/trajectory_selector/check_execution_admission",
            self._check_execution_admission,
            callback_group=self._service_group)
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
            pedestal_dimensions,
            pedestal_enabled,
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
        from luggage_planning.route_producer import (
            certification_policy_from_contract,
            revalidation_policy_from_contract,
        )
        from luggage_planning.trajectory_certifier import TrajectoryCertifier
        import json as _json

        try:
            from ament_index_python.packages import get_package_share_directory

            contract_path = os.path.join(
                get_package_share_directory("st09_contracts"),
                "resolved_planning_contract.json")
            with open(contract_path, encoding="utf-8") as handle:
                contract_document = _json.load(handle)
            cert_policy = certification_policy_from_contract(
                contract_document)
            revalidation_policy = revalidation_policy_from_contract(
                contract_document)
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

        def latest_snapshot():
            """Fresh mapper state at admission time — never the planned
            snapshot. The stats service supplies the live map revision
            and the last REAL observation stamp (the surface records
            are scene-basis stamp-0 by contract); the grid is the
            node's current latched v1 surface. Any unavailable piece
            fails the revalidation closed."""
            import threading

            from luggage_msgs.srv import GetCargoMapStats
            from luggage_perception.cargo_view_integration import (
                planning_snapshot_from_surface,
            )

            surface = dict(self._surface or {})
            if not surface:
                return None
            if not hasattr(self, "_stats_client"):
                self._stats_client = self.create_client(
                    GetCargoMapStats, "/cargo_map/get_stats")
            if not self._stats_client.wait_for_service(timeout_sec=1.0):
                return None
            event = threading.Event()
            future = self._stats_client.call_async(
                GetCargoMapStats.Request())
            future.add_done_callback(lambda _f: event.set())
            if not event.wait(timeout=2.0) or not future.done():
                return None
            stats = future.result()
            if not stats.success:
                return None
            surface.update({
                "acquisition_stamp_ns": int(
                    stats.last_acquisition_stamp_ns),
                "map_revision": int(stats.map_revision),
                # same fallback the planned snapshot uses (the v1 dict
                # carries no separate commit ledger revision)
                "commit_revision": int(stats.map_revision),
                "geometry_hash": str(stats.geometry_hash),
            })
            return planning_snapshot_from_surface(
                surface,
                scene_revision=str(
                    self.get_parameter("frame_scene_revision").value or ""),
                request_id="latest-snapshot")

        def refine(position, quaternion, seed):
            return refine_to_pose(urdf, tcp_path, position, quaternion,
                                  seed, pos_tol=2e-3, rot_tol=2e-2)

        def atlas_prior(targets, request):
            """[ST09-I P3] data-driven reachability prior (empty-load
            atlas: arm truth without payload-envelope conservatism).

            Queries the REAL TCP targets — the ones
            ``_expand_targets`` synthesized through the measured
            attachment (T_world_box x inverse(T_tcp_box): grasp yaw
            deviation, eccentric XY, and the actual tool rotation are
            all composed in) — never the raw slot box pose. Status
            codes: 3 REACHABLE / 2 MARGINAL / 1 UNREACHABLE / 0
            UNKNOWN. A candidate is reported unreachable ONLY when
            EVERY allowed TCP target queries as sampled-unreachable;
            any miss stays unknown — atlas miss is unknown, never
            unreachable. Reachable targets return their contact seeds
            (the atlas' own sampled solutions) for the goal IK, which
            re-refines every seed onto the exact real target and FK
            verifies. MoveIt and certification stay the authority for
            every leg that actually runs.
            """
            atlas, _diag = self._runtime_atlas()
            if atlas is None or not targets:
                return None
            import math as _math

            from st09_contracts.attachment import (
                compose_transform,
                invert_transform,
            )

            transform = request["container_transform"]
            if isinstance(transform, dict):
                translation = tuple(float(v) for v in
                                    transform["translation"])
                quaternion = tuple(float(v) for v in
                                   transform["quaternion_xyzw"])
            else:
                translation = tuple(float(v) for v in transform[2])
                quaternion = tuple(float(v) for v in transform[3])
            inv_xyz, inv_quat = invert_transform(translation, quaternion)

            statuses = []
            seeds = []
            try:
                for target in targets:
                    # world TCP pose -> container frame, query at the
                    # target's OWN yaw (the heading expansion already
                    # enumerates the allowed physical rotations)
                    position, quat = compose_transform(
                        inv_xyz, inv_quat,
                        [float(v) for v in target.position],
                        [float(v) for v in target.quaternion_xyzw])
                    yaw = _math.atan2(
                        2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                        1.0 - 2.0 * (quat[1] ** 2 + quat[2] ** 2))
                    result = atlas.query(
                        float(position[0]), float(position[1]),
                        float(position[2]), yaw)
                    status = int(result.status)
                    statuses.append(status)
                    if status in (2, 3):
                        for seed in (getattr(result, "contact_seeds",
                                             None) or ()):
                            seeds.append(tuple(float(v) for v in seed))
            except Exception:  # noqa: BLE001 - prior fails open
                return {"verdict": "unknown", "seeds": []}
            if statuses and all(status == 1 for status in statuses):
                return {"verdict": "unreachable", "seeds": []}
            verdict = ("reachable"
                       if any(status in (2, 3) for status in statuses)
                       else "unknown")
            return {"verdict": verdict, "seeds": seeds}

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
            revalidator=MapRevalidator(revalidation_policy),
            latest_snapshot_fn=latest_snapshot,
            fk=fk, refine=refine, urdf=urdf, tcp_path=tcp_path,
            clock_ns=lambda: self.get_clock().now().nanoseconds,
            log=lambda message: self.get_logger().info(message),
            opening_info={
                "point": container_opening_target_point_in_world(self._scene),
                "normal": container_opening_normal_in_world(self._scene),
            },
            # [ST09-I loop] the pedestal top is the open-air transit
            # floor: with it the route seeds a high staging anchor so
            # the carried payload never drags across the pedestal.
            transit_clearance_z=(
                pedestal_dimensions(self._scene)[2]
                if pedestal_enabled(self._scene) else None),
            atlas_prior=atlas_prior)
        return self._route_producer

    def _verify_selected(self, request, response):
        """Re-verify the last admitted selection by digest."""
        producer = self._route_producer_or_none()
        if producer is None:
            response.valid = False
            response.reason_code = "ROUTE_PRODUCER_UNAVAILABLE"
            return response
        try:
            ok, reason, detail = producer.verify_selected(
                route_digest=request.route_digest,
                current_joints=list(request.current_joint_state),
                latest_map_revision=int(request.latest_map_revision),
                scene_revision=str(request.scene_revision))
        except Exception as error:  # noqa: BLE001 - service boundary
            import traceback

            self.get_logger().error(
                "route: verify boundary exception %r %s" % (
                    error, traceback.format_exc(limit=4)
                    .replace("\n", " | ")))
            response.valid = False
            response.reason_code = "VERIFY_SERVICE_EXCEPTION"
            response.detail = repr(error)[:300]
            return response
        response.valid = bool(ok)
        response.reason_code = str(reason)
        response.detail = json.dumps(detail, default=str)[:2000]
        response.start_state_delta_rad = float(
            detail.get("start_state_delta_rad", 0.0) or 0.0)
        response.validated_map_revision = int(
            detail.get("validated_map_revision") or 0)
        response.certification_id = str(
            detail.get("certification_id")
            or (producer.last_selection or {}).get("certification_id", ""))
        response.leg_digests = [
            str(digest) for digest in detail.get("leg_digests") or []]
        return response

    def _check_execution_admission(self, request, response):
        """[ST09-I S5.7-3b] Execution-window closure, certifier side."""
        producer = self._route_producer_or_none()
        if producer is None:
            response.live = False
            response.reason_code = "ROUTE_PRODUCER_UNAVAILABLE"
            return response
        try:
            live, reason, detail = producer.execution_admission_is_live(
                route_digest=str(request.route_digest or ""),
                certification_id=str(request.certification_id or ""),
                admission_map_revision=int(
                    request.admission_map_revision or 0),
                scene_revision=str(request.scene_revision or ""))
        except Exception as error:  # noqa: BLE001 - service boundary
            import traceback

            self.get_logger().error(
                "route: admission boundary exception %r %s" % (
                    error, traceback.format_exc(limit=4)
                    .replace("\n", " | ")))
            response.live = False
            response.reason_code = "ADMISSION_SERVICE_EXCEPTION"
            response.detail = repr(error)[:300]
            return response
        response.live = bool(live)
        response.reason_code = str(reason)
        response.detail = json.dumps(detail, default=str)[:2000]
        return response

    def _plan_observation(self, request, response):
        """[ST09-I S5.7-4] PlanObservationRouteV2 service boundary.

        The caller declares observation intent (world camera poses);
        the producer converts camera->TCP, builds the MoveIt edge, and
        returns a COMPLETE certified joint_path under the explicit
        LoadState.UNLOADED admission.
        """
        if getattr(self, "_frame_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                admission = self._frame_runtime.admit_request(
                    str(getattr(request, "request_id", "") or ""),
                    endpoint="/trajectory_selector/plan_observation_route")
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.detail = "%s: %s" % (error.code, error.field)
                return response
        producer = self._route_producer_or_none()
        if producer is None:
            response.success = False
            response.reason_code = "ROUTE_PRODUCER_UNAVAILABLE"
            return response
        targets = [((float(pose.position.x), float(pose.position.y),
                     float(pose.position.z)),
                    (float(pose.orientation.x), float(pose.orientation.y),
                     float(pose.orientation.z), float(pose.orientation.w)))
                   for pose in (request.observation_targets or [])]
        try:
            result = producer.plan_observation_route(
                request_id=str(getattr(request, "request_id", "") or ""),
                scene_revision=str(getattr(request, "scene_revision", "")
                                   or ""),
                current_joints=list(
                    request.current_joint_state or []),
                camera_targets=targets,
                latest_map_revision=int(request.latest_map_revision),
                purpose=str(getattr(request, "purpose", "") or "observation"))
        except Exception as error:  # noqa: BLE001 - service boundary
            import traceback

            self.get_logger().error(
                "route: observation boundary exception %r %s" % (
                    error, traceback.format_exc(limit=4)
                    .replace("\n", " | ")))
            response.success = False
            response.reason_code = "OBSERVATION_SERVICE_EXCEPTION"
            response.detail = repr(error)[:300]
            return response
        if not result.get("success"):
            response.success = False
            response.reason_code = str(
                result.get("reason_code") or "NO_FEASIBLE_ROUTE")
            response.detail = json.dumps(
                result.get("rows") or [], default=str)[:2000]
            return response
        if getattr(self, "_frame_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                self._frame_runtime.authorize_effect(
                    admission, str(getattr(request, "request_id", "") or ""))
            except ContractError as error:
                response.success = False
                response.reason_code = error.code
                response.detail = "%s: %s" % (error.code, error.field)
                return response
        from luggage_planning.place_service import _route_motion_messages

        response.success = True
        response.reason_code = "OK"
        response.route_digest = str(result["path_hash"])
        response.certification_id = str(
            result["certification"].digest())
        response.validated_map_revision = int(
            result["revalidation"].map_revision)
        response.motion_segments = _route_motion_messages(
            result["legs"], result["request"]
            if isinstance(result.get("request"), dict)
            else {"request_id": str(request.request_id),
                  "scene_revision": str(request.scene_revision)})
        response.leg_digests = [
            str(edge.trajectory.digest()) for edge in result["edges"]]
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
        try:
            response = fill_select_response_route(
                request, response, producer=producer)
        except Exception as error:  # noqa: BLE001 - service boundary
            # A refused request carries its own named reason; an
            # UNEXPECTED exception must still never kill the node (an
            # exception escaping a service callback terminates the
            # executor — the [ST09-I] reason-loss lesson).
            import traceback

            self.get_logger().error(
                "route: select boundary exception %r %s" % (
                    error, traceback.format_exc(limit=4)
                    .replace("\n", " | ")))
            response.success = False
            response.reason_code = "SELECT_SERVICE_EXCEPTION"
            response.rows_json = json.dumps(
                {"error": "SELECT_SERVICE_EXCEPTION",
                 "detail": repr(error)[:300]})
            return response
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
