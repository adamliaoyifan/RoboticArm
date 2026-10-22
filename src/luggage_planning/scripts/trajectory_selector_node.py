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
    compute_reachability_scene_hash,
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
        self._reachability_scene_hash = compute_reachability_scene_hash(
            scene_path)
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
        # [ST09-I S5.7-6] authoritative vacuum feed for the carrying
        # admission window (TRANSIENT_LOCAL like the controller's own
        # publish, so a late-joining selector sees the latched state).
        from luggage_msgs.msg import VacuumState
        self._vacuum_state = None
        self.create_subscription(
            VacuumState, "/vacuum/state", self._on_vacuum_state, qos)
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
        # [ST09-I S5.7-6] release-boundary phase transition: the driver
        # owns the release effect and its evidence; this service is the
        # certifier-side verdict that ACTIVATES the unloaded phase.
        from luggage_msgs.srv import TransitionAdmissionPhase
        self.create_service(
            TransitionAdmissionPhase,
            "/trajectory_selector/transition_admission_phase",
            self._transition_admission_phase,
            callback_group=self._service_group)
        # [ST09-I S5.7-6] certified motion to a NAMED site pose: the
        # site-pose file (sim schema) is validated against the live
        # identity; one MoveIt edge + the full certification pipeline
        # under the explicit UNLOADED admission.
        from luggage_msgs.srv import GoToRobotPoseV2
        self.create_service(
            GoToRobotPoseV2,
            "/trajectory_selector/goto_robot_pose_v2",
            self._goto_robot_pose_v2,
            callback_group=self._service_group)
        # [ST09-I S5.7-6 §2] cycle-start adjudication: authoritative
        # state only (latched joints + vacuum + scene config + offline
        # FK), NO route planning and no frame_runtime admission — the
        # same pure-verdict shape as check_execution_admission.
        from luggage_msgs.srv import EnsureCycleStartV2
        self.create_service(
            EnsureCycleStartV2,
            "/trajectory_selector/ensure_cycle_start_v2",
            self._ensure_cycle_start,
            callback_group=self._service_group)
        # ST09-I: the unified route producer (off = legacy selection).
        self.declare_parameter("route_planning", True)
        # graph: GraphRoutePlanner hop search. lattice: corridor lattice
        # for portal..slot..portal, MoveIt joint transit/return only.
        self.declare_parameter("corridor_planner", "graph")
        # Route wall budget is 60 s times this count (1 box 60 s,
        # 3 boxes 180 s). Default 1 keeps a single select at 60 s.
        self.declare_parameter("box_count", 1)
        # [Functional diagnostic, 2026-10-22] Explicit per-Select wall
        # budget override that reaches the searcher. 0.0 (default) keeps
        # the frozen n*60s contract — production behavior is unchanged.
        # Non-scoring diagnostics only; per-edge budgets and every safety
        # gate stay frozen.
        self.declare_parameter("diagnostic_route_budget_s", 0.0)
        self.declare_parameter("frame_scene_revision", "")
        # [ST09-I S5.7-6] site-pose file for GoToRobotPoseV2 (empty =
        # the service refuses POSE_FILE_MISSING — never a default).
        self.declare_parameter("robot_poses_config", "")
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

    def _site_poses(self):
        """[ST09-I S5.7-6] Cached (data, path) for the site-pose file.

        An unreadable or missing file logs and returns None so the
        producer refuses POSE_FILE_MISSING — never an exception at the
        service boundary, never a silent default.
        """
        path = str(self.get_parameter("robot_poses_config").value or "")
        if not path:
            return None
        if getattr(self, "_site_poses_cache", None) is None:
            from luggage_planning.site_pose_schema import load_site_poses

            try:
                self._site_poses_cache = (load_site_poses(path), path)
            except Exception as error:  # noqa: BLE001 - fail closed
                self.get_logger().error(
                    "site poses: %r unreadable: %r" % (path, error))
                return None
        return self._site_poses_cache

    def _runtime_atlas(self):
        cfg = self._atlas_cfg
        if not cfg.get("enabled"):
            return None, {"loaded": False, "reason": "disabled"}
        self._urdf_identity.poll()
        urdf_hash = (
            self._urdf_identity.hash if self._urdf_identity.ready else "")
        key = (self._reachability_scene_hash, self._scene_tf_hash, urdf_hash)
        if key != self._atlas_key:
            self._atlas, self._atlas_diag = load_runtime_atlas(
                cfg, scene_tf_hash=self._scene_tf_hash, urdf_hash=urdf_hash,
                reachability_scene_hash=self._reachability_scene_hash)
            self._atlas_key = key
            reason = str(self._atlas_diag.get("reason") or "")
            note = str(self._atlas_diag.get("hash_message") or "")
            if note and note != "ok":
                reason = "%s (%s)" % (reason, note)
            self.get_logger().info("atlas load: %s" % reason)
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

    def _on_vacuum_state(self, msg):
        """Latch the authoritative vacuum state with a receive stamp.

        The message carries no stamp of its own; the receive time on
        this node's clock (the same clock the producer's clock_ns uses)
        is the freshness basis the admission window compares against.
        """
        self._vacuum_state = {
            "attached": bool(msg.attached),
            "vacuum_on": bool(msg.vacuum_on),
            "fail_reason": str(msg.fail_reason or ""),
            "received_ns": self.get_clock().now().nanoseconds,
        }

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

    def _fetch_cargo_map_stats(self):
        """Cargo-map stats for admission, or None when the service fails.

        Stamp, source, placed count, observation, revision, and geometry
        hash come from ``/cargo_map/get_stats``. A missing piece fails
        revalidation closed.
        """
        import threading

        from luggage_msgs.srv import GetCargoMapStats

        if not hasattr(self, "_stats_client"):
            self._stats_client = self.create_client(
                GetCargoMapStats, "/cargo_map/get_stats")
        if not self._stats_client.wait_for_service(timeout_sec=1.0):
            return None
        event = threading.Event()
        future = self._stats_client.call_async(GetCargoMapStats.Request())
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=2.0) or not future.done():
            return None
        stats = future.result()
        if stats is None or not stats.success:
            return None
        return {
            "success": True,
            "last_acquisition_stamp_ns": int(stats.last_acquisition_stamp_ns),
            "map_source": str(getattr(stats, "map_source", "") or ""),
            "placed_count": int(getattr(stats, "placed_count", 0) or 0),
            "cargo_observed": bool(getattr(stats, "cargo_observed", False)),
            "map_revision": int(stats.map_revision),
            "geometry_hash": str(stats.geometry_hash),
        }

    def _route_producer_or_none(self):
        """Build the unified route producer once, on first use."""
        if self._route_producer is not None:
            return self._route_producer
        from luggage_planning.route_producer_factory import (
            build_production_route_producer,
        )

        self._route_producer = build_production_route_producer(
            self, self._scene_tf_path,
            lambda: self._surface,
            self._fetch_cargo_map_stats)
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
        response.leg_load_states = [
            str(state) for state in detail.get("leg_load_states") or []]
        response.certify_ms = float(detail.get("certify_ms", 0.0) or 0.0)
        response.verify_ms = float(detail.get("verify_ms", 0.0) or 0.0)
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
                scene_revision=str(request.scene_revision or ""),
                leg_digest=str(request.leg_digest or ""),
                leg_load_state=str(request.leg_load_state or ""))
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

    def _transition_admission_phase(self, request, response):
        """[ST09-I S5.7-6] Phase transition verdict, certifier side."""
        producer = self._route_producer_or_none()
        if producer is None:
            response.success = False
            response.reason_code = "ROUTE_PRODUCER_UNAVAILABLE"
            return response
        try:
            ok, reason, detail = producer.transition_admission_phase(
                route_digest=str(request.route_digest or ""),
                certification_id=str(request.certification_id or ""),
                target_phase=str(request.target_phase or ""),
                detach_confirmed=bool(request.detach_confirmed),
                release_evidence=str(request.release_evidence or ""),
                map_revision_after_release=int(
                    request.map_revision_after_release or 0))
        except Exception as error:  # noqa: BLE001 - service boundary
            import traceback

            self.get_logger().error(
                "route: phase transition boundary exception %r %s" % (
                    error, traceback.format_exc(limit=4)
                    .replace("\n", " | ")))
            response.success = False
            response.reason_code = "PHASE_SERVICE_EXCEPTION"
            response.detail = repr(error)[:300]
            return response
        response.success = bool(ok)
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
        response.leg_load_states = [
            str(getattr(edge, "load_state", "") or "").lower()
            for edge in result["edges"]]
        response.certify_ms = float(
            (result.get("stage_timings") or {}).get("certify_ms", 0.0)
            or 0.0)
        return response

    def _goto_robot_pose_v2(self, request, response):
        """[ST09-I S5.7-6] GoToRobotPoseV2 service boundary.

        The caller names an explicit calibrated pose; the producer
        validates the site file against the live identity, builds the
        MoveIt edge, and returns the COMPLETE certified joint_path
        under the explicit LoadState.UNLOADED admission. No
        defaults.observe_pose resolution, no current/here execution —
        caller-side decisions, never server-side fallbacks.
        """
        if getattr(self, "_frame_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                admission = self._frame_runtime.admit_request(
                    str(getattr(request, "request_id", "") or ""),
                    endpoint="/trajectory_selector/goto_robot_pose_v2")
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
        try:
            result = producer.plan_named_pose_route(
                request_id=str(getattr(request, "request_id", "") or ""),
                pose_name=str(getattr(request, "pose_name", "") or ""),
                current_joints=list(
                    request.current_joint_state or []),
                scene_revision=str(getattr(request, "scene_revision", "")
                                   or ""),
                latest_map_revision=int(request.latest_map_revision))
        except Exception as error:  # noqa: BLE001 - service boundary
            import traceback

            self.get_logger().error(
                "route: goto boundary exception %r %s" % (
                    error, traceback.format_exc(limit=4)
                    .replace("\n", " | ")))
            response.success = False
            response.reason_code = "GOTO_SERVICE_EXCEPTION"
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
        response.already_there = bool(result.get("already_there"))
        response.route_digest = str(result.get("path_hash") or "")
        if response.already_there:
            return response
        response.certification_id = str(
            result["certification"].digest())
        response.admission_map_revision = int(
            result["revalidation"].map_revision)
        response.motion_segments = _route_motion_messages(
            result["legs"], result["request"]
            if isinstance(result.get("request"), dict)
            else {"request_id": str(request.request_id),
                  "scene_revision": str(request.scene_revision)})
        response.leg_digests = [
            str(edge.trajectory.digest()) for edge in result["edges"]]
        response.leg_load_states = [
            str(getattr(edge, "load_state", "") or "").lower()
            for edge in result["edges"]]
        response.certify_ms = float(
            (result.get("stage_timings") or {}).get("certify_ms", 0.0)
            or 0.0)
        return response

    def _ensure_cycle_start(self, request, response):
        """[ST09-I S5.7-6 §2] EnsureCycleStartV2 service boundary.

        The selector's OWN latched state answers (joints, vacuum,
        scene); the executor never adjudicates for itself. Pure
        verdict — ALREADY_READY authorizes nothing more than the next
        GoToRobotPoseV2 request.
        """
        from luggage_msgs.srv import EnsureCycleStartV2 as _Ensure

        producer = self._route_producer_or_none()
        if producer is None:
            response.outcome = _Ensure.Response.RECOVERY_REQUIRED
            response.reason_code = "ROUTE_PRODUCER_UNAVAILABLE"
            response.zone = "unknown"
            response.attachment_state = "unknown"
            return response
        try:
            ready, reason, zone, attachment, detail = \
                producer.ensure_cycle_start_state(
                    joints=self._joints,
                    request_scene_revision=str(
                        request.scene_revision or ""),
                    vacuum_state=self._vacuum_state)
        except Exception as error:  # noqa: BLE001 - service boundary
            import traceback

            self.get_logger().error(
                "route: ensure boundary exception %r %s" % (
                    error, traceback.format_exc(limit=4)
                    .replace("\n", " | ")))
            response.outcome = _Ensure.Response.RECOVERY_REQUIRED
            response.reason_code = "ENSURE_SERVICE_EXCEPTION"
            response.zone = "unknown"
            response.attachment_state = "unknown"
            response.detail = repr(error)[:300]
            return response
        response.outcome = (
            _Ensure.Response.ALREADY_READY if ready
            else _Ensure.Response.RECOVERY_REQUIRED)
        response.reason_code = str(reason)
        response.zone = str(zone)
        response.attachment_state = str(attachment)
        response.detail = json.dumps(detail, default=str)[:2000]
        return response

    @staticmethod
    def _surface_validity(request, latched):
        """[B2 single-source] Confirmation verdict source for one select.

        The request's embedded v2 surface carries its own confirmation
        fields when the publishing map declared a snapshot identity (B1:
        non-empty epoch) — overriding them from this node's v1 latch was a
        second verdict source (the two latches can disagree across a
        commit between them; the req-2b freshness race). The latch is the
        fallback for pre-B1 surfaces only.
        """
        request_surface = getattr(request, "surface", None)
        if str(getattr(request_surface, "epoch", "") or ""):
            return None
        latched = latched or {}
        return {
            "occupancy_confirmed": latched.get("occupancy_confirmed", True),
            "change_event": latched.get("change_event") or "",
        }

    def _select_v2(self, request, response):
        # World poses come from transform_pose. v1 keeps _segments.
        # This callback does not execute motion.
        from luggage_planning.boundary_capture import capture_service_request
        capture_service_request(
            request, os.environ.get("ST09_SELECT_REQUEST_CAPTURE_DIR", ""),
            "select_place_path_v2")
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
            latched = getattr(self, "_surface", None) or {}
            response = fill_select_response_route(
                request, response, producer=producer,
                surface_validity=self._surface_validity(request, latched))
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
