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
from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_perception.corridor_audit import (
    corridor_aabb,
    corridor_surface_max,
)
from luggage_msgs.msg import DetectionFrame
from luggage_msgs.srv import BuildMotionSequence, BuildMotionSequenceV2

from luggage_perception.locked_stamp_window import LockedStampWindow
from luggage_planning import ros_message_adapters as adapters
from luggage_planning.reachability_wavefront import (
    SAFE_TOOL_Z,
    container_tool_z_in_frame,
)
from luggage_planning.place_service import handle_build_v2
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
        origin_xyz, origin_rpy = origin_in_world(self._scene_config)
        # container_link safe tool z -> world, the frame build_sequence sees.
        self._retreat_safe_tool_z = container_tool_z_in_frame(
            SAFE_TOOL_Z, origin_xyz[2], origin_rpy)
        inner = container_inner_dimensions(self._scene_config)
        floor_z = container_inner_floor_z(self._scene_config)
        ceiling_z = container_inner_ceiling_z(self._scene_config)
        self._inner_size = [
            float(inner[0]), float(inner[1]), float(ceiling_z - floor_z)]
        self._hull = descriptor_from_scene_config(self._scene_config)
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
        # [ST09-I S5.7-4] pick-ladder certification needs the latched
        # cargo-map surface (the same source the route selector uses).
        self._cargo_surface = None
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d",
            self._on_cargo_surface, latch, callback_group=group)

        self._tf_buffer = tf2_ros.Buffer()
        # use_global_arguments=False: launch -r __node:=... must not rename
        # this sidecar, or ros2 node list reports a duplicate name.
        self._tf_node = Node(
            "waypoint_generator_tf", use_global_arguments=False)
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True)

        from sensor_msgs.msg import JointState

        self._joints_live = None
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)
        self.create_service(
            BuildMotionSequence, "/waypoint_generator/build_motion_sequence",
            self._handle, callback_group=group)
        self.create_service(
            BuildMotionSequenceV2, "/waypoint_generator/build_motion_sequence_v2",
            self._handle_v2, callback_group=group)
        # ST09 F21: identity heartbeat + peer readiness (off = legacy).
        self.declare_parameter("frame_runtime", False)
        self.declare_parameter("frame_identity_json", "")
        self.declare_parameter("frame_peers", "")
        self._runtime = None
        if bool(self.get_parameter("frame_runtime").value):
            import sys as _sys
            from luggage_description.ros_frame_bindings.runtime import (
                node_runtime_from_params)
            self._runtime = node_runtime_from_params(
                self,
                resolved_config={
                    "scene_tf_config": str(
                        self.get_parameter("scene_tf_config").value or ""),
                    "world_frame": str(
                        self.get_parameter("world_frame").value or ""),
                    "place_slot_frame": str(
                        self.get_parameter("place_slot_frame").value or ""),
                },
                modules=[(_sys.modules["__main__"], "module", "application")],
                package_names=["luggage_planning"])
        self.get_logger().info(
            "waypoint_generator ready (clearances %s, place_slot_frame=%s)"
            % (DEFAULT_PICK_CLEARANCES, self._place_slot_frame))

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

    def _on_cargo_surface(self, msg):
        try:
            surface = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(surface, dict):
            self._cargo_surface = surface

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
            self._inner_size, [slot.width, slot.depth, slot.height],
            hull=self._hull)
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

    def _on_joint_states(self, msg):
        names = list(msg.name)
        wanted = ["elfin_joint%d" % i for i in range(1, 7)]
        if msg.position and all(name in names for name in wanted):
            self._joints_live = [
                float(msg.position[names.index(name)]) for name in wanted]

    def _pick_builder(self):
        """Lazy MoveIt edge builder stack for timed pick legs (ST09-I)."""
        if getattr(self, "_pick_stack", None) is not None:
            return self._pick_stack
        from luggage_planning.moveit_edge_adapter import MoveItEdgeAdapter
        from luggage_planning.offline_ik import load_urdf, refine_to_pose
        from luggage_planning.reachable_graph import (
            RouteGraph,
            make_offline_fk,
        )
        urdf = load_urdf(scene_tf_config=str(
            self.get_parameter("scene_tf_config").value or ""))
        # path_to returns the JOINT sequence to the TCP link (tcp() FK
        # walks joints; the link name itself is not a joint).
        tcp_path = urdf.path_to("suction_contact_frame")
        fk = make_offline_fk(urdf, tcp_path)

        def refine(position, quaternion, seed):
            return refine_to_pose(urdf, tcp_path, position, quaternion,
                                  seed, pos_tol=2e-3, rot_tol=2e-2)

        self._pick_stack = {
            "graph": RouteGraph(static_identity="st09-i-pick"),
            "edge_builder": MoveItEdgeAdapter(self),
            "fk": fk, "refine": refine,
        }
        # [ST09-I S5.7-4] pick-ladder certification stack: the same
        # frozen policy + ValidateRouteV2-backed validity adapter the
        # route selector certifies place routes with.
        import json as _json
        import os as _os

        from luggage_planning.moveit_state_validity import (
            MoveItStateValidityAdapter,
        )
        from luggage_planning.planning_scene_client import (
            PlanningSceneClient,
        )
        from luggage_planning.route_producer import (
            certification_policy_from_contract,
        )
        from luggage_planning.trajectory_certifier import TrajectoryCertifier

        try:
            from ament_index_python.packages import (
                get_package_share_directory,
            )

            contract_path = _os.path.join(
                get_package_share_directory("st09_contracts"),
                "resolved_planning_contract.json")
            with open(contract_path, encoding="utf-8") as handle:
                contract_document = _json.load(handle)
            cert_policy = certification_policy_from_contract(
                contract_document)
        except Exception:  # noqa: BLE001 - fail closed, same as selector
            self.get_logger().error(
                "pick: resolved contract unavailable; refusing to build")
            self._pick_stack = None
            return None
        self._pick_stack.update({
            "scene_client": PlanningSceneClient(self),
            "certifier": TrajectoryCertifier(),
            "cert_policy": cert_policy,
        })
        return self._pick_stack

    def _pick_cert_snapshot(self, scene_revision):
        """Fresh mapper state for pick certification — fail-closed.

        No stale fallback: an unavailable surface or stats service
        refuses the pick ([ST09-I S5.7] map admission lock).
        """
        import threading

        from luggage_msgs.srv import GetCargoMapStats
        from luggage_perception.cargo_view_integration import (
            planning_snapshot_from_surface,
        )

        surface = dict(self._cargo_surface or {})
        if not surface:
            return None
        if not hasattr(self, "_map_stats_client"):
            self._map_stats_client = self.create_client(
                GetCargoMapStats, "/cargo_map/get_stats")
        if not self._map_stats_client.wait_for_service(timeout_sec=1.0):
            return None
        event = threading.Event()
        future = self._map_stats_client.call_async(
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
            "map_source": str(getattr(stats, "map_source", "") or ""),
            "placed_count": int(getattr(stats, "placed_count", 0) or 0),
            "cargo_observed": bool(getattr(stats, "cargo_observed", False)),
            "map_revision": int(stats.map_revision),
            "commit_revision": int(stats.map_revision),
            "geometry_hash": str(stats.geometry_hash),
        })
        return planning_snapshot_from_surface(
            surface, scene_revision=str(scene_revision or ""),
            request_id="pick-cert-snapshot")

    def _certify_pick_legs(self, stack, edges, *, request_id,
                           scene_revision, stamp_ns):
        """Certify the unloaded pick ladder ([ST09-I S5.7-4], §4').

        A failed certification is a NAMED refusal — never a degraded
        execution of uncertified legs.
        """
        from luggage_description.scene_tf_config_utils import (
            container_inner_floor_z,
            origin_in_world,
        )
        from luggage_description.scene_tf_publisher import rpy_to_quaternion
        from luggage_planning.grid_path_search import RoutePlanningError
        from luggage_planning.moveit_edge_adapter import (
            OccupancyColumnError,
            SceneState,
            occupancy_boxes_from_surface,
        )
        from luggage_planning.moveit_state_validity import (
            MoveItStateValidityAdapter,
        )
        from luggage_planning.route_producer import certify_unloaded_legs

        snapshot = self._pick_cert_snapshot(scene_revision)
        if snapshot is None:
            raise RoutePlanningError(
                "PICK_CERT_SNAPSHOT_UNAVAILABLE",
                {"leg": "pick", "reason": "latest map unavailable"})
        surface = dict(self._cargo_surface or {})
        origin, rpy = origin_in_world(self._scene_config)
        try:
            placed = occupancy_boxes_from_surface(
                {
                    "state": surface.get("state") or [],
                    "origin_xy_container": (
                        surface.get("origin_xy_container")
                        or surface.get("origin_local")),
                    "resolution_m": (
                        surface.get("resolution_m")
                        or surface.get("resolution")),
                    "floor_z_container": container_inner_floor_z(
                        self._scene_config),
                    "height_above_floor": (
                        surface.get("height_above_floor")
                        or surface.get("height")),
                },
                {
                    "translation": [float(v) for v in origin],
                    "quaternion_xyzw": [
                        float(v) for v in rpy_to_quaternion(rpy)],
                })
        except OccupancyColumnError as exc:
            raise RoutePlanningError(
                "PICK_CERT_SNAPSHOT_UNAVAILABLE",
                {"leg": "pick", "reason": str(exc)})
        adapter = MoveItStateValidityAdapter(
            self, fk=stack["fk"], scene_client=stack["scene_client"],
            log=lambda message: self.get_logger().info(message))
        scene_states = {
            str(edge.edge_id): SceneState(
                load_state="unloaded", payload=None, released_box=None,
                placed_boxes=placed)
            for edge in edges
        }
        adapter.load_edges(edges, scene_states=scene_states)
        result, _route = certify_unloaded_legs(
            edges=edges, snapshot=snapshot,
            request_id=str(request_id),
            route_id="pick-%s" % str(request_id)[:24], purpose="pick",
            certifier=stack["certifier"],
            validity_adapter=adapter, policy=stack["cert_policy"],
            now_ns=int(stamp_ns) or self.get_clock().now().nanoseconds)
        if not result.passed:
            raise RoutePlanningError(
                result.reason.value,
                {"certification_id": result.certification_id,
                 "purpose": "pick"})
        return result

    def _pick_leg_encoder(self):
        """Encode the frozen pick waypoint ladder as timed joint legs."""
        def encode(pick, *, request_id, scene_revision, stamp_ns,
                   suction_candidate=None):
            from luggage_planning.grid_path_search import RoutePlanningError
            from luggage_planning.route_producer import pick_leg_records

            start_q = getattr(self, "_joints_live", None)
            if not start_q:
                raise RoutePlanningError(
                    "EDGE_BUILD_FAILED",
                    {"leg": "pick", "reason": "start joints unknown"})
            # A wrapped equivalent (live j6 = -4.44 after a full cycle)
            # is the same pose, but the offline refine explores from the
            # seed's branch and landed on an elbow family whose goal
            # fouled the box (live: pre_grasp GOAL_STATE_INVALID on the
            # second pick). Seed from the canonical [-pi, pi] range...
            import math as _math

            raw_q = [float(v) for v in start_q]
            start_q = [
                (value + _math.pi) % (2.0 * _math.pi) - _math.pi
                for value in raw_q]

            def _rewrap_legs(legs):
                # ...then shift the built legs back next to the LIVE
                # (possibly wrapped) joints: a constant 2*pi turn per
                # joint changes nothing geometrically, keeps the
                # executor's start-state gate happy, and stops the
                # controller from spinning a full turn to bookkeeping.
                if not legs:
                    return legs
                points = legs[0]["joint_trajectory"]["points"]
                if not points:
                    return legs
                shifts = []
                for ref, value in zip(
                        raw_q, points[0]["positions"]):
                    turns = round((float(value) - ref)
                                  / (2.0 * _math.pi))
                    shifts.append(turns * 2.0 * _math.pi)
                if not any(shifts):
                    return legs
                for leg in legs:
                    for point in leg["joint_trajectory"]["points"]:
                        point["positions"] = [
                            float(v) - s for v, s in
                            zip(point["positions"], shifts)]
                return legs
            stack = self._pick_builder()
            if stack is None:
                raise RoutePlanningError(
                    "PICK_CONTRACT_UNAVAILABLE", {"leg": "pick"})
            waypoints = []
            if suction_candidate is not None:
                from luggage_planning.suction_candidate_waypoints import build_candidate_pick_segments
                segments = build_candidate_pick_segments(
                    suction_candidate, pick_clearances=self._clearances(),
                    detection_yaw=pick.yaw, yaw_valid=pick.yaw_valid)
            else:
                segments = build_sequence(pick, None, "pick",
                                          pick_clearances=self._clearances())
            for segment in segments:
                pose = segment.target_pose
                waypoints.append((
                    str(segment.name),
                    (float(pose.position.x), float(pose.position.y),
                     float(pose.position.z)),
                    (float(pose.orientation.x), float(pose.orientation.y),
                     float(pose.orientation.z), float(pose.orientation.w))))
            import time as _timing

            _t0 = _timing.monotonic_ns()
            legs, edges = pick_leg_records(
                waypoints,
                graph=stack["graph"], edge_builder=stack["edge_builder"],
                refine=stack["refine"], fk=stack["fk"], start_q=start_q,
                request_id=request_id, scene_revision=scene_revision,
                stamp_ns=stamp_ns)
            _t1 = _timing.monotonic_ns()
            self._certify_pick_legs(
                stack, edges, request_id=request_id,
                scene_revision=scene_revision, stamp_ns=stamp_ns)
            _t2 = _timing.monotonic_ns()
            # [ST09-I S5.7-6] frozen-budget attribution for the pick
            # build; stamped into the v2 response by the service handler.
            self._last_pick_stage_ms = {
                "edge_build_ms": round((_t1 - _t0) / 1e6, 1),
                "certification_ms": round((_t2 - _t1) / 1e6, 1),
            }
            return _rewrap_legs(legs)
        return encode

    def _handle(self, request, response):
        """v1 surface preserved as a named refusal (ST09-I).

        Pick waypoints are candidate geometry; timed legs come from the
        v2 build through the MoveIt edge builder, and place motion comes
        from the certified route.
        """
        if getattr(self, "_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                self._runtime.authorize_legacy_effect()
            except ContractError as error:
                response.segments = []
                response.success = False
                response.message = "%s: %s" % (error.code, error.field)
                return response
        response.segments = []
        response.success = False
        response.message = "PLANNING_ROUTE_REQUIRED: v1 build removed; " \
            "use build_motion_sequence_v2 (pick) / select_place_path_v2 " \
            "(place)"
        return response

    def _handle_v2(self, request, response):
        if getattr(self, "_runtime", None) is not None:
            from luggage_description.frame_contracts import ContractError
            try:
                admission = self._runtime.admit_request(
                    str(getattr(request, "request_id", "") or ""),
                    endpoint="/waypoint_generator/build_motion_sequence_v2")
            except ContractError as error:
                response.motion_segments = []
                response.segments = []
                response.success = False
                response.message = "%s: %s" % (error.code, error.field)
                return response
        pick = None
        if str(request.phase or "") == "pick":
            pick = adapters.pick_from_detected(request.pick)
        response = handle_build_v2(
            request, response, pick=pick,
            pick_encoder=self._pick_leg_encoder())
        import json as _json

        response.stage_timings_json = _json.dumps(
            getattr(self, "_last_pick_stage_ms", None) or {})
        if getattr(self, "_runtime", None) is not None and bool(getattr(response, "success", False)):
            # Effect boundary: the builder ran under the admission; recheck.
            from luggage_description.frame_contracts import ContractError
            try:
                self._runtime.authorize_effect(
                    admission, str(getattr(request, "request_id", "") or ""))
            except ContractError as error:
                response.motion_segments = []
                response.segments = []
                response.success = False
                response.message = "%s: %s" % (error.code, error.field)
        return response

    def _handle_candidate(self, request, response, candidate_id):
        """v1 candidate pick build is refused with the v1 surface (ST09-I)."""
        del candidate_id
        response.segments = []
        response.success = False
        response.message = "PLANNING_ROUTE_REQUIRED: v1 candidate build " \
            "removed; candidate waypoints feed the v2 pick encoder"
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
