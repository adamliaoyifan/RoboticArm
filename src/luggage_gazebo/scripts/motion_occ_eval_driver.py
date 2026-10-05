#!/usr/bin/env python3
"""MOTION-OCC: occupancy-aware place through the unified route (ST09-I).

Place motion is the certified route from SelectPlacePathV2 (planned,
certified, and revalidated in the trajectory_selector node); this driver
correlates request ids, probes winners plan-only, executes the typed
joint legs, and records evidence. The in-process geometric selection and
the executor generation bypasses are deleted.
"""

from __future__ import division

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from cargo_map_integrate_eval_driver import (  # noqa: E402
    CargoMapIntegrateDriver,
    parse_args as parse_integrate_args,
)
from luggage_description.scene_tf_config_utils import (  # noqa: E402
    _point_in_container_link,
    container_inner_ceiling_z,
    container_usable_dimensions,
    container_opening_normal_in_world,
    container_opening_target_point_in_world,
    origin_in_world,
    xyz_world_to_base_link,
)
from luggage_gazebo.cargo_view_sequence import (  # noqa: E402
    policy_hash,
    view_policy_record,
)
from luggage_gazebo.place_metrics import (  # noqa: E402
    RETURN_FAIL_CODES,
    place_ok,
    summarize,
    trial_to_dict,
)
from luggage_msgs.msg import MotionSegment  # noqa: E402
from luggage_msgs.srv import ProbeMotionSegment  # noqa: E402
from luggage_planning.occupancy_place_paths import (  # noqa: E402
    PLACE_PATH_INFEASIBLE,
    PAYLOAD_SOURCE_MEASURED,
    dump_rows,
)
from pick_retreat_eval_driver import _git_meta  # noqa: E402


DEFAULT_OUT = os.path.normpath(os.path.join(
    _SCRIPTS, "..", "..", "..", "docs", "status", "evidence",
    "motion_occ", "latest"))

# Selected-route re-verification ceiling (ST09-I S5.7: the probe never
# generates a second path; it re-validates the certified one).
PLACE_PROBE_TIMEOUT_S = 30.0


def _map_to_world(scene, xyz):
    origin, rpy = origin_in_world(scene)
    yaw = float(rpy[2])
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = [float(v) for v in xyz]
    return [
        origin[0] + c * x - s * y,
        origin[1] + s * x + c * y,
        origin[2] + z,
    ]


def carry_frame_convert(selected_path, scene):
    """Map->world convert, or None when the selected path is already world.

    SelectPlacePathV2 ``waypoint_poses`` / ``motion_segments`` are world
    TCP. A second origin shift (rehearsal-20: 1.005 -> 2.505) is
    ``outside_hull``. v1 in-process winners stay map-frame.
    """
    if not selected_path or selected_path.get("waypoint_frame") == "world":
        return None
    return lambda xyz: _map_to_world(scene, xyz)


def split_carry_waypoints(name, waypoints):
    """Split a selected world path into approach vs in-container carry.

    A single cartesian from pick to the slot (2.8 m) unwinds elfin_joint3
    (rehearsal-22/23) and the joint fallback still branch-jumps
    (rehearsal-24/25). Transit still only goes to the first portal pose;
    traverse follows the remaining polyline. The portal hop itself is
    cartesian: pose_target joint interpolation with a centered payload
    sweeps the box through closed hull faces (rehearsal-31). The TCP
    line from pick retreat to that portal already passes occupancy.
    """
    points = [list(xyz) for xyz in (waypoints or [])]
    if not points:
        return None, None
    if str(name) == "transit":
        return "cartesian", [points[0]]
    if str(name) == "traverse":
        rest = points[1:] if len(points) > 1 else points
        return "cartesian", rest
    return None, None


def _quat_yaw(orientation):
    """Planar yaw of a slot orientation (footprint heading for the sweep)."""
    siny = 2.0 * (float(orientation.w) * float(orientation.z)
                  + float(orientation.x) * float(orientation.y))
    cosy = 1.0 - 2.0 * (float(orientation.y) ** 2
                        + float(orientation.z) ** 2)
    return math.atan2(siny, cosy)


def _joint_positions(state):
    if not isinstance(state, dict):
        return None
    raw = state.get("position")
    if raw is None:
        raw = state.get("positions")
    if not raw:
        return None
    return [float(v) for v in list(raw)[:6]]


def _world_to_map(scene, xyz):
    base = xyz_world_to_base_link(scene, xyz)
    return _point_in_container_link(base, scene)


class MotionOccDriver(CargoMapIntegrateDriver):
    def __init__(self, args):
        super().__init__(args)
        self._atlas = None
        self._atlas_diag = {}
        try:
            from luggage_planning.atlas_runtime import (
                certification_block_reason,
                compute_reachability_scene_hash,
                compute_scene_tf_hash,
                load_atlas_runtime_for_scene,
                load_runtime_atlas,
            )
            from luggage_planning.urdf_identity import UrdfIdentity
            from luggage_description.scene_tf_config_utils import (
                resolve_scene_tf_config_path,
            )
            self._certification_block_reason = certification_block_reason
            self._load_runtime_atlas = load_runtime_atlas
            scene_path = resolve_scene_tf_config_path(
                getattr(args, "scene_tf_config", None) or None)
            self._scene_tf_hash = compute_scene_tf_hash(scene_path)
            self._reachability_scene_hash = compute_reachability_scene_hash(
                scene_path)
            self._atlas_cfg = load_atlas_runtime_for_scene(
                scene_path, os.environ.get("ATLAS_RUNTIME_CONFIG") or None)
            self._urdf_identity = UrdfIdentity(self)
            if self._atlas_cfg.get("enabled"):
                self._urdf_identity.start()
            self._atlas_key = None
        except Exception as exc:  # noqa: BLE001 - atlas is a prior, not the map
            self._atlas = None
            self._atlas_diag = {"reason": str(exc)}
            self._atlas_cfg = {"enabled": False}
            self._certification_block_reason = None
            self._load_runtime_atlas = None
            self._urdf_identity = None
            self._scene_tf_hash = ""
            self._atlas_key = None
        self._rehearsal_marks = None
        self._rehearsal_fault_record = None
        self._selected_path = None
        self._selected_carry_v2 = None
        self._execution_admission = None
        self._selector_rows = []
        self._last_boundary = None
        self._probe_place = self.create_client(
            ProbeMotionSegment, "/motion_planner/probe_motion_segment",
            callback_group=self._group)
        boundary_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/motion_planner/last_boundary", self._on_boundary,
            boundary_qos, callback_group=self._group)
        # Typed chain (F22): the v2 surface contract rides verbatim from the
        # mapper into SelectPlacePathV2; hand map/world conversions end here.
        self._surface_v2_msg = None
        if getattr(self, "_typed", None) is not None:
            from luggage_msgs.msg import SurfaceContractV2 as _SurfaceV2
            transient = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                _SurfaceV2, "/luggage/cargo_map/surface_v2",
                self._on_surface_v2, transient, callback_group=self._group)

    def _on_surface_v2(self, msg):
        self._surface_v2_msg = msg

    def _cartesian_min_fraction(self):
        """Selector yaml owns the live gate; tests may stub PlacePathPlanner."""
        selector = getattr(self, "_selector", None)
        if selector is not None:
            return float(selector.config.get("cartesian_min_fraction", 0.95))
        planner = getattr(self, "_planner", None)
        if planner is not None:
            return float(planner.cartesian_min_fraction)
        return 0.95

    def _leg_trajectory_digest(self, segment):
        """Rebuild the certified trajectory digest from a v2 leg message."""
        from st09_contracts.graph import TimedTrajectory
        trajectory = None
        if getattr(segment, "joint_trajectory", None) is not None:
            trajectory = getattr(segment.joint_trajectory, "trajectory",
                                 None)
        if trajectory is None or not trajectory.points:
            return ""
        points = list(trajectory.points)

        def _times():
            return tuple(
                int(p.time_from_start.sec) * 1_000_000_000
                + int(p.time_from_start.nanosec) for p in points)

        try:
            return TimedTrajectory(
                joint_names=tuple(trajectory.joint_names),
                positions=tuple(
                    tuple(float(v) for v in p.positions) for p in points),
                velocities=tuple(
                    tuple(float(v) for v in p.velocities) for p in points),
                accelerations=tuple(
                    tuple(float(v) for v in p.accelerations)
                    for p in points),
                time_from_start_ns=_times()).digest()
        except Exception:  # noqa: BLE001 - malformed leg = no digest match
            return ""

    def _verify_selected_route(self, response, request_id, stats):
        """[ST09-I S5.7] Re-verify the selected certified route.

        The probe NEVER generates a second path (certify A, execute A):
        the selector re-checks identity, start state, and map admission
        and re-certifies the exact edges against a live scene; the
        per-leg trajectory digests must then match what this driver
        holds, proving the legs about to execute are bit-for-bit the
        certified ones. A failure refuses THIS placement.
        """
        stats["mode"] = "verify"
        stats["verified"] = stats.get("verified", 0) + 1
        client = getattr(self, "_verify_route_client", None)
        if client is None:
            from luggage_msgs.srv import ValidateSelectedRoute
            client = self.create_client(
                ValidateSelectedRoute,
                "/trajectory_selector/validate_selected_route",
                callback_group=self._group)
            self._verify_route_client = client
        if not client.wait_for_service(timeout_sec=5.0):
            stats["unavailable"] = stats.get("unavailable", 0) + 1
            return False, "VERIFY_UNAVAILABLE"
        from luggage_msgs.srv import ValidateSelectedRoute
        request = ValidateSelectedRoute.Request()
        request.request_id = str(request_id)
        request.scene_revision = str(self._frame_scene_revision or "")
        request.route_digest = str(getattr(response, "path_hash", "") or "")
        request.current_joint_state = [
            float(v) for v in
            (_joint_positions(getattr(self, "_last_joint_state", None))
             or [])]
        # [ST09-I S5.7 lock] Map admission is fail-closed on the LATEST
        # map: a failed/absent stats fetch is its own named refusal and
        # NEVER falls back to the selection-time revision (a fabricated
        # 0 would masquerade as MAP_RESTART; a stale carry-over would
        # verify against a map nobody can still read).
        stats_client = getattr(self, "_stats", None)
        if stats_client is None:
            stats["map_status_unavailable"] = \
                stats.get("map_status_unavailable", 0) + 1
            return False, "MAP_STATUS_UNAVAILABLE"
        from luggage_msgs.srv import GetCargoMapStats
        stats_response = self.call_srv(
            stats_client, GetCargoMapStats.Request(), timeout=5.0)
        if stats_response is None or not stats_response.success:
            stats["map_status_unavailable"] = \
                stats.get("map_status_unavailable", 0) + 1
            return False, "MAP_STATUS_UNAVAILABLE"
        request.latest_map_revision = int(stats_response.map_revision)
        stats["map_source"] = str(stats_response.map_source or "")
        stats["admitted_map_revision"] = int(stats_response.map_revision)
        result = self.call_srv(client, request, timeout=PLACE_PROBE_TIMEOUT_S)
        if result is None:
            stats["unavailable"] = stats.get("unavailable", 0) + 1
            return False, "VERIFY_UNAVAILABLE"
        stats["reason"] = str(result.reason_code or "")
        if not result.valid:
            stats["rejected"] = stats.get("rejected", 0) + 1
            return False, str(result.reason_code or "VERIFY_REJECTED")
        local = [self._leg_trajectory_digest(segment)
                 for segment in (response.motion_segments or [])]
        certified = [str(digest) for digest in (result.leg_digests or [])]
        if local != certified or any(not digest for digest in local):
            stats["digest_mismatch"] = stats.get("digest_mismatch", 0) + 1
            return False, "LEG_DIGEST_MISMATCH"
        stats["start_state_delta_rad"] = float(
            result.start_state_delta_rad or 0.0)
        # [ST09-I S5.7-6] frozen-budget attribution (re-certification
        # ms vs the whole verify wall time) for the evidence records.
        stats["certify_ms"] = float(result.certify_ms or 0.0)
        stats["verify_ms"] = float(result.verify_ms or 0.0)
        stats["certification_id"] = str(result.certification_id or "")
        stats["validated_map_revision"] = int(
            result.validated_map_revision or 0)
        # [ST09-I S5.7-3b] every leg execution now carries the certified
        # admission; the executor checks the generation is STILL live
        # before each controller submission (certify A, execute A, and
        # the certification holds when A executes).
        self._execution_admission = {
            "route_digest": str(
                getattr(response, "path_hash", "") or ""),
            "certification_id": str(result.certification_id or ""),
            "admission_map_revision": int(
                result.validated_map_revision or 0),
            # [ST09-I S5.7-6] per-leg scopes: every executed leg claims
            # its own certified load-state index (carrying -> release
            # transition -> unloaded retreat).
            "leg_digests": list(certified),
            "leg_load_states": [
                str(state) for state in (result.leg_load_states or [])],
        }
        return True, ""

    def _frame_place_slot(self, pick_msg, trial):
        """Typed v2 chain: ComputePlacementV2 -> SelectPlacePathV2 -> probe.

        The whole sweep runs on the production services; this driver only
        correlates request ids and refuses fail-open paths. Ground truth
        stays the separate oracle it always was.
        """
        import time as _time
        from luggage_description.frame_contracts import ContractError
        from luggage_description.frame_diagnostics import BoundaryContext
        from luggage_description.ros_frame_bindings.poses import (
            pose_from_message)
        from luggage_msgs.msg import SlotSpec
        from luggage_msgs.srv import GetCargoMapStats
        from luggage_planning.typed_driver_adapter import (
            container_transform_fields, tcp_record)

        if not bool(getattr(pick_msg, "height_valid", False)):
            trial.fail_code = trial.fail_code or "DETECT_FULL_GEOMETRY_REQUIRED"
            return None, None
        surface_msg = self._surface_v2_msg
        if surface_msg is None:
            trial.fail_code = trial.fail_code or "SURFACE_V2_MISSING"
            return None, None
        stats = self.call_srv(self._stats, GetCargoMapStats.Request(),
                              timeout=5.0)
        geometry_hash = str(
            (stats.geometry_hash if stats else "")
            or str(surface_msg.geometry_hash or "") or "")
        box_wdh = [float(pick_msg.width), float(pick_msg.depth),
                   float(pick_msg.height)]
        request_id = self._frame_request_id("place")
        try:
            placement = self._typed.compute_placement(
                request_id=request_id, box_wdh=box_wdh,
                height_measured=True,
                scene_revision=self._frame_scene_revision,
                geometry_hash=geometry_hash,
                expected_map_revision=int(surface_msg.map_revision),
                # Deep-first placement keeps the back wall in the first
                # handful of slots. Those container-x cells are outside
                # the arm (run10: world TCP x=-2.06, IK residual 0.5 m).
                # The route producer reorders by increasing depth, so it
                # needs the shallow end of the feasible list in hand.
                max_candidates=2000)
        except ContractError as error:
            trial.fail_code = trial.fail_code or error.code
            trial.extras["compute_placement"] = "%s: %s" % (
                error.code, error.field)
            return None, None
        if placement is None or not placement.success:
            trial.fail_code = trial.fail_code or (
                getattr(placement, "reason_code", None) or "PLACEMENT_FAILED")
            trial.extras["compute_placement"] = (
                placement.message if placement else "timeout")
            return None, None
        trial.extras["compute_placement"] = placement.message
        slot_messages = list(placement.candidates) or [placement.slot]
        scores = list(placement.candidate_scores) or [0.0]
        # ComputePlacementV2 has no request id field; the planner stamps
        # the slots. Select, grasp, and the start TCP join that id.
        slot_request = str(slot_messages[0].identity.request_id or "")
        if slot_request:
            request_id = slot_request

        portal = self._portal_map()
        start_tcp = tcp_record(request_id, self._frame_scene_revision,
                               portal, (1.0, 0.0, 0.0, 0.0))
        grasp = getattr(self, "_frame_grasp", None)
        if grasp is None:
            trial.fail_code = trial.fail_code or "TYPED_PLACE_CONTEXT_MISSING"
            return None, None
        grasp = dict(grasp)
        grasp["request_id"] = request_id
        self._frame_grasp = grasp
        box_id = str(grasp.get("box_id") or "")
        generation = int(grasp.get("generation") or 0)
        probe_stats = {"mode": "on", "probed": 0, "ik_reject": 0,
                       "fraction_reject": 0, "unavailable": 0, "sec": 0.0,
                       "excluded": []}
        excluded = []
        selection = None
        probe_ok = True
        probe_reason = ""

        def _request_doc(current, rows):
            return self._planning_request_document(
                rows, request_id, box_wdh, start_joints, grasp,
                container_transform, surface_msg, stats, geometry_hash,
                self._stage_timings_from_selection(current))

        attempts = max(1, min(5, len(slot_messages)))
        start_joints = (_joint_positions(
            getattr(self, "_last_joint_state", None)) or [])
        container_transform = container_transform_fields(self._scene_config)
        inner_size = list(container_usable_dimensions(self._scene_config))
        for _attempt in range(attempts):
            try:
                selection = self._typed.select_place_path(
                    request_id=request_id, box_wdh=box_wdh,
                    slots=slot_messages, slot_scores=scores,
                    surface_message=surface_msg,
                    inner_size=inner_size,
                    geometry_hash=geometry_hash,
                    map_revision=int(surface_msg.map_revision),
                    start_tcp=start_tcp,
                    start_joints=start_joints,
                    grasp=grasp,
                    container_transform=container_transform,
                    excluded_ids=excluded,
                    box_id=box_id,
                    generation=generation)
            except ContractError as error:
                trial.fail_code = trial.fail_code or error.code
                trial.extras["selector_error"] = "%s: %s" % (
                    error.code, error.field)
                return None, None
            if selection is None or not selection.success:
                trial.fail_code = trial.fail_code or (
                    getattr(selection, "reason_code", None)
                    or PLACE_PATH_INFEASIBLE)
                trial.extras["selector_error"] = (
                    selection.reason_code if selection else "timeout")
                trial.extras["probe"] = probe_stats
                rows = (self._rows_from_selection(selection)
                        if selection is not None else [])
                invalidation = next(
                    (row for row in rows
                     if isinstance(row, dict) and "record_stamp_ns" in row),
                    None)
                if invalidation is not None:
                    trial.extras["attachment_invalidation"] = invalidation
                self._dump_selector(
                    trial, [], rows, None, trial.extras["selector_error"],
                    (self._surface_from_selection(selection)
                     if selection is not None else {}),
                    probe=probe_stats,
                    planning_request=_request_doc(selection, rows))
                return None, None
            if getattr(self._args, "place_probe", "on") != "on":
                probe_ok = True
                break
            t0 = _time.time()
            # [ST09-I S5.7] single re-verification: a failure REFUSES this
            # placement with its named reason — re-selecting another slot
            # is not the answer (the route was certified; what changed is
            # identity/state, which another slot shares).
            probe_ok, probe_reason = self._verify_selected_route(
                selection, request_id, probe_stats)
            probe_stats["sec"] = round(
                float(probe_stats.get("sec") or 0.0) + (_time.time() - t0), 3)
            break
        if getattr(self._args, "place_probe", "on") == "on" and not probe_ok:
            trial.fail_code = trial.fail_code or (
                probe_reason or PLACE_PATH_INFEASIBLE)
            trial.extras["selector"] = {
                "reason": probe_reason, "n_slots": len(slot_messages),
                "n_rows": 0, "winner_method": None,
                "winner_slot": None, "probe": probe_stats,
                "excluded": excluded}
            probe_rows = self._rows_from_selection(selection)
            self._dump_selector(trial, [], probe_rows, None, probe_reason,
                self._surface_from_selection(selection), probe=probe_stats,
                planning_request=_request_doc(selection, probe_rows))
            return None, None

        winner_index = int(selection.slot_index)
        chosen = slot_messages[winner_index]
        context = BoundaryContext(
            "F13", "typed_driver_adapter", "motion_occ_driver", request_id,
            self._frame_scene_revision, "container_link", "box_center",
            "scene")
        try:
            self._frame_slot_record = pose_from_message(
                chosen, "slot", context)
        except ContractError as error:
            trial.fail_code = trial.fail_code or error.code
            return None, None
        self._selected_path = {
            "method": str(selection.path_source),
            "selection_id": str(selection.selection_id),
            "path_hash": str(selection.path_hash),
            "slot_index": winner_index,
            "waypoint_frame": "world",
            "waypoints": [
                [float(pose.pose.position.x), float(pose.pose.position.y),
                 float(pose.pose.position.z)]
                for pose in selection.waypoint_poses],
        }
        self._selected_carry_v2 = list(selection.motion_segments)
        self._selected_carry_v1 = list(selection.carry_segments)
        trial.extras["path_source"] = str(selection.path_source)
        trial.extras["selection_id"] = str(selection.selection_id)
        trial.extras["selector"] = {
            "reason": "", "n_slots": len(slot_messages), "n_rows": 0,
            "winner_method": str(selection.path_source),
            "winner_slot": winner_index, "probe": probe_stats,
            "stage_timings": str(
                getattr(selection, "stage_timings_json", "") or "")}
        # v1 view for the shared trial bookkeeping: container -> base_link
        # through the shared scene helpers, never hand-rolled.
        from luggage_description.scene_tf_config_utils import (
            _local_point_to_base_link)
        record = self._frame_slot_record
        base_xyz = _local_point_to_base_link(
            list(record["position"]), self._scene_config)
        yaw_container = 2.0 * math.atan2(record["quaternion_xyzw"][2],
                                          record["quaternion_xyzw"][3])
        slot = SlotSpec()
        slot.place_pose.position.x = float(base_xyz[0])
        slot.place_pose.position.y = float(base_xyz[1])
        slot.place_pose.position.z = float(base_xyz[2])
        slot.place_pose.orientation.z = math.sin(0.5 * yaw_container)
        slot.place_pose.orientation.w = math.cos(0.5 * yaw_container)
        slot.width, slot.height, slot.depth = box_wdh[0], box_wdh[2], box_wdh[1]
        winner_rows = self._rows_from_selection(selection)
        self._dump_selector(trial, [], winner_rows,
                            self._selected_path,
                            "", self._surface_from_selection(selection),
                            probe=probe_stats,
                            planning_request=_request_doc(
                                selection, winner_rows))
        return slot, self._slot_meta(slot)

    @staticmethod
    def _rows_from_selection(selection):
        """Rows from the v2 response so probe rejects keep their evidence.

        Rehearsal-52's trial_01 candidates.json had ``rows: []`` -- the
        five polylines were lost and the replay had to regenerate them.
        """
        try:
            rows = json.loads(str(getattr(selection, "rows_json", "") or "[]"))
            return rows if isinstance(rows, list) else []
        except ValueError:
            return []

    @staticmethod
    def _surface_from_selection(selection):
        surface = getattr(selection, "surface", None)
        if isinstance(surface, dict) and surface:
            return surface
        return {}

    def _on_boundary(self, msg):
        try:
            boundary = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(boundary, dict) and "t_wall" in boundary:
            self._last_boundary = boundary

    def _wait_boundary(self, t0, timeout=1.5):
        """Latest node boundary record at/after wall time ``t0``."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            boundary = self._last_boundary
            if boundary is not None and boundary.get("t_wall", 0.0) >= t0:
                return boundary
            time.sleep(0.02)
        return self._last_boundary

    def _slot_target_map(self, slot):
        """Map-frame suction contact (box top). Carry Z is not a fixed lift.

        PlacePathPlanner raises from occupied tops and clamps to the lid.
        """
        pos = slot.place_pose.position
        local = _point_in_container_link(
            [pos.x, pos.y, pos.z], self._scene_config)
        height = float(slot.height)
        return (local[0], local[1], local[2] + 0.5 * height)

    def _portal_map(self):
        world = container_opening_target_point_in_world(self._scene_config)
        local = _world_to_map(self._scene_config, world)
        return (local[0], local[1], max(local[2] + 0.35, 0.80))

    def _opening_tangent_map(self):
        normal = container_opening_normal_in_world(self._scene_config)
        return (-float(normal[1]), float(normal[0]))

    def _stage_timings_from_selection(self, selection):
        raw = str(getattr(selection, "stage_timings_json", "") or "")
        if not raw:
            return {}
        try:
            parsed = json.loads(raw)
        except ValueError:
            return {}
        return parsed if isinstance(parsed, dict) else {}

    def _planning_request_document(self, rows, request_id, box_wdh,
                                   start_joints, grasp, container_transform,
                                   surface_msg, stats, geometry_hash,
                                   stage_timings):
        from luggage_gazebo.planning_request_capture import (
            build_planning_request,
            candidate_targets_from_rows,
        )
        from luggage_planning.planning_capture import candidates_from_rows

        translation, quaternion = [], []
        if isinstance(container_transform, (list, tuple)) and len(
                container_transform) >= 4:
            translation = list(container_transform[2])
            quaternion = list(container_transform[3])
        cfg = getattr(self, "_atlas_cfg", None) or {}
        diag = getattr(self, "_atlas_diag", None) or {}
        identity = "|".join((
            str(cfg.get("atlas_dir") or ""),
            str(cfg.get("basename") or ""),
            str(getattr(self, "_scene_tf_hash", "") or ""),
        ))
        snapshot = None
        if surface_msg is not None:
            snapshot = {
                "rows": int(getattr(surface_msg, "rows", 0) or 0),
                "columns": int(getattr(surface_msg, "columns", 0) or 0),
                "resolution_m": float(
                    getattr(surface_msg, "resolution_m", 0.0) or 0.0),
                "map_revision": int(
                    getattr(surface_msg, "map_revision", 0) or 0),
                "geometry_hash": str(
                    getattr(surface_msg, "geometry_hash", "") or ""),
            }
        grasp = grasp or {}
        revision = None
        if surface_msg is not None:
            revision = int(getattr(surface_msg, "map_revision", 0) or 0)
        return build_planning_request({
            "request_id": request_id,
            "box_wdh": box_wdh,
            "start_joints": start_joints,
            "attachment_transform": {
                "translation": list(grasp.get("position") or []),
                "quaternion_xyzw": list(grasp.get("quaternion_xyzw") or []),
            },
            "container_transform": {
                "translation": translation,
                "quaternion_xyzw": quaternion,
            },
            "map_revision": revision,
            "map_source": str(getattr(stats, "map_source", "") or ""),
            "geometry_hash": geometry_hash,
            "candidate_targets": candidate_targets_from_rows(rows),
            "candidates": candidates_from_rows(rows),
            "atlas_identity": {
                "identity": identity,
                "enabled": bool(cfg.get("enabled")),
                "loaded": bool(diag.get("loaded")),
                "reason": str(diag.get("reason") or ""),
            },
            "stage_timings": stage_timings or {},
            "map_snapshot": snapshot,
        })

    def _dump_selector(self, trial, candidates, rows, winner, reason, surface,
                       probe=None, planning_request=None):
        folder = os.path.join(self._args.dump_dir, "box_%02d_paths" % trial.index)
        os.makedirs(folder, exist_ok=True)
        payload = {
            "geometry_hash": (surface or {}).get("geometry_hash"),
            "map_revision": (surface or {}).get("map_revision"),
            "reason": reason,
            "n_slots": len(candidates),
            "n_rows": len(rows),
            "winner": winner,
            "rows": dump_rows(rows),
            "candidates": candidates,
            "probe": probe,
        }
        if planning_request:
            payload["geometry_hash"] = planning_request.get(
                "geometry_hash") or payload["geometry_hash"]
            payload["map_revision"] = planning_request.get("map_revision")
            payload["map_source"] = planning_request.get("map_source")
        with open(os.path.join(folder, "candidates.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        if planning_request:
            with open(os.path.join(folder, "planning_request.json"), "w",
                      encoding="utf-8") as handle:
                json.dump(planning_request, handle, indent=2, sort_keys=True,
                          default=str)
                handle.write("\n")
        trial.extras["selector"] = {
            "reason": reason,
            "n_slots": len(candidates),
            "n_rows": len(rows),
            "winner_method": None if not winner else winner.get("method"),
            "winner_slot": None if not winner else winner.get("slot_index"),
            "probe": probe,
            "dump": folder,
        }
        return folder

    def _runtime_atlas(self):
        """Load the atlas with the live scene and URDF hashes.

        The cache key includes both hashes, so a description that arrives
        after startup replaces an unverified refusal instead of sticking.
        """
        cfg = getattr(self, "_atlas_cfg", None) or {}
        if not cfg.get("enabled") or self._load_runtime_atlas is None:
            return None, {"loaded": False, "reason": "disabled"}
        if self._urdf_identity is not None:
            self._urdf_identity.poll()
        urdf_hash = (
            self._urdf_identity.hash
            if self._urdf_identity is not None and self._urdf_identity.ready
            else "")
        key = (getattr(self, "_reachability_scene_hash", ""),
               self._scene_tf_hash, urdf_hash)
        if key != self._atlas_key:
            self._atlas, self._atlas_diag = self._load_runtime_atlas(
                cfg, scene_tf_hash=self._scene_tf_hash, urdf_hash=urdf_hash,
                reachability_scene_hash=getattr(
                    self, "_reachability_scene_hash", ""))
            self._atlas_key = key
        return self._atlas, self._atlas_diag

    def _place_slot_for(self, pick_msg, box_msg, trial):
        self._rehearsal_mark("placement_request")
        try:
            return self._place_slot_resolved(pick_msg, box_msg, trial)
        finally:
            self._rehearsal_mark("placement_response")

    def _place_slot_resolved(self, pick_msg, box_msg, trial):
        del box_msg
        if getattr(self, "_typed", None) is None:
            # [ST09-I] the in-process selection was the geometric bypass;
            # the typed chain (ComputePlacementV2 -> SelectPlacePathV2
            # certified route) is the only production path.
            trial.fail_code = trial.fail_code or "PLANNING_ROUTE_REQUIRED"
            trial.extras["selector_error"] = (
                "legacy in-process selection removed (ST09-I)")
            return None, None
        if not bool(getattr(pick_msg, "height_valid", False)):
            trial.fail_code = trial.fail_code or "DETECT_FULL_GEOMETRY_REQUIRED"
            return None, None
        return self._frame_place_slot(pick_msg, trial)

    def _placement_atlas_extra(self, slot):
        """Atlas status of the selected slot from the placement dump.

        The ComputePlacement response carries no atlas fields; the latched
        /placement_planner/last_result does. The motion selector may pick
        any of the ranked candidates, so match the chosen slot's container
        x/y against the dumped candidates.
        """
        last = getattr(self, "_last_result", None)
        if not isinstance(last, dict):
            return None
        extra = {"gate": last.get("atlas") or None, "selected": None}
        target = self._slot_target_map(slot)
        best = None
        for cand in last.get("candidates") or []:
            center = cand.get("center_local")
            if not center or len(center) < 2:
                continue
            dist = math.hypot(
                float(center[0]) - target[0], float(center[1]) - target[1])
            if best is None or dist < best[0]:
                best = (dist, cand)
        if best is not None and best[0] <= 0.15:
            cand = best[1]
            extra["selected"] = {
                "atlas_status": cand.get("atlas_status"),
                "atlas_status_name": cand.get("atlas_status_name"),
                "reachability_prior": cand.get("reachability_prior"),
                "atlas_contact_z": cand.get("atlas_contact_z"),
                "match_dist_m": round(best[0], 4),
            }
        return extra

    def _apply_selected_carry(self, segment):
        """[ST09-I] no-op: certified route legs carry their own timed
        trajectories; the driver never rewrites motion specs."""

    def _execute_segment(self, segment, trial):
        self._apply_selected_carry(segment)
        t0 = time.time()
        ok, code, rec = super()._execute_segment(segment, trial)
        if rec is not None:
            rec["occupancy_checked"] = bool(self._surface_2d)
            rec["selector_method"] = (
                None if not self._selected_path
                else self._selected_path.get("method"))
            if not ok and PLACE_PATH_INFEASIBLE in str(
                    rec.get("message") or ""):
                code = PLACE_PATH_INFEASIBLE
        if str(segment.name) in ("transit", "traverse"):
            boundary = self._wait_boundary(t0)
            sources = trial.extras.setdefault("payload_sources", {})
            sources[str(segment.name)] = (
                None if not boundary else boundary.get("payload_source"))
            if rec is not None and boundary is not None:
                rec["payload_source"] = boundary.get("payload_source")
                rec["payload_wdh"] = boundary.get("payload_wdh")
        return ok, code, rec

    def _rehearsal_mark(self, name):
        marks = self._rehearsal_marks
        if marks is None:
            return
        marks.append({
            "name": str(name),
            "t_ros": float(self.ros_now_sec()),
            "t_wall": time.time(),
        })

    def _effect_window(self):
        state = dict(self._vacuum_state or {})
        place_state = str(self._place_state or "")
        return {
            "segments": len(getattr(self, "_segments_log", None) or []),
            "commits": 1 if place_state in ("COMMITTED", "VERIFIED", "HOME") else 0,
            "attached": bool(state.get("attached")) and not state.get("fail_reason"),
            "place_state": place_state,
        }

    def _rehearsal_refuse(self, trial, phase):
        """Drop peer observation, require the gate to refuse, then recover.

        The refusal itself performs no motion and no map commit. Recovery
        only resumes heartbeats; it does not start a place.
        """
        from luggage_description.frame_contracts import ContractError
        from luggage_gazebo.rehearsal_evidence import fault_window_ok

        before = self._effect_window()
        runtime = self._frame_runtime_handle
        code = "REHEARSAL_FAULT_NO_RUNTIME"
        if runtime is not None:
            runtime.suppress_peer_observation = True
            with runtime._lock:
                runtime.gate.reports.clear()
            try:
                runtime.require_graph_ready()
                code = "REHEARSAL_FAULT_NOT_REFUSED"
            except ContractError as error:
                code = str(error.code)
        after = self._effect_window()
        recovered = False
        if runtime is not None:
            runtime.suppress_peer_observation = False
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if runtime.readiness() == "READY":
                    recovered = True
                    break
                time.sleep(0.05)
        after_recovery = self._effect_window()
        record = {
            "fault": phase,
            "refusal_code": code,
            "before": before,
            "after": after,
            "after_recovery": after_recovery,
            "recovered_ready": recovered,
            "counters": dict(runtime.counters) if runtime is not None else {},
        }
        record["ok"] = fault_window_ok(
            phase, code, before, after, after_recovery, recovered)
        trial.fail_code = code
        trial.extras["rehearsal_fault"] = record
        self._rehearsal_fault_record = record
        return record

    def _pick_carry(self, index, trial, already_spawned=False):
        if getattr(self._args, "rehearsal_fault", "") == "pre_pick":
            self._rehearsal_refuse(trial, "pre_pick")
            return None
        result = super()._pick_carry(
            index, trial, already_spawned=already_spawned)
        if result is not None:
            self._rehearsal_mark("observation")
        return result

    def run_place_from_carry(self, pick_msg, trial, slot, slot_meta):
        if getattr(self._args, "rehearsal_fault", "") == "pre_place":
            self._rehearsal_refuse(trial, "pre_place")
            return trial
        return super().run_place_from_carry(pick_msg, trial, slot, slot_meta)

    def _integrate_after_place(self, trial, slot, slot_meta, pick_msg,
                               view="lift", record=True, expected_revision=None):
        result = super()._integrate_after_place(
            trial, slot, slot_meta, pick_msg, view=view, record=record,
            expected_revision=expected_revision)
        self._rehearsal_mark("integration")
        return result

    def run_trial(self, index, slot=None, slot_meta=None,
                  already_spawned=False, keep_placed=False):
        active = bool(getattr(self._args, "rehearsal_dir", ""))
        self._rehearsal_marks = [] if active else None
        trial = super().run_trial(
            index, slot=slot, slot_meta=slot_meta,
            already_spawned=already_spawned, keep_placed=keep_placed)
        if self._rehearsal_marks is not None:
            integrate = trial.extras.get("integrate") or {}
            geometry = str(integrate.get("geometry_hash") or "")
            if not geometry:
                geometry = str((self._surface_2d or {}).get("geometry_hash") or "")
            trial.extras["rehearsal_marks"] = list(self._rehearsal_marks)
            trial.extras["state_timeline"] = [dict(row) for row in self._timeline]
            trial.extras["vacuum_event"] = dict(self._vacuum_events or {})
            trial.extras["live_scene_revision"] = str(
                getattr(self, "_frame_scene_revision", "") or "")
            trial.extras["live_geometry_hash"] = geometry
            if not trial.extras.get("rehearsal_request_id"):
                trial.extras["rehearsal_request_id"] = "%s-%s" % (
                    trial.catalog_id or "box", trial.index)
        return trial


def parse_args(argv=None):
    argv = list(argv or [])
    # This driver's own flags; strip them so the shared place parser (and
    # every other driver reusing it) never sees unknown options.
    local = argparse.ArgumentParser(add_help=False)
    local.add_argument("--place-probe", choices=("on", "off"), default="on")
    local.add_argument("--frame-runtime", action="store_true",
                       help="Adopt the v2 runtime chain from the stack graph")
    local.add_argument("--frame-peer", default="motion_occ_driver")
    local.add_argument("--rehearsal", default="",
                       help="Write non-scoring rehearsal evidence to this directory")
    local.add_argument("--rehearsal-fault", choices=("", "pre_pick", "pre_place"),
                       default="",
                       help="Inject one readiness refusal without scoring")
    local_ns, remaining = local.parse_known_args(argv)
    # MOTION-OCC is an n=2 eval; default it here once. (The old
    # n==3 post-normalization never fired: main() had already appended
    # --n 2, and an explicit --n 3 suppressed it on purpose.)
    if "--n" not in remaining:
        remaining.extend(["--n", "2"])
    args = parse_integrate_args(remaining)
    args.place_probe = local_ns.place_probe
    args.frame_runtime = local_ns.frame_runtime
    args.frame_peer = local_ns.frame_peer
    args.rehearsal_dir = str(local_ns.rehearsal or "")
    args.rehearsal_fault = str(local_ns.rehearsal_fault or "")
    return args


def _place_reached(record):
    if place_ok(record):
        return True
    if record.fail_code in ("", None, "COVERAGE_LOW"):
        state = str(getattr(record, "place_state", "") or "")
        return state in ("VERIFIED", "HOME", "COMMITTED", "RELEASED")
    return False


def _write_suite(out, records, driver):
    dumps = driver._args.dump_dir
    complete = True
    occupancy_blind = False
    # Privilege gate: every carry-segment occupancy sweep from trial 1 on
    # must have used the perception-measured payload, never the static
    # fallback envelope.
    payload_measured = True
    for rec in records:
        if rec.index < 1:
            continue
        sources = rec.extras.get("payload_sources") or {}
        carry = [v for k, v in sources.items() if k in ("transit", "traverse")]
        if not carry or any(s != PAYLOAD_SOURCE_MEASURED for s in carry):
            payload_measured = False
    for rec in records:
        folder = os.path.join(dumps, "box_%02d_paths" % rec.index)
        if rec.index >= 1 or rec.fail_code == PLACE_PATH_INFEASIBLE:
            if not os.path.isfile(os.path.join(folder, "candidates.json")):
                complete = False
        if rec.fail_code in RETURN_FAIL_CODES + (PLACE_PATH_INFEASIBLE,):
            replay = os.path.join(
                dumps, "trial_%02d_%s" % (
                    rec.index, rec.fail_code), "replay", "manifest.json")
            if not os.path.isfile(replay):
                complete = False
            else:
                try:
                    with open(replay, "r", encoding="utf-8") as handle:
                        manifest = json.load(handle)
                    if not manifest.get("replay_possible"):
                        complete = False
                except (OSError, ValueError):
                    complete = False
        if rec.fail_code and rec.fail_code.startswith("PLACE_PLAN_"):
            extra = rec.extras.get("selector") or {}
            if int(extra.get("n_rows") or 0) < 2:
                occupancy_blind = True
    probe_stats = [
        (r.extras.get("selector") or {}).get("probe") for r in records]
    probe_stats = [s for s in probe_stats if s]
    payload = {
        "n": len(records),
        "place_ok": sum(1 for r in records if _place_reached(r)),
        "fail_codes": [r.fail_code for r in records],
        "clock_publishers": driver.clock_publishers(),
        "capture_complete": complete,
        "occupancy_blind": occupancy_blind,
        "payload_measured": payload_measured,
        "place_probe": {
            "mode": driver._args.place_probe,
            "modes": [s.get("mode") for s in probe_stats],
            "probed": sum(int(s.get("probed") or 0) for s in probe_stats),
            "ik_reject": sum(int(s.get("ik_reject") or 0) for s in probe_stats),
            "fraction_reject": sum(
                int(s.get("fraction_reject") or 0) for s in probe_stats),
            "unavailable": sum(
                int(s.get("unavailable") or 0) for s in probe_stats),
            "sec": round(sum(float(s.get("sec") or 0.0)
                             for s in probe_stats), 3),
        },
        "selectors": [r.extras.get("selector") for r in records],
        "atlas": {
            "selected": [
                (r.extras.get("placement_atlas") or {}).get("selected")
                for r in records],
            "gate": [
                (r.extras.get("placement_atlas") or {}).get("gate")
                for r in records],
            "n_selected_reachable": sum(
                1 for r in records
                if ((r.extras.get("placement_atlas") or {})
                    .get("selected") or {}).get("atlas_status") in (2, 3)),
        },
    }
    with open(os.path.join(out, "suite.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return payload


def _write_t0(out, extra, args, records, suite, started_at):
    payload = {
        "case_ids": [
            {"index": r.index, "catalog_id": r.catalog_id,
             "fail_code": r.fail_code}
            for r in records],
        "commit": extra.get("commit"),
        "dirty": extra.get("dirty"),
        "n": args.n,
        "driver": "motion_occ_eval_driver.py",
        "ros_domain_id": extra.get("ros_domain_id"),
        "started_at": started_at,
        "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "clock_publishers": suite.get("clock_publishers"),
        "suite": suite,
        "dump_matrix": {
            "T0": "t0_manifest.json",
            "T1": "dumps/box_NN_paths/candidates.json",
            "T2": "replay/last_boundary.json + planning_scene.json on fail",
        },
        "cargo_view_policy": dict(view_policy_record(), hash=policy_hash()),
        "cargo_view_choices": [
            {
                "index": r.index,
                "aim": (r.extras.get("cargo_view_aim") or {}).get("chosen"),
                "lookback": (r.extras.get("cargo_view_lookback") or {}).get("chosen"),
            }
            for r in records],
    }
    path = os.path.join(out, "t0_manifest.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return path


def _atomic_json(path, payload):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    return path


def _write_rehearsal(directory, records):
    """Non-scoring evidence. The extractor binds hashes in a later step."""
    from luggage_gazebo.rehearsal_evidence import (
        build_evidence, case_from_artifacts,
    )
    cases = [case_from_artifacts(trial_to_dict(record)) for record in records]
    evidence = build_evidence(cases)
    path = _atomic_json(os.path.join(directory, "rehearsal-evidence.json"), evidence)
    return path, evidence


def main(argv=None):
    argv = list(argv or sys.argv[1:])
    if "--out" not in argv:
        argv.extend(["--out", DEFAULT_OUT])
    # --n defaults to 2 inside parse_args (single source of that default).
    args = parse_args(argv)
    if args.rehearsal_dir and args.out == DEFAULT_OUT:
        args.out = args.rehearsal_dir
    os.makedirs(args.out, exist_ok=True)
    args.dump_dir = args.dump_dir or os.path.join(args.out, "dumps")
    os.makedirs(args.dump_dir, exist_ok=True)
    jsonl = os.path.join(args.out, "trials.jsonl")
    with open(jsonl, "w", encoding="utf-8"):
        pass
    rclpy.init()
    driver = MotionOccDriver(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    records = []
    stop = {"flag": False}

    def _on_sigint(_signum, _frame):
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_sigint)
    extra = _git_meta(os.path.normpath(os.path.join(_SCRIPTS, "..", "..", "..")))
    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    try:
        if not args.skip_graph_check:
            err = driver.wait_graph(timeout=180.0)
            if err:
                print("graph: %s" % err)
                if args.rehearsal_dir or args.rehearsal_fault:
                    _atomic_json(
                        os.path.join(args.rehearsal_dir or args.out,
                                     "startup_failure.json"),
                        {"reason": err, "cases_attempted": 0,
                         "scoring_writes": False})
                return 2
        print("graph ok; MOTION-OCC n=%d" % args.n, flush=True)
        time.sleep(8.0)
        if args.rehearsal_fault:
            rec = driver.run_trial(0)
            records.append(rec)
            with open(jsonl, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(trial_to_dict(rec), sort_keys=True) + "\n")
            fault = driver._rehearsal_fault_record or rec.extras.get(
                "rehearsal_fault") or {}
            path = _atomic_json(
                os.path.join(args.rehearsal_dir or args.out,
                             "rehearsal-fault.json"), fault)
            print(json.dumps({"fault": path, "ok": fault.get("ok"),
                              "refusal_code": fault.get("refusal_code")},
                             sort_keys=True), flush=True)
            return 0 if fault.get("ok") else 1
        for index in range(args.n):
            if stop["flag"] or not rclpy.ok():
                break
            rec = driver.run_trial(index)
            records.append(rec)
            with open(jsonl, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(trial_to_dict(rec), sort_keys=True) + "\n")
            print(json.dumps({
                "index": rec.index, "fail_code": rec.fail_code,
                "selector": rec.extras.get("selector"),
            }, sort_keys=True), flush=True)
            if rec.fail_code and args.on_place_fail == "stop":
                if rec.fail_code not in RETURN_FAIL_CODES + ("COVERAGE_LOW",):
                    break
        ws = os.path.normpath(os.path.join(_SCRIPTS, "..", "..", ".."))
        extra.update({
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "dirty": subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=ws, text=True).count("\n"),
        })
        if args.rehearsal_dir:
            path, evidence = _write_rehearsal(args.rehearsal_dir, records)
            note = {"scoring_writes": False, "namespace": "rehearsal",
                    "evidence": path, "complete": evidence.get("complete")}
            _write_t0(args.out, extra, args, records, note, started_at)
            print(json.dumps(note, sort_keys=True), flush=True)
            if not records:
                return 2
            return 0 if evidence.get("complete") else 1
        suite = _write_suite(args.out, records, driver)
        _write_t0(args.out, extra, args, records, suite, started_at)
        summary = summarize(records)
        with open(os.path.join(args.out, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        print(json.dumps(suite, sort_keys=True, default=str), flush=True)
        if (suite.get("occupancy_blind") or not suite.get("capture_complete")
                or not suite.get("payload_measured")):
            return 1
        if suite.get("clock_publishers") not in (1, -1):
            return 1
        if any(r.fail_code == "BIN_FULL" for r in records):
            return 1
        # Trial 1 (index 1) placed, or PLACE_PATH_INFEASIBLE with dumped set.
        if len(records) >= 2:
            second = records[1]
            extra = second.extras.get("selector") or {}
            if _place_reached(second):
                return 0
            if (second.fail_code == PLACE_PATH_INFEASIBLE
                    and int(extra.get("n_slots") or 0) >= 2
                    and int(extra.get("n_rows") or 0) >= 4):
                return 0
            return 1
        return 0 if records and _place_reached(records[-1]) else 1
    finally:
        executor.remove_node(driver)
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
