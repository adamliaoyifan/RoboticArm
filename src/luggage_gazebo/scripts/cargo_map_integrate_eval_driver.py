#!/usr/bin/env python3
"""OCC-1 cargo-map integrate eval.

Place-smoke n=3 (keep placed, no cargo-map AddPlacedBox). After each
PLACE_RETREAT, Cartesian-lift the wrist (camera-down) so the D435 is
outside min-range and the box top fills the FOV, then call
IntegrateCargoView on depth points inside the placed box volume.
The slot and the suitcase size only crop the cloud; an empty crop does
not paint the planned slot. Score sensor
coverage against ign AABB (score-only). Accumulation is the RS-1 claim:
after trial 3 every GT footprint still has a sensor cell.
"""

from __future__ import division

import argparse
import json
import math
import os
import re
import signal
import shutil
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from luggage_msgs.action import PlanMotion
from luggage_msgs.msg import MotionSegment, SlotSpec
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from luggage_description.scene_tf_config_utils import (
    _local_point_to_base_link,
    _point_in_container_link,
    container_in_base_link,
    container_opening_normal_in_world,
    xyz_base_link_to_world,
    xyz_world_to_base_link,
    yaw_base_link_to_world,
    yaw_world_to_base_link,
)
from luggage_gazebo.cargo_view_sequence import (
    BLENDS,
    RIM_MARGIN_M,
    box_corners,
    footprint_sensor_hits,
    lookback_eye,
    next_expected_revision,
    observation_target,
    rank_observation_aims,
    view_policy_record,
    view_stop,
)
from luggage_perception.cargo_view_diagnosis import (
    _polygon_mask,
    classify_empty_view,
    project_box_corners,
    region_depth_stats,
    rim_occlusion_fraction,
)
from luggage_msgs.srv import ComputePlacement, GetCargoMapStats, IntegrateCargoView
from luggage_perception import ros_message_adapters as adapters
from luggage_gazebo.eval_metrics import depth_to_camera_xyz
from luggage_perception.cargo_instance_tracker import (
    rotation_from_xyzw,
    transform_points,
)
from luggage_perception.cargo_view_integration import (
    SCHEMA_VERSION,
    box_point_mask,
    floor_through_count,
    footprint_sensor_coverage,
    footprint_sensor_height,
    occupancy_height_matches,
    occupied_sensor_aabb,
    optical_axis_angle_rad,
    stamp_to_sec,
)
from luggage_planning.container_aim_utils import look_at_quaternion

_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)
from place_smoke_driver import (  # noqa: E402
    PlaceSmokeDriver,
    _yaw_quat,
    parse_args as parse_place_args,
)
from luggage_gazebo.place_metrics import (  # noqa: E402
    RETURN_FAIL_CODES,
    place_ok,
    summarize,
    trial_to_dict,
)
from pick_retreat_eval_driver import _git_meta  # noqa: E402

DEFAULT_OUT = os.path.normpath(os.path.join(
    _SCRIPTS, "..", "..", "..", "docs", "status", "evidence",
    "cargo_map_integrate", "latest"))
COVERAGE_MIN = 0.40
BOX_SYNTHETIC = [0.55, 0.40, 0.25]
# PLACE_RETREAT is 0.15 m above the box top, below D555 VGA Min-Z (0.26 m).
# A 0.25 m camera-down lift is the longest cartesian hop that still reaches
# fraction 1.0 (0.50 m was 0.94, hop-2 0.885). That view tops out at 0.29
# footprint coverage because the wrist still occludes the lid. A second
# IntegrateCargoView from the portal, camera aimed at the box, fills the rest.
VIEW_LIFT_Z = 0.25
# One voxel. Admits the settled surface without pulling the container wall in.
BOX_DEPTH_MARGIN_M = 0.05
BOX_DEPTH_MIN_POINTS = 30


def _quat_from_R(rot):
    """3x3 rotation matrix -> (x, y, z, w). Same conversion as look_at_quaternion."""
    trace = rot[0][0] + rot[1][1] + rot[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2][1] - rot[1][2]) / s
        y = (rot[0][2] - rot[2][0]) / s
        z = (rot[1][0] - rot[0][1]) / s
    elif rot[0][0] > rot[1][1] and rot[0][0] > rot[2][2]:
        s = math.sqrt(1.0 + rot[0][0] - rot[1][1] - rot[2][2]) * 2.0
        w = (rot[2][1] - rot[1][2]) / s
        x = 0.25 * s
        y = (rot[0][1] + rot[1][0]) / s
        z = (rot[0][2] + rot[2][0]) / s
    elif rot[1][1] > rot[2][2]:
        s = math.sqrt(1.0 + rot[1][1] - rot[0][0] - rot[2][2]) * 2.0
        w = (rot[0][2] - rot[2][0]) / s
        x = (rot[0][1] + rot[1][0]) / s
        y = 0.25 * s
        z = (rot[1][2] + rot[2][1]) / s
    else:
        s = math.sqrt(1.0 + rot[2][2] - rot[0][0] - rot[1][1]) * 2.0
        w = (rot[1][0] - rot[0][1]) / s
        x = (rot[0][2] + rot[2][0]) / s
        y = (rot[1][2] + rot[2][1]) / s
        z = 0.25 * s
    return (float(x), float(y), float(z), float(w))


class CargoMapIntegrateDriver(PlaceSmokeDriver):

    def __init__(self, args):
        super().__init__(args)
        self._out = str(args.out)
        self._surface_2d = None
        self._untracked = {"msg": None, "recv": None}
        self._gt_aabbs = []
        self._t1 = []
        group = self._group
        latch = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            String, "/luggage/cargo_map/surface_2d",
            self._on_surface, latch, callback_group=group)
        self.create_subscription(
            PointCloud2, "/luggage/semantic/cargo_points_untracked",
            self._on_untracked, sensor, callback_group=group)
        self._geometry_cloud_pub = self.create_publisher(
            PointCloud2, "/luggage/semantic/cargo_points_untracked", sensor)
        self._integrate = self.create_client(
            IntegrateCargoView, "/cargo_map/integrate_cargo_view",
            callback_group=group)
        self._stats = self.create_client(
            GetCargoMapStats, "/cargo_map/get_stats",
            callback_group=group)
        self._compute = self.create_client(
            ComputePlacement, "/placement_planner/compute_placement",
            callback_group=group)
        self._pending_integrate = None
        self._last_result = None
        self.create_subscription(
            String, "/placement_planner/last_result",
            self._on_last_result, latch, callback_group=group)

    def graph_error(self):
        err = super().graph_error()
        if err:
            return err
        names = set(self.get_node_names())
        missing = [
            n for n in ("cargo_volume_mapper", "placement_planner",
                        "semantic_point_filter")
            if n not in names]
        if missing:
            return "missing nodes: %s" % ", ".join(missing)
        return ""

    def _on_surface(self, msg):
        try:
            self._surface_2d = json.loads(msg.data)
        except ValueError:
            pass

    def _on_untracked(self, msg):
        self._untracked["msg"] = msg
        self._untracked["recv"] = time.time()

    def run_trial(self, index, slot=None, slot_meta=None,
                  already_spawned=False, keep_placed=False):
        del keep_placed
        return super().run_trial(
            index, slot=slot, slot_meta=slot_meta,
            already_spawned=already_spawned, keep_placed=True)

    def run_place_from_carry(self, pick_msg, trial, slot, slot_meta):
        self._pending_integrate = (trial, slot, slot_meta, pick_msg)
        try:
            return super().run_place_from_carry(pick_msg, trial, slot, slot_meta)
        finally:
            self._pending_integrate = None

    def _home_arm(self):
        pending = self._pending_integrate
        if pending and self._place_state in ("VERIFIED", "COMMITTED", "HOME"):
            trial, slot, slot_meta, pick_msg = pending
            self._pending_integrate = None
            self._run_post_place_views(trial, slot, slot_meta, pick_msg)
        return super()._home_arm()

    def _accept_view(self, trial, final, traces):
        if not final:
            trial.fail_code = trial.fail_code or "VIEW_EMPTY"
            return
        final = dict(final)
        final["views"] = traces
        if not view_stop(final):
            trial.fail_code = trial.fail_code or (
                final.get("empty_reason")
                or final.get("reason_code")
                or final.get("drop_reason")
                or "VIEW_EMPTY")
        self._t1.append(final)
        trial.extras["integrate"] = final
        trial.extras["coverage"] = final.get("coverage")
        dump = os.path.join(
            self._args.dump_dir, "box_%02d_integrate" % trial.index)
        self._dump_json(dump, "t1_trace.json", final)
        view = str(final.get("view") or "aim")
        for name in (
                "integrate_request", "integrate_response",
                "surface_2d", "reference", "solver_replay"):
            src = os.path.join(dump, "%s_%s.json" % (name, view))
            dst = os.path.join(dump, "%s.json" % name)
            if os.path.isfile(src):
                shutil.copyfile(src, dst)

    def _run_post_place_views(self, trial, slot, slot_meta, pick_msg):
        """Aim from the portal, then look back. Stop on the first footprint hit."""
        dump = os.path.join(
            self._args.dump_dir, "box_%02d_integrate" % trial.index)
        os.makedirs(dump, exist_ok=True)
        exit_ok, exit_msg = self._exit_to_portal()
        self._portal_exited = bool(exit_ok)
        trial.extras["cargo_view_portal"] = {
            "ok": bool(exit_ok), "message": exit_msg}
        if not exit_ok:
            trial.fail_code = trial.fail_code or "PORTAL_EXIT_FAILED"
            self._dump_json(dump, "aim_attempts.json", [{
                "view": "aim", "skipped": True, "reason": exit_msg}])
            return
        time.sleep(0.4)
        try:
            cam_xyz, cam_q = self._lookup_xyz_quat(
                "world", "camera_depth_optical_frame")
            suc_t, suc_q = self._lookup_xyz_quat(
                "camera_depth_optical_frame", "suction_contact_frame")
        except Exception as exc:  # noqa: BLE001 - T1 evidence
            trial.fail_code = trial.fail_code or "VIEW_TF_MISSING"
            trial.extras["cargo_view_aim"] = {"ok": False, "message": str(exc)}
            return
        slot_position = list(
            (slot_meta or {}).get("pose_world", {}).get("position") or [])
        if len(slot_position) < 3:
            trial.fail_code = trial.fail_code or "VIEW_EMPTY"
            return
        # [S5.7-4] the observation TARGET is policy-owned.
        target = observation_target(slot_position, pick_msg.height)
        center, size, yaw = self._placed_box_container(slot, pick_msg)
        stats = self.call_srv(self._stats, GetCargoMapStats.Request(), timeout=5.0)
        stats_rev = int(stats.map_revision) if stats else 0
        aim_attempts, aim_chosen = self._execute_view_candidates(
            dump, "aim", "cargo_view_aim", cam_xyz, cam_q, target,
            suc_t, suc_q, center, size, yaw)
        trial.extras["cargo_view_aim"] = {
            "ok": aim_chosen is not None,
            "chosen": aim_chosen,
            "n_attempts": len(aim_attempts),
            "message": "" if aim_chosen else "no aim candidate executed",
        }
        if aim_chosen is not None:
            time.sleep(0.4)
        t_aim = self._integrate_after_place(
            trial, slot, slot_meta, pick_msg, view="aim", record=False,
            expected_revision=stats_rev)
        traces = {"aim": t_aim}
        final = t_aim
        if not view_stop(t_aim):
            normal = container_opening_normal_in_world(self._scene_config)
            eye = lookback_eye(cam_xyz, normal)
            look_attempts, look_chosen = self._execute_view_candidates(
                dump, "lookback", "cargo_view_lookback", eye, cam_q, target,
                suc_t, suc_q, center, size, yaw, eye_raises=(0.0,))
            trial.extras["cargo_view_lookback"] = {
                "ok": look_chosen is not None,
                "chosen": look_chosen,
                "n_attempts": len(look_attempts),
                "eye": [float(v) for v in eye],
            }
            if look_chosen is not None:
                time.sleep(0.4)
            t_look = self._integrate_after_place(
                trial, slot, slot_meta, pick_msg, view="lookback",
                record=False,
                expected_revision=next_expected_revision(t_aim, stats_rev))
            traces["lookback"] = t_look
            final = t_look
        self._accept_view(trial, final, traces)
        # The aim/lookback route leaves the portal. Re-mint the exit
        # proof at the certified portal pose before _home_arm classifies
        # the live TCP. A failed re-exit leaves the proof void so home
        # refuses by name instead of trusting the pre-view joints.
        self._restore_portal_after_views(trial)

    def _restore_portal_after_views(self, trial):
        back_ok, back_msg = self._exit_to_portal()
        joints = (getattr(self, "_last_joint_state", None) or {}).get(
            "position") or []
        proof = getattr(self, "_portal_exit_proof", None) or {}
        trial.extras["cargo_view_portal_return"] = {
            "ok": bool(back_ok),
            "message": str(back_msg or ""),
            "t_ros": float(self.ros_now_sec()),
            "final_joint_values": [float(v) for v in list(joints)[:6]],
            "proof_ns": int(proof.get("proof_ns") or 0) if back_ok else 0,
        }
        if not back_ok:
            trial.fail_code = trial.fail_code or "PORTAL_EXIT_FAILED"

    def _world_box_corners(self, center, size, yaw):
        corners = []
        for local in box_corners(center, size, yaw):
            base = _local_point_to_base_link(local, self._scene_config)
            corners.append(xyz_base_link_to_world(self._scene_config, base))
        return corners

    def _latest_map_revision_or_refuse(self):
        """[S5.7 lock] Latest map revision, fail-closed (never 0)."""
        stats = self.call_srv(
            self._stats, GetCargoMapStats.Request(), timeout=5.0)
        if stats is None or not stats.success:
            return None, "MAP_STATUS_UNAVAILABLE"
        return int(stats.map_revision), ""

    def _execute_observation_route(self, name, camera_xyz, camera_q):
        """[ST09-I S5.7-4] Certified motion to an observation camera pose.

        The camera pose is the policy's declared intent; the producer
        owns the camera->TCP conversion, the MoveIt edge, and the
        certification. What executes is the returned COMPLETE certified
        joint_path (typed legs, executed in order) — never a pose spec.
        """
        if getattr(self, "_typed", None) is None:
            return False, "TYPED_UNAVAILABLE"
        from luggage_msgs.srv import PlanObservationRouteV2
        client = getattr(self, "_observation_route_client", None)
        if client is None:
            client = self.create_client(
                PlanObservationRouteV2,
                "/trajectory_selector/plan_observation_route",
                callback_group=self._group)
            self._observation_route_client = client
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "OBSERVATION_ROUTE_UNAVAILABLE"
        revision, refuse = self._latest_map_revision_or_refuse()
        if revision is None:
            return False, refuse
        request = PlanObservationRouteV2.Request()
        request.request_id = self._frame_request_id(name)
        request.scene_revision = str(self._frame_scene_revision or "")
        joints = (self._last_joint_state or {}).get("position") or []
        request.current_joint_state = [float(v) for v in list(joints)[:6]]
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x = float(camera_xyz[0])
        pose.position.y = float(camera_xyz[1])
        pose.position.z = float(camera_xyz[2])
        pose.orientation.x = float(camera_q[0])
        pose.orientation.y = float(camera_q[1])
        pose.orientation.z = float(camera_q[2])
        pose.orientation.w = float(camera_q[3])
        request.observation_targets = [pose]
        request.latest_map_revision = int(revision)
        request.purpose = str(name)
        result = self.call_srv(client, request, timeout=30.0)
        if result is None:
            return False, "OBSERVATION_ROUTE_UNAVAILABLE"
        if not result.success:
            return False, str(result.reason_code
                              or "OBSERVATION_ROUTE_REJECTED")
        executed = 0
        admission = {
            "route_digest": str(result.route_digest or ""),
            "certification_id": str(result.certification_id or ""),
            "admission_map_revision": int(
                result.validated_map_revision or 0),
        }
        digests = [str(d) for d in (result.leg_digests or [])]
        states = [str(v) for v in (result.leg_load_states or [])]
        for index, segment in enumerate(result.motion_segments or []):
            # [ST09-I S5.7-6] each leg claims ITS OWN certified scope
            # (observation routes certify under the unloaded index).
            leg_admission = dict(admission)
            if index < len(digests):
                leg_admission["leg_digest"] = digests[index]
                leg_admission["load_state"] = (
                    states[index] if index < len(states) else "unloaded")
            ok, message, _rec = self._typed.plan_motion(
                segment, self._args.plan_timeout,
                name="PlanMotionV2:%s" % segment.name,
                admission=leg_admission)
            executed += 1
            if not ok:
                return False, "OBSERVATION_LEG_FAILED:%s" % (
                    str(message or segment.name)[:120])
        if executed == 0:
            return False, "OBSERVATION_ROUTE_EMPTY"
        self._last_observation_certification = {
            "route_digest": str(result.route_digest or ""),
            "certification_id": str(result.certification_id or ""),
            "validated_map_revision": int(result.validated_map_revision or 0),
            "leg_digests": [str(d) for d in (result.leg_digests or [])],
        }
        return True, ""

    def _execute_view_candidates(
            self, dump, view, segment_name, cam_xyz, cam_q, target,
            suc_t, suc_q, center, size, yaw, eye_raises=None):
        """[ST09-I S5.7-4] ObservationPolicy ranks; certified routes move.

        The aim grid, target, and in-frame prediction live in the
        ObservationPolicy (``rank_observation_aims`` — at most two
        camera targets per request); each candidate's motion is a
        COMPLETE certified joint_path from PlanObservationRouteV2. The
        driver-side pose_target construction is deleted (the executor
        answers those with SEGMENT_GENERATION_REMOVED).
        """
        corners = self._world_box_corners(center, size, yaw)
        info = getattr(self, "_camera_info", None)
        camera_k = None
        image_wh = (0, 0)
        if info is not None:
            k = getattr(info, "k", None)
            if k is not None and len(k) >= 6:
                camera_k = [float(v) for v in list(k)[:9]]
            image_wh = (int(getattr(info, "width", 0) or 0),
                        int(getattr(info, "height", 0) or 0))
        kwargs = {}
        if eye_raises is not None:
            kwargs["eye_raises"] = eye_raises
        rows = rank_observation_aims(
            cam_xyz, cam_q, target, suc_t, suc_q, corners,
            camera_k=camera_k, image_wh=image_wh, **kwargs)
        attempts = []
        chosen = None
        for row in rows:
            record = {
                "view": view,
                "index": row["index"],
                "blend": row["blend"],
                "eye_raise_m": row["eye_raise_m"],
                "camera_xyz": row["camera_xyz"],
                "camera_q_xyzw": row["camera_q_xyzw"],
                "target_xyz": row["target_xyz"],
                "suction_xyz": row["suction_xyz"],
                "suction_q_xyzw": row["suction_q_xyzw"],
                "t_cam_suction": row["t_cam_suction"],
                "r_cam_suction_xyzw": row["r_cam_suction_xyzw"],
            }
            projected = row.get("projection")
            record["projection"] = {
                key: projected.get(key)
                for key in ("in_frame", "in_frame_fraction",
                            "behind_camera", "reason")
                if isinstance(projected, dict) and key in projected}
            if not row.get("in_frame"):
                record["skipped"] = True
                record["reason"] = "TARGET_NOT_IN_FRAME"
                attempts.append(record)
                continue
            ok, message = self._execute_observation_route(
                segment_name, row["camera_xyz"], row["camera_q_xyzw"])
            record.update({
                "ok": bool(ok),
                "message": str(message or ""),
                "certified_route": dict(
                    getattr(self, "_last_observation_certification", {})
                    or {}) if ok else None,
            })
            attempts.append(record)
            if ok:
                chosen = {
                    "index": row["index"],
                    "blend": row["blend"],
                    "eye_raise_m": row["eye_raise_m"],
                    "camera_xyz": row["camera_xyz"],
                    "suction_xyz": row["suction_xyz"],
                }
                break
        self._dump_json(dump, "%s_attempts.json" % view, attempts)
        return attempts, chosen

    def _exit_to_portal(self):
        """Post-place-view positioning through the certified goto service.

        [ST09-I S5.7] The shared ad-hoc cartesian portal escape is deleted
        (the executor answers driver-built cartesian with
        SEGMENT_GENERATION_REMOVED); the observation camera rides the
        NAMED portal pose through GoToRobotPoseV2 — complete certified
        joint legs, never a pose spec. The pose is a calibrated
        configuration asset (robot_poses yaml); a general exit from
        inside the container is the certified route's own retreat/return
        legs (EnsureCycleStartV2 recovery), never this call.

        [ST09-I S5.7-6 评审点 1] Success freezes a STRUCTURED exit
        proof — request/route/cert/scene identity, the final joint
        state, and the unloaded phase — the evidence chain _home_arm's
        EXIT_ROUTE_UNAVAILABLE exception adjudicates. Any drift (scene,
        joints moved, vacuum re-attached, new cycle start) invalidates
        it; it is never a cross-cycle boolean bypass.
        """
        ok, message = self._goto_named_pose_v2(
            "portal_observe_sim", timeout=self._args.goto_timeout)
        self._portal_exited = bool(ok)
        self._exit_yaw = ({"source": "named_portal_pose"}
                          if ok else self._exit_yaw)
        if ok:
            route = getattr(self, "_last_goto_route", None) or {}
            # [G0-1] only an ACTUALLY-certified exit route mints a
            # proof. already_there executes zero legs and promotes
            # nothing (_last_goto_route is None): it may only INHERIT
            # the still-valid original proof from the real exit — an
            # empty/already-there record is never frozen as a new one.
            certified = (
                str(route.get("route_digest") or "").strip()
                and str(route.get("certification_id") or "").strip())
            if certified:
                joints = (self._last_joint_state or {}).get(
                    "position") or []
                self._portal_exit_proof = {
                    "request_id": str(route.get("request_id", "")),
                    "route_digest": str(route.get("route_digest", "")),
                    "certification_id": str(
                        route.get("certification_id", "")),
                    "scene_revision": str(route.get("scene_revision", "")),
                    "final_joint_values": [
                        float(v) for v in list(joints)[:6]],
                    "attachment_state": "unloaded",
                    "proof_ns": int(self.get_clock().now().nanoseconds),
                }
        else:
            self._portal_exit_proof = None
        return bool(ok), str(
            message or ("portal goto %s" % ("ok" if ok else "failed")))

    def _execute_segment(self, segment, trial):
        ok, code, rec = super()._execute_segment(segment, trial)
        invalidated = (
            rec is not None
            and int(rec.get("moveit_error_code") or 0) in (-2, -4))
        if (not ok and invalidated
                and str(getattr(segment, "name", "")) in (
                    "traverse", "transit")):
            self.get_logger().warn(
                "retry %s after MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE"
                % segment.name)
            time.sleep(0.3)
            ok, code, rec = super()._execute_segment(segment, trial)
            if rec is not None:
                rec["retried_invalidated"] = True
        return ok, code, rec

    def _on_last_result(self, msg):
        try:
            self._last_result = json.loads(msg.data)
        except ValueError:
            pass

    def _yaw_abs(self, yaw):
        wrapped = (float(yaw) + math.pi) % (2.0 * math.pi) - math.pi
        return abs(wrapped)

    def _slot_from_candidate(self, candidate, pick_msg):
        if candidate.get("center_base_link"):
            base = candidate["center_base_link"]
            if candidate.get("yaw_base_link") is not None:
                yaw_base = float(candidate["yaw_base_link"])
            else:
                yaw_base = yaw_world_to_base_link(
                    self._scene_config, float(candidate.get("yaw") or 0.0))
        else:
            local = candidate.get("center_local") or candidate.get(
                "center_base") or [0.0, 0.0, 0.0]
            base = _local_point_to_base_link(local, self._scene_config)
            yaw_base = yaw_world_to_base_link(
                self._scene_config, float(candidate.get("yaw") or 0.0))
        slot = SlotSpec()
        slot.layer, slot.row, slot.col = 0, 0, 0
        slot.width = float(pick_msg.width)
        slot.depth = float(pick_msg.depth)
        slot.height = float(pick_msg.height)
        slot.place_pose.position.x = float(base[0])
        slot.place_pose.position.y = float(base[1])
        slot.place_pose.position.z = float(base[2])
        slot.place_pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(yaw_base * 0.5), w=math.cos(yaw_base * 0.5))
        return slot

    def _prefer_reachable_slot(self, fallback_slot, pick_msg, trial):
        """Prefer a yaw-0 floor candidate near the container origin.

        ComputePlacement's top score can be a far-wall cell whose transit IK
        fails (OCC-1 trial 0: x=1.03 m, error_code=-2). Near-center floor
        slots keep 3-box XY unique once sensor occupancy fills the origin.
        """
        dump = self._last_result or {}
        if not dump.get("candidates"):
            deadline = time.time() + 1.0
            while time.time() < deadline and not (
                    self._last_result or {}).get("candidates"):
                time.sleep(0.05)
            dump = self._last_result or {}
        occupied = occupied_sensor_aabb(self._surface_2d or {})
        feasible = [
            cand for cand in dump.get("candidates") or ()
            if cand.get("feasible")]
        ranked = []
        for cand in feasible:
            local = cand.get("center_local") or cand.get("center_base") or [
                0.0, 0.0, 0.0]
            inner = (self._surface_2d or {}).get("inner_size") or [0, 0, 1.5]
            inner_h = float(inner[2])
            peak = float(local[2]) + 0.5 * inner_h - 0.5 * float(pick_msg.height)
            fp = cand.get("footprint") or [
                float(pick_msg.width), float(pick_msg.depth)]
            half_l = 0.5 * float(fp[0])
            half_w = 0.5 * float(fp[1])
            cand_xy = (
                float(local[0]) - half_l, float(local[1]) - half_w,
                float(local[0]) + half_l, float(local[1]) + half_w,
            )
            overlaps = bool(
                occupied
                and cand_xy[0] < occupied[2] and cand_xy[2] > occupied[0]
                and cand_xy[1] < occupied[3] and cand_xy[3] > occupied[1])
            is_floor = peak <= 0.05
            # Floor-through (floor pose on the occupied patch) is last.
            # Stacking on that patch reuses the proven center traverse; a
            # lateral floor neighbor at hypot 0.37 failed cartesian 0.854
            # (2026-09-20_1240 trial 1).
            floor_through = 1 if (is_floor and overlaps) else 0
            prefer_stack = 0 if ((not is_floor) and overlaps) else 1
            ranked.append((
                floor_through,
                prefer_stack,
                0 if self._yaw_abs(cand.get("yaw") or 0.0) <= 0.20 else 1,
                math.hypot(float(local[0]), float(local[1])),
                cand,
            ))
        if not ranked:
            trial.extras["slot_pick"] = "compute_placement_default"
            return fallback_slot
        ranked.sort(key=lambda row: row[:4])
        chosen = ranked[0][4]
        local = chosen.get("center_local") or [0.0, 0.0, 0.0]
        dist = math.hypot(float(local[0]), float(local[1]))
        if dist > 0.35 and self._sensor_occupied_count(self._surface_2d or {}) < 1:
            slot, _meta = self._fixed_slot(pick_msg)
            trial.extras["slot_pick"] = {
                "source": "fixed_floor_center",
                "reason": "empty-map last_result nearest hypot %.3f > 0.35" % dist,
                "n_feasible": len(feasible),
            }
            return slot
        trial.extras["slot_pick"] = {
            "source": "last_result_near_origin",
            "center_local": local,
            "yaw": chosen.get("yaw"),
            "n_feasible": len(feasible),
        }
        return self._slot_from_candidate(chosen, pick_msg)

    def _place_slot_for(self, pick_msg, box_msg, trial):
        del box_msg
        if not bool(getattr(pick_msg, "height_valid", False)):
            trial.fail_code = trial.fail_code or "DETECT_FULL_GEOMETRY_REQUIRED"
            return None, None
        stats = self.call_srv(self._stats, GetCargoMapStats.Request(), timeout=5.0)
        request = ComputePlacement.Request()
        request.box = pick_msg
        request.geometry_hash = str(
            (stats.geometry_hash if stats else "")
            or (self._surface_2d or {}).get("geometry_hash") or "")
        placement = self.call_srv(self._compute, request, timeout=30.0)
        if placement is None or not placement.success:
            trial.fail_code = trial.fail_code or (
                getattr(placement, "reason_code", None) or "PLACEMENT_FAILED")
            trial.extras["compute_placement"] = (
                placement.message if placement else "timeout")
            return None, None
        trial.extras["compute_placement"] = placement.message
        slot = self._prefer_reachable_slot(placement.slot, pick_msg, trial)
        return slot, self._slot_meta(slot)

    def _slot_meta(self, slot):
        pos = slot.place_pose.position
        base_xyz = [pos.x, pos.y, pos.z]
        world_xyz = xyz_base_link_to_world(self._scene_config, base_xyz)
        yaw_base = 2.0 * math.atan2(
            slot.place_pose.orientation.z, slot.place_pose.orientation.w)
        yaw_world = yaw_base_link_to_world(self._scene_config, yaw_base)
        return {
            "planning_frame": "world",
            "pose_world": {"position": world_xyz, "yaw": yaw_world},
            "pose_base_link": {"position": base_xyz, "yaw": yaw_base},
            "source": "compute_placement",
        }

    def _wait_untracked(self, timeout=5.0, min_points=1):
        self._untracked["msg"] = None
        deadline = time.time() + timeout
        last = None
        last_dt = None
        while time.time() < deadline:
            msg = self._untracked.get("msg")
            if msg is not None:
                dt = abs(stamp_to_sec(msg.header.stamp) - self.ros_now_sec())
                n_pts = int(msg.width * msg.height)
                if dt <= 0.20:
                    last, last_dt = msg, dt
                    if n_pts >= int(min_points):
                        return msg, dt
            time.sleep(0.05)
        return last, last_dt

    def _wait_surface(self, revision, timeout=2.0):
        deadline = time.time() + timeout
        want = int(revision)
        last = dict(self._surface_2d or {})
        while time.time() < deadline:
            last = dict(self._surface_2d or {})
            have = last.get("map_revision")
            if have is not None and int(have) >= want:
                return last
            time.sleep(0.05)
        return last

    def _tf_at_stamp(self, frame_id, stamp):
        if not frame_id:
            return False, "empty frame_id"
        try:
            self._tf_buffer.lookup_transform(
                "container_link", str(frame_id),
                rclpy.time.Time(
                    seconds=int(stamp.sec), nanoseconds=int(stamp.nanosec)),
                timeout=rclpy.duration.Duration(seconds=0.05))
            return True, ""
        except Exception as exc:  # noqa: BLE001 - T1 evidence
            return False, str(exc)

    def _latest_buffer_msg(self, name):
        buf = (getattr(self, "_buffers", None) or {}).get(name)
        if buf is None:
            return None
        snap = buf.snapshot()
        if not snap:
            return None
        return snap[max(snap)]

    def _dump_view_bundle(self, dump, view, slot, slot_meta, pick_msg):
        """Raw frames, intrinsics, stamps, YOLO, and the projected footprint."""
        diag = self._dump_cargo_view(dump, view, slot_meta, pick_msg)
        info = getattr(self, "_camera_info", None)
        k = None if info is None else getattr(info, "k", None)
        if info is not None and k is not None and len(k) >= 6:
            self._dump_json(dump, "%s_camera_info.json" % view, {
                "frame_id": str(info.header.frame_id or ""),
                "width": int(info.width),
                "height": int(info.height),
                "k": [float(v) for v in k],
                "d": [float(v) for v in (info.d or [])],
                "stamp_sec": stamp_to_sec(info.header.stamp),
            })
            diag["files"]["camera_info"] = "%s_camera_info.json" % view
        depth_msg = self._latest_buffer_msg("depth")
        color_msg = self._latest_buffer_msg("color")
        mask_msg = self._latest_buffer_msg("mask")
        stamps = {
            "depth": stamp_to_sec(depth_msg.header.stamp) if depth_msg is not None else None,
            "color": stamp_to_sec(color_msg.header.stamp) if color_msg is not None else None,
            "mask": stamp_to_sec(mask_msg.header.stamp) if mask_msg is not None else None,
        }
        diag["acquisition_stamp_sec"] = stamps
        if stamps["depth"] is not None and stamps["color"] is not None:
            diag["color_depth_dt_sec"] = abs(stamps["color"] - stamps["depth"])
        if stamps["depth"] is not None and stamps["mask"] is not None:
            diag["mask_depth_dt_sec"] = abs(stamps["mask"] - stamps["depth"])
        millimetres = (
            adapters.depth_array_from_msg(depth_msg) if depth_msg is not None else None)
        if millimetres is not None:
            raw = np.array(millimetres)
            npy = os.path.join(dump, "%s_depth_raw.npy" % view)
            np.save(npy, raw)
            diag["files"]["depth_raw"] = os.path.basename(npy)
        mask_img = None
        try:
            from pick_retreat_eval_driver import _decode_dump_image
            mask_img = _decode_dump_image(mask_msg)
        except Exception:  # noqa: BLE001 - diagnostics only
            mask_img = None
        if mask_img is not None:
            grey = mask_img[:, :, 0] if getattr(mask_img, "ndim", 0) == 3 else mask_img
            diag["mask_pixels"] = int(np.count_nonzero(grey))
            diag["semantic_mask_empty"] = diag["mask_pixels"] == 0
            if millimetres is not None and grey.shape == np.asarray(millimetres).shape:
                valid = np.asarray(millimetres) > 0
                diag["mask_valid_depth"] = int(np.count_nonzero(
                    valid & (np.asarray(grey) > 0)))
        else:
            diag["semantic_mask_empty"] = True
            diag["mask_pixels"] = 0
        yolo = diag.get("yolo") or []
        diag["semantic_mask_empty"] = bool(
            diag.get("semantic_mask_empty") or not yolo)
        diag["yolo"] = [
            {
                "prompt": row.get("prompt"),
                "confidence": row.get("confidence"),
                "bbox": list(row.get("bbox") or []),
            }
            for row in yolo]
        center, size, yaw = self._placed_box_container(slot, pick_msg)
        projected = {"in_frame_fraction": 0.0, "behind_camera": False, "pixels": []}
        try:
            cam_xyz, cam_q = self._lookup_xyz_quat(
                "world", "camera_depth_optical_frame")
            from luggage_gazebo.cargo_view_sequence import (
                corners_in_camera,
            )
            corners = corners_in_camera(
                self._world_box_corners(center, size, yaw), cam_xyz, cam_q)
            if k is not None and len(k) >= 6 and getattr(info, "width", 0):
                projected = project_box_corners(
                    corners, float(k[0]), float(k[4]), float(k[2]), float(k[5]),
                    int(info.width), int(info.height))
        except Exception as exc:  # noqa: BLE001 - view evidence
            diag.setdefault("errors", []).append("project: %s" % exc)
        diag["in_frame_fraction"] = projected.get("in_frame_fraction")
        diag["behind_camera"] = projected.get("behind_camera")
        diag["projected_corners"] = projected.get("pixels")
        polygon = [p for p in (projected.get("pixels") or []) if p is not None]
        if millimetres is not None:
            stats = region_depth_stats(np.array(millimetres), polygon)
        else:
            stats = {"n_valid": 0, "n_invalid": 0, "n_zero": 0,
                     "min_m": None, "median_m": None, "max_m": None}
        diag["depth_region"] = stats
        diag["n_valid_depth_in_footprint"] = stats["n_valid"]
        diag["depth_range_m"] = {
            "min": stats["min_m"], "median": stats["median_m"], "max": stats["max_m"]}
        diag["rim_occlusion_fraction"] = None
        distance = diag.get("camera_to_box_m")
        if millimetres is not None and polygon and distance is not None:
            image = np.array(millimetres)
            mask = _polygon_mask(image.shape[0], image.shape[1], polygon)
            if mask.shape == image.shape and bool(mask.any()):
                ranges = np.asarray(image[mask], dtype=np.float64)
                ranges = ranges[np.isfinite(ranges) & (ranges > 0.0)] * 0.001
                diag["rim_occlusion_fraction"] = rim_occlusion_fraction(
                    ranges, distance, RIM_MARGIN_M)
                diag["rim_margin_m"] = RIM_MARGIN_M
        self._dump_json(dump, "view_diag_%s.json" % view, diag)
        return diag

    def _close_view(self, t1, dump, view, slot, pick_msg, gate, surface, view_diag):
        """Footprint hits and one empty-view reason. Does not treat empty as success."""
        center, size, yaw = self._placed_box_container(slot, pick_msg)
        hits = footprint_sensor_hits(surface or {}, center, size, yaw)
        t1["footprint_hits"] = int(hits)
        t1["integrated"] = "resulting_map_revision" in t1
        gate_diag = (gate or {}).get("diag") or {}
        message = str(t1.get("message") or "")
        kept_match = re.search(r"n_kept=(\d+)", message)
        finite_match = re.search(r"n_finite=(\d+)", message)
        merged = {
            "in_frame_fraction": (view_diag or {}).get("in_frame_fraction"),
            "behind_camera": (view_diag or {}).get("behind_camera"),
            "tf_stamped_ok": gate_diag.get("tf_stamped_ok"),
            "tf_used_latest": gate_diag.get("tf_used_latest"),
            "tf_fallback_age_sec": gate_diag.get("tf_fallback_age_sec"),
            "n_valid_depth_in_footprint": (view_diag or {}).get(
                "n_valid_depth_in_footprint"),
            "n_kept": gate_diag.get("n_kept") or 0,
        }
        if hits < 1:
            t1["empty_reason"] = classify_empty_view(merged)
        else:
            t1["empty_reason"] = None
        diag = dict(view_diag or {})
        diag.update({
            "expected_map_revision": t1.get("expected_map_revision"),
            "resulting_map_revision": t1.get("resulting_map_revision"),
            "footprint_hits": t1["footprint_hits"],
            "n_sensor_occupied": t1.get("n_sensor_occupied"),
            "empty_reason": t1.get("empty_reason"),
            "n_depth": gate_diag.get("n_depth"),
            "n_kept": gate_diag.get("n_kept"),
            "n_published": gate_diag.get("n_published"),
            "mapper_n_finite": int(finite_match.group(1)) if finite_match else None,
            "mapper_n_kept": int(kept_match.group(1)) if kept_match else None,
            "tf_stamped_ok": gate_diag.get("tf_stamped_ok"),
            "tf_used_latest": gate_diag.get("tf_used_latest"),
            "tf_fallback_age_sec": gate_diag.get("tf_fallback_age_sec"),
            "tf_stamp_error": gate_diag.get("tf_stamp_error"),
            "integrate_success": t1.get("success"),
            "reason_code": t1.get("reason_code") or t1.get("drop_reason"),
            "semantic_mask_empty": (view_diag or {}).get("semantic_mask_empty"),
        })
        if diag.get("camera_to_box_m") is not None and diag.get("depth_region"):
            diag["rim_margin_m"] = RIM_MARGIN_M
        self._dump_json(dump, "view_diag_%s.json" % view, diag)
        t1["view_diag"] = diag
        return t1

    def _dump_cargo_view(self, dump, view, slot_meta, pick_msg):
        """Color, depth, mask, optical-axis angle, and YOLO count for one view."""
        from luggage_perception.eval.detection_gate_sampling import write_png
        from pick_retreat_eval_driver import _decode_dump_image

        diag = {"view": view, "files": {}, "errors": []}
        color = _decode_dump_image(self._latest_buffer_msg("color"))
        mask = _decode_dump_image(self._latest_buffer_msg("mask"))
        depth_msg = self._latest_buffer_msg("depth")
        depth = adapters.image_array_from_msg(depth_msg) if depth_msg is not None else None
        try:
            if color is not None:
                path = os.path.join(dump, "%s_color.png" % view)
                write_png(path, color)
                diag["files"]["color"] = os.path.basename(path)
            else:
                diag["errors"].append("no color frame")
            if mask is not None:
                path = os.path.join(dump, "%s_mask.png" % view)
                grey = mask[:, :, 0] if getattr(mask, "ndim", 0) == 3 else mask
                write_png(path, grey)
                diag["files"]["mask"] = os.path.basename(path)
            else:
                diag["errors"].append("no mask frame")
            if depth is not None:
                npy = os.path.join(dump, "%s_depth.npy" % view)
                np.save(npy, np.asarray(depth))
                diag["files"]["depth"] = os.path.basename(npy)
                finite = np.asarray(depth, dtype=np.float64)
                finite = finite[np.isfinite(finite)]
                if finite.size:
                    lo, hi = float(finite.min()), float(finite.max())
                    span = max(hi - lo, 1e-6)
                    preview = np.clip((np.asarray(depth, dtype=np.float64) - lo) / span * 255.0, 0, 255)
                    preview = np.nan_to_num(preview, nan=0.0).astype(np.uint8)
                    if preview.ndim == 3:
                        preview = preview[:, :, 0]
                    png = os.path.join(dump, "%s_depth.png" % view)
                    write_png(png, preview)
                    diag["files"]["depth_png"] = os.path.basename(png)
            else:
                diag["errors"].append("no depth frame")
        except (OSError, ValueError) as exc:
            diag["errors"].append(str(exc))
        slot_position = list(
            (slot_meta or {}).get("pose_world", {}).get("position") or [])
        target = (observation_target(slot_position, pick_msg.height)
                  if len(slot_position) >= 3 else [])
        if target:
            try:
                cam_xyz, cam_q = self._lookup_xyz_quat(
                    "world", "camera_depth_optical_frame")
                angle, distance = optical_axis_angle_rad(cam_xyz, cam_q, target)
                diag["optical_axis_angle_rad"] = angle
                diag["camera_to_box_m"] = distance
                diag["camera_xyz"] = [float(v) for v in cam_xyz]
                diag["target_xyz"] = [float(v) for v in target]
            except Exception as exc:  # noqa: BLE001 - view evidence
                diag["errors"].append("axis: %s" % exc)
        frames = getattr(self, "_detection_frames", None)
        frame = None
        if frames is not None:
            frame, _dt = frames.nearest(self.ros_now_sec())
        yolo = list(getattr(frame, "yolo", None) or []) if frame is not None else []
        diag["yolo_count"] = len(yolo)
        diag["yolo"] = [
            {"prompt": str(getattr(box, "prompt", "")),
             "confidence": float(getattr(box, "confidence", 0.0)),
             "bbox": [int(v) for v in (getattr(box, "bbox", None) or [])]}
            for box in yolo]
        self._dump_json(dump, "view_diag_%s.json" % view, diag)
        return diag

    def _dump_cloud_ply(self, dump, cloud, name="untracked_container.ply"):
        if cloud is None:
            return ""
        pts = adapters.cloud_points_from_msg(cloud)
        if pts is None or pts.shape[0] == 0:
            return ""
        if pts.shape[0] > 20000:
            pts = pts[::max(1, pts.shape[0] // 20000)]
        path = os.path.join(dump, name)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write("ply\nformat ascii 1.0\n")
            handle.write("element vertex %d\n" % int(pts.shape[0]))
            handle.write("property float x\nproperty float y\nproperty float z\n")
            handle.write("end_header\n")
            for row in pts:
                handle.write("%.4f %.4f %.4f\n" % (
                    float(row[0]), float(row[1]), float(row[2])))
        return path

    def _sensor_occupied_count(self, surface):
        if not surface:
            return 0
        state = surface.get("state") or []
        conf = surface.get("confidence") or []
        n = 0
        for ix in range(int(surface.get("nx") or 0)):
            for iy in range(int(surface.get("ny") or 0)):
                if (state[ix][iy] == "occupied"
                        and conf[ix][iy] == "sensor"):
                    n += 1
        return n

    def _placed_box_container(self, slot, pick_msg):
        """Resting box center, size, and yaw in container_link.

        The slot pose is the planned box center. Size is the suitcase that
        was placed. This volume only selects depth points.
        """
        pos = slot.place_pose.position
        base = [float(pos.x), float(pos.y), float(pos.z)]
        center = _point_in_container_link(base, self._scene_config)
        quat = slot.place_pose.orientation
        yaw_base = 2.0 * math.atan2(float(quat.z), float(quat.w))
        _origin, rpy = container_in_base_link(self._scene_config)
        yaw = yaw_base - float(rpy[2])
        physics = getattr(self, "_release_physics", None) or {}
        command_size = list(physics.get("command_size") or [])
        if len(command_size) == 3:
            size = [float(v) for v in command_size]
        else:
            size = [
                float(pick_msg.width), float(pick_msg.depth),
                float(pick_msg.height)]
        return center, size, yaw

    def _placed_box_depth_cloud(self, slot, pick_msg):
        """Publish depth points that fall inside the placed box.

        Points stay in the depth optical frame so the mapper raycasts from
        the camera. The stamp is the depth stamp plus 1 us, which keeps this
        cloud distinct from an empty semantic frame of the same acquisition.
        Returns ``{"msg": None, "diag": ...}`` when the crop is empty.
        """
        diag = {
            "source": "placed_box_depth",
            "margin_m": BOX_DEPTH_MARGIN_M,
            "min_points": BOX_DEPTH_MIN_POINTS,
        }
        info = getattr(self, "_camera_info", None)
        k = None if info is None else getattr(info, "k", None)
        if k is None or len(k) < 6:
            diag["reason"] = "no_camera_info"
            return {"msg": None, "diag": diag}
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            diag["reason"] = "bad_intrinsics"
            return {"msg": None, "diag": diag}
        depth_msg = self._latest_buffer_msg("depth")
        if depth_msg is None:
            diag["reason"] = "no_depth"
            return {"msg": None, "diag": diag}
        millimetres = adapters.depth_array_from_msg(depth_msg)
        if millimetres is None:
            diag["reason"] = "depth_decode_fail"
            return {"msg": None, "diag": diag}
        depth_m = np.asarray(millimetres, dtype=np.float64) * 0.001
        camera = depth_to_camera_xyz(
            depth_m, fx, fy, cx, cy, max_depth=4.0, stride=4)
        diag["n_depth"] = int(camera.shape[0])
        frame_id = str(depth_msg.header.frame_id or "camera_depth_optical_frame")
        diag["tf_stamped_ok"] = False
        diag["tf_used_latest"] = False
        diag["tf_fallback_age_sec"] = None
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "container_link", frame_id, depth_msg.header.stamp,
                rclpy.duration.Duration(seconds=0.2))
            diag["tf_stamped_ok"] = True
        except Exception as exc:  # noqa: BLE001 - fall back to the latest TF
            diag["tf_stamp_error"] = str(exc)
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    "container_link", frame_id, rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5))
                diag["tf_used_latest"] = True
                diag["tf_fallback_age_sec"] = abs(
                    stamp_to_sec(depth_msg.header.stamp) - self.ros_now_sec())
            except Exception as latest_exc:  # noqa: BLE001 - T1 evidence
                diag["reason"] = "tf:%s" % latest_exc
                return {"msg": None, "diag": diag}
        translation = tf_msg.transform.translation
        rotation = tf_msg.transform.rotation
        rot = rotation_from_xyzw(
            rotation.x, rotation.y, rotation.z, rotation.w)
        trans = (float(translation.x), float(translation.y), float(translation.z))
        container = transform_points(camera, rot, trans)
        diag["n_container"] = int(getattr(container, "shape", (0,))[0])
        center, size, yaw = self._placed_box_container(slot, pick_msg)
        diag["center_container"] = [float(v) for v in center]
        diag["size_wdh"] = [float(v) for v in size]
        diag["yaw_container"] = float(yaw)
        mask = box_point_mask(
            container, center, size, yaw=yaw, margin=BOX_DEPTH_MARGIN_M)
        kept_camera = camera[mask] if camera.shape[0] else camera
        diag["n_kept"] = int(kept_camera.shape[0])
        if kept_camera.shape[0] < BOX_DEPTH_MIN_POINTS:
            diag["reason"] = "too_few_box_points"
            return {"msg": None, "diag": diag, "container_points": container[mask]}
        if kept_camera.shape[0] > 4000:
            step = int(math.ceil(kept_camera.shape[0] / 4000.0))
            kept_camera = kept_camera[::step]
            diag["n_published"] = int(kept_camera.shape[0])
        stamp = depth_msg.header.stamp
        nanosec = int(stamp.nanosec) + 1000
        sec = int(stamp.sec)
        if nanosec >= 1000000000:
            sec += 1
            nanosec -= 1000000000
        from builtin_interfaces.msg import Time
        shifted = Time(sec=sec, nanosec=nanosec)
        msg = adapters.cloud_msg_from_points(kept_camera, shifted, frame_id)
        self._geometry_cloud_pub.publish(msg)
        diag["reason"] = "published"
        diag["frame_id"] = frame_id
        diag["stamp_sec"] = stamp_to_sec(shifted)
        return {"msg": msg, "diag": diag}

    def _integrate_after_place(
            self, trial, slot, slot_meta, pick_msg, view="lift", record=True,
            expected_revision=None):
        dump = os.path.join(
            self._args.dump_dir, "box_%02d_integrate" % trial.index)
        os.makedirs(dump, exist_ok=True)
        view_diag = self._dump_view_bundle(dump, view, slot, slot_meta, pick_msg)
        gate = self._placed_box_depth_cloud(slot, pick_msg)
        self._dump_json(dump, "geometry_gate_%s.json" % view, gate.get("diag") or {})
        stats = self.call_srv(self._stats, GetCargoMapStats.Request(), timeout=5.0)
        if gate.get("msg") is not None:
            time.sleep(0.15)
            cloud = gate["msg"]
            freshness = abs(
                stamp_to_sec(cloud.header.stamp) - self.ros_now_sec())
        else:
            cloud, freshness = self._wait_untracked(timeout=5.0, min_points=100)
        t1 = {
            "view_request_id": "occ1-%d-%s" % (trial.index, view),
            "view": view,
            "cloud_n": int(cloud.width * cloud.height) if cloud else 0,
            "cloud_frame": str(cloud.header.frame_id) if cloud else "",
            "cloud_stamp": stamp_to_sec(cloud.header.stamp) if cloud else None,
            "freshness_sec": freshness,
            "expected_map_revision": (
                int(expected_revision) if expected_revision is not None
                else (int(stats.map_revision) if stats else -1)),
            "geometry_hash": str(stats.geometry_hash) if stats else "",
            "cargo_view_aim": trial.extras.get("cargo_view_aim"),
            "view_diag": view_diag,
            "cloud_source": (
                "placed_box_depth" if gate.get("msg") is not None
                else "semantic_untracked"),
            "geometry_gate": gate.get("diag") or {},
        }
        try:
            filt = json.loads(self._filter_stats.get("payload") or "{}")
        except ValueError:
            filt = {}
        t1["untracked_publish_count"] = filt.get("untracked_publish_count")
        t1["untracked_point_count"] = filt.get("untracked_point_count")
        t1["filter_source"] = filt.get("source")
        trace_name = "t1_trace_%s.json" % view
        if cloud is None or freshness is None:
            t1["drop_reason"] = "VIEW_EMPTY"
            t1["integrated"] = False
            self._close_view(
                t1, dump, view, slot, pick_msg, gate, None, view_diag)
            self._dump_cloud_ply(dump, cloud, name="untracked_%s.ply" % view)
            self._dump_json(dump, trace_name, t1)
            if record:
                trial.fail_code = trial.fail_code or (
                    t1.get("empty_reason") or "VIEW_EMPTY")
                self._dump_json(dump, "t1_trace.json", t1)
                self._t1.append(t1)
            return t1
        request = IntegrateCargoView.Request()
        request.schema_version = SCHEMA_VERSION
        request.view_request_id = t1["view_request_id"]
        request.candidate_id = str(trial.catalog_id or trial.index)
        request.source_acquisition_stamp = cloud.header.stamp
        request.expected_map_revision = int(t1["expected_map_revision"])
        request.geometry_hash = str(stats.geometry_hash) if stats else ""
        request.settled = True
        request.frame_count = 1
        self._dump_json(dump, "integrate_request_%s.json" % view, {
            "view_request_id": request.view_request_id,
            "expected_map_revision": request.expected_map_revision,
            "geometry_hash": request.geometry_hash,
            "settled": True,
            "stamp_sec": stamp_to_sec(request.source_acquisition_stamp),
        })
        pre_surface = dict(self._surface_2d or {})
        self._dump_json(dump, "surface_2d_pre_%s.json" % view, pre_surface)
        tf_ok, tf_err = self._tf_at_stamp(cloud.header.frame_id, cloud.header.stamp)
        t1["tf_target"] = "container_link"
        t1["tf_source"] = str(cloud.header.frame_id)
        t1["tf_ok"] = bool(tf_ok)
        t1["tf_error"] = tf_err
        t0 = time.monotonic()
        resp = self.call_srv(self._integrate, request, timeout=10.0)
        latency = time.monotonic() - t0
        if resp is None:
            t1["drop_reason"] = "INTEGRATE_TIMEOUT"
            t1["integrated"] = False
            self._close_view(
                t1, dump, view, slot, pick_msg, gate, None, view_diag)
            self._dump_cloud_ply(dump, cloud, name="untracked_%s.ply" % view)
            self._dump_json(dump, trace_name, t1)
            if record:
                trial.fail_code = trial.fail_code or "INTEGRATE_TIMEOUT"
                self._dump_json(dump, "t1_trace.json", t1)
                self._t1.append(t1)
            return t1
        t1.update({
            "success": bool(resp.success),
            "reason_code": str(resp.reason_code or ""),
            "message": str(resp.message or ""),
            "resulting_map_revision": int(resp.resulting_map_revision),
            "latency_sec": latency,
        })
        self._dump_json(dump, "integrate_response_%s.json" % view, {
            "success": t1["success"],
            "reason_code": t1["reason_code"],
            "message": t1["message"],
            "resulting_map_revision": t1["resulting_map_revision"],
            "latency_sec": latency,
            "unknown_ratio": float(resp.unknown_ratio),
            "occupancy_ratio": float(resp.occupancy_ratio),
        })
        surface = self._wait_surface(resp.resulting_map_revision)
        self._dump_json(dump, "surface_2d_%s.json" % view, surface)
        ign = self._gz_box_pose()
        aabb = self._gt_aabb(ign, pick_msg)
        coverage, hit, total = (0.0, 0, 0)
        if aabb is not None and surface:
            coverage, hit, total = footprint_sensor_coverage(surface, aabb)
        physics = getattr(self, "_release_physics", None) or {}
        command_size = list(physics.get("command_size") or [])
        if len(command_size) == 3:
            box_height = float(command_size[2])
        else:
            box_height = float(pick_msg.height)
        occ_height = (
            footprint_sensor_height(surface, aabb)
            if aabb is not None and surface else None)
        resolution = float(
            (surface or {}).get("resolution")
            or (surface or {}).get("resolution_m")
            or 0.05)
        reference = {
            "ign_pose": ign,
            "aabb_xy_container": aabb,
            "coverage": coverage,
            "hit_cells": hit,
            "total_cells": total,
            "size_wdh": [
                float(pick_msg.width), float(pick_msg.depth),
                float(pick_msg.height)],
            "box_height_m": box_height,
            "occupancy_height_m": occ_height,
        }
        self._dump_json(dump, "reference_%s.json" % view, reference)
        if aabb is not None:
            self._gt_aabbs = [
                row for row in self._gt_aabbs if row.get("index") != trial.index]
            self._gt_aabbs.append({
                "index": trial.index, "aabb": aabb, "coverage": coverage})
        patch = occupied_sensor_aabb(surface) if surface else None
        through, solver = floor_through_count(
            surface, BOX_SYNTHETIC, patch) if surface else (0, {})
        self._dump_json(dump, "solver_replay_%s.json" % view, solver)
        t1["coverage"] = coverage
        t1["floor_through"] = through
        t1["n_sensor_occupied"] = self._sensor_occupied_count(surface)
        t1["hit_cells"] = int(hit)
        t1["total_cells"] = int(total)
        t1["occupancy_height_m"] = occ_height
        t1["box_height_m"] = box_height
        t1["occupancy_height_resolution_m"] = resolution
        t1["occupancy_height_ok"] = occupancy_height_matches(
            occ_height, box_height, resolution)
        self._close_view(
            t1, dump, view, slot, pick_msg, gate, surface, view_diag)
        if (not resp.success) or t1["n_sensor_occupied"] < 1:
            self._dump_cloud_ply(dump, cloud, name="untracked_%s.ply" % view)
        self._dump_json(dump, trace_name, t1)
        if record:
            self._dump_json(dump, "t1_trace.json", t1)
            self._dump_json(dump, "integrate_request.json", {
                "view_request_id": request.view_request_id,
                "expected_map_revision": request.expected_map_revision,
                "geometry_hash": request.geometry_hash,
                "settled": True,
                "stamp_sec": stamp_to_sec(request.source_acquisition_stamp),
            })
            self._dump_json(dump, "surface_2d.json", surface)
            self._dump_json(dump, "reference.json", reference)
            self._dump_json(dump, "solver_replay.json", solver)
            self._t1.append(t1)
        trial.extras["integrate"] = t1
        trial.extras["coverage"] = coverage
        if not resp.success:
            if record:
                trial.fail_code = trial.fail_code or (
                    t1.get("empty_reason") or resp.reason_code
                    or "INTEGRATE_FAILED")
            return t1
        trial.extras["integrate_ok"] = True
        if not record:
            return t1
        if coverage >= COVERAGE_MIN and not through:
            if trial.fail_code in ("COVERAGE_LOW", "FLOOR_THROUGH"):
                trial.fail_code = None
        elif coverage < COVERAGE_MIN:
            trial.fail_code = trial.fail_code or "COVERAGE_LOW"
        elif through:
            trial.fail_code = trial.fail_code or "FLOOR_THROUGH"
        if (coverage >= COVERAGE_MIN and not through
                and occ_height is not None
                and not t1["occupancy_height_ok"]):
            trial.fail_code = trial.fail_code or "OCC_HEIGHT"
        if int(t1.get("footprint_hits") or 0) < 1:
            trial.fail_code = trial.fail_code or (
                t1.get("empty_reason") or "VIEW_EMPTY")
        return t1

    def _gt_aabb(self, ign_pose, pick_msg):
        if not ign_pose or len(ign_pose) < 3:
            return None
        world_xyz = [float(ign_pose[0]), float(ign_pose[1]), float(ign_pose[2])]
        base = xyz_world_to_base_link(self._scene_config, world_xyz)
        local = _point_in_container_link(base, self._scene_config)
        half_w = 0.5 * float(pick_msg.width)
        half_d = 0.5 * float(pick_msg.depth)
        return (
            local[0] - half_w, local[1] - half_d,
            local[0] + half_w, local[1] + half_d,
        )

    def _dump_json(self, folder, name, payload):
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, name)
        with open(path, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        return path

    def clock_publishers(self):
        try:
            infos = self.get_publishers_info_by_topic("/clock")
            return len(infos)
        except Exception:  # noqa: BLE001
            return -1

    def accumulation(self):
        surface = self._surface_2d or {}
        revision = surface.get("map_revision")
        rows = []
        ok = True
        for item in self._gt_aabbs:
            cov, hit, total = footprint_sensor_coverage(surface, item["aabb"])
            rows.append({
                "index": item["index"], "coverage": cov,
                "hit_cells": hit, "total_cells": total})
            if hit < 1:
                ok = False
        return {
            "ok": ok and len(rows) >= 3,
            "map_revision": revision,
            "boxes": rows,
        }


def parse_args(argv):
    args = parse_place_args(argv)
    args.out = args.out or DEFAULT_OUT
    return args


def _write_suite(out, records, driver, extra):
    accumulation = driver.accumulation()
    t1 = list(driver._t1)
    hold = [
        row for row in t1
        if "hold_track" in str(row.get("reason_code") or "").lower()
        or row.get("reason_code") == "STAMP_STALE" and row.get("success")
    ]
    dumps = driver._args.dump_dir
    complete = True
    required_names = ("t1_trace.json", "integrate_request.json", "surface_2d.json",
                      "reference.json", "solver_replay.json")
    for rec in records:
        folder = os.path.join(dumps, "box_%02d_integrate" % rec.index)
        if rec.fail_code in ("VIEW_EMPTY", "INTEGRATE_TIMEOUT"):
            complete = complete and os.path.isfile(os.path.join(
                folder, "t1_trace.json"))
            continue
        for name in required_names:
            if not os.path.isfile(os.path.join(folder, name)):
                complete = False
    payload = {
        "n": len(records),
        "place_ok": sum(1 for r in records if place_ok(r)),
        "integrate_ok": sum(1 for row in t1 if row.get("success")),
        "coverage_min": COVERAGE_MIN,
        "coverage": [row.get("coverage") for row in t1],
        "n_sensor_occupied": [row.get("n_sensor_occupied") for row in t1],
        "accumulation": accumulation,
        "clock_publishers": driver.clock_publishers(),
        "hold_track_accepts": len(hold),
        "floor_through": [row.get("floor_through") for row in t1],
        "fail_codes": [r.fail_code for r in records],
        "t1": t1,
        "git": extra,
        "capture_complete": complete,
        "replay_possible": complete,
    }
    path = os.path.join(out, "suite.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return payload


def _write_t0(out, extra, args, records, suite, started_at):
    payload = {
        "case_ids": [
            {"index": r.index, "catalog_id": r.catalog_id, "fail_code": r.fail_code}
            for r in records],
        "commit": extra.get("commit"),
        "dirty": extra.get("dirty"),
        "porcelain": extra.get("porcelain"),
        "n": args.n,
        "launch": (
            "ros2 launch luggage_gazebo sim_world.launch.py gui:=false "
            "use_rviz:=false use_semantic:=true use_motion:=true "
            "use_vacuum:=true use_cargo_map:=true use_packing:=true "
            "visual_kind:=mesh size_mode:=catalog "
            "sequence_ids:=carryon,standard,large "
            "observe_pose_name:=pickup_observe "
            "semantic_require_backend:=bbox_fill"),
        "driver": "cargo_map_integrate_eval_driver.py",
        "ros_domain_id": extra.get("ros_domain_id"),
        "started_at": started_at,
        "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "clock_publishers": suite.get("clock_publishers"),
        "suite": {
            "integrate_ok": suite.get("integrate_ok"),
            "accumulation": suite.get("accumulation"),
            "coverage": suite.get("coverage"),
            "capture_complete": suite.get("capture_complete"),
            "replay_possible": suite.get("replay_possible"),
        },
        "dump_matrix": {
            "T0": "t0_manifest.json",
            "T1": "dumps/box_NN_integrate/{t1_trace,integrate_request,integrate_response,surface_2d,reference,solver_replay}.json",
            "T2": "untracked_container.ply + surface_2d_pre.json on VIEW_EMPTY/no sensor cells",
            "GT": "reference.json (ign pose, score-only AABB; not a mapper input)",
        },
    }
    path = os.path.join(out, "t0_manifest.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    return path


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    os.makedirs(args.out, exist_ok=True)
    args.dump_dir = args.dump_dir or os.path.join(args.out, "dumps")
    os.makedirs(args.dump_dir, exist_ok=True)
    jsonl = os.path.join(args.out, "trials.jsonl")
    with open(jsonl, "w", encoding="utf-8"):
        pass

    rclpy.init()
    driver = CargoMapIntegrateDriver(args)
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
            err = driver.wait_graph(timeout=90.0)
            if err:
                print("graph: %s" % err)
                return 2
        print("graph ok; OCC-1 n=%d" % args.n, flush=True)
        time.sleep(8.0)
        for index in range(args.n):
            if stop["flag"] or not rclpy.ok():
                break
            rec = driver.run_trial(index)
            records.append(rec)
            with open(jsonl, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(trial_to_dict(rec), sort_keys=True) + "\n")
            print(json.dumps({
                "index": rec.index, "fail_code": rec.fail_code,
                "coverage": rec.extras.get("coverage"),
                "integrate": (rec.extras.get("integrate") or {}).get("success"),
            }, sort_keys=True), flush=True)
            if rec.fail_code and args.on_place_fail == "stop":
                if rec.fail_code not in RETURN_FAIL_CODES + ("COVERAGE_LOW",):
                    break
        ws = os.path.normpath(os.path.join(_SCRIPTS, "..", "..", ".."))
        extra.update({
            "date": time.strftime("%Y-%m-%d"),
            "n_requested": args.n,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
            "dirty": subprocess.check_output(
                ["git", "status", "--porcelain"], cwd=ws,
                text=True).count("\n"),
        })
        suite = _write_suite(args.out, records, driver, extra)
        _write_t0(args.out, extra, args, records, suite, started_at)
        summary = summarize(records)
        with open(os.path.join(args.out, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        print(json.dumps({
            "integrate_ok": suite["integrate_ok"],
            "accumulation": suite["accumulation"]["ok"],
            "clock_publishers": suite["clock_publishers"],
            "capture_complete": suite["capture_complete"],
        }, sort_keys=True), flush=True)
        if (suite["integrate_ok"] != args.n
                or not suite["accumulation"]["ok"]
                or suite["clock_publishers"] not in (1, -1)
                or suite["hold_track_accepts"]
                or not suite["capture_complete"]):
            return 1
        if any(r.fail_code and r.fail_code not in RETURN_FAIL_CODES
               for r in records):
            return 1
        return 0
    finally:
        executor.remove_node(driver)
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
