#!/usr/bin/env python3
"""OCC-1 cargo-map integrate eval.

Place-smoke n=3 (keep placed, no cargo-map AddPlacedBox). After each
PLACE_RETREAT, Cartesian-lift the wrist (camera-down) so the D435 is
outside min-range and the box top fills the FOV, then call
IntegrateCargoView on the untracked semantic cloud. Score sensor
coverage against ign AABB (score-only). Accumulation is the RS-1 claim:
after trial 3 every GT footprint still has a sensor cell.
"""

from __future__ import division

import argparse
import json
import math
import os
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
    xyz_base_link_to_world,
    xyz_world_to_base_link,
    yaw_base_link_to_world,
    yaw_world_to_base_link,
)
from luggage_msgs.srv import ComputePlacement, GetCargoMapStats, IntegrateCargoView
from luggage_perception import ros_message_adapters as adapters
from luggage_perception.cargo_instance_tracker import rotation_from_xyzw
from luggage_perception.cargo_view_integration import (
    SCHEMA_VERSION,
    floor_through_count,
    footprint_sensor_coverage,
    occupied_sensor_aabb,
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
            lift_ok, lift_msg, lift_xyz = self._lift_for_cargo_view()
            trial.extras["cargo_view_lift"] = {
                "ok": bool(lift_ok),
                "message": lift_msg,
                "suction_xyz": lift_xyz,
                "delta_z": VIEW_LIFT_Z,
            }
            time.sleep(0.4)
            t_lift = self._integrate_after_place(
                trial, slot, slot_meta, pick_msg, view="lift", record=False)
            # Portal look-at reaches IK (2026-09-20_1350) but bbox_fill
            # publishes 0 cargo points from that viewpoint. Coverage has
            # to come from the lift cloud (optionally with obstacle).
            final = t_lift
            if final:
                final = dict(final)
                final["view_lift"] = t_lift
                final["view_aim"] = None
                if not final.get("success"):
                    trial.fail_code = trial.fail_code or (
                        final.get("reason_code") or final.get("drop_reason")
                        or "VIEW_EMPTY")
                self._t1.append(final)
                trial.extras["integrate"] = final
                trial.extras["coverage"] = final.get("coverage")
                dump = os.path.join(
                    self._args.dump_dir, "box_%02d_integrate" % trial.index)
                self._dump_json(dump, "t1_trace.json", final)
                view = str(final.get("view") or "lift")
                for name in (
                        "integrate_request", "integrate_response",
                        "surface_2d", "reference", "solver_replay"):
                    src = os.path.join(dump, "%s_%s.json" % (name, view))
                    dst = os.path.join(dump, "%s.json" % name)
                    if os.path.isfile(src):
                        shutil.copyfile(src, dst)
        return super()._home_arm()

    def _lookup_xyz_quat(self, target, source):
        tf_msg = self._tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(),
            rclpy.duration.Duration(seconds=1.0))
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        return (
            (float(t.x), float(t.y), float(t.z)),
            (float(r.x), float(r.y), float(r.z), float(r.w)),
        )

    def _aim_from_portal(self, slot_meta, pick_msg):
        """Exit to the opening and aim camera_depth_optical_frame at the lid.

        Tool-down at the portal looks at the threshold (VIEW_EMPTY,
        2026-09-20_1230). Keep the camera eye, yaw optical +Z toward the
        placed box, and command the equivalent suction pose.
        """
        exit_ok, exit_msg = self._exit_to_portal()
        if not exit_ok:
            return False, "portal: %s" % exit_msg
        try:
            cam_xyz, _cam_q = self._lookup_xyz_quat(
                "world", "camera_depth_optical_frame")
            suc_t, suc_q = self._lookup_xyz_quat(
                "camera_depth_optical_frame", "suction_contact_frame")
        except Exception as exc:  # noqa: BLE001 - T1 evidence
            return False, str(exc)
        target = list((slot_meta or {}).get("pose_world", {}).get("position") or [])
        if len(target) < 3:
            return False, "no slot world position"
        target[2] = float(target[2]) + 0.5 * float(pick_msg.height)
        q_wc = look_at_quaternion(cam_xyz, target)
        r_wc = rotation_from_xyzw(*q_wc)
        r_cs = rotation_from_xyzw(*suc_q)
        t_cs = np.asarray(suc_t, dtype=np.float64)
        r_ws = r_wc.dot(r_cs)
        t_ws = r_wc.dot(t_cs) + np.asarray(cam_xyz, dtype=np.float64)
        q_ws = _quat_from_R(r_ws)
        segment = MotionSegment()
        segment.name = "cargo_view_aim"
        segment.type = "pose_target"
        segment.keep_tool_down = False
        segment.keep_camera_down = False
        segment.allow_ompl_fallback = True
        segment.target_pose = Pose(
            position=Point(x=float(t_ws[0]), y=float(t_ws[1]), z=float(t_ws[2])),
            orientation=Quaternion(x=q_ws[0], y=q_ws[1], z=q_ws[2], w=q_ws[3]),
        )
        ok, message, _result = self.send_action(
            self._plan, PlanMotion.Goal(segment=segment),
            timeout=self._args.plan_timeout,
            name="PlanMotion:cargo_view_aim")
        return ok, message

    def _lift_for_cargo_view(self):
        """Raise the wrist in place, keeping the retreat orientation.

        Cartesian-only: OMPL fallback can spin the camera off the box top.
        One 0.50 m hop was fraction 0.940 (2026-09-20_1320); two 0.25 m hops
        stay above cartesian_min_fraction. A partial lift still integrates.
        """
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "world", "suction_contact_frame",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0))
        except Exception as exc:  # noqa: BLE001 - T1 evidence
            return False, str(exc), None
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        before = (float(t.x), float(t.y), float(t.z))
        hops = []
        remaining = float(VIEW_LIFT_Z)
        hop_z = 0.25
        while remaining > 1e-6:
            step = min(hop_z, remaining)
            hops.append(step)
            remaining -= step
        last_msg = ""
        achieved = 0.0
        z = float(t.z)
        for i, step in enumerate(hops):
            segment = MotionSegment()
            segment.name = "cargo_view_lift_%d" % i
            segment.type = "cartesian"
            segment.keep_tool_down = True
            segment.keep_camera_down = True
            segment.allow_ompl_fallback = False
            z = z + step
            segment.target_pose = Pose(
                position=Point(x=t.x, y=t.y, z=z),
                orientation=Quaternion(x=r.x, y=r.y, z=r.z, w=r.w),
            )
            ok, last_msg, _result = self.send_action(
                self._plan, PlanMotion.Goal(segment=segment),
                timeout=self._args.plan_timeout,
                name="PlanMotion:cargo_view_lift_%d" % i)
            if not ok:
                after, _err = self.suction_xyz()
                return (
                    achieved > 0.0,
                    "hop %d/%d failed after +%.2f: %s" % (
                        i + 1, len(hops), achieved, last_msg),
                    {"before": before, "after": after, "achieved_z": achieved},
                )
            achieved += step
        after, _err = self.suction_xyz()
        return True, last_msg, {
            "before": before, "after": after, "achieved_z": achieved}

    def _execute_segment(self, segment, trial):
        ok, code, rec = super()._execute_segment(segment, trial)
        invalidated = (
            rec is not None
            and int(rec.get("moveit_error_code") or 0) in (-2, -4))
        if (not ok and invalidated
                and str(getattr(segment, "name", "")) in (
                    "traverse", "transit", "cargo_view_lift")):
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

    def _integrate_after_place(
            self, trial, slot, slot_meta, pick_msg, view="lift", record=True):
        del slot, slot_meta
        dump = os.path.join(
            self._args.dump_dir, "box_%02d_integrate" % trial.index)
        os.makedirs(dump, exist_ok=True)
        stats = self.call_srv(self._stats, GetCargoMapStats.Request(), timeout=5.0)
        cloud, freshness = self._wait_untracked(timeout=5.0, min_points=100)
        t1 = {
            "view_request_id": "occ1-%d-%s" % (trial.index, view),
            "view": view,
            "cloud_n": int(cloud.width * cloud.height) if cloud else 0,
            "cloud_frame": str(cloud.header.frame_id) if cloud else "",
            "cloud_stamp": stamp_to_sec(cloud.header.stamp) if cloud else None,
            "freshness_sec": freshness,
            "expected_map_revision": int(stats.map_revision) if stats else -1,
            "geometry_hash": str(stats.geometry_hash) if stats else "",
            "cargo_view_lift": trial.extras.get("cargo_view_lift"),
            "cargo_view_aim": trial.extras.get("cargo_view_aim"),
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
            self._dump_cloud_ply(dump, cloud, name="untracked_%s.ply" % view)
            self._dump_json(dump, trace_name, t1)
            if record:
                trial.fail_code = trial.fail_code or "VIEW_EMPTY"
                self._dump_json(dump, "t1_trace.json", t1)
                self._t1.append(t1)
            return t1
        request = IntegrateCargoView.Request()
        request.schema_version = SCHEMA_VERSION
        request.view_request_id = t1["view_request_id"]
        request.candidate_id = str(trial.catalog_id or trial.index)
        request.source_acquisition_stamp = cloud.header.stamp
        request.expected_map_revision = int(stats.map_revision) if stats else 0
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
        reference = {
            "ign_pose": ign,
            "aabb_xy_container": aabb,
            "coverage": coverage,
            "hit_cells": hit,
            "total_cells": total,
            "size_wdh": [
                float(pick_msg.width), float(pick_msg.depth),
                float(pick_msg.height)],
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
            if record or not trial.extras.get("integrate_ok"):
                trial.fail_code = trial.fail_code or (
                    resp.reason_code or "INTEGRATE_FAILED")
            return t1
        trial.extras["integrate_ok"] = True
        if coverage >= COVERAGE_MIN and not through:
            if trial.fail_code in ("COVERAGE_LOW", "FLOOR_THROUGH"):
                trial.fail_code = None
        elif coverage < COVERAGE_MIN:
            trial.fail_code = trial.fail_code or "COVERAGE_LOW"
        elif through:
            trial.fail_code = trial.fail_code or "FLOOR_THROUGH"
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
                if rec.fail_code not in ("GOTO_FAILED", "COVERAGE_LOW"):
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
        if any(r.fail_code and r.fail_code != "GOTO_FAILED" for r in records):
            return 1
        return 0
    finally:
        executor.remove_node(driver)
        driver.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
