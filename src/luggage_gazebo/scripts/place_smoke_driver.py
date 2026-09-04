#!/usr/bin/env python3
"""Slice A place smoke: pick-carry into a known container slot.

State machine and dump layout: docs/plans/todo5_place_action.md.
Does not modify pick_retreat_eval_driver.py. Reuses its graph / pick helpers
by subclassing, then continues into place instead of vacuum-off + observe.

Order after descend: RELEASE_SETTLE → VacuumCommand(false) → retreat →
AddPlacedBox. Adding the placed box while the cup is still on the lid makes
retreat start in-collision.
"""

from __future__ import division

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from geometry_msgs.msg import Quaternion
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from luggage_msgs.action import PlanMotion
from luggage_msgs.msg import DetectedLuggage, SlotSpec
from luggage_msgs.srv import (
    AddPlacedBox,
    BuildMotionSequence,
    ClearCurrentBox,
    DetectLuggage,
    GetCurrentBox,
    RemovePlacedBox,
    SpawnNextBox,
)

from luggage_description.scene_tf_config_utils import (
    container_inner_floor_z,
    load_scene_tf_config,
    origin_in_world,
    point_inside_container_inner_box,
    point_inside_opening_aperture,
    resolve_scene_tf_config_path,
    xyz_world_to_base_link,
    yaw_world_to_base_link,
)
from luggage_description.scene_tf_publisher import rpy_to_quaternion
from luggage_planning.motion_executor import MotionExecutor

from luggage_gazebo.place_metrics import (
    PlaceTrial,
    parse_ign_model_pose,
    place_ok,
    summarize,
    trial_to_dict,
)

# Import the eval driver as a library (same package scripts dir).
_SCRIPTS = os.path.dirname(os.path.abspath(__file__))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)
from pick_retreat_eval_driver import (  # noqa: E402
    DEFAULT_VISUAL_TOL_XY,
    DEFAULT_VISUAL_TOL_Z,
    PickRetreatEvalDriver,
    _decode_dump_image,
    _git_meta,
    _parse_json,
    stamp_sec_from_key,
    write_trial_dump,
)
from luggage_gazebo.place_gt_dump import (  # noqa: E402
    build_place_gt,
    write_ply_xyzrgb,
)
from luggage_perception.cargo_instance_tracker import (  # noqa: E402
    parse_current_box_payload,
)
from luggage_gazebo.eval_metrics import tracker_wait_fail_code  # noqa: E402

DEFAULT_OUT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "docs", "status", "evidence", "place_smoke_n3"))

CARRYING_ABORTS = {
    "PLACE_PRECOND", "PLACE_SLOT_INVALID", "PLACE_SLOT_UNREACHABLE",
    "PLACE_BUILD_FAILED", "PLACE_LOST_PAYLOAD", "PLACE_APERTURE",
    "PLACE_FOLLOW_DRIFT", "PLACE_PLAN_stage", "PLACE_PLAN_stage_mid",
    "PLACE_PLAN_stage_late", "PLACE_PLAN_transit", "PLACE_PLAN_traverse",
    "PLACE_PLAN_insert", "PLACE_FRACTION_insert", "PLACE_PLAN_descend",
    "PLACE_FRACTION_descend", "RELEASE_SETTLE_FAILED", "VACUUM_DETACH",
}

STAGING_NAMES = ("stage_mid", "stage_late", "stage")
PLACE_CORE = ("transit", "traverse", "insert", "descend", "retreat")


def _pose_to_dict(pose):
    return {
        "position": [
            float(pose.position.x), float(pose.position.y),
            float(pose.position.z)],
        "orientation": [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w)],
    }


def _xyz3(sample):
    if sample is None:
        return None
    return [float(sample[0]), float(sample[1]), float(sample[2])]


def _ign_model_bin():
    for name in ("ign", "gz"):
        path = shutil.which(name)
        if path and os.path.basename(path) in ("ign", "gz"):
            return path
    return "ign"


def _yaw_quat(yaw):
    qx, qy, qz, qw = rpy_to_quaternion([0.0, 0.0, float(yaw)])
    q = Quaternion()
    q.x, q.y, q.z, q.w = qx, qy, qz, qw
    return q


class PlaceSmokeDriver(PickRetreatEvalDriver):

    def __init__(self, args):
        super().__init__(args)
        self._scene_config = load_scene_tf_config(
            resolve_scene_tf_config_path(
                getattr(args, "scene_tf_config", None) or None))
        self._timeline = []
        self._place_state = "INIT"
        self._segments_log = []
        self._tf_trace = []
        self._probe_joints = None
        self._follow_skipped0 = 0
        latch = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            String, "/luggage/place/state", latch)
        self._add_placed = self.create_client(
            AddPlacedBox, "/scene_manager/add_placed_box",
            callback_group=self._group)
        self._remove_placed = self.create_client(
            RemovePlacedBox, "/scene_manager/remove_placed_box",
            callback_group=self._group)
        self._box_model = ""
        self._place_touch = self.create_client(
            SetBool, "/scene_manager/set_place_support_touch",
            callback_group=self._group)
        self._get_scene = self.create_client(
            GetPlanningScene, "/get_planning_scene",
            callback_group=self._group)
        self._probe = MotionExecutor(self)
        self._keep_placed = False

    def _exit_to_portal(self):
        """Cartesian reverse of traverse so HOME does not start inside the box."""
        dummy = self._dummy_pick((0.55, 0.40, 0.25))
        slot, _meta = self._fixed_slot(dummy)
        req = BuildMotionSequence.Request()
        req.phase = "place"
        req.pick = dummy
        req.place_slot = slot
        built = self.call_srv(self._build, req, timeout=15.0)
        if built is None or not built.success:
            return False, built.message if built else "build timeout"
        transit = next((s for s in built.segments if s.name == "transit"), None)
        if transit is None:
            return False, "no transit segment"
        transit.name = "place_exit"
        transit.type = "cartesian"
        transit.allow_ompl_fallback = True
        ok, message, _result = self.send_action(
            self._plan, PlanMotion.Goal(segment=transit),
            timeout=self._args.plan_timeout, name="PlanMotion:place_exit")
        return ok, message

    def _home_arm(self):
        suction, _err = self.suction_xyz()
        notes = []
        t_wall0 = time.time()
        t_ros0 = self.ros_now_sec()
        if suction is not None and float(suction[0]) > 0.2:
            exit_ok, exit_msg = self._exit_to_portal()
            notes.append("exit=%s" % exit_msg)
            if not exit_ok:
                return False, "; ".join(notes), None
        goto_ok, goto_msg, result = self.goto_observe()
        notes.append("goto=%s" % goto_msg)
        dt_wall = time.time() - t_wall0
        dt_ros = self.ros_now_sec() - t_ros0
        rtf = (dt_ros / dt_wall) if dt_wall > 1e-3 else 0.0
        notes.append("rtf=%.3f (sim %.2fs / wall %.2fs)" % (
            rtf, dt_ros, dt_wall))
        return goto_ok, "; ".join(notes), result

    def graph_error(self):
        err = super().graph_error()
        if err:
            return err
        names = set(self.get_node_names())
        if "scene_manager" not in names:
            return "missing nodes: scene_manager"
        return ""

    def _set_state(self, new_state, guard="", note=""):
        record = {
            "t_ros": self.ros_now_sec(),
            "t_wall": time.time(),
            "from": self._place_state,
            "to": new_state,
            "guard": guard,
            "note": note,
        }
        self._timeline.append(record)
        self._place_state = new_state
        self._state_pub.publish(String(data=json.dumps(record, sort_keys=True)))
        self.get_logger().info(
            "place %s -> %s %s %s" % (record["from"], new_state, guard, note))

    def _sample_tf_box(self, label="", query_gz=False):
        suction, err = self.suction_xyz()
        box = self._gz_box_pose() if query_gz else None
        sample = {
            "label": label,
            "t_ros": self.ros_now_sec(),
            "suction": list(suction) if suction else None,
            "suction_err": err,
            "box_gz": box,
            "vacuum": dict(self._vacuum_state or {}),
        }
        self._tf_trace.append(sample)
        return sample

    def _gz_box_pose(self, model=None):
        model = str(model or self._box_model or "")
        if not model:
            payload = _parse_json(self._current_box_topic.get("payload")) or {}
            model = str(payload.get("model_name") or "")
        if not model:
            return None
        try:
            out = subprocess.check_output(
                [_ign_model_bin(), "model", "-m", model, "--pose"],
                text=True, timeout=5.0, stderr=subprocess.STDOUT)
        except (OSError, subprocess.CalledProcessError, subprocess.TimeoutExpired):
            return None
        return parse_ign_model_pose(out)

    def _fixed_slot(self, box):
        height = float(box.height)
        width = float(box.width)
        depth = float(box.depth)
        origin, _rpy = origin_in_world(self._scene_config)
        floor_z = container_inner_floor_z(self._scene_config)
        world_xyz = [
            origin[0], origin[1], origin[2] + floor_z + 0.5 * height]
        world_yaw = 0.0
        if float(getattr(self._args, "release_gap", 0.0) or 0.0) > 0.0:
            world_xyz[2] += float(self._args.release_gap)
        base_xyz = xyz_world_to_base_link(self._scene_config, world_xyz)
        base_yaw = yaw_world_to_base_link(self._scene_config, world_yaw)
        slot = SlotSpec()
        slot.layer, slot.row, slot.col = 0, 0, 0
        slot.width, slot.depth, slot.height = width, depth, height
        slot.place_pose.position.x = float(base_xyz[0])
        slot.place_pose.position.y = float(base_xyz[1])
        slot.place_pose.position.z = float(base_xyz[2])
        slot.place_pose.orientation = _yaw_quat(base_yaw)
        return slot, {
            "planning_frame": "world",
            "pose_world": {"position": world_xyz, "yaw": world_yaw},
            "pose_base_link": {
                "position": list(base_xyz), "yaw": float(base_yaw)},
            "source": "fixed_floor_center",
        }

    def _slot_world_msg(self, slot, slot_meta):
        out = SlotSpec()
        out.layer, out.row, out.col = slot.layer, slot.row, slot.col
        out.width, out.depth, out.height = slot.width, slot.depth, slot.height
        pos = slot_meta["pose_world"]["position"]
        out.place_pose.position.x = float(pos[0])
        out.place_pose.position.y = float(pos[1])
        out.place_pose.position.z = float(pos[2])
        out.place_pose.orientation = _yaw_quat(slot_meta["pose_world"]["yaw"])
        return out

    def _scene_snapshot(self):
        request = GetPlanningScene.Request()
        request.components.components = int(
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = self.call_srv(self._get_scene, request, timeout=5.0)
        if response is None:
            return {"error": "get_planning_scene timeout"}
        scene = response.scene
        acm = scene.allowed_collision_matrix
        return {
            "world_objects": [obj.id for obj in scene.world.collision_objects],
            "attached": [
                a.object.id for a in scene.robot_state.attached_collision_objects],
            "acm_names": list(acm.entry_names),
            "place_state": self._place_state,
        }

    def _follow_skipped(self):
        events = dict(self._vacuum_events or {})
        try:
            return int(events.get("follow_skipped") or 0)
        except (TypeError, ValueError):
            return 0

    def _clear_smoke_slot(self, slot, delete_model=False):
        resp = self.call_srv(
            self._remove_placed, RemovePlacedBox.Request(slot=slot), timeout=5.0)
        if delete_model:
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
        return resp

    def _set_place_touch(self, allowed):
        request = SetBool.Request()
        request.data = bool(allowed)
        return self.call_srv(self._place_touch, request, timeout=10.0)

    def _check_i1(self):
        if not getattr(self._args, "use_vacuum", False):
            return True, ""
        state = dict(self._vacuum_state or {})
        if not state.get("attached") or state.get("fail_reason"):
            return False, "PLACE_LOST_PAYLOAD"
        return True, ""

    def _execute_segment(self, segment, trial):
        t0 = time.time()
        before = self._sample_tf_box("before_%s" % segment.name)
        ok_i1, code = self._check_i1()
        if not ok_i1 and segment.name != "retreat":
            return False, code, None
        if self._args.plan_only:
            probe = self._probe.probe_segment(
                segment, start_joints=self._probe_joints)
            if probe.get("ik_joints"):
                self._probe_joints = probe["ik_joints"]
            cartesian_ok = probe.get("cartesian_ok")
            rec = {
                "name": segment.name, "type": segment.type,
                "ok": bool(probe.get("ik_ok")) and cartesian_ok is not False,
                "fraction": probe.get("fraction"),
                "ik_ok": probe.get("ik_ok"),
                "cartesian_ok": cartesian_ok,
                "used_ompl_fallback": False,
                "message": "probe",
                "plan_sec": time.time() - t0,
                "target_pose_world": _pose_to_dict(segment.target_pose),
            }
            self._segments_log.append(rec)
            self.get_logger().info(
                "place probe %s ik=%s frac=%s cart=%s ok=%s"
                % (segment.name, rec.get("ik_ok"), rec.get("fraction"),
                   rec.get("cartesian_ok"), rec["ok"]))
            if rec["ok"]:
                return True, "", rec
            if cartesian_ok is False:
                return False, "PLACE_FRACTION_%s" % segment.name, rec
            return False, "PLACE_PLAN_%s" % segment.name, rec
        goal = PlanMotion.Goal()
        goal.segment = segment
        ok, message, result = self.send_action(
            self._plan, goal, timeout=self._args.plan_timeout,
            name="PlanMotion:%s" % segment.name)
        after = self._sample_tf_box("after_%s" % segment.name)
        fraction = float(getattr(result, "fraction", 0.0) or 0.0) if result else 0.0
        used_fb = bool(getattr(result, "used_ompl_fallback", False)) if result else False
        rec = {
            "name": str(segment.name),
            "type": str(segment.type),
            "reference_frame": "world",
            "target_pose_world": _pose_to_dict(segment.target_pose),
            "keep_tool_down": bool(segment.keep_tool_down),
            "keep_camera_down": bool(segment.keep_camera_down),
            "lock_wrist": bool(segment.lock_wrist),
            "allow_ompl_fallback": bool(segment.allow_ompl_fallback),
            "ok": bool(ok),
            "message": message,
            "fraction": fraction,
            "used_ompl_fallback": used_fb,
            "moveit_error_code": int(
                getattr(result, "moveit_error_code", 0) or 0) if result else 0,
            "settle_json": (
                getattr(result, "settle_json", "") or "") if result else "",
            "plan_sec": time.time() - t0,
            "suction_before": before.get("suction"),
            "suction_after": after.get("suction"),
            "box_gz_before": before.get("box_gz"),
            "box_gz_after": after.get("box_gz"),
            "vacuum": after.get("vacuum"),
        }
        self._segments_log.append(rec)
        self.get_logger().info(
            "place %s ok=%s frac=%.3f f_ompl=%s vac=%s"
            % (segment.name, ok, fraction, int(used_fb),
               int(bool((after.get("vacuum") or {}).get("attached")))))
        if not ok:
            if segment.name == "descend" and "settle" in (message or "").lower():
                return False, "RELEASE_SETTLE_FAILED", rec
            if "fraction" in (message or "") and not segment.allow_ompl_fallback:
                return False, "PLACE_FRACTION_%s" % segment.name, rec
            return False, "PLACE_PLAN_%s" % segment.name, rec
        return True, "", rec

    def _latest_color(self, min_stamp=None, timeout=2.5):
        buf = (self._buffers or {}).get("color")
        if buf is None:
            return None, "no color buffer (dump_dir unset)"
        deadline = time.time() + float(timeout)
        best = None
        while time.time() < deadline:
            snap = buf.snapshot()
            if snap:
                key = max(snap)
                stamp = stamp_sec_from_key(key)
                if min_stamp is None or stamp is None or stamp > float(min_stamp):
                    best = snap[key]
                    break
            time.sleep(0.05)
        if best is None:
            snap = buf.snapshot()
            if snap:
                best = snap[max(snap)]
        if best is None:
            return None, "no color frame"
        return _decode_dump_image(best), ""

    def _capture_success_debug(self, trial, slot, slot_meta, pose_world):
        """Camera at retreat pose, GT occupancy image, interior+box cloud."""
        if not self._args.plan_only:
            time.sleep(0.35)
        color, color_err = self._latest_color(
            min_stamp=(self.ros_now_sec() - 1.0) if not self._args.plan_only else None,
            timeout=2.5 if not self._args.plan_only else 0.1)
        size = None
        if slot is not None:
            size = [float(slot.width), float(slot.depth), float(slot.height)]
        center_world = None
        rpy_world = None
        center_base = None
        yaw_base = 0.0
        if pose_world is not None and len(pose_world) >= 3:
            center_world = [float(pose_world[i]) for i in range(3)]
            yaw_w = float(pose_world[5]) if len(pose_world) >= 6 else 0.0
            rpy_world = [
                float(pose_world[3]) if len(pose_world) >= 4 else 0.0,
                float(pose_world[4]) if len(pose_world) >= 5 else 0.0,
                yaw_w,
            ]
            center_base = xyz_world_to_base_link(self._scene_config, center_world)
            yaw_base = yaw_world_to_base_link(self._scene_config, yaw_w)
        elif slot_meta:
            center_world = [float(v) for v in slot_meta["pose_world"]["position"]]
            yaw_w = float(slot_meta["pose_world"].get("yaw") or 0.0)
            rpy_world = [0.0, 0.0, yaw_w]
            center_base = xyz_world_to_base_link(self._scene_config, center_world)
            yaw_base = yaw_world_to_base_link(self._scene_config, yaw_w)
        gt = build_place_gt(
            self._scene_config, center_base, size, yaw_base,
            box_center_world=center_world, box_rpy_world=rpy_world)
        surface = gt["surface_map"]
        trial.extras["place_debug"] = {
            "color_err": color_err,
            "occupancy": {
                key: surface[key] for key in (
                    "nx", "ny", "resolution", "inner_size",
                    "occupancy_ratio", "occupied_count", "committed_box_count")
                if key in surface
            },
            "gt_stats": {
                key: gt["stats"][key] for key in (
                    "occupancy_ratio", "occupied_count", "committed_box_count",
                    "n_gt_points", "n_wall_points", "n_box_points")
                if key in gt["stats"]
            },
        }
        return {
            "place_retreat_color": color,
            "occupancy_gt": gt["occupancy_image"],
            "cloud_xyz": gt["cloud_xyz"],
            "cloud_rgb": gt["cloud_rgb"],
            "occupancy_meta": gt["surface_map"],
        }

    def _dump_place(self, trial, extra_files=None, extra=None, debug=None):
        dump_dir = getattr(self._args, "dump_dir", "") or ""
        if not dump_dir:
            return None
        images = {}
        if debug:
            if debug.get("place_retreat_color") is not None:
                images["place_retreat_color"] = debug["place_retreat_color"]
            if debug.get("occupancy_gt") is not None:
                images["occupancy_gt"] = debug["occupancy_gt"]
        dest = write_trial_dump(
            dump_dir, trial_to_dict(trial), images=images,
            exact_dir=bool(getattr(self._args, "dump_exact", False)))
        if not dest:
            return None
        with open(os.path.join(dest, "segments.jsonl"), "w", encoding="utf-8") as handle:
            for row in self._segments_log:
                handle.write(json.dumps(row, sort_keys=True, default=str) + "\n")
        with open(os.path.join(dest, "state_timeline.jsonl"), "w", encoding="utf-8") as handle:
            for row in self._timeline:
                handle.write(json.dumps(row, sort_keys=True, default=str) + "\n")
        with open(os.path.join(dest, "tf_trace.jsonl"), "w", encoding="utf-8") as handle:
            for row in self._tf_trace:
                handle.write(json.dumps(row, sort_keys=True, default=str) + "\n")
        events = dict(self._vacuum_events or {})
        with open(os.path.join(dest, "vacuum_events.jsonl"), "w", encoding="utf-8") as handle:
            handle.write(json.dumps(events, sort_keys=True, default=str) + "\n")
        merged = {"scene.json": self._scene_snapshot()}
        if extra_files:
            merged.update(extra_files)
        if extra:
            merged.update(extra)
        if debug and debug.get("occupancy_meta") is not None:
            merged["occupancy_gt.json"] = debug["occupancy_meta"]
        for name, payload in merged.items():
            with open(os.path.join(dest, name), "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2, sort_keys=True, default=str)
                handle.write("\n")
        if debug and debug.get("cloud_xyz"):
            n = write_ply_xyzrgb(
                os.path.join(dest, "gt_interior_box.ply"),
                debug["cloud_xyz"], debug.get("cloud_rgb"))
            trial.extras.setdefault("place_debug", {})["gt_ply_points"] = n
            trial.extras.setdefault("place_debug", {})["gt_ply"] = "gt_interior_box.ply"
            with open(os.path.join(dest, "trial.json"), "w", encoding="utf-8") as handle:
                json.dump(trial_to_dict(trial), handle, indent=2, sort_keys=True, default=str)
                handle.write("\n")
        return dest

    def _abort(self, trial, code, carrying, extra=None):
        trial.fail_code = code
        if carrying:
            self._set_state("ABORT_CARRYING", guard=code)
        else:
            self._set_state("ABORT_RELEASED", guard=code)
            if not self._args.plan_only:
                self._home_arm()
        trial.place_state = self._place_state
        trial.lost_payload = code == "PLACE_LOST_PAYLOAD"
        trial.extras["follow_skipped_delta"] = (
            self._follow_skipped() - self._follow_skipped0)
        self._dump_place(trial, extra=extra)
        return trial

    def run_place_from_carry(self, pick_msg, trial, slot, slot_meta):
        self._probe_joints = None
        self._follow_skipped0 = self._follow_skipped()
        extras = {"slot.json": slot_meta}
        if not self._box_model:
            payload = _parse_json(self._current_box_topic.get("payload")) or {}
            self._box_model = str(payload.get("model_name") or "")
        extras["box_model"] = self._box_model
        if not getattr(self, "_keep_placed", False):
            self._clear_smoke_slot(slot, delete_model=False)
        self._set_state("CARRY_READY")
        if getattr(self._args, "use_vacuum", False):
            ok_i1, code = self._check_i1()
            if not ok_i1:
                return self._abort(trial, code, True, extra=extras)

        self._set_state("SLOT_RESOLVED")
        base_xyz = [
            slot.place_pose.position.x,
            slot.place_pose.position.y,
            slot.place_pose.position.z,
        ]
        if not point_inside_container_inner_box(
                base_xyz, self._scene_config, margin=0.0):
            return self._abort(trial, "PLACE_SLOT_INVALID", True, extra=extras)

        req = BuildMotionSequence.Request()
        req.phase = "place"
        req.pick = pick_msg
        req.place_slot = slot
        built = self.call_srv(self._build, req, timeout=15.0)
        if built is None or not built.success:
            trial.extras["build"] = built.message if built else "timeout"
            return self._abort(trial, "PLACE_BUILD_FAILED", True, extra=extras)
        trial.staging_degenerate = "staging_degenerate" in (built.message or "")
        trial.extras["build"] = built.message
        segments = list(built.segments)
        trial.segments_planned = len(segments)
        self._set_state("SEQUENCE_BUILT", note=built.message)

        names = [s.name for s in segments]
        retreat_seg = None
        remaining = []
        for segment in segments:
            if segment.name == "retreat":
                retreat_seg = segment
            else:
                remaining.append(segment)
        if any(n in names for n in STAGING_NAMES):
            self._set_state("STAGING")
        state_by_name = {
            "transit": "TRANSIT", "traverse": "TRAVERSE",
            "insert": "INSERT", "descend": "DESCEND",
        }
        for segment in remaining:
            if segment.name == "insert":
                suction, _ = self.suction_xyz()
                if suction is not None:
                    base = xyz_world_to_base_link(self._scene_config, suction)
                    if not point_inside_opening_aperture(
                            base, self._scene_config, margin=0.02):
                        return self._abort(trial, "PLACE_APERTURE", True,
                                           extra=extras)
                self._set_state("INSERT")
            elif segment.name == "descend":
                touch = self._set_place_touch(True)
                trial.extras["place_support_touch"] = (
                    touch.message if touch else "timeout")
                if touch is None or not touch.success:
                    return self._abort(
                        trial, "PLACE_PLAN_descend", True, extra=extras)
                self._set_state("DESCEND")
            elif segment.name in state_by_name:
                self._set_state(state_by_name[segment.name])
            ok, code, rec = self._execute_segment(segment, trial)
            if rec and segment.name == "descend":
                trial.descend_fraction = rec.get("fraction")
                trial.used_ompl_fallback_descend = rec.get("used_ompl_fallback")
            if ok:
                trial.segments_succeeded += 1
            else:
                return self._abort(trial, code, True, extra=extras)
            if segment.name in ("transit", "traverse") and rec:
                suction = rec.get("suction_after")
                box = _xyz3(rec.get("box_gz_after"))
                if suction and box:
                    dxy = math.hypot(
                        float(suction[0]) - float(box[0]),
                        float(suction[1]) - float(box[1]))
                    trial.extras["follow_offset_xy_%s" % segment.name] = dxy
                    trial.extras["follow_offset_z_%s" % segment.name] = (
                        float(suction[2]) - float(box[2]))
                    # Plan I-3 is 5 cm; kinematic follow lags ~6–12 cm (Todo 4
                    # T3). Abort only when the box is left behind.
                    if dxy > 0.5:
                        return self._abort(
                            trial, "PLACE_FOLLOW_DRIFT", True, extra=extras)

        self._set_state("RELEASE_SETTLE")
        descend_rec = next(
            (row for row in reversed(self._segments_log)
             if row.get("name") == "descend"),
            {})
        trial.extras["release_settle"] = descend_rec.get("settle_json") or "probe"
        self._set_state("RELEASED")
        if getattr(self._args, "use_vacuum", False) and not self._args.plan_only:
            off_ok, off_msg = self.vacuum_command(False)
            trial.extras["vac_detach"] = off_msg
            if not off_ok:
                return self._abort(trial, "VACUUM_DETACH", True, extra=extras)

        if retreat_seg is not None:
            self._set_state("PLACE_RETREAT")
            ok, code, rec = self._execute_segment(retreat_seg, trial)
            if ok:
                trial.segments_succeeded += 1
            else:
                return self._abort(trial, code, False, extra=extras)

        pose_release = self._gz_box_pose()
        if not self._args.plan_only:
            time.sleep(float(self._args.drift_wait))
        pose_drift = self._gz_box_pose()
        self._set_state("COMMITTED")
        if (not self._args.plan_only) and getattr(self._args, "use_vacuum", False):
            add = AddPlacedBox.Request()
            add.slot = self._slot_world_msg(slot, slot_meta)
            resp = self.call_srv(self._add_placed, add, timeout=10.0)
            trial.extras["add_placed"] = resp.message if resp else "timeout"
            if resp is None or not resp.success:
                return self._abort(trial, "COMMIT_FAILED", False, extra=extras)
            self._set_place_touch(False)

        self._set_state("VERIFIED")
        expected = slot_meta["pose_world"]["position"]
        actual = _xyz3(pose_drift or pose_release)
        if (actual is None
                and getattr(self._args, "use_vacuum", False)
                and not self._args.plan_only):
            return self._abort(trial, "PLACE_VERIFY_XY", False, extra=extras)
        if actual is not None:
            trial.err_xy = math.hypot(
                actual[0] - expected[0], actual[1] - expected[1])
            trial.err_z = actual[2] - expected[2]
            rel = _xyz3(pose_release)
            drift = _xyz3(pose_drift)
            if rel and drift:
                trial.drift = math.sqrt(sum(
                    (drift[i] - rel[i]) ** 2 for i in range(3)))
            if pose_drift is not None and len(pose_drift) >= 5:
                trial.roll = float(pose_drift[3])
                trial.pitch = float(pose_drift[4])
            base_actual = xyz_world_to_base_link(self._scene_config, actual)
            trial.inside_inner_box = point_inside_container_inner_box(
                base_actual, self._scene_config, margin=0.0)
            if trial.err_xy > 0.04:
                return self._abort(trial, "PLACE_VERIFY_XY", False, extra=extras)
            if abs(trial.err_z) > 0.03:
                return self._abort(trial, "PLACE_VERIFY_Z", False, extra=extras)
            if trial.drift is not None and trial.drift > 0.02:
                return self._abort(
                    trial, "PLACE_VERIFY_DRIFT", False, extra=extras)
            if trial.roll is not None and abs(trial.roll) > math.radians(2.0):
                return self._abort(trial, "PLACE_VERIFY_TIP", False, extra=extras)
            if trial.pitch is not None and abs(trial.pitch) > math.radians(2.0):
                return self._abort(trial, "PLACE_VERIFY_TIP", False, extra=extras)
            if not trial.inside_inner_box:
                return self._abort(trial, "PLACE_VERIFY_XY", False, extra=extras)
        trial.extras["verify"] = {
            "box_pose_after_release": pose_release,
            "box_pose_after_drift_wait": pose_drift,
            "err_xy": trial.err_xy, "err_z": trial.err_z,
            "drift": trial.drift,
            "inside_inner_box": trial.inside_inner_box,
            "roll": trial.roll, "pitch": trial.pitch,
        }
        trial.extras["follow_skipped_delta"] = (
            self._follow_skipped() - self._follow_skipped0)

        debug = self._capture_success_debug(
            trial, slot, slot_meta, pose_drift or pose_release)
        trial.extras.setdefault("place_debug", {})["gt_ply"] = "gt_interior_box.ply"

        self._set_state("HOME")
        if not self._args.plan_only:
            goto_ok, goto_msg, _ = self._home_arm()
            trial.extras["return_observe"] = goto_msg
            if not goto_ok:
                trial.fail_code = "GOTO_FAILED"
        trial.place_state = self._place_state
        self._dump_place(trial, extra=extras, debug=debug)
        if (not self._args.plan_only) and getattr(self._args, "use_vacuum", False):
            if not getattr(self, "_keep_placed", False):
                self._clear_smoke_slot(slot, delete_model=True)
        return trial

    def _dummy_pick(self, size):
        msg = DetectedLuggage()
        msg.width, msg.depth, msg.height = [
            float(size[0]), float(size[1]), float(size[2])]
        msg.yaw_valid = False
        msg.pose.orientation.w = 1.0
        return msg

    def run_trial(self, index, slot=None, slot_meta=None,
                  already_spawned=False, keep_placed=False):
        t0 = time.time()
        self._timeline = []
        self._segments_log = []
        self._tf_trace = []
        self._place_state = "INIT"
        self._box_model = ""
        self._keep_placed = bool(keep_placed)
        trial = PlaceTrial(index=index)
        trial.extras = {}

        if self._args.plan_only or self._args.payload == "none":
            size = (0.55, 0.40, 0.25)
            pick = self._dummy_pick(size)
            if slot is None:
                slot, slot_meta = self._fixed_slot(pick)
            trial.catalog_id = "carryon"
            if self._args.payload == "none" and not self._args.plan_only:
                self._home_arm()
            result = self.run_place_from_carry(pick, trial, slot, slot_meta)
            result.wall_time_sec = time.time() - t0
            return result

        carry = self._pick_carry(index, trial, already_spawned=already_spawned)
        trial.wall_time_sec = time.time() - t0
        if carry is None:
            trial.place_state = self._place_state or "CARRY_READY"
            self._dump_place(trial)
            return trial
        pick_msg, box_msg = carry
        self._box_model = str(getattr(box_msg, "id", "") or self._box_model or "")
        if slot is None:
            slot, slot_meta = self._fixed_slot(box_msg)
        result = self.run_place_from_carry(pick_msg, trial, slot, slot_meta)
        result.wall_time_sec = time.time() - t0
        if (not keep_placed) and result.fail_code not in CARRYING_ABORTS:
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
        return result

    def _pick_carry(self, index, trial, already_spawned=False):
        """Pick through vacuum-follow. Returns (pick_msg, box_msg) or None."""
        goto_ok, goto_msg, _ = self._home_arm()
        if not goto_ok:
            trial.fail_code = "GOTO_FAILED"
            trial.extras["goto"] = goto_msg
            return None
        if already_spawned:
            current = self.call_srv(
                self._current, GetCurrentBox.Request(), timeout=10.0)
            if current is None or not current.success:
                trial.fail_code = "GT_UNAVAILABLE"
                trial.extras["current"] = (
                    current.message if current else "timeout")
                return None
            spawn = current
            # Spawn happened before ComputePlacement; accept YOLO already
            # published in the geometry window rather than requiring a new
            # frame after this call.
            spawn_stamp = max(
                0.0, self.ros_now_sec() - float(self._args.geometry_timeout))
        else:
            dirty = self.ensure_clean()
            if dirty:
                trial.fail_code = dirty.split(":")[0]
                trial.extras["clean"] = dirty
                return None
            spawn = self.call_srv(
                self._spawn, SpawnNextBox.Request(), timeout=30.0)
            if spawn is None or not spawn.success:
                trial.fail_code = "SPAWN_FAILED"
                trial.extras["spawn"] = spawn.message if spawn else "timeout"
                return None
            spawn_stamp = self.ros_now_sec()
        size_eval = _parse_json(self._size_eval["payload"]) or {}
        trial.catalog_id = str(size_eval.get("catalog_id") or "")
        box_id, generation = parse_current_box_payload(
            self._current_box_topic["payload"])
        if spawn.box.id and box_id != spawn.box.id:
            deadline = time.time() + 2.0
            while time.time() < deadline:
                box_id, generation = parse_current_box_payload(
                    self._current_box_topic["payload"])
                if box_id == spawn.box.id:
                    break
                time.sleep(0.05)
        yolo_stats = self.wait_yolo_boxes(
            generation, spawn.box.id, spawn_stamp,
            self._args.geometry_timeout)
        if not yolo_stats:
            trial.fail_code = "YOLO_NOT_READY"
            trial.extras["spawn_id"] = spawn.box.id
            trial.extras["spawn_generation"] = generation
            trial.extras["topic_box_id"] = box_id
            trial.extras["seg_stats"] = _parse_json(self._seg_stats["payload"])
            return None
        if not self.wait_tracked_cargo(
                generation, self._args.geometry_timeout, spawn.box.id):
            trial.fail_code = tracker_wait_fail_code(
                _parse_json(self._filter_stats["payload"]),
                generation, spawn.box.id)
            return None
        detect = self.call_srv(
            self._detect, DetectLuggage.Request(),
            timeout=self._args.detect_timeout)
        current = self.call_srv(
            self._current, GetCurrentBox.Request(), timeout=10.0)
        if detect is None:
            trial.fail_code = "DETECT_TIMEOUT"
            return None
        if current is None or not current.success:
            trial.fail_code = "GT_UNAVAILABLE"
            return None
        if not detect.success or not detect.luggage:
            trial.fail_code = detect.message or "MEASURED_NONE"
            return None
        pick_msg = detect.luggage[0]
        if self._args.use_vacuum:
            scene_ok, scene_msg = self.add_scene_box(pick_msg)
            trial.extras["scene_add"] = scene_msg
            if not scene_ok:
                trial.fail_code = "SCENE_ADD_FAILED"
                return None
        req = BuildMotionSequence.Request()
        req.phase = "pick"
        req.pick = pick_msg
        built = self.call_srv(self._build, req, timeout=15.0)
        if built is None or not built.success:
            trial.fail_code = "BUILD_FAILED"
            trial.extras["build_pick"] = built.message if built else "timeout"
            return None
        for segment in list(built.segments):
            goal = PlanMotion.Goal()
            goal.segment = segment
            ok, message, _res = self.send_action(
                self._plan, goal, timeout=self._args.plan_timeout,
                name="PlanMotion:%s" % segment.name)
            if not ok:
                trial.fail_code = "PLAN_%s" % segment.name
                trial.extras["plan_%s" % segment.name] = message
                return None
            if segment.name == "attach" and self._args.use_vacuum:
                vac_ok, vac_msg = self.vacuum_command(True)
                trial.vac_attach = vac_ok
                trial.extras["vac_attach"] = vac_msg
                if not vac_ok:
                    trial.fail_code = "VACUUM_ATTACH"
                    return None
        if self._args.use_vacuum:
            state = dict(self._vacuum_state or {})
            if not (state.get("attached") and not state.get("fail_reason")):
                trial.fail_code = "VACUUM_FOLLOW"
                return None
        return pick_msg, current.box


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=3)
    parser.add_argument("--out", default=DEFAULT_OUT)
    parser.add_argument("--dump-dir", default="")
    parser.add_argument("--observe-pose", default="pickup_observe")
    parser.add_argument("--plan-timeout", type=float, default=60.0)
    parser.add_argument("--goto-timeout", type=float, default=30.0)
    parser.add_argument("--detect-timeout", type=float, default=15.0)
    parser.add_argument("--geometry-timeout", type=float, default=15.0)
    parser.add_argument("--expected-retreat-dz", type=float, default=0.35)
    parser.add_argument("--retreat-tol", type=float, default=0.08)
    parser.add_argument("--tol-xy", type=float, default=0.03)
    parser.add_argument("--tol-z", type=float, default=0.02)
    parser.add_argument("--tol-size", type=float, default=0.05)
    parser.add_argument("--tol-yaw", type=float, default=0.15)
    parser.add_argument("--tol-iou", type=float, default=0.60)
    parser.add_argument("--tol-visual-xy", type=float, default=DEFAULT_VISUAL_TOL_XY)
    parser.add_argument("--tol-visual-z", type=float, default=DEFAULT_VISUAL_TOL_Z)
    parser.add_argument("--skip-graph-check", action="store_true")
    parser.add_argument("--strict-gt", action="store_true")
    parser.add_argument(
        "--payload", choices=("vacuum", "none"), default="vacuum")
    parser.add_argument(
        "--dry-run", dest="plan_only", action="store_true",
        help="IK + cartesian probe only, do not execute.")
    parser.add_argument("--slot-source", default="fixed_floor_center")
    parser.add_argument("--on-place-fail", default="stop")
    parser.add_argument("--release-gap", type=float, default=0.0)
    parser.add_argument("--drift-wait", type=float, default=1.0)
    parser.add_argument("--scene-tf-config", default="")
    args = parser.parse_args(argv)
    args.use_vacuum = args.payload == "vacuum" and not args.plan_only
    args.dump_dir = args.dump_dir or os.path.join(args.out, "dumps")
    return args


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    os.makedirs(args.out, exist_ok=True)
    os.makedirs(args.dump_dir, exist_ok=True)
    jsonl = os.path.join(args.out, "trials.jsonl")
    with open(jsonl, "w", encoding="utf-8"):
        pass

    rclpy.init()
    driver = PlaceSmokeDriver(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    records = []
    stop = {"flag": False}

    def _on_sigint(_signum, _frame):
        stop["flag"] = True
        print("interrupt: flushing %d trials" % len(records), flush=True)

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        if not args.skip_graph_check:
            err = driver.wait_graph(timeout=90.0)
            if err:
                print("graph: %s" % err)
                return 2
        if not driver._probe.wait_ready(timeout_sec=30.0 if args.plan_only else 5.0):
            print("move_group unavailable")
            return 2
        print("graph ok; running %d trials (payload=%s dry_run=%s)" % (
            args.n, args.payload, int(bool(args.plan_only))), flush=True)
        for index in range(args.n):
            if stop["flag"] or not rclpy.ok():
                break
            rec = driver.run_trial(index)
            records.append(rec)
            with open(jsonl, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(trial_to_dict(rec), sort_keys=True) + "\n")
            print(json.dumps({
                "index": rec.index, "fail_code": rec.fail_code,
                "place_state": rec.place_state,
                "segments": "%s/%s" % (
                    rec.segments_succeeded, rec.segments_planned),
            }, sort_keys=True), flush=True)
            if rec.fail_code in CARRYING_ABORTS and args.on_place_fail == "stop":
                break
    finally:
        extra = _git_meta(os.path.normpath(os.path.join(_SCRIPTS, "..", "..", "..")))
        extra.update({
            "date": time.strftime("%Y-%m-%d"),
            "n_requested": args.n,
            "payload": args.payload,
            "plan_only": bool(args.plan_only),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        })
        summary = summarize(records)
        summary.update(extra)
        with open(os.path.join(args.out, "summary.json"), "w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        listener = getattr(driver, "_tf_listener", None)
        tf_exec = getattr(listener, "executor", None) if listener else None
        if tf_exec is not None:
            tf_exec.shutdown()
            thread = getattr(listener, "dedicated_listener_thread", None)
            if thread is not None:
                thread.join(timeout=2.0)
        executor.shutdown()
        tf_node = getattr(driver, "_tf_node", None)
        driver.destroy_node()
        if tf_node is not None:
            tf_node.destroy_node()
        rclpy.shutdown()
    n_ok = sum(1 for r in records if place_ok(r))
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if n_ok == len(records) and records else 1


if __name__ == "__main__":
    sys.exit(main())
