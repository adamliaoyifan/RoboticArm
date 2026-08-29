#!/usr/bin/env python3
"""Closed-loop pick/retreat eval: spawn → observe → detect → plan → execute → clear.

State machine per docs/plans/closed_loop_eval_driver.md. ROS interfaces only;
no YOLO / RANSAC / MoveIt imports. Ctrl-C flushes completed trials.

Three independent rates (see luggage_gazebo.eval_metrics.summarize):
  detect_pass_rate, plan_pass_rate, retreat_pass_rate.
Geometric pick accuracy is suction_contact_frame vs the measured box after
attach (no vacuum in this stack).
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
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from luggage_msgs.action import GoToRobotPose, PlanMotion
from luggage_msgs.srv import (
    BuildMotionSequence,
    ClearCurrentBox,
    DetectLuggage,
    GetCurrentBox,
    SpawnNextBox,
)

from luggage_perception.eval.detection_accuracy import (
    BoxObservation,
    DetectionAccuracy,
)
from luggage_perception.eval.detection_gate_sampling import (
    FRESH_SEC,
    is_gt_fallback,
    is_perception_estimate,
    perception_reason,
    wait_ready,
)

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.isdir(os.path.join(_PKG_ROOT, "luggage_gazebo")):
    sys.path.insert(0, _PKG_ROOT)
from luggage_gazebo.eval_metrics import (  # noqa: E402
    TrialRecord,
    summarize,
    trial_from_dict,
    trial_to_dict,
)

DUPLICATE_PROCS = (
    ("camera_bridge", r"__node:=camera_bridge"),
    ("move_group", r"moveit_ros_move_group/move_group"),
    ("robot_state_publisher", r"/robot_state_publisher/robot_state_publisher"),
    ("sensor_preprocessor", r"lib/luggage_perception/sensor_preprocessor_node.py"),
    ("pickup_box_spawner", r"lib/luggage_gazebo/pickup_box_spawner_node.py"),
    ("motion_planner", r"lib/luggage_planning/motion_planner_node.py"),
)
REQUIRED_NODES = (
    "camera_bridge",
    "sensor_preprocessor",
    "pickup_box_spawner",
    "luggage_detector",
    "waypoint_generator",
    "motion_planner",
    "move_group",
)
DEFAULT_OUT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..",
    "docs", "status", "evidence", "pick_eval",
)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def observation_from_detected(msg):
    return BoxObservation(
        x=float(msg.pose.position.x),
        y=float(msg.pose.position.y),
        z=float(msg.pose.position.z),
        yaw=yaw_from_quaternion(msg.pose.orientation),
        width=float(msg.width),
        depth=float(msg.depth),
        height=float(msg.height),
    )


def _parse_json(payload):
    if not payload:
        return None
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _git_meta(workspace):
    meta = {"commit": "", "dirty": False, "porcelain": ""}
    try:
        meta["commit"] = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=workspace, text=True).strip()
        porcelain = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=workspace, text=True).strip()
        meta["porcelain"] = porcelain
        meta["dirty"] = bool(porcelain)
    except (OSError, subprocess.CalledProcessError):
        pass
    return meta


class PickRetreatEvalDriver(Node):

    def __init__(self, args):
        super().__init__(
            "pick_retreat_eval_driver",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ])
        self._args = args
        self._group = ReentrantCallbackGroup()
        self._status = {"payload": None, "recv": None}
        self._cloud = {"recv": None, "stamp": 0.0}
        self._cargo = {"recv": None, "stamp": 0.0}
        self._diag = {"payload": None}
        self._seg_stats = {"payload": None}
        self._size_eval = {"payload": None}
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._accuracy = DetectionAccuracy(
            tol_xy=args.tol_xy, tol_z=args.tol_z,
            tol_size=args.tol_size, tol_yaw=args.tol_yaw,
            tol_iou=args.tol_iou)

        latch = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            String, "/luggage/preprocessed/status",
            self._on_status, latch, callback_group=self._group)
        self.create_subscription(
            String, "/luggage_detector/diagnostics_json",
            lambda m: self._diag.__setitem__("payload", m.data),
            latch, callback_group=self._group)
        self.create_subscription(
            String, "/semantic_segmenter/stats_json",
            lambda m: self._seg_stats.__setitem__("payload", m.data),
            latch, callback_group=self._group)
        self.create_subscription(
            String, "/luggage/perception/size_eval/spawned",
            lambda m: self._size_eval.__setitem__("payload", m.data),
            latch, callback_group=self._group)
        self.create_subscription(
            PointCloud2, "/luggage/preprocessed/camera/depth/points",
            self._on_cloud, sensor, callback_group=self._group)
        self.create_subscription(
            PointCloud2, "/luggage/semantic/cargo_points",
            self._on_cargo, sensor, callback_group=self._group)

        self._spawn = self.create_client(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box",
            callback_group=self._group)
        self._clear = self.create_client(
            ClearCurrentBox, "/pickup_box_spawner/clear_current_box",
            callback_group=self._group)
        self._current = self.create_client(
            GetCurrentBox, "/pickup_box_spawner/get_current_box",
            callback_group=self._group)
        self._detect = self.create_client(
            DetectLuggage, "/luggage_detector/detect_luggage",
            callback_group=self._group)
        self._build = self.create_client(
            BuildMotionSequence, "/waypoint_generator/build_motion_sequence",
            callback_group=self._group)
        self._plan = ActionClient(
            self, PlanMotion, "/motion_planner/plan_motion",
            callback_group=self._group)
        self._goto = ActionClient(
            self, GoToRobotPose, "/motion_planner/go_to_robot_pose",
            callback_group=self._group)

    def _on_status(self, msg):
        self._status["payload"] = msg.data
        self._status["recv"] = time.time()

    def _on_cloud(self, msg):
        self._cloud["recv"] = time.time()
        stamp = msg.header.stamp
        self._cloud["stamp"] = float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def _on_cargo(self, msg):
        self._cargo["recv"] = time.time()
        stamp = msg.header.stamp
        self._cargo["stamp"] = float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    def call_srv(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=min(timeout, 15.0)):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        return future.result()

    def send_action(self, client, goal, timeout, name):
        if not client.wait_for_server(timeout_sec=min(timeout, 20.0)):
            return False, "%s server unavailable" % name, None
        accepted = threading.Event()
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda _f: accepted.set())
        if not accepted.wait(20.0):
            return False, "%s goal send timeout" % name, None
        handle = future.result()
        if handle is None or not handle.accepted:
            return False, "%s rejected" % name, None
        done = threading.Event()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            try:
                handle.cancel_goal()
            except Exception:
                pass
            return False, "%s timeout" % name, None
        wrapped = result_future.result()
        result = wrapped.result
        ok = (wrapped.status == GoalStatus.STATUS_SUCCEEDED
              and bool(getattr(result, "success", False)))
        return ok, str(getattr(result, "message", "")), result

    def _proc_count(self, pattern):
        try:
            out = subprocess.check_output(
                ["pgrep", "-c", "-f", pattern], text=True)
            return int(out.strip() or "0")
        except (OSError, subprocess.CalledProcessError, ValueError):
            return 0

    def graph_error(self):
        # Process counts, not DDS names: Fast-DDS keeps ghost participants
        # for tens of seconds after SIGTERM, which would false-positive a
        # ros2-node-list uniqueness check without actually dual-feeding
        # /camera/depth/points.
        for label, pattern in DUPLICATE_PROCS:
            count = self._proc_count(pattern)
            if count > 1:
                return "duplicate process %s (x%d); residual graph" % (
                    label, count)
        names = set(self.get_node_names())
        missing = [n for n in REQUIRED_NODES if n not in names]
        if missing:
            return "missing nodes: %s" % ", ".join(missing)
        n_bridge = self._proc_count("ros_gz_bridge/parameter_bridge")
        if n_bridge > 3:
            return "duplicate process parameter_bridge (x%d); residual graph" % (
                n_bridge)
        return ""

    def wait_graph(self, timeout=90.0):
        deadline = time.time() + timeout
        last = "not checked"
        while time.time() < deadline:
            err = self.graph_error()
            last = err
            if err.startswith("duplicate"):
                return err
            if not err:
                return ""
            time.sleep(1.0)
        return last or "graph wait timeout"

    def wait_geometry_ok(self, timeout, stamp0=None):
        wait_started = time.time()
        if stamp0 is None:
            snapshot = _parse_json(self._status["payload"]) or {}
            try:
                stamp0 = float(snapshot.get("primary_stamp") or 0.0)
            except (TypeError, ValueError):
                stamp0 = 0.0
        deadline = wait_started + timeout
        while time.time() < deadline:
            if wait_ready(
                    status_data=_parse_json(self._status["payload"]),
                    cloud_recv=self._cloud["recv"],
                    stamp_at_start=stamp0,
                    wait_started=wait_started,
                    now=time.time(),
                    fresh_sec=FRESH_SEC):
                return True
            time.sleep(0.05)
        return False

    def wait_new_cargo(self, stamp0, timeout):
        """Wait for a cargo cloud from a camera frame after *stamp0*."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if float(self._cargo["stamp"] or 0.0) > float(stamp0) + 1e-6:
                return True
            time.sleep(0.05)
        return False

    def suction_xyz(self, timeout=1.0):
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "world", "suction_contact_frame",
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout))
        except TransformException as exc:
            return None, str(exc)
        t = tf_msg.transform.translation
        return (float(t.x), float(t.y), float(t.z)), ""

    def ensure_clean(self):
        current = self.call_srv(
            self._current, GetCurrentBox.Request(), timeout=10.0)
        if current is None:
            return "GT_UNAVAILABLE:get_current timeout"
        if not current.success:
            return ""
        cleared = self.call_srv(
            self._clear, ClearCurrentBox.Request(), timeout=15.0)
        if cleared is None or not cleared.success:
            return "CLEAR_FAILED:%s" % (
                cleared.message if cleared else "timeout")
        return ""

    def run_trial(self, index):
        t0 = time.time()
        fields = {
            "index": index,
            "catalog_id": "",
            "visual_id": "",
            "detect_failure": "",
            "accuracy_ok": None,
            "accuracy_reason": "",
            "err_xy": None,
            "err_z": None,
            "err_width": None,
            "err_depth": None,
            "err_height": None,
            "iou": None,
            "segments_planned": 0,
            "segments_succeeded": 0,
            "segment_failures": (),
            "attach_xy_err": None,
            "attach_z_err": None,
            "attach_xy_gt": None,
            "attach_z_gt": None,
            "retreat_delta_z": None,
            "retreat_ok": None,
            "fail_code": "",
            "extras": {},
        }
        dirty = self.ensure_clean()
        if dirty:
            fields["fail_code"] = dirty.split(":")[0]
            fields["extras"]["clean"] = dirty
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        snapshot = _parse_json(self._status["payload"]) or {}
        try:
            stamp0 = float(snapshot.get("primary_stamp") or 0.0)
        except (TypeError, ValueError):
            stamp0 = 0.0

        spawn = self.call_srv(
            self._spawn, SpawnNextBox.Request(), timeout=30.0)
        if spawn is None or not spawn.success:
            fields["fail_code"] = "SPAWN_FAILED"
            fields["extras"]["spawn"] = (
                spawn.message if spawn else "timeout")
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        size_eval = _parse_json(self._size_eval["payload"]) or {}
        fields["catalog_id"] = str(size_eval.get("catalog_id") or "")
        fields["visual_id"] = str(size_eval.get("visual_id") or "")
        if not fields["catalog_id"] and spawn.box.id:
            parts = spawn.box.id.rsplit("_", 1)
            if len(parts) == 2:
                fields["catalog_id"] = parts[1]
        fields["extras"]["spawn_id"] = spawn.box.id

        goto_ok, goto_msg, _ = self.send_action(
            self._goto,
            GoToRobotPose.Goal(pose_name=self._args.observe_pose),
            timeout=self._args.goto_timeout,
            name="GoToRobotPose")
        if not goto_ok:
            fields["fail_code"] = "GOTO_FAILED"
            fields["extras"]["goto"] = goto_msg
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        if not self.wait_geometry_ok(self._args.geometry_timeout, stamp0=stamp0):
            fields["fail_code"] = "GEOMETRY_NOT_STABLE"
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        if not self.wait_new_cargo(stamp0, self._args.geometry_timeout):
            fields["fail_code"] = "CARGO_NOT_READY"
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        detect = self.call_srv(
            self._detect, DetectLuggage.Request(),
            timeout=self._args.detect_timeout)
        current = self.call_srv(
            self._current, GetCurrentBox.Request(), timeout=10.0)
        diag = self._diag["payload"]
        if detect is None:
            fields["fail_code"] = "DETECT_TIMEOUT"
            fields["detect_failure"] = "DETECT_TIMEOUT"
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        fields["detect_failure"] = detect.message or ""
        fields["extras"]["detect_message"] = detect.message
        fields["extras"]["backend"] = (
            (_parse_json(self._seg_stats["payload"]) or {}).get("backend"))

        if current is None or not current.success:
            fields["fail_code"] = "GT_UNAVAILABLE"
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        if is_gt_fallback(detect.message, diag):
            fields["fail_code"] = "DETECT_GT_FALLBACK"
            fields["detect_failure"] = perception_reason(
                detect.message, diag) or "DETECT_GT_FALLBACK"
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        if not is_perception_estimate(
                detect.success, detect.message, bool(detect.luggage), diag):
            fields["fail_code"] = detect.message or "MEASURED_NONE"
            fields["detect_failure"] = fields["fail_code"]
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        measured = observation_from_detected(detect.luggage[0])
        gt = observation_from_detected(current.box)
        result = self._accuracy.compare(measured, gt)
        fields["accuracy_ok"] = bool(result.ok)
        fields["accuracy_reason"] = result.reason
        fields["err_xy"] = result.err_xy
        fields["err_z"] = result.err_z
        fields["err_width"] = result.err_width
        fields["err_depth"] = result.err_depth
        fields["err_height"] = result.err_height
        fields["iou"] = result.iou
        fields["extras"]["measured"] = measured.__dict__
        fields["extras"]["gt"] = gt.__dict__

        if not result.ok:
            fields["fail_code"] = "DETECT_GATE:%s" % result.reason
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        req = BuildMotionSequence.Request()
        req.phase = "pick"
        req.pick = detect.luggage[0]
        built = self.call_srv(self._build, req, timeout=15.0)
        if built is None or not built.success:
            fields["fail_code"] = "BUILD_FAILED"
            fields["extras"]["build"] = (
                built.message if built else "timeout")
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
            fields["wall_time_sec"] = time.time() - t0
            return TrialRecord(**fields)

        segments = list(built.segments)
        fields["segments_planned"] = len(segments)
        attach_z = None
        attach_xyz = None
        retreat_xyz = None
        failures = []
        succeeded = 0
        for segment in segments:
            goal = PlanMotion.Goal()
            goal.segment = segment
            ok, message, _res = self.send_action(
                self._plan, goal,
                timeout=self._args.plan_timeout,
                name="PlanMotion:%s" % segment.name)
            if ok:
                succeeded += 1
            else:
                failures.append((str(segment.name), message))
                fields["fail_code"] = "PLAN_%s" % segment.name
                fields["extras"]["plan_%s" % segment.name] = message
                break
            if segment.name == "attach":
                attach_xyz, tf_err = self.suction_xyz()
                attach_z = (
                    attach_xyz[2] if attach_xyz is not None
                    else float(segment.target_pose.position.z))
                if attach_xyz is not None:
                    fields["attach_xy_err"] = math.hypot(
                        attach_xyz[0] - measured.x,
                        attach_xyz[1] - measured.y)
                    box_top = measured.z + 0.5 * measured.height
                    fields["attach_z_err"] = attach_xyz[2] - box_top
                    fields["attach_xy_gt"] = math.hypot(
                        attach_xyz[0] - gt.x, attach_xyz[1] - gt.y)
                    fields["attach_z_gt"] = (
                        attach_xyz[2] - (gt.z + 0.5 * gt.height))
                else:
                    fields["extras"]["attach_tf"] = tf_err
            if segment.name == "pick_retreat":
                retreat_xyz, tf_err = self.suction_xyz()
                if retreat_xyz is None:
                    fields["extras"]["retreat_tf"] = tf_err
        fields["segments_succeeded"] = succeeded
        fields["segment_failures"] = tuple(failures)

        if attach_xyz is not None and retreat_xyz is not None:
            fields["retreat_delta_z"] = retreat_xyz[2] - attach_xyz[2]
        elif attach_z is not None and retreat_xyz is not None:
            fields["retreat_delta_z"] = retreat_xyz[2] - attach_z
        if fields["retreat_delta_z"] is not None:
            fields["retreat_ok"] = (
                abs(fields["retreat_delta_z"] - self._args.expected_retreat_dz)
                <= self._args.retreat_tol)
            if not failures and not fields["retreat_ok"]:
                fields["fail_code"] = "RETREAT_HEIGHT"

        self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
        fields["wall_time_sec"] = time.time() - t0
        return TrialRecord(**fields)


def write_jsonl_line(path, record):
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(json.dumps(trial_to_dict(record), sort_keys=True) + "\n")
        handle.flush()


def write_summary(out_dir, records, args, extra):
    summary = summarize(records, expected_retreat_dz=args.expected_retreat_dz)
    summary.update(extra)
    path = os.path.join(out_dir, "summary.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
    return summary


def format_md(summary, records, extra):
    lines = [
        "# Closed-loop pick / retreat eval",
        "",
        "- Date: %s" % extra.get("date", ""),
        "- Commit: `%s`%s" % (
            extra.get("commit", ""),
            " (dirty)" if extra.get("dirty") else ""),
        "- `ROS_DOMAIN_ID`: %s" % extra.get("ros_domain_id", ""),
        "- N requested / completed: %s / %s" % (
            extra.get("n_requested"), summary.get("n")),
        "- Config: `use_semantic=%s` visual_kind=%s sequence_ids=%s "
        "observe=%s backend=%s" % (
            extra.get("use_semantic"), extra.get("visual_kind"),
            extra.get("sequence_ids"), extra.get("observe_pose"),
            extra.get("backend") or "unknown"),
        "- Detection tols: xy=%.3f z=%.3f size=%.3f yaw=%.3f iou=%.2f" % (
            extra.get("tol_xy", 0), extra.get("tol_z", 0),
            extra.get("tol_size", 0), extra.get("tol_yaw", 0),
            extra.get("tol_iou", 0)),
        "- Retreat check: expected ΔZ=%.3f m, tol=%.3f m" % (
            extra.get("expected_retreat_dz", 0.35),
            extra.get("retreat_tol", 0.08)),
        "",
        "## Rates (independent denominators)",
        "",
        "| Metric | Rate | n |",
        "|---|---|---|",
        "| Detect pass | %.1f%% | %s / %s |" % (
            100.0 * summary["detect_pass_rate"],
            summary["n_detect_ok"], summary["n_detect_compared"]),
        "| Plan pass (4 segments, among detect-pass) | %.1f%% | %s / %s |" % (
            100.0 * summary["plan_pass_rate"],
            summary["n_plan_ok"], summary["n_detect_ok"]),
        "| Retreat height (among plan-pass) | %.1f%% | %s / %s |" % (
            100.0 * summary["retreat_pass_rate"],
            summary["n_retreat_ok"], summary["n_plan_ok"]),
        "| End-to-end pick (retreat-ok / N) | %.1f%% | %s / %s |" % (
            100.0 * summary["pick_pass_rate"],
            summary["n_retreat_ok"], summary["n"]),
        "",
        "## Geometric pick (attach, no vacuum)",
        "",
        "Attach XY error vs measured box centre; Z vs measured box top.",
        "",
        "| | n | mean | std | p50 | p95 |",
        "|---|---|---|---|---|---|",
    ]
    for key, label in (
            ("attach_xy_gt", "attach XY vs GT (m)"),
            ("attach_z_gt_abs", "attach |Z| vs GT (m)"),
            ("attach_xy", "attach XY vs measured (m)"),
            ("attach_z_abs", "attach |Z| vs measured (m)"),
            ("retreat_delta_z", "retreat ΔZ (m)")):
        slot = summary.get(key) or {}
        def _fmt(v):
            return "—" if v is None else "%.4f" % v
        lines.append("| %s | %s | %s | %s | %s | %s |" % (
            label, slot.get("n", 0),
            _fmt(slot.get("mean")), _fmt(slot.get("std")),
            _fmt(slot.get("p50")), _fmt(slot.get("p95"))))
    lines.extend(["", "## Fail codes", "", "```json",
                  json.dumps(summary.get("fail_codes") or {}, indent=2),
                  "```", "", "## Per catalog", "",
                  "```json",
                  json.dumps(summary.get("by_catalog") or {}, indent=2),
                  "```", "", "## Per visual", "",
                  "```json",
                  json.dumps(summary.get("by_visual") or {}, indent=2),
                  "```", "", "## Trials", "",
                  "| i | catalog | visual | detect | plan | retreat | "
                  "attach XY | ΔZ | fail |",
                  "|---|---|---|---|---|---|---|---|---|"])
    for rec in records:
        lines.append(
            "| %02d | %s | %s | %s | %s/%s | %s | %s | %s | %s |" % (
                rec.index, rec.catalog_id or "?", rec.visual_id or "?",
                "ok" if rec.accuracy_ok else (
                    rec.detect_failure or rec.accuracy_reason or "—"),
                rec.segments_succeeded, rec.segments_planned,
                "ok" if rec.retreat_ok else (
                    "—" if rec.retreat_ok is None else "fail"),
                   "—" if rec.attach_xy_gt is None else "%.3f" % rec.attach_xy_gt,
                "—" if rec.retreat_delta_z is None else "%.3f" % rec.retreat_delta_z,
                rec.fail_code or "",
            ))
    lines.extend([
        "",
        "## Known deviations",
        "",
        "- No vacuum / ACM attach: a successful pick is geometric "
        "(PlanMotion success + retreat ΔZ), not grasp retention.",
        "- MoveIt planning scene does not contain the suitcase collision "
        "object; attach may physically contact the box in Gazebo.",
        "- `keep_camera_down` / `lock_wrist` are not implemented.",
        "- Detection tols were not retuned for this run.",
        "",
    ])
    return "\n".join(lines) + "\n"


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", type=int, default=20)
    parser.add_argument("--out", default=os.path.normpath(DEFAULT_OUT))
    parser.add_argument("--observe-pose", default="pickup_observe")
    parser.add_argument("--expected-retreat-dz", type=float, default=0.35)
    parser.add_argument("--retreat-tol", type=float, default=0.08)
    parser.add_argument("--geometry-timeout", type=float, default=15.0)
    parser.add_argument("--detect-timeout", type=float, default=15.0)
    parser.add_argument("--plan-timeout", type=float, default=60.0)
    parser.add_argument("--goto-timeout", type=float, default=30.0)
    parser.add_argument("--tol-xy", type=float, default=0.03)
    parser.add_argument("--tol-z", type=float, default=0.02)
    parser.add_argument("--tol-size", type=float, default=0.05)
    parser.add_argument("--tol-yaw", type=float, default=0.15)
    parser.add_argument("--tol-iou", type=float, default=0.60)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-graph-check", action="store_true")
    return parser.parse_args(argv)


def dry_run(args):
    rec = TrialRecord(
        index=0, catalog_id="standard", visual_id="vintage",
        accuracy_ok=True, accuracy_reason="ok",
        segments_planned=4, segments_succeeded=4,
        retreat_ok=True, retreat_delta_z=0.35,
        attach_xy_err=0.01, attach_z_err=0.0,
        wall_time_sec=1.0)
    os.makedirs(args.out, exist_ok=True)
    jsonl = os.path.join(args.out, "trials.jsonl")
    with open(jsonl, "w", encoding="utf-8"):
        pass
    write_jsonl_line(jsonl, rec)
    extra = {
        "date": time.strftime("%Y-%m-%d"),
        "n_requested": args.n, "dry_run": True,
        "expected_retreat_dz": args.expected_retreat_dz,
        "retreat_tol": args.retreat_tol,
        "tol_xy": args.tol_xy, "tol_z": args.tol_z,
        "tol_size": args.tol_size, "tol_yaw": args.tol_yaw,
        "tol_iou": args.tol_iou,
    }
    summary = write_summary(args.out, [rec], args, extra)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    if args.dry_run:
        return dry_run(args)

    os.makedirs(args.out, exist_ok=True)
    jsonl = os.path.join(args.out, "trials.jsonl")
    with open(jsonl, "w", encoding="utf-8"):
        pass

    workspace = os.path.normpath(os.path.join(_PKG_ROOT, "..", ".."))
    extra = _git_meta(workspace)
    extra.update({
        "date": time.strftime("%Y-%m-%d"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "n_requested": args.n,
        "observe_pose": args.observe_pose,
        "expected_retreat_dz": args.expected_retreat_dz,
        "retreat_tol": args.retreat_tol,
        "tol_xy": args.tol_xy, "tol_z": args.tol_z,
        "tol_size": args.tol_size, "tol_yaw": args.tol_yaw,
        "tol_iou": args.tol_iou,
        "use_semantic": True,
        "visual_kind": os.environ.get("PICK_EVAL_VISUAL_KIND", ""),
        "sequence_ids": os.environ.get("PICK_EVAL_SEQUENCE_IDS", ""),
        "interrupted": False,
        "graph_error": "",
        "backend": "",
    })

    rclpy.init()
    node = PickRetreatEvalDriver(args)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    records = []
    stop = {"flag": False}

    def _on_sigint(_signum, _frame):
        stop["flag"] = True
        extra["interrupted"] = True
        print("interrupt: flushing %d trials" % len(records), flush=True)

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        if not args.skip_graph_check:
            err = node.wait_graph(timeout=90.0)
            extra["graph_error"] = err
            if err:
                print("graph check failed: %s" % err, flush=True)
                write_summary(args.out, records, args, extra)
                return 2
        print("graph ok; running %d trials" % args.n, flush=True)
        for index in range(args.n):
            if stop["flag"] or not rclpy.ok():
                break
            rec = node.run_trial(index)
            records.append(rec)
            write_jsonl_line(jsonl, rec)
            backend = rec.extras.get("backend")
            if backend:
                extra["backend"] = backend
            print(
                "trial %02d catalog=%s visual=%s detect=%s plan=%s/%s "
                "retreat=%s xy=%s fail=%s (%.1fs)"
                % (rec.index, rec.catalog_id, rec.visual_id,
                   rec.accuracy_ok, rec.segments_succeeded,
                   rec.segments_planned, rec.retreat_ok,
                   "—" if rec.attach_xy_gt is None else "%.3f" % rec.attach_xy_gt,
                   rec.fail_code, rec.wall_time_sec),
                flush=True)
        summary = write_summary(args.out, records, args, extra)
        md = format_md(summary, records, extra)
        md_path = os.path.join(args.out, "closed_loop_eval.md")
        with open(md_path, "w", encoding="utf-8") as handle:
            handle.write(md)
        print("summary detect=%.1f%% plan=%.1f%% retreat=%.1f%% pick=%.1f%%"
              % (100 * summary["detect_pass_rate"],
                 100 * summary["plan_pass_rate"],
                 100 * summary["retreat_pass_rate"],
                 100 * summary["pick_pass_rate"]), flush=True)
        print("wrote %s" % args.out, flush=True)
        return 0
    finally:
        if records:
            write_summary(args.out, records, args, extra)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
