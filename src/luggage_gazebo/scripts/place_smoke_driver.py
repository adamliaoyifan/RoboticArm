#!/usr/bin/env python3
"""Slice A place smoke: pick-carry into a known container slot.

State machine and dump layout: docs/plans/todo5_place_action.md.
Does not modify pick_retreat_eval_driver.py. Reuses its graph / pick helpers
by subclassing, then continues into place instead of vacuum-off + observe.

Order after descend: RELEASE_SETTLE → VacuumCommand(false) → retreat →
cargo-map commit ([G3 wave5] the slot-named scene AddPlacedBox channel is
removed; occupancy columns are the single rigid-obstacle source).
"""

from __future__ import division

import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
from collections import deque

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import PlanningSceneComponents
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from moveit_msgs.srv import GetPlanningScene
from luggage_msgs.action import PlanMotion
from luggage_msgs.msg import DetectedLuggage, MotionSegment, SlotSpec
from luggage_msgs.srv import (
    BuildMotionSequence,
    CheckExecutionAdmission,
    ClearCurrentBox,
    DetectLuggage,
    GetCurrentBox,
    SpawnNextBox,
)

from luggage_description.scene_tf_config_utils import (
    container_inner_floor_z,
    container_opening_normal_in_world,
    container_opening_target_point_in_world,
    load_scene_tf_config,
    origin_in_world,
    point_inside_container_inner_box,
    point_inside_opening_aperture,
    resolve_scene_tf_config_path,
    xyz_world_to_base_link,
    yaw_world_to_base_link,
)
from luggage_description.scene_tf_publisher import rpy_to_quaternion
from luggage_description.suitcase_visual import pickup_visual_sdf
from luggage_planning.motion_executor import MotionExecutor
from luggage_planning.motion_boundary import (
    load_latest_boundary,
    replay_manifest,
)
from luggage_planning.planning_scene_client import summarize_planning_scene
from luggage_planning.waypoint_generator import nearest_box_yaw, tool_down_yaw

from luggage_gazebo.pose_text import PoseTextDecoder, parse_pose_samples
from luggage_gazebo.capture_budget import CONTROLLER_STATE_TOPIC
from luggage_gazebo.handoff_settle import (
    HandoffPolicy,
    await_handoff,
    summarize_for_extras,
)
from luggage_gazebo.place_metrics import (
    SETTLE_BUDGET_SEC,
    SETTLE_MOVE_M,
    SETTLE_WINDOW_SEC,
    PlaceTrial,
    parse_ign_model_pose,
    place_ok,
    settle_verdict,
    summarize,
    trial_to_dict,
    xyz_separation,
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

PLACE_CORE = ("transit", "traverse", "insert", "descend", "retreat")
# Legs that still hold the box. The attached check applies only here,
# and only before a successful detach. Explicit
# ``requires_payload_attached`` or a certified ``load_state`` wins over
# these names.
_CARRYING_LEG_NAMES = frozenset((
    "transit", "traverse", "insert", "descend",
))
# Legs after the box is supposed to be down. ``attached == false`` is
# the expected vacuum state, not a lost payload.
_UNLOADED_LEG_NAMES = frozenset((
    "retreat", "return", "place_exit", "home",
))
GZ_WORLD = "airport_loading"

# [ST09-I S5.7] Safety margin in the DERIVED opening-corridor depth:
# corridor = wall thickness ((outer-inner)/2, scene data)
#          + TCP/camera envelope (|T_tcp_camera|, live static TF; the
#            fallback below is the URDF-derived conservative bound)
#          + this margin. Deploy numbers: 0.04 + ~0.27 + 0.10 ~ 0.41 m —
# the pickup observe (1.70 m outside the face) classifies outside while
# the mouth (+0.055 m) stays in the corridor.
APERTURE_ESCAPE_MARGIN_M = 0.10
CAMERA_ENVELOPE_FALLBACK_M = 0.30

# [ST09-I G0-1] Receive-time freshness the exit-proof adjudication
# demands of /vacuum/state — mirrors the selector's
# route_producer.VACUUM_STATE_FRESHNESS_S (the message carries no
# stamp, so receive time on the node clock is the only basis).
PORTAL_PROOF_VACUUM_FRESHNESS_S = 2.0

# Match elfin_controllers_sim.yaml goal_time. The previous 80-sample ring
# was ~1.6 s at 50 Hz and froze after a 15 s abort. Keep at least this
# many seconds; grow if a planned segment plus goal_time plus tail is longer.
JOINT_RING_FLOOR_SEC = 20.0
JOINT_RING_GOAL_TIME_SEC = 5.0
JOINT_RING_TAIL_SEC = 2.0
# Bound the ring against a segment that hangs until the execute timeout.
JOINT_RING_CEILING_SEC = 120.0


def joint_ring_horizon_sec(segment_duration=None):
    """Seconds of joint history to retain for a segment of this length.

    The planned duration is not known when a segment starts - the driver only
    reads it back from the boundary dump after the action returns - so the
    caller grows the horizon from elapsed segment time while the segment runs.
    The 1838 place_exit ran 24.3 s against the bare 20 s floor and lost its
    first 4.35 s, which is the start of the execute a divergence is measured
    against.
    """
    duration = 0.0 if segment_duration is None else float(segment_duration)
    return min(
        JOINT_RING_CEILING_SEC,
        max(JOINT_RING_FLOOR_SEC,
            duration + JOINT_RING_GOAL_TIME_SEC + JOINT_RING_TAIL_SEC))


def prune_joint_ring(ring, now_stamp, horizon_sec):
    cutoff = float(now_stamp) - float(horizon_sec)
    while ring and float(ring[0].get("stamp") or 0.0) < cutoff:
        ring.popleft()


def joint_ring_from(ring, t0):
    rows = list(ring or ())
    if t0 is None:
        return rows
    start = float(t0)
    return [row for row in rows if float(row.get("stamp") or 0.0) >= start]


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


def segment_requires_payload_attached(segment, load_state=None, detached=False):
    """Whether this leg may start only while vacuum still holds the box.

    ``requires_payload_attached`` on the segment is the contract. Else a
    certified ``load_state`` (``carrying`` / ``unloaded``). Else the leg
    name. After a successful detach, a leg without an explicit True
    requirement does not need ``attached``.
    """
    explicit = getattr(segment, "requires_payload_attached", None)
    if explicit is not None:
        return bool(explicit)
    if detached:
        return False
    if load_state is not None and str(load_state) != "":
        return str(load_state) == "carrying"
    name = str(getattr(segment, "name", "") or "")
    if name in _UNLOADED_LEG_NAMES:
        return False
    if name in _CARRYING_LEG_NAMES:
        return True
    return True


def payload_attachment_gate(segment, vacuum_state, use_vacuum=True,
                            detached=False, load_state=None):
    """Pre-leg vacuum gate.

    Returns ``(ok, code)``. A carrying leg with ``attached`` false or a
    ``fail_reason`` is ``PLACE_LOST_PAYLOAD``. An unloaded leg
    (retreat, return, home, or any leg after a successful detach) may
    run with ``attached`` false. A non-empty ``fail_reason`` on that
    leg is a release/vacuum failure, ``VACUUM_DETACH:<reason>``.
    """
    if not use_vacuum:
        return True, ""
    state = dict(vacuum_state or {})
    reason = str(state.get("fail_reason") or "")
    attached = bool(state.get("attached"))
    requires = segment_requires_payload_attached(
        segment, load_state=load_state, detached=detached)
    if requires:
        if reason or not attached:
            return False, "PLACE_LOST_PAYLOAD"
        return True, ""
    if reason:
        return False, "VACUUM_DETACH:%s" % reason
    return True, ""


# Same gate as motion_executor._execute_timed_joint_v2: drift greater than
# this refuses the timed path. Do not raise it; wait for the arm instead.
QP_START_TOLERANCE_RAD = 0.01
CANONICAL_ARM_JOINTS = tuple("elfin_joint%d" % i for i in range(1, 7))
_RETREAT_START_WAIT_SEC = 3.0


def _angular_gap(left, right):
    """Shortest absolute joint distance, matching the executor QP gate."""
    gap = abs(float(left) - float(right))
    return min(gap, (2.0 * math.pi) - gap)


def canonical_joint_positions(names, positions, wanted=CANONICAL_ARM_JOINTS):
    """Order ``positions`` as elfin_joint1..6. None when the names cannot."""
    if names is None or positions is None:
        return None
    try:
        names = [str(name) for name in names]
        values = [float(value) for value in positions]
    except (TypeError, ValueError):
        return None
    if len(names) != len(values) or len(names) != len(wanted):
        return None
    if len(set(names)) != len(names):
        return None
    if tuple(names) == tuple(wanted):
        return values
    index = dict(zip(names, values))
    if any(name not in index for name in wanted):
        return None
    return [index[name] for name in wanted]


def trajectory_endpoint(segment, which):
    """``(names, positions)`` of the first or last timed knot, or Nones."""
    if segment is None or which not in ("first", "last"):
        return None, None
    if isinstance(segment, dict):
        joint = segment.get("joint_trajectory")
    else:
        joint = getattr(segment, "joint_trajectory", None)
    if joint is None:
        return None, None
    if isinstance(joint, dict):
        names = list(joint.get("joint_names") or [])
        points = list(joint.get("points") or [])
        if not points:
            return names, None
        point = points[0] if which == "first" else points[-1]
        if isinstance(point, dict):
            raw = point.get("positions")
        else:
            raw = point
        if raw is None:
            return names, None
        return names, [float(value) for value in raw]
    trajectory = getattr(joint, "trajectory", None)
    if trajectory is None:
        return None, None
    names = list(trajectory.joint_names)
    points = list(trajectory.points)
    if not points:
        return names, None
    point = points[0] if which == "first" else points[-1]
    return names, [float(value) for value in point.positions]


def trajectory_dump(segment, max_points=5000):
    """Lossless submitted-trajectory record: names and every timed point.

    Capture only. ``t`` is ``time_from_start`` in seconds; ``q`` and ``v``
    are the point's positions and velocities (``v`` None when absent).
    Returns None when the segment carries no timed trajectory. A trajectory
    longer than ``max_points`` is cut and marked ``truncated``.
    """
    if segment is None:
        return None
    if isinstance(segment, dict):
        joint = segment.get("joint_trajectory")
    else:
        joint = getattr(segment, "joint_trajectory", None)
    if joint is None:
        return None
    if isinstance(joint, dict):
        names = list(joint.get("joint_names") or [])
        raw = list(joint.get("points") or [])
    else:
        trajectory = getattr(joint, "trajectory", None)
        if trajectory is None:
            return None
        names = list(trajectory.joint_names)
        raw = list(trajectory.points)
    points = []
    for point in raw[:int(max_points)]:
        if isinstance(point, dict):
            q = point.get("positions")
            v = point.get("velocities")
            t = point.get("time_from_start", point.get("t"))
            if isinstance(t, dict):
                t = float(t.get("sec", 0)) + 1e-9 * float(t.get("nanosec", 0))
        else:
            q = point.positions
            v = point.velocities
            stamp = point.time_from_start
            t = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
        points.append({
            "t": None if t is None else round(float(t), 6),
            "q": None if q is None else [float(x) for x in q],
            "v": [float(x) for x in v] if v else None,
        })
    return {
        "joint_names": [str(n) for n in names],
        "n_points": len(raw),
        "truncated": len(raw) > int(max_points),
        "points": points,
    }


def _round_joints(values):
    if values is None:
        return None
    return [round(float(value), 6) for value in values]


def retreat_start_decision(samples, detach_stamp, retreat_names,
                           retreat_positions, descend_names,
                           descend_positions,
                           tolerance=QP_START_TOLERANCE_RAD):
    """Decide whether retreat may submit against the descend-final contract.

    A sample counts only when its stamp is strictly after ``detach_stamp``.
    Ready requires that retreat's first knot and descend's last knot agree
    within ``tolerance``, and that the newest post-detach joint state is
    within ``tolerance`` of both. The tolerance is the executor QP gate.
    """
    tolerance = float(tolerance)
    record = {
        "tolerance_rad": tolerance,
        "detach_stamp": (
            None if detach_stamp is None else float(detach_stamp)),
        "retreat_first_joints": None,
        "descend_final_joints": None,
        "joint_states": None,
        "joint_names": list(CANONICAL_ARM_JOINTS),
        "joint_state_stamp": None,
        "stamp_after_detach": False,
        "delta_per_joint": None,
        "max_delta": None,
        "delta_vs_retreat_per_joint": None,
        "max_delta_vs_retreat": None,
        "status": "missing_contract",
    }
    if retreat_positions is None or descend_positions is None:
        return record
    retreat = canonical_joint_positions(retreat_names, retreat_positions)
    descend = canonical_joint_positions(descend_names, descend_positions)
    if retreat is None or descend is None:
        record["status"] = "joint_order"
        return record
    record["retreat_first_joints"] = _round_joints(retreat)
    record["descend_final_joints"] = _round_joints(descend)
    contract_delta = [
        _angular_gap(left, right) for left, right in zip(retreat, descend)]
    record["contract_delta_per_joint"] = _round_joints(contract_delta)
    record["contract_max_delta"] = round(max(contract_delta), 6)

    ordered = []
    for sample in samples or []:
        if not isinstance(sample, dict):
            continue
        try:
            stamp = float(sample.get("stamp"))
        except (TypeError, ValueError):
            continue
        ordered.append((stamp, sample))
    newest = ordered[-1][1] if ordered else None
    if newest is not None:
        live = canonical_joint_positions(
            newest.get("name"), newest.get("position"))
        record["joint_state_stamp"] = float(newest.get("stamp"))
        record["stamp_after_detach"] = bool(
            detach_stamp is not None
            and float(newest.get("stamp")) > float(detach_stamp))
        if live is not None:
            record["joint_states"] = _round_joints(live)

    if max(contract_delta) > tolerance:
        record["status"] = "contract_mismatch"
        record["delta_per_joint"] = record["contract_delta_per_joint"]
        record["max_delta"] = record["contract_max_delta"]
        return record

    fresh = [
        sample for stamp, sample in ordered
        if detach_stamp is not None and stamp > float(detach_stamp)]
    if not fresh:
        record["status"] = "stale_joint_state"
        if newest is not None:
            held = canonical_joint_positions(
                newest.get("name"), newest.get("position"))
            if held is not None:
                versus_descend = [
                    _angular_gap(left, right)
                    for left, right in zip(held, descend)]
                versus_retreat = [
                    _angular_gap(left, right)
                    for left, right in zip(held, retreat)]
                record["delta_per_joint"] = _round_joints(versus_descend)
                record["max_delta"] = round(max(versus_descend), 6)
                record["delta_vs_retreat_per_joint"] = _round_joints(
                    versus_retreat)
                record["max_delta_vs_retreat"] = round(max(versus_retreat), 6)
        return record
    sample = max(fresh, key=lambda item: float(item.get("stamp")))
    live = canonical_joint_positions(sample.get("name"), sample.get("position"))
    record["joint_state_stamp"] = float(sample.get("stamp"))
    record["stamp_after_detach"] = True
    if live is None:
        record["status"] = "joint_order"
        record["joint_states"] = None
        record["joint_states_raw_names"] = [
            str(name) for name in (sample.get("name") or [])]
        return record
    versus_descend = [
        _angular_gap(left, right) for left, right in zip(live, descend)]
    versus_retreat = [
        _angular_gap(left, right) for left, right in zip(live, retreat)]
    record["joint_states"] = _round_joints(live)
    record["delta_per_joint"] = _round_joints(versus_descend)
    record["max_delta"] = round(max(versus_descend), 6)
    record["delta_vs_retreat_per_joint"] = _round_joints(versus_retreat)
    record["max_delta_vs_retreat"] = round(max(versus_retreat), 6)
    if (max(versus_descend) > tolerance) or (max(versus_retreat) > tolerance):
        record["status"] = "mismatch"
        return record
    record["status"] = "ready"
    return record


def detach_joint_motion(before, after):
    """Change between two joint samples. ``moved`` is a gap above 1e-4 rad."""
    out = {
        "before_stamp": None,
        "after_stamp": None,
        "delta_per_joint": None,
        "max_delta": None,
        "moved": None,
    }
    if not isinstance(before, dict) or not isinstance(after, dict):
        return out
    try:
        out["before_stamp"] = float(before.get("stamp"))
        out["after_stamp"] = float(after.get("stamp"))
    except (TypeError, ValueError):
        return out
    left = canonical_joint_positions(before.get("name"), before.get("position"))
    right = canonical_joint_positions(after.get("name"), after.get("position"))
    if left is None or right is None:
        return out
    deltas = [_angular_gap(a, b) for a, b in zip(left, right)]
    out["delta_per_joint"] = _round_joints(deltas)
    out["max_delta"] = round(max(deltas), 6)
    out["moved"] = bool(max(deltas) > 1e-4)
    return out


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


class ReleasePoseStream:
    """[r7 release-boundary fix] Continuous pose/info subscription cache.

    The release boundary used to read the box pose via one-shot
    `ign model -m <name> --pose` CLI calls — the read failed with no
    diagnostic twice (r4 box0, r6 box1: "no release pose" 4x fast
    empty). This class keeps a daemon thread reading
    `ign topic -e -t /world/<w>/pose/info` and caches the LATEST pose
    per model with (sim stamp ns, wall receive s).

    Freshness contract (r7 frozen thresholds): sim age <= 100 ms,
    wall-receive age <= 500 ms; a FUTURE stamp or a stamp older than
    the epoch floor (first stamp the cache ever observed, minus
    tolerance) is a previous-instance / time-jump-back message and is
    REJECTED — never served. A missing or stale sample refuses the
    release by name; there is no fallback read and no reuse of
    pre-delete samples after the swap (the frozen pose is the value
    read BEFORE the static delete).
    """

    SIM_AGE_MAX_S = 0.100
    WALL_AGE_MAX_S = 0.500
    EPOCH_TOL_S = 0.250

    def __init__(self, world=GZ_WORLD, ign_bin=None):
        self._world = str(world)
        self._ign = ign_bin or _ign_model_bin()
        self._lock = threading.Lock()
        self._latest = {}        # name -> (stamp_ns, pose7, recv_wall)
        self._epoch_floor_ns = None
        self._proc = None
        self._thread = None
        self._stop = threading.Event()
        self._decoder = PoseTextDecoder()
        self.errors = self._decoder.errors

    # -- lifecycle ------------------------------------------------------
    def start(self):
        if self._thread is not None:
            return self
        topic = "/world/%s/pose/info" % self._world
        self._proc = subprocess.Popen(
            [self._ign, "topic", "-e", "-t", topic],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, errors="replace")
        self._thread = threading.Thread(
            target=self._read_loop, daemon=True,
            name="release_pose_stream")
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._proc is not None:
            try:
                self._proc.terminate()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def _read_loop(self):
        try:
            for line in self._proc.stdout:
                if self._stop.is_set():
                    break
                self._ingest_line(line)
        except (OSError, ValueError) as exc:  # keep the cache; record why
            self.errors.append({"reason": "reader_error", "detail": repr(exc)})
        finally:
            with self._lock:
                self._decoder.finish()

    # -- parsing --------------------------------------------------------
    def _ingest_line(self, line):
        with self._lock:
            for sample in self._decoder.feed_line(line):
                if (self._epoch_floor_ns is None
                        or sample.stamp_ns < self._epoch_floor_ns):
                    self._epoch_floor_ns = (
                        sample.stamp_ns - int(self.EPOCH_TOL_S * 1e9))
                self._latest[sample.model] = (
                    sample.stamp_ns, list(sample.pose7), time.monotonic())

    # -- reads ----------------------------------------------------------
    def latest(self, model):
        with self._lock:
            return self._latest.get(str(model))

    def read_release_pose(self, model, now_sim_s,
                          timeout_s=2.0, wall_now=None):
        """(pose7|None, record). Refuses stale/missing/future/old-epoch
        samples by name; never falls back."""
        wall = wall_now or time.monotonic
        deadline = time.monotonic() + float(timeout_s)
        reason = None
        while True:
            with self._lock:
                entry = self._latest.get(str(model))
                floor = self._epoch_floor_ns
                decoder_evidence = self._decoder.diagnostics()
                parse_reason = self._decoder.last_rejections.get(str(model))
            record = {"model": str(model), "epoch_floor_ns": floor,
                      "decoder": decoder_evidence}
            if entry is None:
                reason = parse_reason or "no_message"
            else:
                stamp_ns, pose6, recv_wall = entry
                now_ns = int(float(now_sim_s) * 1e9)
                sim_age_s = (now_ns - stamp_ns) / 1e9
                wall_age_s = wall() - recv_wall
                record.update({
                    "stamp_ns": stamp_ns,
                    "sim_age_ms": round(sim_age_s * 1e3, 1),
                    "wall_age_ms": round(wall_age_s * 1e3, 1),
                })
                if sim_age_s < -self.EPOCH_TOL_S:
                    reason = "future_stamp"
                elif floor is not None and stamp_ns < floor:
                    reason = "old_epoch_stamp"
                elif sim_age_s > self.SIM_AGE_MAX_S:
                    reason = "stale_sim_age"
                elif wall_age_s > self.WALL_AGE_MAX_S:
                    reason = "stale_wall_age"
                else:
                    record["verdict"] = "fresh"
                    return list(pose6), record
            if time.monotonic() >= deadline:
                record["verdict"] = "refused"
                record["reason"] = reason or "no_message"
                return None, record
            time.sleep(0.02)


def parse_pose_stream_sample(text, model):
    """Replay the same Pose_V text decoder used by the live release cache."""
    return parse_pose_samples(text, model)


def _quat7_to_rpy6(pose7):
    """[x,y,z,qx,qy,qz,qw] -> [x,y,z,roll,pitch,yaw] (ZYX, clamped)."""
    x, y, z, qx, qy, qz, qw = [float(v) for v in pose7]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, 1.0) if abs(sinp) >= 1.0 \
        else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [x, y, z, roll, pitch, yaw]


def _quat_angle_deg(qa, qb):
    """Geodesic angle between two xyzw quaternions, degrees."""
    dot = abs(sum(float(a) * float(b)
                  for a, b in zip(qa, qb)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def _first_dynamic_state(model, timeout_s=3.0, world=GZ_WORLD):
    """First two dynamic_pose/info samples of `model` after the swap.

    Returns a record with the first pose (spawn-consistency evidence:
    compare vs the spawn request in the verdict), the finite-difference
    first-step velocity (the stream carries no twist fields), or a
    named gap. Evidence-only: it never gates the motion itself.
    """
    try:
        proc = subprocess.run(
            [_ign_model_bin(), "topic", "-e", "-t",
             "/world/%s/dynamic_pose/info" % world, "--num", "60"],
            capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "reason": str(exc)}
    samples = parse_pose_stream_sample(proc.stdout or "", model)
    if len(samples) < 2:
        return {"available": False, "reason": "insufficient_samples",
                "n": len(samples)}
    (t0, p0), (t1, p1) = samples[0], samples[1]
    dt = (t1 - t0) / 1e9
    if dt <= 0:
        return {"available": False, "reason": "non_monotone_stamps"}
    vel = [(b - a) / dt for a, b in zip(p0[:3], p1[:3])]
    return {
        "available": True, "t_sim_ns": [t0, t1],
        "pose6": [round(v, 6) for v in p0],
        "dt_s": round(dt, 4),
        "first_step_velocity_mps": [round(v, 5) for v in vel],
        "velocity_method": "finite_difference(pose_stream)",
    }


def _parse_synthetic_size(text):
    parts = [p.strip() for p in str(text or "").split(",") if p.strip()]
    if len(parts) != 3:
        return None
    try:
        size = tuple(float(part) for part in parts)
    except ValueError:
        return None
    if any(value <= 0.0 for value in size):
        return None
    return size


def _pick_detection_record(pick):
    """Z inputs the pick waypoints are derived from.

    `pick_contact_top_z` prefers `top_surface_pose.z` and falls back to
    `pose.z + height/2`, and the detector only puts the true centre in
    `pose.z` when `height_valid`. A planning failure cannot be attributed
    without these fields.
    """
    return {
        "id": str(pick.id),
        "frame_id": str(pick.header.frame_id),
        "pose_position": [pick.pose.position.x, pick.pose.position.y,
                          pick.pose.position.z],
        "size_wdh": [float(pick.width), float(pick.depth), float(pick.height)],
        "height_valid": bool(getattr(pick, "height_valid", False)),
        "height_source": int(getattr(pick, "height_source", 0)),
        "top_surface_valid": bool(getattr(pick, "top_surface_valid", False)),
        "top_surface_z": float(pick.top_surface_pose.position.z),
        "yaw_valid": bool(pick.yaw_valid),
    }


class PlaceSmokeDriver(PickRetreatEvalDriver):

    def __init__(self, args):
        super().__init__(args)
        self._scene_config = load_scene_tf_config(
            resolve_scene_tf_config_path(
                getattr(args, "scene_tf_config", None) or None))
        self._scene_config_path = resolve_scene_tf_config_path(
            getattr(args, "scene_tf_config", None) or None)
        # F23: admitted GT envelopes render through scene_viz (latched).
        self._dump_viz_pub = self.create_publisher(
            String, "/luggage/frame_dump/viz",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._timeline = []
        self._place_state = "INIT"
        self._payload_detached = False
        self._segments_log = []
        self._tf_trace = []
        self._probe_joints = None
        self._follow_skipped0 = 0
        self._last_joint_state = None
        #: [B3] Live per-leg execution admission (armed only by a
        #: successful ValidateSelectedRoute after Select; None = no leg
        #: may claim a certified scope).
        self._execution_admission = None
        self._verify_route_client = None
        self._map_stats_client = None
        #: [release-failure contract] True from vacuum-off until the next
        #: cycle: a physically released box stays in the world (and in
        #: the map as pending) whatever the acceptance verdict.
        self._frame_released = False
        self._note_event_client = None
        self._joint_ring = deque()
        self._joint_ring_horizon_sec = JOINT_RING_FLOOR_SEC
        self._segment_exec_t0 = None
        self._segment_exec_name = None
        self._handoff_policy = HandoffPolicy()
        self._last_submitted_leg = None
        self._handoff_failure_record = None
        self._controller_state_ring = deque()
        self._controller_state_count = 0
        self._admission_check_client = None
        self._last_pick_detection = None
        self._home_arm_state = {}
        self._exit_yaw = {}
        latch = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # IK seed for a planning failure: without it a PLAN_<segment> dump
        # cannot tell an unreachable target from a bad start state.
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10,
            callback_group=self._group)
        self._subscribe_controller_state()
        self._state_pub = self.create_publisher(
            String, "/luggage/place/state", latch)
        self._box_model = ""
        self._release_physics = None
        self._gz_create = self.create_client(
            SpawnEntity, "/world/%s/create" % GZ_WORLD,
            callback_group=self._group)
        self._gz_delete = self.create_client(
            DeleteEntity, "/world/%s/remove" % GZ_WORLD,
            callback_group=self._group)
        self._place_touch = self.create_client(
            SetBool, "/scene_manager/set_place_support_touch",
            callback_group=self._group)
        self._get_scene = self.create_client(
            GetPlanningScene, "/get_planning_scene",
            callback_group=self._group)
        self._probe = MotionExecutor(self)
        self._keep_placed = False
        self._last_authorized_pick = None

    def wait_arm_settle(self, reason="", timeout=6.0, still_eps=0.005,
                        quiet_sec=0.4):
        """Block until `/joint_states` stops changing, bounded.

        A goal-time abort leaves the arm still decelerating when the driver
        regains control, and a plan from a moving arm reports a start state in
        collision that the same plan accepts a fraction of a second later.
        Polls the persistent joint subscription; do not build a second one.
        """
        t0 = time.time()
        deadline = t0 + float(timeout)
        last = None
        still_since = None
        while time.time() < deadline and rclpy.ok():
            record = self._last_joint_state
            positions = list((record or {}).get("position") or [])
            if positions:
                moved = (
                    last is None
                    or len(last) != len(positions)
                    or max(abs(a - b) for a, b in zip(positions, last))
                    > float(still_eps))
                if moved:
                    still_since = None
                elif still_since is None:
                    still_since = time.time()
                elif time.time() - still_since >= float(quiet_sec):
                    return "still after %.2fs" % (time.time() - t0)
                last = positions
            time.sleep(0.05)
        return "not still within %.1fs (%s)" % (timeout, reason or "settle")

    #: [ST09-I S5.7-6] named stops _home_arm surfaces verbatim (the
    #: four-state machine's own reasons — never re-mapped into a
    #: generic failure).
    ENSURE_NAMED_STOPS = frozenset((
        "CARRY_EXIT_UNSUPPORTED", "EXIT_ROUTE_UNAVAILABLE",
        "IDENTITY_UNKNOWN", "ATTACHMENT_STATE_UNKNOWN",
        "JOINT_STATE_UNAVAILABLE", "CAMERA_FRAME_UNAVAILABLE",
        "ZONE_UNKNOWN", "SCENE_CONFIG_UNAVAILABLE",
        "ENSURE_CYCLE_START_UNAVAILABLE", "ENSURE_SERVICE_EXCEPTION",
        "ROUTE_PRODUCER_UNAVAILABLE", "GOTO_V2_UNAVAILABLE",
    ))

    def home_arm_code(self):
        """Reason code for the last `_home_arm`, or "" when it returned."""
        state = dict(self._home_arm_state or {})
        if not state or state.get("ok"):
            return ""
        if not state.get("exit_ok"):
            # [ST09-I S5.7-6] the ensure verdict's named stops pass
            # through verbatim; anything else means the arm arrived
            # somewhere it may not start a cycle from without a
            # certified exit route.
            reason = str(state.get("cycle_reason") or "")
            if reason in self.ENSURE_NAMED_STOPS:
                return reason
            return "EXIT_REQUIRES_CERTIFIED_ROUTE"
        return "GOTO_FAILED"

    def _ensure_cycle_start(self):
        """[ST09-I S5.7-6 §2] The selector's authoritative cycle verdict.

        Returns ``(ok_service, ready, reason, zone, attachment, detail)``
        — the service absent fails CLOSED (the home path then refuses
        by name; it never adjudicates for itself).
        """
        from luggage_msgs.srv import EnsureCycleStartV2

        client = getattr(self, "_ensure_client", None)
        if client is None:
            client = self.create_client(
                EnsureCycleStartV2,
                "/trajectory_selector/ensure_cycle_start_v2",
                callback_group=getattr(self, "_group", None))
            self._ensure_client = client
        if not client.wait_for_service(timeout_sec=5.0):
            return False, False, "ENSURE_CYCLE_START_UNAVAILABLE", \
                "unknown", "unknown", ""
        request = EnsureCycleStartV2.Request()
        request.scene_revision = str(self._frame_scene_revision or "")
        result = self.call_srv(client, request, timeout=5.0) \
            if hasattr(self, "call_srv") else None
        if result is None:
            return False, False, "ENSURE_CYCLE_START_UNAVAILABLE", \
                "unknown", "unknown", ""
        return (True, bool(result.outcome == (
                    EnsureCycleStartV2.Response.ALREADY_READY)),
                str(result.reason_code or ""),
                str(result.zone or "unknown"),
                str(result.attachment_state or "unknown"),
                str(result.detail or ""))

    def _portal_exit_proof_valid(self):
        """[ST09-I S5.7-6 评审点 1 / G0-1] Adjudicate the structured
        exit proof.

        The proof is an evidence chain, not a boolean bypass: it holds
        only while NOTHING has changed since the certified exit — a
        NON-EMPTY certified exit route completed in THIS cycle (an
        empty/already-there record is not a proof and never passes the
        exception), same scene identity, the arm still at the exit's
        final joints (0.02 rad wrap-aware), and a PRESENT and FRESH
        un-attached vacuum feed (an absent or stale feed cannot vouch
        that the cycle never restarted). Any drift voids it and the
        four-state verdict stands.
        """
        proof = getattr(self, "_portal_exit_proof", None)
        if not isinstance(proof, dict):
            return False, "absent"
        # [G0-1] bind the exception to the ACTUALLY-certified exit
        # route: request/route/cert must all be non-empty.
        for field in ("request_id", "route_digest", "certification_id"):
            if not str(proof.get(field) or "").strip():
                return False, "uncertified_exit(%s)" % field
        if str(proof.get("scene_revision") or "") != str(
                getattr(self, "_frame_scene_revision", "") or ""):
            return False, "scene_changed"
        final = [float(v) for v in proof.get("final_joint_values") or []]
        live = list((getattr(self, "_last_joint_state", None) or {})
                    .get("position") or [])[:len(final)]
        if len(final) != 6 or len(live) != 6:
            return False, "joints_unavailable"

        def _wrap(delta):
            return (delta + math.pi) % (2.0 * math.pi) - math.pi

        delta = max(abs(_wrap(c - t)) for c, t in zip(live, final))
        if delta > 0.02:
            return False, "joints_moved(%.3frad)" % delta
        vacuum = getattr(self, "_vacuum_state", None)
        if not isinstance(vacuum, dict) or not vacuum:
            return False, "vacuum_state_missing"
        if bool(vacuum.get("attached")):
            return False, "vacuum_reattached"
        received_ns = int(vacuum.get("received_ns") or 0)
        age_s = max(0.0, (
            int(self.get_clock().now().nanoseconds) - received_ns) / 1e9)
        if received_ns <= 0 or age_s > PORTAL_PROOF_VACUUM_FRESHNESS_S:
            return False, "vacuum_state_stale(%.1fs)" % age_s
        return True, "valid(delta=%.3frad)" % delta

    def _start_new_cycle(self):
        """[ST09-I S5.7-6 评审点 1 / G0-1] A new attach STARTS a cycle:
        the previous cycle's exit proof is void from here — it must
        never carry across a new carrying phase (cross-cycle rejection
        happens at the voiding site, not at adjudication time)."""
        self._portal_exit_proof = None
        self._portal_exited = False

    def _aperture_corridor_depth(self):
        """Derived opening-corridor depth (design S5.7 section 1).

        wall thickness from scene data + TCP/camera envelope from the
        live static TF (URDF-derived fallback when TF is unavailable,
        recorded in the return) + declared safety margin. Never a bare
        constant.
        """
        scene = getattr(self, "_scene_config", None) or {}
        outer = scene.get("container", {}).get("outer", {}) or {}
        inner = scene.get("container", {}).get("inner", {}) or {}
        wall = 0.04
        try:
            if outer.get("length") and inner.get("length"):
                wall = max(0.0, (float(outer["length"])
                                 - float(inner["length"])) * 0.5)
        except (TypeError, ValueError):
            pass
        envelope, source = CAMERA_ENVELOPE_FALLBACK_M, "fallback"
        try:
            from rclpy.time import Time as _RclpyTime
            tf_msg = self._tf_buffer.lookup_transform(
                "suction_contact_frame", "camera_depth_optical_frame",
                _RclpyTime())
            t = tf_msg.transform.translation
            magnitude = math.sqrt(
                float(t.x) ** 2 + float(t.y) ** 2 + float(t.z) ** 2)
            if magnitude > 1e-3:
                envelope, source = magnitude, "tf"
        except Exception:  # noqa: BLE001 - classification must not die
            pass
        return (wall + envelope + APERTURE_ESCAPE_MARGIN_M,
                {"wall_m": round(wall, 4),
                 "camera_envelope_m": round(envelope, 4),
                 "envelope_source": source,
                 "margin_m": APERTURE_ESCAPE_MARGIN_M})

    def _container_zone(self):
        """Classify the cup in CONTAINER-LOCAL coordinates (S5.7 taxonomy).

        Returns ``(zone, detail)`` with zone in CONTAINER_ZONES: the
        explicit scene-state classification `_home_arm` acts on. UNKNOWN
        whenever TF fails or (under the typed chain) the scene identity
        is missing — a named refusal, never a world-X fallback; the
        legacy world-X guess survives only for non-frame-runtime callers.
        """
        from luggage_description.scene_tf_config_utils import container_zone
        suction, err = self.suction_xyz()
        if suction is None:
            return "unknown", str(err)
        scene = getattr(self, "_scene_config", None)
        if scene is None:
            if getattr(self, "_typed", None) is not None:
                return "unknown", "SCENE_IDENTITY_MISSING"
            # Legacy eval driver without a scene: world-X guess (sim
            # layout, container on +X) — never reachable on the typed
            # production chain.
            return ("opening_corridor" if float(suction[0]) > 0.2
                    else "outside_container"), ""
        depth, source = self._aperture_corridor_depth()
        base = xyz_world_to_base_link(scene, suction)
        zone = container_zone(base, scene, depth)
        return zone, "corridor=%s" % source

    def _suction_clear_of_container(self):
        """True only in the OUTSIDE_CONTAINER zone (classification, not
        authorization: the named-pose motion itself still rides the
        GoToRobotPose surface)."""
        zone, _err = self._container_zone()
        return zone == "outside_container"

    def _capture_gz_contacts(self):
        """One physics-contact sample. Absent publishers stay an explicit gap."""
        topic = "/world/%s/physics/contacts" % GZ_WORLD
        try:
            proc = subprocess.run(
                [_ign_model_bin(), "topic", "-e", "-t", topic, "--num", "1"],
                capture_output=True, text=True, timeout=2.0)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"available": False, "topic": topic, "reason": str(exc)}
        text = (proc.stdout or "") + (proc.stderr or "")
        if proc.returncode != 0 or "collision" not in text.lower():
            return {
                "available": False,
                "topic": topic,
                "reason": (text.strip() or "no contact sample")[:500],
            }
        return {"available": True, "topic": topic, "sample": text[:4000]}

    def _home_arm(self):
        """[ST09-I S5.7-6 §2] EnsureCycleStartV2 verdict + sanctioned goto.

        The selector adjudicates the authoritative state (latched
        joints, vacuum, scene identity — never a driver-side guess);
        this driver only obeys. ALREADY_READY -> the named-pose goto
        (GoToRobotPoseV2 under workstream C). RECOVERY_REQUIRED is a
        named stop, with ONE exception: EXIT_ROUTE_UNAVAILABLE plus a
        still-valid structured exit proof (评审点 1 — the certified
        portal exit already happened and nothing has drifted since;
        the portal pose legitimately classifies corridor).
        """
        (_ok_service, ready, cycle_reason, zone, attachment, detail) = \
            self._ensure_cycle_start()
        notes = ["cycle=%s zone=%s attachment=%s reason=%s" % (
            "ALREADY_READY" if ready else "RECOVERY_REQUIRED",
            zone, attachment, cycle_reason)]
        t_wall0 = time.time()
        t_ros0 = self.ros_now_sec()
        exit_ok = bool(ready)
        proof_note = ""
        if not exit_ok and cycle_reason == "EXIT_ROUTE_UNAVAILABLE":
            valid, why = self._portal_exit_proof_valid()
            if valid:
                exit_ok = True
                proof_note = "exit=portal_proof(%s)" % why
                notes.append(proof_note)
            elif why != "absent":
                # [G0-1] the void reason is evidence — record which
                # field drifted (absent is the ordinary never-exited
                # case and stays out of the notes).
                notes.append("exit_proof_void(%s)" % why)
        goto_ok = True
        goto_msg = ""
        result = None
        if not exit_ok:
            goto_ok = False
            goto_msg = "recovery required; arm %s (%s)" % (
                zone, cycle_reason)
            notes.append("goto=%s" % goto_msg)
        else:
            goto_ok, goto_msg, result = self.goto_observe()
            notes.append("goto=%s" % goto_msg)
        dt_wall = time.time() - t_wall0
        dt_ros = self.ros_now_sec() - t_ros0
        rtf = (dt_ros / dt_wall) if dt_wall > 1e-3 else 0.0
        notes.append("rtf=%.3f (sim %.2fs / wall %.2fs)" % (
            rtf, dt_ros, dt_wall))
        ok = bool(exit_ok and goto_ok)
        self._home_arm_state = {
            "ok": ok, "exit_ok": bool(exit_ok), "goto_ok": bool(goto_ok),
            "zone": zone, "zone_reason": cycle_reason,
            "cycle_reason": cycle_reason,
            "attachment_state": attachment,
            "portal_proof": bool(proof_note),
            "ensure_detail": str(detail or ""),
            "t_ros": float(self.ros_now_sec()),
        }
        if not ok:
            capture = getattr(self, "_capture_gz_contacts", None)
            if callable(capture):
                self._home_arm_state["contacts"] = capture()
        return ok, "; ".join(notes), result

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

    def _release_pose_stream(self):
        """Lazily started pose/info cache (driver-lifetime: the epoch
        floor is set at first message, well before any release)."""
        stream = getattr(self, "_pose_stream", None)
        if stream is None:
            stream = ReleasePoseStream().start()
            self._pose_stream = stream
        return stream

    def _gz_box_pose(self, model=None):
        model = str(model or self._box_model or "") or self.gt_model_name()
        self._last_gz_pose_probe = {"model": model}
        if not model:
            self._last_gz_pose_probe["error"] = "missing model name"
            return None
        # G3 wave4 (multi_run01): a single ign CLI read raced the sim and
        # returned nothing right at the release boundary ("no release
        # pose"), aborting a healthy cycle. Bounded retry over ~2 s;
        # the caller's own semantics are unchanged. [prior-g1 merge]
        # Each probe also feeds the prior branch's diagnostic record
        # (_last_gz_pose_probe) — observability only.
        command = [_ign_model_bin(), "model", "-m", model, "--pose"]
        for attempt in range(4):
            self._last_gz_pose_probe["command"] = command
            try:
                out = subprocess.check_output(
                    command,
                    text=True, timeout=5.0, stderr=subprocess.STDOUT)
            except (OSError, subprocess.CalledProcessError,
                    subprocess.TimeoutExpired) as exc:
                output = getattr(exc, "output", "") or ""
                if isinstance(output, bytes):
                    output = output.decode(errors="replace")
                self._last_gz_pose_probe.update(
                    error=type(exc).__name__, detail=str(exc),
                    output=str(output)[-16000:],
                    returncode=getattr(exc, "returncode", None))
                out = ""
            pose = parse_ign_model_pose(out) if out else None
            if pose is not None:
                self._last_gz_pose_probe.update(
                    output=out[-16000:], parsed=pose)
                return pose
            # Unparseable-but-returned output is recorded too (the prior
            # branch's forensic contract: the raw text must survive).
            # Empty `out` means the exception path already recorded the
            # raw text — do not clobber it.
            if out:
                self._last_gz_pose_probe.update(
                    output=out[-16000:], parsed=None)
            if attempt < 3:
                time.sleep(0.5)
        return None

    def on_detected_luggage(self, pick_msg, trial, spawn):
        """Hook after DetectLuggage. Return a fail_code or empty string.

        Pack-eval computes the place slot from this detection. Spawn GT
        must not become ComputePlacement input.

        [placement-prior opt-in] With --placement-prior the measured
        prior is prepared and bound HERE — strictly BEFORE the grasp —
        and narrows the measured suction candidates to the chosen one.
        The typed prior hint only REORDERS Select candidates (Select,
        admission and route certification are unchanged); the actual
        attachment after the grasp is the measured attach record, and
        nothing re-chooses the grasp after it. Any failure is a NAMED
        fallback to the original flow (adopted=false + reason); a
        stale/old prior is never used.
        """
        if not bool(getattr(self._args, "placement_prior", False)):
            return ""
        record = self._apply_placement_prior(pick_msg)
        if record:
            trial.extras.setdefault("placement_prior", record)
        return ""

    def _placement_prior_client(self):
        client = getattr(self, "_placement_prior_client_obj", None)
        if client is None:
            from luggage_planning.placement_prior_client import (
                PlacementPriorClient)
            client = PlacementPriorClient(
                self, group=getattr(self, "_group", None))
            self._placement_prior_client_obj = client
        return client

    def _apply_placement_prior(self, pick_msg):
        """One measured prior attempt; returns the evidence record."""
        started = time.monotonic()
        self._typed.set_placement_prior(None)   # cycle start: no stale hint
        record = {"adopted": False}
        try:
            client = self._placement_prior_client()
            response = client.prepare(
                pick_msg, self._frame_request_id("prior"))
            if response is None:
                record["reason"] = "PRIOR_UNAVAILABLE_OR_STALE_SURFACE"
                self._typed.set_placement_prior(None)  # uniform reset
                return record
            from luggage_planning.placement_prior_requests import (
                measured_detection_view)
            _bound, choice = client.bind(
                response, measured_detection_view(pick_msg),
                self._frame_scene_revision)
            selected = [candidate for candidate
                        in pick_msg.suction_candidates
                        if candidate.candidate_id == choice.candidate_id]
            if len(selected) != 1:
                raise ValueError(
                    "selected measured suction candidate is not unique")
            # Retain the original measured contact and rank; only narrow
            # the candidate list consumed by the existing pick builder.
            pick_msg.suction_candidates = selected
            self._typed.set_placement_prior(response)
            surface = getattr(client, "_surface", None)
            record.update({
                "adopted": True,
                "slot_id": choice.slot_id,
                "candidate_id": choice.candidate_id,
                "prior_elapsed_s": getattr(response, "elapsed_s", None),
                "cache_hit": bool(getattr(response, "cache_hit", False)),
                "driver_hook_s": round(time.monotonic() - started, 4),
                "snapshot_identity": {
                    "map_revision": getattr(surface, "map_revision", None),
                    "geometry_hash": getattr(surface, "geometry_hash", ""),
                    "epoch": getattr(surface, "epoch", ""),
                    "content_digest": getattr(
                        surface, "content_digest", ""),
                    "scene_revision": getattr(
                        getattr(surface, "identity", None),
                        "scene_revision", "")},
                "grasp_candidate": {
                    "candidate_id": choice.candidate_id,
                    "observation_id": getattr(
                        selected[0], "observation_id", "")},
            })
        except Exception as exc:  # noqa: BLE001 - named fallback only
            record["reason"] = "%s:%s" % (type(exc).__name__, exc)[:200]
            self._typed.set_placement_prior(None)
        return record

    def _place_slot_for(self, pick_msg, box_msg, trial):
        """Place slot from measured detection, never from GetCurrentBox."""
        del box_msg
        if not bool(getattr(pick_msg, "height_valid", False)):
            trial.fail_code = trial.fail_code or "DETECT_FULL_GEOMETRY_REQUIRED"
            return None, None
        # Typed chain: the slot and the place motion come from
        # ComputePlacementV2 -> SelectPlacePathV2. The fixed floor-center
        # slot stays a named fixture for callers without that chain.
        if getattr(self, "_typed", None) is not None:
            return self._typed_place_slot(pick_msg, trial)
        slot, meta = self._fixed_slot(pick_msg)
        meta["slot_source"] = "test_fixture"
        return slot, meta

    def _typed_place_slot(self, pick_msg, trial):
        """Certified-route slot: ComputePlacementV2 then SelectPlacePathV2.

        A certified route that is refused fails the box with the
        producer's reason. It does not fall back to the fixture slot.
        """
        import math as _math  # noqa: F401 - legacy local alias
        # [B3] A stale admission from a previous cycle must never arm the
        # next cycle's legs; the new one is armed only after re-verify.
        self._execution_admission = None
        # [release-failure contract] per-cycle release boundary.
        self._frame_released = False
        from luggage_description.frame_contracts import ContractError
        from luggage_description.frame_diagnostics import BoundaryContext
        from luggage_description.ros_frame_bindings.poses import (
            pose_from_message)
        from luggage_description.scene_tf_config_utils import (
            _local_point_to_base_link, _point_in_container_link,
            container_opening_target_point_in_world,
            container_usable_dimensions, xyz_world_to_base_link)
        from luggage_msgs.msg import SlotSpec
        from luggage_planning.grid_path_search import route_budget_s
        from luggage_planning.typed_driver_adapter import (
            container_transform_fields, tcp_record)

        surface_msg = getattr(self, "_surface_v2_msg", None)
        if surface_msg is None:
            trial.fail_code = trial.fail_code or "SURFACE_V2_MISSING"
            return None, None
        geometry_hash = str(surface_msg.geometry_hash or "")
        # box_wdh = placement semantics: footprint_x, footprint_y,
        # vertical_extent (see docs/architecture/size_semantics.md).
        box_wdh = [float(pick_msg.width), float(pick_msg.depth),
                   float(pick_msg.height)]
        self._frame_box_wdh = list(box_wdh)
        self._frame_selection_capture_dir = None
        attachment_geometry = getattr(self, '_frame_attachment_geometry', None)
        if attachment_geometry is None:
            trial.fail_code = trial.fail_code or 'ATTACHMENT_GEOMETRY_MISSING'
            return None, None
        request_id = self._frame_request_id("place")
        try:
            placement = self._typed.compute_placement(
                request_id=request_id, box_wdh=box_wdh,
                conservative_geometry=attachment_geometry,
                height_measured=True,
                scene_revision=self._frame_scene_revision,
                geometry_hash=geometry_hash,
                expected_map_revision=int(surface_msg.map_revision),
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
        slot_request = str(slot_messages[0].identity.request_id or "")
        if slot_request:
            request_id = slot_request

        grasp = getattr(self, "_frame_grasp", None)
        if grasp is None:
            trial.fail_code = trial.fail_code or "TYPED_PLACE_CONTEXT_MISSING"
            return None, None
        grasp = dict(grasp)
        grasp["request_id"] = request_id
        self._frame_grasp = grasp

        world = container_opening_target_point_in_world(self._scene_config)
        base = xyz_world_to_base_link(self._scene_config, world)
        local = _point_in_container_link(base, self._scene_config)
        portal = (local[0], local[1], max(float(local[2]) + 0.35, 0.80))
        start_tcp = tcp_record(request_id, self._frame_scene_revision,
                               portal, (1.0, 0.0, 0.0, 0.0))
        live = getattr(self, "_last_joint_state", None) or {}
        start_joints = [float(v) for v in (live.get("position")
                                           or live.get("positions") or [])]
        box_count = max(1, int(getattr(self._args, "max_boxes", 1) or 1))
        # Client wait = the SERVICE-side select budget (which must match
        # what the searcher actually enforces) plus an explicit 30 s
        # communication overhead. --select-budget-s > 0 mirrors the
        # selector's diagnostic_route_budget_s override so the client
        # never cuts a diagnostic search short; 0 (default) keeps the
        # frozen n*60s formula — production behavior unchanged.
        budget_override_s = float(
            getattr(self._args, "select_budget_s", 0.0) or 0.0)
        service_budget_s = (
            budget_override_s if budget_override_s > 0.0
            else route_budget_s(box_count))
        select_timeout_s = service_budget_s + 30.0
        from luggage_gazebo.selection_capture import capture_selection
        capture_root = os.environ.get("ELFIN_SELECTION_CAPTURE_DIR", "").strip()
        selection_only = bool(getattr(self._args, "diagnostic_selection_only", False))
        if selection_only and not capture_root:
            capture_root = os.path.join(self._args.out, "selection_boundary")
        selection_boundary = dict(
            request_id=request_id, scene_revision=self._frame_scene_revision,
            conservative_geometry=attachment_geometry.to_dict(),
            box_wdh=box_wdh, grasp=grasp, start_tcp=start_tcp,
            start_joints=start_joints, geometry_hash=geometry_hash,
            map_revision=int(surface_msg.map_revision),
            box_id=str(grasp.get("box_id") or ""),
            generation=int(grasp.get("generation") or 0),
            mono=time.monotonic(), sim=float(self.ros_now_sec()))
        if capture_root:
            from rosidl_runtime_py.convert import message_to_ordereddict
            selection_boundary.update(
                slots=[message_to_ordereddict(slot) for slot in slot_messages],
                slot_scores=list(scores),
                surface=message_to_ordereddict(surface_msg),
                inner_size=list(container_usable_dimensions(self._scene_config)),
                container_transform=container_transform_fields(self._scene_config),
                excluded_ids=[])
        self._frame_selection_capture_dir = capture_selection(capture_root, selection_boundary)
        if self._frame_selection_capture_dir:
            snapshot = self._scene_snapshot(raw_capture_path=os.path.join(
                self._frame_selection_capture_dir, 'planning_scene.cdr'))
            if snapshot.get('error'):
                trial.fail_code = trial.fail_code or 'SELECTION_SCENE_CAPTURE_FAILED'
                return None, None
        try:
            selection = self._typed.select_place_path(
                request_id=request_id, box_wdh=box_wdh,
                conservative_geometry=attachment_geometry,
                slots=slot_messages, slot_scores=scores,
                surface_message=surface_msg,
                inner_size=list(
                    container_usable_dimensions(self._scene_config)),
                geometry_hash=geometry_hash,
                map_revision=int(surface_msg.map_revision),
                start_tcp=start_tcp,
                start_joints=start_joints,
                grasp=grasp,
                container_transform=container_transform_fields(
                    self._scene_config),
                excluded_ids=(),
                box_id=str(grasp.get("box_id") or ""),
                generation=int(grasp.get("generation") or 0),
                timeout_sec=select_timeout_s)
        except ContractError as error:
            trial.fail_code = trial.fail_code or error.code
            trial.extras["selector_error"] = "%s: %s" % (
                error.code, error.field)
            return None, None
        capture_selection(capture_root, selection_boundary, selection)
        if selection_only and selection is not None and selection.success:
            trial.fail_code = "DIAGNOSTIC_SELECTION_CAPTURED"
            trial.extras["selection_boundary"] = selection_boundary
            return None, None
        if selection is None or not selection.success:
            trial.fail_code = trial.fail_code or (
                getattr(selection, "reason_code", None)
                or "PLANNING_ROUTE_REQUIRED")
            trial.extras["selector_error"] = (
                selection.reason_code if selection else "timeout")
            # [Functional diagnostic, 2026-10-22] Keep the selector's own
            # per-candidate/per-edge failure rows and stage timings
            # (prep/search/certify/revalidate/total) with the trial —
            # budget exhaustion closes with named evidence, not just the
            # terminal reason code.
            if selection is not None:
                trial.extras["selector_stage_timings"] = json.loads(
                    getattr(selection, "stage_timings_json", None) or "{}")
                trial.extras["selector_rows"] = json.loads(
                    getattr(selection, "rows_json", None) or "[]")
            return None, None

        winner_index = int(selection.slot_index)
        chosen = slot_messages[winner_index]
        context = BoundaryContext(
            "F13", "typed_driver_adapter", "place_smoke_driver", request_id,
            self._frame_scene_revision, "container_link", "box_center",
            "scene")
        try:
            record = pose_from_message(chosen, "slot", context)
        except ContractError as error:
            trial.fail_code = trial.fail_code or error.code
            return None, None
        self._frame_slot_record = record
        self._selected_carry_v2 = list(selection.motion_segments)
        self._selected_carry_v1 = list(selection.carry_segments)
        # [B3 execution invalidation] With verify_selected_route on (the
        # production launches), the selected route must be re-verified and
        # carry a live per-leg admission before any leg executes: handoff
        # polling, the release transition, and the executor's
        # pre-submission gate all read _execution_admission, which stayed
        # None on this path before (every check no-oped). Legacy harnesses
        # keep the pre-B3 behavior with the flag off.
        if getattr(self._args, "verify_selected_route", False):
            ok, reason = self._verify_selected_admission(
                selection, request_id)
            if not ok:
                trial.fail_code = trial.fail_code or reason
                trial.extras["verify_selected"] = reason
                self._execution_admission = None
                self._selected_carry_v2 = None
                self._selected_carry_v1 = None
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
        trial.extras["path_source"] = str(selection.path_source)
        trial.extras["selection_id"] = str(selection.selection_id)
        trial.extras["slot_source"] = "certified_route"
        base_xyz = _local_point_to_base_link(
            list(record["position"]), self._scene_config)
        yaw_container = 2.0 * _math.atan2(record["quaternion_xyzw"][2],
                                          record["quaternion_xyzw"][3])
        slot = SlotSpec()
        slot.place_pose.position.x = float(base_xyz[0])
        slot.place_pose.position.y = float(base_xyz[1])
        slot.place_pose.position.z = float(base_xyz[2])
        slot.place_pose.orientation.z = _math.sin(0.5 * yaw_container)
        slot.place_pose.orientation.w = _math.cos(0.5 * yaw_container)
        slot.width, slot.height, slot.depth = (
            box_wdh[0], box_wdh[2], box_wdh[1])
        return slot, self._slot_meta(slot)

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

    def _scene_snapshot(self, raw_capture_path=None):
        request = GetPlanningScene.Request()
        request.components.components = int(
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.OCTOMAP
            | PlanningSceneComponents.TRANSFORMS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = self.call_srv(self._get_scene, request, timeout=5.0)
        if response is None:
            return {"error": "get_planning_scene timeout"}
        if raw_capture_path:
            from rclpy.serialization import serialize_message
            with open(raw_capture_path, 'wb') as handle:
                handle.write(serialize_message(response))
        scene = response.scene
        acm = scene.allowed_collision_matrix
        summary = summarize_planning_scene(scene)
        summary.update({
            "acm_names": list(acm.entry_names),
            "place_state": self._place_state,
        })
        return summary

    def _follow_skipped(self):
        events = dict(self._vacuum_events or {})
        try:
            return int(events.get("follow_skipped") or 0)
        except (TypeError, ValueError):
            return 0

    def _clear_smoke_slot(self, slot, delete_model=False):
        # [G3 wave5] no scene placed object to remove (occ-only); the
        # map rollback happens through the cargo-map chain.
        if delete_model:
            return self.call_srv(
                self._clear, ClearCurrentBox.Request(), timeout=15.0)
        return None

    def _commit_to_cargo_map(self, trial, slot, slot_meta):
        """Commit the released box to the cargo-map ledger.

        The base driver measures the box with the integration itself and
        leaves the ledger alone; drivers that exercise the commit path
        override this and return False on a refused commit.
        """
        del trial, slot, slot_meta
        return True

    def _set_place_touch(self, allowed):
        request = SetBool.Request()
        request.data = bool(allowed)
        resp = self.call_srv(self._place_touch, request, timeout=10.0)
        if resp is None or not resp.success:
            # A loaded stack can stall this ACM round-trip past the first
            # 10 s call (streak 2 P2: the touch, clear, build, and vacuum
            # services all timed out in the same window while the sim
            # itself kept running). Idempotent: one settle-and-retry
            # before the descend branch fails the case on it.
            time.sleep(2.0)
            resp = self.call_srv(self._place_touch, request, timeout=20.0)
        return resp

    def _check_i1(self):
        if not getattr(self._args, "use_vacuum", False):
            return True, ""
        state = dict(self._vacuum_state or {})
        if not state.get("attached") or state.get("fail_reason"):
            return False, "PLACE_LOST_PAYLOAD"
        return True, ""

    def _carrying_vacuum_holds(self):
        """[ST09-I S5.7-6] Live vacuum-held check for a carrying leg."""
        state = dict(getattr(self, "_vacuum_state", None) or {})
        if not state:
            return True, ""
        if state.get("fail_reason"):
            return False, "PAYLOAD_LOST:%s" % str(
                state.get("fail_reason"))[:80]
        if not bool(state.get("attached")):
            return False, "PAYLOAD_LOST:vacuum_released"
        return True, ""

    def _transition_admission_unloaded(self, release_evidence):
        """[ST09-I S5.7-6] Activate the unloaded phase after release.

        CARRYING phase -> release side effect -> detach confirmed ->
        UNLOADED phase -> retreat. No admission (pick legs, legacy) is
        a no-op success: there is nothing to transition. The service
        being absent fails CLOSED — the retreat legs would then be
        refused per leg by the executor gate anyway.
        """
        admission = getattr(self, "_execution_admission", None)
        if not admission:
            return True, ""
        from luggage_msgs.srv import TransitionAdmissionPhase

        client = getattr(self, "_phase_client", None)
        if client is None:
            client = self.create_client(
                TransitionAdmissionPhase,
                "/trajectory_selector/transition_admission_phase",
                callback_group=getattr(self, "_group", None))
            self._phase_client = client
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "SERVICE_UNAVAILABLE"
        request = TransitionAdmissionPhase.Request()
        request.route_digest = str(admission.get("route_digest", "") or "")
        request.certification_id = str(
            admission.get("certification_id", "") or "")
        request.target_phase = "unloaded"
        request.detach_confirmed = True
        request.release_evidence = str(release_evidence or "")[:500]
        result = self.call_srv(client, request, timeout=5.0) \
            if hasattr(self, "call_srv") else None
        if result is None:
            return False, "SERVICE_TIMEOUT"
        if not result.success:
            return False, str(result.reason_code or "TRANSITION_REFUSED")
        return True, str(result.reason_code or "OK")

    def _leg_execution_admission(self, motion_segment):
        """[ST09-I S5.7-6] Per-leg admission scope for one typed leg.

        The stored admission carries the certified per-leg digests and
        load-state indices from ValidateSelectedRoute; the leg about to
        execute claims ITS OWN scope (carrying legs before the release
        boundary, unloaded legs after it) instead of a route-level
        blanket claim — the executor's liveness check then refuses a
        carrying scope authorizing a post-release leg by name.
        """
        base = getattr(self, "_execution_admission", None)
        if not base:
            return None
        admission = dict(base)
        digests = list(admission.pop("leg_digests", None) or [])
        states = list(admission.pop("leg_load_states", None) or [])
        digest_fn = getattr(self, "_leg_trajectory_digest", None)
        digest = (digest_fn(motion_segment)
                  if callable(digest_fn) else None)
        if digest and str(digest) in [str(d) for d in digests]:
            index = [str(d) for d in digests].index(str(digest))
            admission["leg_digest"] = str(digest)
            if index < len(states):
                admission["load_state"] = str(states[index])
        return admission

    def _evaluate_place_verify(self, trial, expected, actual, pose_release,
                               pose_drift):
        """[release-failure contract] Fill trial metrics + the SHARED
        verify-evidence dict, then judge the gates.

        The evidence (poses, direction, errors, inside flag) is written
        BEFORE any gate decision, so a failing verify dumps exactly what
        a passing one would. Returns the failing gate name or None.
        """
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
        trial.extras["verify"].update({
            "err_xy": trial.err_xy, "err_z": trial.err_z,
            "drift": trial.drift,
            "release_drift_direction_world": [
                actual[0] - expected[0], actual[1] - expected[1]],
            "inside_inner_box": trial.inside_inner_box,
        })
        if trial.err_xy > 0.04:
            return "PLACE_VERIFY_XY"
        if abs(trial.err_z) > 0.03:
            return "PLACE_VERIFY_Z"
        if trial.drift is not None and trial.drift > 0.02:
            return "PLACE_VERIFY_DRIFT"
        if trial.roll is not None and abs(trial.roll) > math.radians(2.0):
            return "PLACE_VERIFY_TIP"
        if trial.pitch is not None and abs(trial.pitch) > math.radians(2.0):
            return "PLACE_VERIFY_TIP"
        if not trial.inside_inner_box:
            return "PLACE_VERIFY_XY"
        return None

    def _notify_box_released(self):
        """[release-failure contract] Record the box_released fact with
        the commanded footprint on the cargo map at vacuum-off.

        Returns (ok, message). The map enters pending immediately; the
        confirmation gate then blocks next-box planning until the fact
        resolves (anchored accepted commit, or an observing view).
        """
        record = getattr(self, "_frame_slot_record", None) or {}
        box_wdh = getattr(self, "_frame_box_wdh", None) or []
        position = [float(v) for v in (record.get("position")
                                       or [0.0, 0.0, 0.0])]
        if len(position) < 3 or len(box_wdh) < 3 \
                or any(v <= 0.0 for v in box_wdh[:3]):
            return False, "no commanded footprint to report"
        yaw = 2.0 * math.atan2(
            float((record.get("quaternion_xyzw") or [0, 0, 0, 1])[2]),
            float((record.get("quaternion_xyzw") or [0, 0, 0, 1])[3]))
        client = getattr(self, "_note_event_client", None)
        if client is None:
            from luggage_msgs.srv import NotifyOccupancyEvent
            client = self.create_client(
                NotifyOccupancyEvent, "/cargo_map/note_occupancy_event")
            self._note_event_client = client
        from luggage_msgs.srv import NotifyOccupancyEvent
        request = NotifyOccupancyEvent.Request()
        request.event = "box_released"
        request.box_id = str(
            (getattr(self, "_frame_grasp", None) or {}).get("box_id")
            or "")
        request.box_center_container = position[:3]
        request.box_size_wdh = [float(v) for v in box_wdh[:3]]
        request.box_yaw = float(yaw)
        request.source = "place_smoke_driver"
        response = self._call_srv_sync(client, request, timeout=10.0)
        if response is None:
            return False, "service unavailable"
        if response.status != "accepted":
            return False, "%s: %s" % (response.status, response.reason)
        return True, ("pending=%s epoch=%s" % (
            not response.occupancy_confirmed, response.epoch[:8]))

    def _leg_trajectory_digest(self, segment):
        """Rebuild the certified trajectory digest from a v2 leg message.

        [B3] Ported from motion_occ_eval_driver: _leg_execution_admission
        always looked for this helper and silently degraded to a
        scope-less admission when it was absent; the B3 verify wiring
        makes the absence a named refusal (LEG_DIGEST_UNAVAILABLE), so
        the helper now exists here too.
        """
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

    def _call_srv_sync(self, client, request, timeout=30.0):
        """Minimal synchronous service call (event-wait, None on timeout)."""
        import threading
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        return future.result()

    def _verify_selected_admission(self, selection, request_id):
        """[B3] Re-verify the selected route and arm the leg admission.

        Certify A, execute A: the selector re-checks identity, start
        state, and live map admission, re-certifies the exact edges, and
        the returned per-leg digests must match what this driver holds.
        Only then does _execution_admission arm — handoff polling, the
        release transition, and the executor's pre-submission gate all
        become live (they read the same record and no-oped while it was
        None). Wiring mirrors motion_occ_eval_driver._verify_selected_route.
        Returns (True, "") or (False, reason).
        """
        client = getattr(self, "_verify_route_client", None)
        if client is None:
            from luggage_msgs.srv import ValidateSelectedRoute
            client = self.create_client(
                ValidateSelectedRoute,
                "/trajectory_selector/validate_selected_route")
            self._verify_route_client = client
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "VERIFY_UNAVAILABLE"
        from luggage_msgs.srv import ValidateSelectedRoute, GetCargoMapStats
        stats_client = getattr(self, "_map_stats_client", None)
        if stats_client is None:
            stats_client = self.create_client(
                GetCargoMapStats, "/cargo_map/get_stats")
            self._map_stats_client = stats_client
        # Fail-closed on the LATEST map: a failed stats fetch never falls
        # back to the selection-time revision.
        stats_response = self._call_srv_sync(
            stats_client, GetCargoMapStats.Request(), timeout=5.0)
        if stats_response is None or not stats_response.success:
            return False, "MAP_STATUS_UNAVAILABLE"
        live = getattr(self, "_last_joint_state", None) or {}
        request = ValidateSelectedRoute.Request()
        request.request_id = str(request_id)
        request.scene_revision = str(self._frame_scene_revision or "")
        request.route_digest = str(
            getattr(selection, "path_hash", "") or "")
        request.current_joint_state = [float(v) for v in
                                       (live.get("position")
                                        or live.get("positions") or [])]
        request.latest_map_revision = int(stats_response.map_revision)
        result = self._call_srv_sync(client, request, timeout=30.0)
        if result is None:
            return False, "VERIFY_UNAVAILABLE"
        if not result.valid:
            return False, str(result.reason_code or "VERIFY_REJECTED")
        digest_fn = getattr(self, "_leg_trajectory_digest", None)
        if not callable(digest_fn):
            return False, "LEG_DIGEST_UNAVAILABLE"
        local = [digest_fn(segment)
                 for segment in (selection.motion_segments or [])]
        certified = [str(digest) for digest in (result.leg_digests or [])]
        if local != certified or any(not digest for digest in local):
            return False, "LEG_DIGEST_MISMATCH"
        self._execution_admission = {
            "route_digest": str(
                getattr(selection, "path_hash", "") or ""),
            "certification_id": str(result.certification_id or ""),
            "admission_map_revision": int(
                result.validated_map_revision or 0),
            "leg_digests": list(certified),
            "leg_load_states": [
                str(state) for state in (result.leg_load_states or [])],
        }
        return True, ""

    def _peek_leg_load_state(self, segment):
        """Certified load_state for this leg, without consuming the index."""
        explicit = getattr(segment, "load_state", None)
        if explicit:
            return str(explicit)
        segments_v2 = getattr(self, "_frame_motion_segments", None) or []
        index = int(getattr(self, "_frame_motion_index", 0) or 0)
        if index >= len(segments_v2):
            return None
        motion = segments_v2[index]
        if str(getattr(motion, "name", "")) != str(getattr(segment, "name", "")):
            return None
        admission = self._leg_execution_admission(motion)
        if not admission:
            return None
        state = admission.get("load_state")
        return str(state) if state else None

    def _payload_attachment_gate(self, segment):
        """Attached is required only on a carrying leg before detach.

        Carrying legs still go through ``_check_i1`` so a subclass can
        keep its own retry policy. Unloaded legs do not.
        """
        if not getattr(self._args, "use_vacuum", False):
            return True, ""
        detached = bool(getattr(self, "_payload_detached", False))
        load_state = self._peek_leg_load_state(segment)
        requires = segment_requires_payload_attached(
            segment, load_state=load_state, detached=detached)
        if requires:
            return self._check_i1()
        state = dict(getattr(self, "_vacuum_state", None) or {})
        reason = str(state.get("fail_reason") or "")
        if reason:
            return False, "VACUUM_DETACH:%s" % reason
        return True, ""

    def _timed_leg(self, name):
        for segment in list(getattr(self, "_frame_motion_segments", None) or []):
            if str(getattr(segment, "name", "") or "") == str(name):
                return segment
        return None

    def _joint_sample(self):
        record = getattr(self, "_last_joint_state", None)
        if not isinstance(record, dict):
            return None
        try:
            stamp = float(record.get("stamp"))
        except (TypeError, ValueError):
            return None
        return {
            "stamp": stamp,
            "name": list(record.get("name") or []),
            "position": [float(value) for value in (record.get("position") or [])],
        }

    # ------------------------------------------------------------------
    # Leg-to-leg handoff: controller success is not arrival.
    # See luggage_gazebo/handoff_settle.py for the contract and Run08.

    # Live publisher verified 2026-10-08 (handoff_verify01). The namespaced
    # /S20/... name has no publisher. Measured on an idle arm, ``feedback``
    # equals ``reference`` (open_loop_control), so measured joint position
    # comes from /joint_states, not from this message.
    CONTROLLER_STATE_TOPIC = CONTROLLER_STATE_TOPIC
    # Controller reference/feedback is kept this long for handoff capture.
    _CONTROLLER_STATE_HORIZON_SEC = 30.0
    # Success-path handoff evidence keeps at most this many controller rows.
    _CONTROLLER_STATE_SUCCESS_ROWS = 30
    _HANDOFF_ADMISSION_POLL_SEC = 1.0
    _HANDOFF_ADMISSION_TIMEOUT_SEC = 1.0

    def _subscribe_controller_state(self):
        """Record controller reference/feedback/error for the handoff window.

        Optional: a missing message type or topic is recorded explicitly as
        ``unavailable`` in the handoff evidence, never silently skipped.
        """
        try:
            from control_msgs.msg import JointTrajectoryControllerState
        except ImportError:
            self._controller_state_import_error = "control_msgs unavailable"
            return
        self._controller_state_import_error = ""
        self.create_subscription(
            JointTrajectoryControllerState, self.CONTROLLER_STATE_TOPIC,
            self._on_controller_state,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self._group)

    @staticmethod
    def _controller_point(point, names):
        if point is None:
            return None
        positions = canonical_joint_positions(
            names, list(getattr(point, "positions", []) or []))
        return positions

    def _on_controller_state(self, msg):
        names = list(msg.joint_names)
        # humble: reference/feedback/error. foxy: desired/actual/error.
        reference = getattr(msg, "reference", None)
        if reference is None or not list(getattr(reference, "positions", [])):
            reference = getattr(msg, "desired", None)
        feedback = getattr(msg, "feedback", None)
        if feedback is None or not list(getattr(feedback, "positions", [])):
            feedback = getattr(msg, "actual", None)
        row = {
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "rx_mono": time.monotonic(),
            "reference": self._controller_point(reference, names),
            "feedback": self._controller_point(feedback, names),
            "error": self._controller_point(
                getattr(msg, "error", None), names),
            "output": self._controller_point(
                getattr(msg, "output", None), names),
        }
        self._controller_state_count += 1
        self._controller_state_ring.append(row)
        cutoff = row["rx_mono"] - self._CONTROLLER_STATE_HORIZON_SEC
        while (self._controller_state_ring
               and self._controller_state_ring[0]["rx_mono"] < cutoff):
            self._controller_state_ring.popleft()

    def _controller_state_window(self, since_mono, until_mono, max_rows=None):
        rows = []
        for row in list(self._controller_state_ring):
            if since_mono <= row["rx_mono"] <= until_mono:
                rows.append({
                    "stamp": row["stamp"],
                    "reference": _round_joints(row["reference"]),
                    "feedback": _round_joints(row["feedback"]),
                    "error": _round_joints(row["error"]),
                    "output": _round_joints(row["output"]),
                })
        if max_rows is not None and len(rows) > max_rows:
            step = max(1, len(rows) // int(max_rows))
            picked = rows[::step]
            if picked[-1] is not rows[-1]:
                picked.append(rows[-1])
            rows = picked
        return {
            "topic": self.CONTROLLER_STATE_TOPIC,
            "available": bool(rows),
            "messages_total": int(self._controller_state_count),
            "unavailable_reason": (
                "" if rows else (
                    getattr(self, "_controller_state_import_error", "")
                    or "no controller_state message in the handoff window")),
            "rows": rows,
        }

    def _note_leg_submitted(self, motion_segment, name, ok):
        """Remember the leg that was actually submitted and its result time.

        The endpoint comes from the typed segment handed to the controller
        path, not from the certified route plan, so the next leg is compared
        against what was really submitted.
        """
        if not ok:
            self._last_submitted_leg = None
            return
        names, last = trajectory_endpoint(motion_segment, "last")
        _first_names, first = trajectory_endpoint(motion_segment, "first")
        self._last_submitted_leg = {
            "name": str(name),
            "joint_names": list(names) if names is not None else None,
            "last_knot": None if last is None else list(last),
            "first_knot": None if first is None else list(first),
            "result_mono": time.monotonic(),
            "result_sim_stamp": float(self.ros_now_sec()),
        }

    def _record_submitted_leg(self, trial, motion_segment, name, ok, message,
                              submit_times):
        """Capture only: the exact submitted trajectory and action times.

        The PlanMotionV2 path writes no boundary dump, so this is the only
        record of the time-parameterised trajectory handed to the controller.
        Controller-level accept/success times come from the controller log
        and are aligned to these by wall clock.
        """
        trial.extras.setdefault("submitted_legs", []).append({
            "name": str(name),
            "ok": bool(ok),
            "message": str(message or ""),
            "submit_wall": submit_times.get("submit_wall"),
            "submit_sim_stamp": submit_times.get("submit_sim_stamp"),
            "response_wait_s": submit_times.get("response_wait_s"),
            "result_wall": time.time(),
            "result_sim_stamp": float(self.ros_now_sec()),
            "trajectory": trajectory_dump(motion_segment),
        })

    def _handoff_samples_reader(self, since_mono):
        """``read_samples`` closure: new ring rows received after ``since``."""
        cursor = {"stamp": None}

        def read():
            try:
                rows = list(self._joint_ring)
            except RuntimeError:  # ring mutated while copying; retry later
                return []
            fresh = []
            for row in rows:
                rx = row.get("rx_mono")
                if rx is None or rx < since_mono:
                    continue
                stamp = float(row.get("stamp") or 0.0)
                if cursor["stamp"] is not None and stamp <= cursor["stamp"]:
                    continue
                fresh.append((row, rx))
            if fresh:
                cursor["stamp"] = float(fresh[-1][0].get("stamp") or 0.0)
            return fresh

        return read

    def _handoff_admission_reason(self, motion_segment, leg_admission):
        """Revocation reason while waiting, or None. Fail-closed."""
        if not leg_admission:
            return None
        client = self._admission_check_client
        if client is None:
            client = self.create_client(
                CheckExecutionAdmission,
                "/trajectory_selector/check_execution_admission",
                callback_group=getattr(self, "_group", None))
            self._admission_check_client = client
        request = CheckExecutionAdmission.Request()
        request.route_digest = str(leg_admission.get("route_digest", "") or "")
        request.certification_id = str(
            leg_admission.get("certification_id", "") or "")
        request.admission_map_revision = int(
            leg_admission.get("admission_map_revision", 0) or 0)
        request.scene_revision = str(
            getattr(motion_segment, "scene_revision", "") or "")
        request.leg_digest = str(leg_admission.get("leg_digest", "") or "")
        request.leg_load_state = str(leg_admission.get("load_state", "") or "")
        response = self.call_srv(
            client, request, timeout=self._HANDOFF_ADMISSION_TIMEOUT_SEC)
        if response is None:
            return "HANDOFF_ADMISSION_CHECK_UNAVAILABLE"
        if not bool(response.live):
            return "HANDOFF_ADMISSION_%s" % str(
                response.reason_code or "REVOKED")
        return None

    def _await_leg_handoff(self, motion_segment, name, leg_admission, trial):
        """Gate the submission of ``name`` on the previous leg's arrival.

        Returns ``(ok, code)``. ``ok`` False means the leg must not be
        submitted: ``HANDOFF_ENDPOINT_MISMATCH`` (structural),
        ``HANDOFF_SETTLE_TIMEOUT`` (bounded wait expired), or the abort
        reason (cancellation, vacuum loss, admission revoked). No motion is
        commanded here and the executor start gate is unchanged.
        """
        previous = self._last_submitted_leg
        if previous is None:
            return True, ""
        evidence = {
            "previous_leg": previous.get("name"),
            "next_leg": str(name),
            "previous_result_sim_stamp": previous.get("result_sim_stamp"),
        }
        if previous.get("name") == "descend" and str(name) == "retreat":
            # Retreat has its own detach-aware start contract.
            evidence["status"] = "skipped"
            evidence["reason"] = "retreat_owns_descend_handoff"
            trial.extras.setdefault("handoffs", []).append(evidence)
            return True, ""
        next_names, next_first = trajectory_endpoint(motion_segment, "first")
        if previous.get("last_knot") is None or next_first is None:
            # No timed endpoint on one side: nothing to wait for. Recorded
            # so a missing contract is visible, not silently passed.
            evidence["status"] = "skipped"
            evidence["reason"] = "no_timed_endpoint"
            evidence["previous_last_knot_present"] = (
                previous.get("last_knot") is not None)
            evidence["next_first_knot_present"] = next_first is not None
            trial.extras.setdefault("handoffs", []).append(evidence)
            return True, ""

        carrying = (leg_admission or {}).get("load_state") == "carrying"
        wait_begun = {"mono": time.monotonic(), "polled": None}

        def should_abort():
            if not rclpy.ok():
                return "HANDOFF_CANCELLED"
            lifecycle = getattr(self, "_executor_lifecycle", None)
            if lifecycle is not None and lifecycle.failure is not None:
                return "DRIVER_EXECUTOR_FAILED"
            if carrying:
                holds, reason = self._carrying_vacuum_holds()
                if not holds:
                    return reason
            now = time.monotonic()
            last = wait_begun["polled"]
            if last is None:
                last = wait_begun["mono"]
            if now - last >= self._HANDOFF_ADMISSION_POLL_SEC:
                wait_begun["polled"] = now
                return self._handoff_admission_reason(
                    motion_segment, leg_admission)
            return None

        record = await_handoff(
            self._handoff_policy,
            (previous.get("joint_names"), previous.get("last_knot")),
            (next_names, next_first),
            read_samples=self._handoff_samples_reader(
                float(previous["result_mono"])),
            now_mono=time.monotonic,
            sleep=time.sleep,
            started_mono=float(previous["result_mono"]),
            should_abort=should_abort)
        record.update(evidence)
        record["action_result"] = {
            "previous_leg_success": True,
            "result_mono": previous.get("result_mono"),
            "result_sim_stamp": previous.get("result_sim_stamp"),
        }
        window = self._controller_state_window(
            float(previous["result_mono"]) - 0.5, time.monotonic(),
            max_rows=(
                self._CONTROLLER_STATE_SUCCESS_ROWS
                if record.get("status") == "ready" else None))
        record["controller_state"] = window
        if record.get("status") != "ready":
            from luggage_gazebo.handoff_failure_capture import retain_failure
            record["post_verdict_capture"] = retain_failure(self, record)
        self.get_logger().info(
            "handoff %s->%s status=%s waited=%.3fs samples=%d" % (
                previous.get("name"), name, record.get("status"),
                float(record.get("waited_sec") or 0.0),
                len(record.get("samples") or [])))
        if record.get("status") == "ready":
            trial.extras.setdefault("handoffs", []).append(
                summarize_for_extras(record))
            return True, ""
        self._handoff_failure_record = record
        trial.extras.setdefault("handoffs", []).append({
            "previous_leg": evidence["previous_leg"],
            "next_leg": evidence["next_leg"],
            "status": record.get("status"),
            "code": record.get("code"),
            "waited_sec": record.get("waited_sec"),
            "detail": "full record in handoff.json",
        })
        return False, str(record.get("code") or "HANDOFF_SETTLE_TIMEOUT")

    def _await_retreat_start_contract(self, retreat_seg, detach_request_stamp,
                                      detach_response_stamp,
                                      joints_before_detach,
                                      timeout=_RETREAT_START_WAIT_SEC):
        """Wait until a post-detach joint sample matches descend's final knot.

        Descend can report success while joint 2 is still outside the 0.01 rad
        QP gate (run17c/d). Stillness is not enough: the residual creeps slower
        than the settle epsilon. This does not widen the gate.
        """
        retreat_names, retreat_q = trajectory_endpoint(
            self._timed_leg("retreat"), "first")
        descend_names, descend_q = trajectory_endpoint(
            self._timed_leg("descend"), "last")
        if detach_response_stamp is None:
            detach_response_stamp = float(self.ros_now_sec())
        started = time.time()
        deadline = started + float(timeout)
        seen = []
        decision = None
        while True:
            sample = self._joint_sample()
            if sample is not None and (
                    not seen or sample["stamp"] != seen[-1]["stamp"]):
                seen.append(sample)
            decision = retreat_start_decision(
                seen,
                detach_stamp=float(detach_response_stamp),
                retreat_names=retreat_names,
                retreat_positions=retreat_q,
                descend_names=descend_names,
                descend_positions=descend_q)
            if decision["status"] == "ready":
                break
            if decision["status"] in (
                    "contract_mismatch", "joint_order", "missing_contract"):
                break
            if time.time() >= deadline or not rclpy.ok():
                break
            time.sleep(0.05)
        fresh = [
            sample for sample in seen
            if sample["stamp"] > float(detach_response_stamp)]
        decision["detach_request_stamp"] = detach_request_stamp
        decision["detach_response_stamp"] = float(detach_response_stamp)
        decision["detach_motion"] = detach_joint_motion(
            joints_before_detach, fresh[0] if fresh else None)
        decision["controller_settle"] = detach_joint_motion(
            fresh[0] if fresh else None,
            fresh[-1] if fresh else None)
        decision["driver_motion_submitted"] = False
        decision["waited_sec"] = round(time.time() - started, 3)
        decision["timeout_sec"] = float(timeout)
        load_state = self._peek_leg_load_state(retreat_seg)
        decision["load_state"] = load_state
        decision["requires_payload_attached"] = bool(
            segment_requires_payload_attached(
                retreat_seg, load_state=load_state,
                detached=bool(getattr(self, "_payload_detached", False))))
        self.get_logger().info(
            "retreat start %s max_delta=%s stamp=%s" % (
                decision["status"], decision.get("max_delta"),
                decision.get("joint_state_stamp")))
        return decision

    def _execute_segment(self, segment, trial):
        t0 = time.time()
        if self._last_joint_state:
            stamp0 = float(self._last_joint_state["stamp"])
        else:
            stamp0 = self.ros_now_sec()
        name = str(segment.name)
        if getattr(self, "_segment_exec_name", None) != name:
            self._segment_exec_name = name
            self._segment_exec_t0 = stamp0
        self._joint_ring_horizon_sec = joint_ring_horizon_sec()
        before = self._sample_tf_box("before_%s" % segment.name)
        ok_i1, code = self._payload_attachment_gate(segment)
        if not ok_i1:
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
        use_typed = False
        if self._typed is not None:
            # Typed chain: execute the v2 segment paired by build order.
            # place_exit is built here, after release, and is not in that
            # sequence. It still has to reach the cartesian replan.
            segments_v2 = getattr(self, "_frame_motion_segments", None) or []
            index = getattr(self, "_frame_motion_index", 0)
            motion_segment = None
            if index < len(segments_v2):
                candidate = segments_v2[index]
                if str(candidate.name) == name:
                    motion_segment = candidate
                    self._frame_motion_index = index + 1
            if motion_segment is not None:
                use_typed = True
                leg_admission = self._leg_execution_admission(
                    motion_segment)
                # [ST09-I S5.7-6] live payload state for carrying legs:
                # the producer binds the certificate scope and the
                # measured digest; the VACUUM feed lives here. A box
                # that fell off refuses the leg by name BEFORE any
                # controller submission (absent feed = no signal, not
                # payload-lost; the producer-side gates still hold).
                # No certified admission scope (Select's response alone
                # carries no ValidateSelectedRoute record): the same
                # no-admission semantics as the transition/handoff/peek
                # helpers, with the carrying determination falling back
                # to the segment contract the payload gate uses.
                if leg_admission:
                    leg_carries_payload = (
                        leg_admission.get("load_state") == "carrying")
                else:
                    leg_carries_payload = segment_requires_payload_attached(
                        motion_segment,
                        load_state=self._peek_leg_load_state(
                            motion_segment),
                        detached=bool(getattr(
                            self, "_payload_detached", False)))
                if leg_carries_payload:
                    holds, lost_reason = self._carrying_vacuum_holds()
                    if not holds:
                        return False, lost_reason, None
                # Controller success is not arrival (Run08): the previous
                # leg's submitted endpoint must be continuous with this
                # leg's first knot, and the measured joints must have
                # arrived, before this leg is submitted. No motion is
                # commanded and the executor start gate is unchanged.
                handoff_ok, handoff_code = self._await_leg_handoff(
                    motion_segment, name, leg_admission, trial)
                if not handoff_ok:
                    return False, handoff_code, None
                # [ST09-I S5.7-6] during-leg poll for carrying legs:
                # payload loss between the pre-submission check above
                # and the controller commit cancels mid-wait (the
                # adapter refuses by name and waits for the cancel
                # receipt — no detach, no map commit, no recovery).
                submit_times = {
                    "submit_wall": time.time(),
                    "submit_mono": time.monotonic(),
                    "submit_sim_stamp": float(self.ros_now_sec()),
                }
                from luggage_gazebo.observation_execution_budget import certified_execution_wait
                try:
                    response_wait = certified_execution_wait(
                        motion_segment, self._args.plan_timeout)
                except (ValueError, AttributeError) as exc:
                    return False, "INVALID_CERTIFIED_TIMING:%s" % exc, None
                submit_times["response_wait_s"] = response_wait
                ok, message, result = self._typed.plan_motion(
                    motion_segment, response_wait,
                    name="PlanMotionV2:%s" % name,
                    admission=leg_admission,
                    should_abort=(
                        (lambda: not self._carrying_vacuum_holds()[0])
                        if leg_carries_payload
                        else None))
                self._note_leg_submitted(motion_segment, name, ok)
                self._record_submitted_leg(
                    trial, motion_segment, name, ok, message, submit_times)
            else:
                # [ST09-I S5.7] every executed leg pairs with a certified
                # route leg; the driver-side place_exit fall-through to
                # raw v1 PlanMotion is deleted with the ad-hoc builder.
                return False, "TYPED_SEGMENT_MISMATCH:%s" % name, None
        if not use_typed:
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
            "joint_ring_t0": self._segment_exec_t0,
        }
        rec["boundary"] = load_latest_boundary(
            os.environ.get("MOTION_BOUNDARY_DUMP", ""))
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
        # F23: the same GT fact as one explicit frame_dump/2 envelope
        # (record_kind gt_reference), written next to the legacy artifacts.
        gt["dump_v2"] = self._build_gt_envelope(center_base, size, yaw_base)
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

    def _build_gt_envelope(self, center_base, size, yaw_base):
        """GT dump envelope for this trial, or None with the refusal logged.

        The scene revision is the measured scene-file digest, so the
        envelope is self-consistent in v1 and frame-runtime modes alike.
        """
        from luggage_description.runtime_identity import scene_revision_from_path
        from luggage_gazebo.place_gt_dump import gt_place_dump
        self._gt_request_seq = getattr(self, "_gt_request_seq", 0) + 1
        request_id = "gt-place-%d" % self._gt_request_seq
        try:
            return gt_place_dump(
                self._scene_config,
                scene_revision=scene_revision_from_path(
                    self._scene_config_path),
                request_id=request_id,
                box_center_base=center_base, box_size_wdh=size,
                box_yaw_base=yaw_base)
        except Exception as exc:  # noqa: BLE001 - GT envelope is additive
            self.get_logger().warning(
                "gt dump envelope refused (%s); legacy GT artifacts only"
                % exc)
            return None

    def _scene_box_record(self):
        """Collision box the pick planner had to avoid, as it was added."""
        detection = self._last_pick_detection
        if not detection:
            return None
        center = detection["pose_position"]
        size = detection["size_wdh"]
        return {
            "center": center,
            "size_wdh": size,
            "z_bottom": center[2] - size[2] * 0.5,
            "z_top": center[2] + size[2] * 0.5,
        }

    def _on_joint_state(self, msg):
        names = list(msg.name)
        positions = [float(v) for v in msg.position]
        # [ST09-I G2] The Gazebo joint_state_broadcaster publishes in
        # controller-registration order (observed j1,j2,j5,j3,j4,j6) —
        # NOT the canonical elfin_joint1..6 every consumer of this
        # record assumes (pose files, already-there wrap compare, exit
        # proofs, V2 requests). Normalize AT INGESTION by name, the
        # same discipline the selector's own _on_joints uses; a
        # message without the full name set keeps raw order (and its
        # evidence trail) exactly as before.
        wanted = ["elfin_joint%d" % i for i in range(1, 7)]
        # Reported velocity, aligned by name like the positions. None when
        # the message carries no velocity for every joint: a missing field
        # is never zero (handoff_settle treats None as "not stable").
        velocity = None
        raw_velocity = [float(v) for v in msg.velocity]
        if len(raw_velocity) == len(names) and names:
            velocity = raw_velocity
        if names and positions and all(name in names for name in wanted):
            index = {name: pos for name, pos in zip(names, positions)}
            if velocity is not None:
                velocity_index = dict(zip(names, velocity))
                velocity = [velocity_index[name] for name in wanted]
            positions = [index[name] for name in wanted]
            names = list(wanted)
        record = {
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "name": names,
            "position": positions,
            "velocity": velocity,
            "rx_mono": time.monotonic(),
        }
        self._last_joint_state = record
        self._joint_ring.append(record)
        if self._segment_exec_t0 is not None:
            self._joint_ring_horizon_sec = max(
                self._joint_ring_horizon_sec,
                joint_ring_horizon_sec(
                    record["stamp"] - float(self._segment_exec_t0)))
        prune_joint_ring(
            self._joint_ring, record["stamp"], self._joint_ring_horizon_sec)

    @staticmethod
    def _segment_record(segment):
        pose = segment.target_pose
        return {
            "name": segment.name,
            "type": segment.type,
            "target_position": [pose.position.x, pose.position.y,
                                pose.position.z],
            "target_orientation": [pose.orientation.x, pose.orientation.y,
                                   pose.orientation.z, pose.orientation.w],
            "n_waypoints": len(segment.waypoints),
            "keep_tool_down": bool(segment.keep_tool_down),
            "keep_camera_down": bool(segment.keep_camera_down),
            "lock_wrist": bool(segment.lock_wrist),
            "allow_ompl_fallback": bool(segment.allow_ompl_fallback),
            "required_cartesian_fraction": float(
                segment.required_cartesian_fraction),
        }

    def _plan_failure_record(self, segments, failed, message):
        """Planning-boundary payload for a PLAN_<segment> failure.

        Without the requested pose and the start joint state, the dump cannot
        separate an unreachable target from a bad start state or a collision,
        and the next run repeats the same blind failure.
        """
        waypoints = []
        for pose in getattr(failed, "waypoints", ()) or ():
            waypoints.append([
                float(pose.position.x), float(pose.position.y),
                float(pose.position.z)])
        release_poses = []
        events = self._vacuum_events if isinstance(
            self._vacuum_events, dict) else {}
        for event in events.values():
            if isinstance(event, dict) and event.get("release_pose"):
                release_poses.append(event["release_pose"])
        return {
            "failed_segment": self._segment_record(failed),
            "failed_leg_waypoints": waypoints,
            "planner_message": str(message),
            "sequence": [self._segment_record(s) for s in segments],
            "joint_state_at_failure": self._last_joint_state,
            "pick_detection": self._last_pick_detection,
            "scene_pickup_box": self._scene_box_record(),
            "place_state": self._place_state,
            "release_poses": release_poses,
        }

    def _dump_place(self, trial, extra_files=None, extra=None, debug=None):
        dump_dir = getattr(self._args, "dump_dir", "") or ""
        if not dump_dir:
            return None
        contacts = (getattr(self, "_home_arm_state", None) or {}).get("contacts")
        if contacts and str(getattr(trial, "fail_code", "") or "").startswith(
                "EXIT_FAILED"):
            trial.extras["gz_contacts"] = contacts
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
        self._freeze_motion_replay(dest, trial)
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
        boundary = (getattr(trial, "extras", None) or {}).get(
            "perception_boundary")
        if boundary:
            merged["perception_boundary.json"] = boundary
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
            dump_v2 = debug.get("dump_v2")
            if dump_v2 is not None:
                from luggage_gazebo.frame_dump import write_dump
                try:
                    write_dump(os.path.join(dest, "gt_place_dump_v2.json"),
                               dump_v2,
                               scene_revision=dump_v2["scene_revision"],
                               geometry_hash=dump_v2["geometry_hash"],
                               request_id=dump_v2["request_id"],
                               map_revision=dump_v2["map_revision"])
                    trial.extras.setdefault(
                        "place_debug", {})["gt_dump_v2"] = "gt_place_dump_v2.json"
                    viz = String()
                    viz.data = json.dumps(dump_v2, sort_keys=True,
                                          allow_nan=False)
                    self._dump_viz_pub.publish(viz)
                except FileExistsError:
                    self.get_logger().warning(
                        "gt dump v2 already present in %s; not overwritten"
                        % dest)
                except OSError as exc:
                    self.get_logger().warning("gt dump v2 write failed: %s" % exc)
            with open(os.path.join(dest, "trial.json"), "w", encoding="utf-8") as handle:
                json.dump(trial_to_dict(trial), handle, indent=2, sort_keys=True, default=str)
                handle.write("\n")
        return dest

    def _freeze_motion_replay(self, dest, trial):
        """T2 motion/perception bundle: planned traj, scene geometry, joints."""
        replay_dir = os.path.join(dest, "replay")
        os.makedirs(replay_dir, exist_ok=True)
        artifacts = {}
        missing = []
        boundary = load_latest_boundary(
            os.environ.get("MOTION_BOUNDARY_DUMP", ""))
        fail = str(getattr(trial, "fail_code", "") or "")
        from luggage_gazebo.motion_replay_boundary import replay_boundary
        boundary = replay_boundary(
            trial.extras.get("submitted_legs", []),
            getattr(self, "_segment_exec_name", None), boundary,
            typed=getattr(self, "_typed", None) is not None)
        detect_stage = fail.startswith("DETECT_")
        from luggage_gazebo.selection_capture import preserve_failed_selection
        selection_failure = preserve_failed_selection(
            getattr(self, '_frame_selection_capture_dir', None), replay_dir)
        if selection_failure is not None and not detect_stage:
            selection_artifacts, selection_missing = selection_failure
            artifacts.update(selection_artifacts)
            missing.extend(selection_missing)
        elif boundary and not detect_stage:
            with open(os.path.join(replay_dir, "last_boundary.json"), "w",
                      encoding="utf-8") as handle:
                json.dump(boundary, handle, indent=2, sort_keys=True,
                          default=str)
                handle.write("\n")
            artifacts["last_boundary"] = "replay/last_boundary.json"
            if not (boundary.get("trajectory") or boundary.get("fjt_trajectory")):
                missing.append("trajectory")
            elif (boundary.get("trajectory") or {}).get("truncated"):
                missing.append("trajectory_truncated")
        elif not detect_stage:
            missing.append("last_boundary")
        scene = self._scene_snapshot()
        with open(os.path.join(replay_dir, "planning_scene.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(scene, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        artifacts["planning_scene"] = "replay/planning_scene.json"
        if scene.get("error"):
            missing.append("planning_scene")
        fail = str(getattr(trial, "fail_code", "") or "")
        if fail:
            joints = joint_ring_from(
                getattr(self, "_joint_ring", ()),
                getattr(self, "_segment_exec_t0", None))
        else:
            joints = list(getattr(self, "_joint_ring", ()) or ())
        with open(os.path.join(replay_dir, "joint_ring.jsonl"), "w",
                  encoding="utf-8") as handle:
            for row in joints:
                handle.write(json.dumps(row, sort_keys=True) + "\n")
        artifacts["joint_ring"] = "replay/joint_ring.jsonl"
        if not joints and not detect_stage:
            missing.append("joint_ring")
        surface = getattr(self, "_surface_2d", None)
        if surface:
            with open(os.path.join(replay_dir, "surface_2d.json"), "w",
                      encoding="utf-8") as handle:
                json.dump(surface, handle, indent=2, sort_keys=True,
                          default=str)
                handle.write("\n")
            artifacts["surface_2d"] = "replay/surface_2d.json"
        # Node-side failure capture (occ snapshot + source cloud + depth),
        # referenced by the boundary record's "capture" block. Listed in
        # artifacts but never in missing this wave: absence must not flip
        # replay_possible before the capture has proven reliable in sim.
        capture = (boundary or {}).get("capture") or {}
        for key in ("surface_2d_dump", "sidecar"):
            src = capture.get(key)
            if src and os.path.isfile(src):
                shutil.copy(src, os.path.join(
                    replay_dir, "capture_%s" % os.path.basename(src)))
                artifacts["capture_%s" % key] = (
                    "replay/capture_%s" % os.path.basename(src))
        for key in ("cloud", "depth"):
            block = capture.get(key) or {}
            src = block.get("path")
            if src and os.path.isfile(src):
                shutil.copy(src, os.path.join(
                    replay_dir, "capture_%s" % os.path.basename(src)))
                artifacts["capture_%s" % key] = (
                    "replay/capture_%s" % os.path.basename(src))
        if capture.get("missing"):
            trial.extras.setdefault("replay", {})[
                "capture_missing"] = list(capture["missing"])
        suction, serr = self.suction_xyz()
        extras = getattr(trial, "extras", None) or {}
        perception = {
            "suction_xyz": suction,
            "suction_error": serr,
            "pick_detection": self._last_pick_detection,
            "gz_box": self._gz_box_pose() if hasattr(self, "_gz_box_pose")
            else None,
            "detect_message": extras.get("detect_message"),
            "support_stream": extras.get("support_stream"),
        }
        with open(os.path.join(replay_dir, "perception_refs.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(perception, handle, indent=2, sort_keys=True, default=str)
            handle.write("\n")
        artifacts["perception_refs"] = "replay/perception_refs.json"
        stage = "motion"
        if selection_failure is not None and not detect_stage:
            stage = 'selection'
        elif detect_stage:
            stage = "detect"
        elif fail.startswith("PLACE_PLAN_"):
            stage = fail[len("PLACE_PLAN_"):]
        elif fail == "GOTO_FAILED":
            stage = "goto_observe"
        elif self._segments_log:
            stage = str(self._segments_log[-1].get("name") or "motion")
        manifest = replay_manifest(
            "trial_%02d" % int(getattr(trial, "index", 0) or 0),
            str(getattr(trial, "fail_code", "") or ""),
            stage, artifacts, missing)
        with open(os.path.join(replay_dir, "manifest.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(manifest, handle, indent=2, sort_keys=True)
            handle.write("\n")
        trial.extras.setdefault("replay", {})["dir"] = replay_dir
        trial.extras["replay"]["capture_complete"] = manifest["capture_complete"]
        trial.extras["replay"]["replay_possible"] = manifest["replay_possible"]
        trial.extras["replay"]["stage"] = stage
        return manifest

    def _placed_primitive_size(self, visual, command_size):
        """[G3 wave3] Placed-body primitive extents from the tier manifest.

        The manifest is the single machine-readable truth for asset,
        GT-reference and primitive collision sizes (pipeline:
        ``generate_sized_suitcases.py sync-manifest``); the spawner's
        command size stays the fallback when no record exists.
        """
        try:
            from luggage_description.suitcase_visual import (
                sized_model_name,
                sized_suitcases_manifest_path,
                size_tier_name,
            )
        except ImportError:
            return None
        tier = size_tier_name(command_size)
        if not tier:
            return None
        root = getattr(self, "_placed_models_root", None)
        if root is None:
            try:
                from ament_index_python.packages import (
                    get_package_share_directory)
                root = os.path.join(
                    get_package_share_directory("luggage_gazebo"), "models")
            except Exception:  # noqa: BLE001 - package share may be absent
                root = ""
            self._placed_models_root = root
        if not root:
            return None
        path = sized_suitcases_manifest_path(root)
        if not os.path.isfile(path):
            return None
        manifest = getattr(self, "_placed_manifest", None)
        if manifest is None:
            with open(path, "r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            self._placed_manifest = manifest
        entry = manifest.get(sized_model_name(str(visual), tier)) or {}
        primitive = entry.get("primitive_collision_size")
        if not primitive:
            return None
        return [float(v) for v in primitive]

    def _placed_lid_offset(self, visual, command_size):
        """Tier lid_offset for the contact-recess collision (manifest
        constant, same loader as _placed_primitive_size; None makes the
        SDF builder fail closed rather than measure online)."""
        try:
            from luggage_description.suitcase_visual import (
                sized_model_name,
                sized_suitcases_manifest_path,
                size_tier_name,
            )
        except ImportError:
            return None
        tier = size_tier_name(command_size)
        if not tier:
            return None
        root = getattr(self, "_placed_models_root", None)
        if root is None:
            try:
                from ament_index_python.packages import (
                    get_package_share_directory)
                root = os.path.join(
                    get_package_share_directory("luggage_gazebo"), "models")
                self._placed_models_root = root
            except Exception:  # noqa: BLE001 - share dir absent offline
                return None
        path = sized_suitcases_manifest_path(root)
        if not os.path.isfile(path):
            return None
        manifest = getattr(self, "_placed_manifest", None)
        if manifest is None:
            with open(path, "r", encoding="utf-8") as handle:
                manifest = json.load(handle)
            self._placed_manifest = manifest
        entry = manifest.get(sized_model_name(str(visual), tier)) or {}
        off = entry.get("lid_offset")
        return float(off) if off else None

    def _release_capture_event(self, event, **details):
        """[prior-g1 merge] Measured-replay provenance (env-gated, inert
        unless ELFIN_RELEASE_CAPTURE_DIR is set)."""
        directory = getattr(self, '_release_capture_directory', None)
        if directory:
            with open(os.path.join(directory, 'events.jsonl'), 'a') as handle:
                handle.write(json.dumps(dict(event=event, wall=time.time(),
                    monotonic=time.monotonic(), ros=self.ros_now_sec(),
                    model=self._box_model, details=details))+'\n')

    def _stop_release_capture(self):
        recorder = getattr(self, '_release_capture_process', None)
        if recorder is not None:
            if recorder.poll() is None:
                recorder.terminate()
            recorder.wait(timeout=5.)
            self._release_capture_process = None

    def _start_release_capture(self):
        root = os.environ.get('ELFIN_RELEASE_CAPTURE_DIR', '').strip()
        if not root:
            return
        directory = os.path.join(root, str(self._box_model))
        os.makedirs(directory, exist_ok=False)
        self._release_capture_directory = directory
        topic = os.environ.get('ELFIN_BOX_CONTACT_TOPIC', '').strip()
        if not topic:
            raise RuntimeError('release capture requires contact topic')
        # Owned child remains in the driver process group and also self-expires.
        with open(os.path.join(directory, 'recorder.log'), 'w') as log:
            self._release_capture_process = subprocess.Popen([
                sys.executable, '-m', 'luggage_gazebo.release_capture',
                '--output', directory, '--contact-topic', topic],
                stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic()+3.
        while not os.path.isfile(os.path.join(directory, 'ready.json')):
            if self._release_capture_process.poll() is not None or time.monotonic() >= deadline:
                self._release_capture_process.terminate()
                self._release_capture_process.wait(timeout=5.)
                raise RuntimeError('release recorder failed to start')
            time.sleep(.02)
        self._release_capture_event('before_vacuum_off')

    def _release_kinematic(self, model, slot_meta):
        """[planning-verification mode, user directive 2026-10-19]

        Keep the carried body as a STATIC obstacle at the DECLARED
        release end-state: no delete, no dynamic respawn, no free fall,
        no ejection — the contact-solver artifacts of the dynamic track
        are out of scope here by declaration. ONE declared transition:
        set_pose to the slot's support-rest pose; presence re-read via
        the fresh-only pose stream ("the released box must not vanish").

        Proves: planning + state-chain feasibility (placement legality,
        full-route legality incl. payload, multi-box state via the
        committed ACTUAL pose). Does NOT prove contact-safety, suction
        reliability, or landing accuracy — those stay on the separate
        dynamic track. The support/end-state conditions are still
        checked: the declared pose must match the slot's support rest
        and the world must confirm it (<=5 mm).
        """
        pose_world = (slot_meta or {}).get("pose_world") or {}
        expected = pose_world.get("position")
        if not expected or len(expected) < 3:
            return False, "no declared slot pose", {"mode": "kinematic"}
        yaw = float(pose_world.get("yaw") or 0.0)
        declared = [float(expected[0]), float(expected[1]),
                    float(expected[2]), 0.0, 0.0, yaw]
        record = {"mode": "kinematic",
                  "declared_pose_rpy6": [round(v, 6) for v in declared]}
        ok, msg = self._set_model_pose_declared(model, declared)
        record["set_pose"] = msg
        if not ok:
            return False, "kinematic set_pose refused: %s" % msg, record
        pose7, evidence = self._release_pose_stream().read_release_pose(
            model, self.ros_now_sec())
        record["presence_evidence"] = evidence
        if pose7 is None:
            return False, "kinematic presence refused (%s)" % (
                evidence.get("reason")), record
        err = math.dist(declared[:3], pose7[:3])
        record["declared_vs_world_m"] = round(err, 6)
        if err > 0.005:
            return False, "kinematic pose mismatch %.4f m" % err, record
        record["verdict"] = "declared_state_confirmed"
        return True, "kinematic", record

    def _set_model_pose_declared(self, model, pose_rpy6):
        """One declared set_pose via the bridged world service (the same
        channel the spawner/vacuum follow use). Fail-closed message."""
        from ros_gz_interfaces.srv import SetEntityPose
        cli = getattr(self, "_set_pose_cli", None)
        if cli is None:
            cli = self.create_client(
                SetEntityPose, "/world/%s/set_pose" % GZ_WORLD)
            self._set_pose_cli = cli
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, "set_pose service unavailable"
        request = SetEntityPose.Request()
        request.entity.name = str(model)
        x, y, z, roll, pitch, yaw = [float(v) for v in pose_rpy6]
        qx, qy, qz, qw = rpy_to_quaternion([roll, pitch, yaw])
        request.pose.position.x, request.pose.position.y = x, y
        request.pose.position.z = z
        request.pose.orientation.x, request.pose.orientation.y = qx, qy
        request.pose.orientation.z, request.pose.orientation.w = qz, qw
        resp = self.call_srv(cli, request, timeout=10.0)
        if resp is None or not getattr(resp, "success", False):
            return False, "set_pose rejected"
        return True, "ok"

    def _release_as_dynamic(self):
        """Replace the static carry body with the same mesh, dynamic, in place.

        Carry stays static so ``set_pose`` follow does not fight the solver.
        Called after vacuum-off and before retreat. Returns
        ``(ok, message, release_pose)``.
        """
        physics = getattr(self, "_release_physics", None) or {}
        size = list(physics.get("command_size") or [])
        command_size = list(size)
        visual = str(physics.get("visual_id") or "")
        mass = physics.get("mass_kg")
        model = str(self._box_model or physics.get("model_name") or "")
        # [G3 wave3] single source: the tier manifest's registered
        # primitive_collision_size (falls back to the command size).
        size = list(self._placed_primitive_size(visual, size) or size)
        # [r7] Release pose from the continuous pose/info cache —
        # fresh-only, instance- and epoch-matched, NO CLI fallback. The
        # pose is FROZEN here, before the static delete; a refusal is a
        # named stop that keeps the entity and the pending obstacle
        # (vacuum is already off; nothing is deleted below this point).
        release, pose_evidence = self._release_pose_stream(
            ).read_release_pose(model, self.ros_now_sec())
        self._release_pose_evidence = pose_evidence
        if release is None:
            return False, "release pose refused (%s)" % (
                pose_evidence.get("reason")), None
        # The cache speaks quaternion; the release path's frozen record
        # keeps the historical rpy6 shape (spawn + verify read it).
        release = _quat7_to_rpy6(release)
        pose_evidence["pose_rpy6"] = [round(v, 6) for v in release]
        # [prior-g1 merge] measured-replay provenance hook (record-only)
        self._release_capture_event('pre_recreation_pose', pose=release)
        if len(size) != 3 or not visual or mass is None or not model:
            return False, "missing physics identity", release
        try:
            mass_kg = float(mass)
            topic = os.environ.get("ELFIN_BOX_CONTACT_TOPIC", "").strip() or None
            from luggage_description.gazebo_spawn_collision import BOX_FULL_MASK
            # Placed body carries a box-primitive collision (authorized
            # 2026-10-15): a resting suitcase STL on the container
            # trimesh drives ODE trimesh-trimesh contact pathological
            # (RTF ~0.03 measured, rehearsal_06); the primitive keeps
            # the outer extents and the mesh visual for perception.
            sdf = pickup_visual_sdf(
                model, size, mass_kg, visual, static=False,
                contact_topic=topic, collide_bitmask=BOX_FULL_MASK,
                collision_kind="box_recess",
                lid_offset=self._placed_lid_offset(visual, command_size))
            release_dump = os.environ.get("ELFIN_BOX_RELEASE_SDF", "").strip()
            if release_dump:
                with open(release_dump, "w", encoding="utf-8") as handle:
                    handle.write(sdf)
            if getattr(self, '_release_capture_directory', None):
                with open(os.path.join(self._release_capture_directory, 'release.sdf'), 'w') as handle:
                    handle.write(sdf)
        except (TypeError, ValueError) as exc:
            return False, str(exc), release
        if release is None or len(release) < 6:
            return False, "release pose refused (malformed)", release
        # [release observation] Sim-clock timing of the static->dynamic
        # swap, for the synchronized release-window capture (the pose
        # stream and these stamps share the simulation clock).
        self._release_dynamic_timing = {
            "t_delete_req_sim": float(self.ros_now_sec())}
        delete = DeleteEntity.Request()
        delete.entity.name = model
        delete.entity.type = Entity.MODEL
        removed = self.call_srv(self._gz_delete, delete, timeout=15.0)
        self._release_capture_event('delete_response', success=bool(removed and removed.success))
        if removed is None or not getattr(removed, "success", False):
            return False, "delete failed", release
        self._release_dynamic_timing["t_delete_done_sim"] = float(
            self.ros_now_sec())
        spawn = SpawnEntity.Request()
        spawn.entity_factory.name = model
        spawn.entity_factory.sdf = sdf
        qx, qy, qz, qw = rpy_to_quaternion(release[3:6])
        spawn.entity_factory.pose.position.x = float(release[0])
        spawn.entity_factory.pose.position.y = float(release[1])
        spawn.entity_factory.pose.position.z = float(release[2])
        spawn.entity_factory.pose.orientation.x = float(qx)
        spawn.entity_factory.pose.orientation.y = float(qy)
        spawn.entity_factory.pose.orientation.z = float(qz)
        spawn.entity_factory.pose.orientation.w = float(qw)
        created = self.call_srv(self._gz_create, spawn, timeout=15.0)
        self._release_capture_event('create_response', success=bool(created and created.success))
        if created is None or not getattr(created, "success", False):
            return False, "create failed", release
        self._release_dynamic_timing["t_spawn_done_sim"] = float(
            self.ros_now_sec())
        self._release_dynamic_timing["spawn_pose_world"] = [
            float(v) for v in release[:3]]
        # [r7 evidence] First physical state of the dynamic body vs the
        # spawn request (<=1 mm / <=0.5 deg is a verdict gate, not a
        # motion gate) + finite-difference first-step velocity.
        first = _first_dynamic_state(model)
        if first.get("available"):
            dx = math.dist(release[:3], first["pose6"][:3])
            first["spawn_request_rpy6"] = [round(v, 6) for v in release]
            first["position_delta_m"] = round(dx, 6)
            first["rotation_delta_deg"] = round(_quat_angle_deg(
                rpy_to_quaternion(release[3:6]),
                first["pose6"][3:7]), 4)
        self._release_dynamic_timing["first_dynamic_state"] = first
        return True, "dynamic", release

    def _release_dwell(self, dwell_s, anchor_stamp):
        """[release dwell, user directive 2026-10-19] Hold the retreat
        knot while the released box falls clear of the panel.

        r5 evidence: the retreat began 0.39 s after vacuum-off while the
        box still sat at the release height (top ~= panel underside at
        zero vacuum-grasp clearance); the first, lateral stroke swept
        the box top for 20 ms. Sim-clock dwell from the anchor (swap
        completion if known, else the detach response); the box pose at
        dwell end is recorded as separation evidence. Never aborts —
        an unreadable pose is recorded and the caller's existing
        contracts decide the rest.
        """
        record = {"dwell_s": float(dwell_s),
                  "anchor_sim": float(anchor_stamp or 0.0)}
        if float(dwell_s) <= 0.0:
            record["skipped"] = True
            return record
        deadline = float(anchor_stamp or 0.0) + float(dwell_s)
        while True:
            now = float(self.ros_now_sec())
            if now >= deadline:
                break
            time.sleep(min(0.05, max(deadline - now, 1e-3)))
        record["t_end_sim"] = float(self.ros_now_sec())
        record["box_pose_at_dwell_end"] = self._gz_box_pose()
        return record

    def _poses_after_settle(self):
        """Return ``(residual_start, residual_end, verdict)``.

        After the body has stopped, residual drift is the motion during
        ``drift_wait``. If it never settles, the pair spans the poll so the
        fall still fails the drift gate. ``err_z`` uses the end pose against
        the slot support rest, not the planned release height.
        """
        if self._args.plan_only:
            pose = self._gz_box_pose()
            return pose, pose, {
                "settled": False, "pose": _xyz3(pose), "window_move_m": None}
        samples = []
        captured = []
        t0 = time.monotonic()
        verdict = {"settled": False, "pose": None, "window_move_m": None}
        last = None
        while True:
            last = self._gz_box_pose()
            xyz = _xyz3(last)
            now = time.monotonic() - t0
            if xyz is not None:
                samples.append((now, xyz))
                captured.append(last)
                verdict = settle_verdict(
                    samples, move_m=SETTLE_MOVE_M, window_sec=SETTLE_WINDOW_SEC)
            # One ``ign model --pose`` call can outlast the budget. Keep
            # sampling until the window can be judged, then stop.
            if verdict.get("settled"):
                break
            if now >= SETTLE_BUDGET_SEC and len(samples) >= 2:
                break
            if now >= SETTLE_BUDGET_SEC + 5.0:
                break
            time.sleep(0.05)
        if verdict.get("settled"):
            start = last
            time.sleep(float(self._args.drift_wait))
            end = self._gz_box_pose()
            return start, end, verdict
        start = captured[0] if captured else last
        return start, last, verdict

    def _abort(self, trial, code, carrying, extra=None):
        self._release_capture_event('abort', code=code)
        self._stop_release_capture()
        trial.fail_code = code
        pending_handoff = getattr(self, "_handoff_failure_record", None)
        if pending_handoff is not None:
            extra = dict(extra or {})
            extra["handoff.json"] = pending_handoff
            self._handoff_failure_record = None
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

    def _contact_capture_gate(self, trial, extras):
        """Refuse the first certified leg until contact capture is live.

        Active only when ``M3_CONTACT_CAPTURE=1``. The check is content:
        the spawned box SDF is the static carry model and carries the
        contact sensor, the follow trace is being written now, pose/info
        is being recorded, and the contact topic has a publisher. An empty
        contact log is not treated as ready or as proof of no contact.
        """
        if os.environ.get("M3_CONTACT_CAPTURE", "") != "1":
            return None
        script_dir = os.environ.get("ELFIN_ST09_SCRIPTS", "")
        if script_dir and script_dir not in sys.path:
            sys.path.insert(0, script_dir)
        import contact_capture
        topic = os.environ.get("ELFIN_BOX_CONTACT_TOPIC", "").strip()
        info = ""
        if topic:
            try:
                proc = subprocess.run(
                    ["ign", "topic", "-i", "-t", topic],
                    capture_output=True, text=True, timeout=8.0, check=False)
                info = (proc.stdout or "") + (proc.stderr or "")
            except (OSError, subprocess.TimeoutExpired) as exc:
                info = "ign topic failed: %s" % exc
        ready, detail = contact_capture.capture_ready(
            follow_trace=os.environ.get("ELFIN_FOLLOW_TRACE", ""),
            topic_info=info,
            box_sdf_path=os.environ.get("ELFIN_BOX_SDF_DUMP", ""),
            robot_urdf_path=os.environ.get(
                "ELFIN_ROBOT_URDF", "/tmp/elfin_s20_gazebo.urdf"),
            recorder_status_path=os.environ.get(
                "ELFIN_CONTACT_STATUS", ""),
            now=time.time(),
        )
        token = os.environ.get("ELFIN_GZ_STRIP_PANEL_COLLISION", "").strip()
        if token:
            from luggage_description.gazebo_spawn_collision import (
                panel_collision_absent)
            strip_ok, strip_detail = panel_collision_absent(
                os.environ.get("ELFIN_ROBOT_URDF", "/tmp/elfin_s20_gazebo.urdf"),
                token)
            detail["strip"] = strip_detail
            ready = ready and strip_ok
        trial.extras["contact_capture_gate"] = detail
        extras["contact_capture_gate.json"] = json.dumps(detail)
        if ready:
            return None
        return self._abort(trial, "CONTACT_CAPTURE_NOT_READY", True, extra=extras)

    def run_place_from_carry(self, pick_msg, trial, slot, slot_meta):
        self._probe_joints = None
        self._last_submitted_leg = None
        self._handoff_failure_record = None
        self._payload_detached = False
        self._follow_skipped0 = self._follow_skipped()
        extras = {"slot.json": slot_meta}
        if not self._box_model:
            self._box_model = self.gt_model_name()
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

        route_v2 = list(getattr(self, "_selected_carry_v2", None) or ())
        route_v1 = list(getattr(self, "_selected_carry_v1", None) or ())
        if route_v2:
            # [ST09-I] certified route legs from SelectPlacePathV2 are
            # the place motion; the fixed-clearance build was a bypass.
            if len(route_v1) != len(route_v2):
                return self._abort(trial, "PLACE_ROUTE_LEGS_MISMATCH", True,
                                   extra=extras)
            segments = route_v1
            self._frame_motion_segments = route_v2
            self._frame_motion_index = 0
            trial.segments_planned = len(segments)
            trial.extras["build"] = "certified route (%d legs)" % len(route_v2)
            self._set_state("SEQUENCE_BUILT", note="certified route")
            blocked = self._contact_capture_gate(trial, extras)
            if blocked is not None:
                return blocked
        elif self._typed is not None or True:
            # The build endpoint no longer produces place legs.
            return self._abort(trial, "PLANNING_ROUTE_REQUIRED", True,
                               extra=extras)

        names = [s.name for s in segments]
        post_release = []
        remaining = []
        for segment in segments:
            if segment.name in ("retreat", "return"):
                post_release.append(segment)
            else:
                remaining.append(segment)
        retreat_seg = post_release[0] if post_release else None
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
            diagnostic = getattr(self, "_transit_capture_exit", None)
            if diagnostic is not None:
                reason = diagnostic.before(segment.name)
                if reason:
                    return self._abort(trial, reason, True, extra=extras)
            ok, code, rec = self._execute_segment(segment, trial)
            if diagnostic is not None:
                if ok:
                    trial.segments_succeeded += 1
                return diagnostic.finish(self, trial, ok, code, extras)
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
        joints_before_detach = None
        detach_request_stamp = None
        detach_response_stamp = None
        if getattr(self._args, "use_vacuum", False) and not self._args.plan_only:
            self._start_release_capture()
            joints_before_detach = self._joint_sample()
            detach_request_stamp = float(self.ros_now_sec())
            # Unlock follow at this instant, before retreat plans a new TCP.
            off_ok, off_msg = self.vacuum_command(False)
            self._release_capture_event('vacuum_off_response', success=off_ok)
            detach_response_stamp = float(self.ros_now_sec())
            trial.extras["vac_detach"] = off_msg
            if off_ok:
                # Later legs are unloaded. attached=false is expected.
                self._payload_detached = True
            if not off_ok:
                # [S5.7-6 negative] release backend refused: the
                # retreat never submits (named stop, zero controller
                # submissions past the release boundary).
                return self._abort(trial, "VACUUM_DETACH", True, extra=extras)
            # [release-failure contract] The world changed the instant
            # the box left the tool — notify the map BEFORE any
            # acceptance decision so a later verify failure can never
            # leave this spot readable as free. The commanded footprint
            # (slot record + box wdh) is container_link, matching the
            # mapper's frame. Fail-closed: no notification = the map is
            # blind to a landed box; stop the session by name rather
            # than continue.
            self._frame_released = True
            notify_ok, notify_msg = self._notify_box_released()
            trial.extras["release_notify"] = notify_msg
            if not notify_ok:
                return self._abort(
                    trial, "RELEASE_NOTIFY_FAILED", False, extra=extras)
            release_mode = str(
                getattr(self._args, "release_mode", "kinematic")
                or "kinematic").lower()
            if release_mode == "kinematic":
                k_ok, k_msg, k_rec = self._release_kinematic(
                    str(self._box_model or ""), slot_meta)
                trial.extras["release_kinematic"] = k_rec
                trial.extras["box_pose_at_release"] =                     k_rec.get("declared_pose_rpy6")
                if not k_ok:
                    return self._abort(
                        trial, "KINEMATIC_RELEASE(%s)" % k_msg,
                        False, extra=extras)
                release_pose = list(
                    k_rec.get("declared_pose_rpy6") or [])
            else:
                dyn_ok, dyn_msg, release_pose = self._release_as_dynamic()
                trial.extras["dynamic_body"] = dyn_msg
                trial.extras["box_pose_at_release"] = release_pose
                trial.extras["release_pose"] = getattr(
                    self, "_release_pose_evidence", None)
                trial.extras["release_dynamic_timing"] = getattr(
                    self, "_release_dynamic_timing", None)
                if not dyn_ok:
                    return self._abort(
                        trial, "DYNAMIC_BODY", False, extra=extras)
            # [ST09-I S5.7-6] explicit release-boundary phase
            # transition BEFORE any post-release leg: the certifier
            # side activates the UNLOADED phase only against verified
            # detach evidence; a refusal stops the session by name
            # (retreat never reaches the executor).
            phase_ok, phase_reason = self._transition_admission_unloaded(
                str(off_msg or ""))
            trial.extras["admission_phase"] = phase_reason
            if not phase_ok:
                return self._abort(
                    trial, "ADMISSION_PHASE_%s" % (
                        phase_reason or "TRANSITION_FAILED"),
                    False, extra=extras)

        if (retreat_seg is not None and not self._args.plan_only
                and str(getattr(self._args, "release_mode", "kinematic")
                        or "kinematic").lower() != "kinematic"):
            # [release dwell, dynamic mode only] The box falls DURING
            # this dwell; the arm stays on the certified knot the whole
            # time, so the retreat_start contract below is unchanged.
            # Kinematic mode has no dynamic body to wait for.
            timing = getattr(self, "_release_dynamic_timing", None) or {}
            dwell_anchor = timing.get("t_spawn_done_sim") \
                or detach_response_stamp
            dwell = self._release_dwell(
                getattr(self._args, "release_dwell_s", 2.0), dwell_anchor)
            trial.extras["release_dwell"] = dwell
            extras["release_dwell.json"] = dwell

        if retreat_seg is not None and not self._args.plan_only:
            # The certified retreat knot is descend's final waypoint. Submit
            # only after a joint sample newer than the detach response sits
            # inside the existing QP tolerance of that knot.
            evidence = self._await_retreat_start_contract(
                retreat_seg,
                detach_request_stamp=detach_request_stamp,
                detach_response_stamp=detach_response_stamp,
                joints_before_detach=joints_before_detach)
            trial.extras["retreat_start"] = evidence
            extras["retreat_start.json"] = evidence
            if evidence.get("status") != "ready":
                return self._abort(
                    trial, "QP_START_STATE_MISMATCH", False, extra=extras)

        if retreat_seg is not None:
            self._set_state("PLACE_RETREAT")
            for post_seg in post_release:
                self._release_capture_event('post_release_segment_start')
                ok, code, rec = self._execute_segment(post_seg, trial)
                self._release_capture_event('post_release_segment_end', success=ok, code=code,
                                           name=(rec or {}).get('name'))
                if ok:
                    trial.segments_succeeded += 1
                else:
                    return self._abort(trial, code, False, extra=extras)

        pose_release, pose_drift, settle = self._poses_after_settle()
        self._release_capture_event('settle_complete', pose=pose_drift, verdict=settle)
        self._stop_release_capture()
        # Preserve the raw comparison inputs before commit or verification can
        # abort. The success-only summary below cannot diagnose a failed gate.
        trial.extras["verify_inputs"] = {
            "box_pose_after_release": pose_release,
            "box_pose_after_drift_wait": pose_drift,
            "box_pose_at_release": trial.extras.get("box_pose_at_release"),
            "expected_box_position_world": list(slot_meta["pose_world"]["position"]),
        }
        trial.extras["settle"] = {
            "settled": bool(settle.get("settled")),
            "window_move_m": settle.get("window_move_m"),
            "budget_sec": SETTLE_BUDGET_SEC,
        }
        release_xyz = _xyz3(trial.extras.get("box_pose_at_release"))
        landed_xyz = _xyz3(pose_drift or pose_release)
        if release_xyz and landed_xyz:
            trial.extras["fall_m"] = xyz_separation(release_xyz, landed_xyz)
        self._set_state("COMMITTED")
        if (not self._args.plan_only) and getattr(self._args, "use_vacuum", False):
            # [G3 wave5] the slot-named scene object channel is gone; the
            # confirmed cargo-map commit is the single rigid-obstacle
            # source (occupancy columns, map_revision-bound).
            trial.extras["add_placed"] = "occ-only"
            if not self._commit_to_cargo_map(trial, slot, slot_meta):
                return self._abort(trial, "COMMIT_FAILED", False, extra=extras)
            self._set_place_touch(False)

        self._set_state("VERIFIED")
        expected = slot_meta["pose_world"]["position"]
        actual = _xyz3(pose_drift or pose_release)
        # [release-failure contract] Evidence lands BEFORE the gates:
        # success and failure share one dump path, and a failing verify
        # must not discard the poses/direction/timing that explain it
        # (witness r10: box 1's settled direction was lost exactly here).
        trial.extras["verify"] = {
            "box_pose_after_release": pose_release,
            "box_pose_after_drift_wait": pose_drift,
            "box_pose_at_release": trial.extras.get("box_pose_at_release"),
            "expected_pose_world": expected,
            "err_xy": None, "err_z": None, "drift": None,
            "release_drift_direction_world": None,
            "fall_m": trial.extras.get("fall_m"),
            "settled": bool(settle.get("settled")),
            "inside_inner_box": None,
            "detach_request_stamp": locals().get("detach_request_stamp"),
            "detach_response_stamp": locals().get("detach_response_stamp"),
            "verdict": "pending",
        }
        if (actual is None
                and getattr(self._args, "use_vacuum", False)
                and not self._args.plan_only):
            trial.extras["verify"]["verdict"] = "PLACE_VERIFY_XY"
            return self._abort(trial, "PLACE_VERIFY_XY", False, extra=extras)
        if actual is not None:
            gate = self._evaluate_place_verify(
                trial, expected, actual, pose_release, pose_drift)
            if gate is not None:
                trial.extras["verify"]["verdict"] = gate
                return self._abort(trial, gate, False, extra=extras)
        # Success completes the shared evidence record (same path the
        # failures already wrote; only the verdict flips).
        trial.extras["verify"].update({
            "roll": trial.roll, "pitch": trial.pitch,
            "support_rest": list(expected) if expected is not None else None,
            "verdict": "ok",
        })
        trial.extras["follow_skipped_delta"] = (
            self._follow_skipped() - self._follow_skipped0)

        debug = self._capture_success_debug(
            trial, slot, slot_meta, pose_drift or pose_release)
        trial.extras.setdefault("place_debug", {})["gt_ply"] = "gt_interior_box.ply"

        self._set_state("HOME")
        if not self._args.plan_only:
            goto_ok, goto_msg, _ = self._home_arm()
            trial.extras["return_observe"] = goto_msg
            trial.extras["exit_yaw"] = dict(self._exit_yaw or {})
            if not goto_ok:
                trial.fail_code = self.home_arm_code()
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
        msg.height_valid = False
        msg.height_source = DetectedLuggage.HEIGHT_SOURCE_UNAVAILABLE
        msg.top_surface_valid = False
        return msg

    def run_trial(self, index, slot=None, slot_meta=None,
                  already_spawned=False, keep_placed=False):
        t0 = time.time()
        self._yolo_decision_record = None
        self._frozen_bundle = None
        self._timeline = []
        self._segments_log = []
        self._tf_trace = []
        self._segment_exec_t0 = None
        self._segment_exec_name = None
        self._joint_ring_horizon_sec = JOINT_RING_FLOOR_SEC
        self._place_state = "INIT"
        self._box_model = ""
        self._release_physics = None
        self._keep_placed = bool(keep_placed)
        trial = PlaceTrial(index=index)
        trial.extras = {}

        if self._args.plan_only or self._args.payload == "none":
            size = _parse_synthetic_size(
                getattr(self._args, "synthetic_size", ""))
            if size is None:
                trial.fail_code = "SYNTHETIC_SIZE_REQUIRED"
                trial.extras["synthetic_plan_only"] = True
                return trial
            pick = self._dummy_pick(size)
            trial.extras["synthetic_plan_only"] = True
            trial.extras["synthetic_size_wdh"] = list(size)
            if slot is None:
                slot, slot_meta = self._fixed_slot(pick)
            trial.catalog_id = "synthetic"
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
            slot, slot_meta = self._place_slot_for(pick_msg, box_msg, trial)
            if slot is None:
                trial.place_state = self._place_state or "CARRY_READY"
                self._dump_place(trial)
                return trial
        result = self.run_place_from_carry(pick_msg, trial, slot, slot_meta)
        result.wall_time_sec = time.time() - t0
        if (not keep_placed) and result.fail_code not in CARRYING_ABORTS:
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
        return result

    def _authorized_pick_record(self, auth):
        """Stamp, source and size of the sample that authorized this pick.

        The scene AABB is computed from that same message. The selected
        detector-window row carries its own size, so a later read can see
        whether authorization, the window row and the scene box agree.
        """
        from luggage_planning.pick_authorization import (
            authorized_pick_boundary,
            observation_gate,
        )
        from luggage_planning.pickup_collision import pickup_collision_aabb

        pick = auth.box
        fields = {}
        scene_size = None
        gate = ""
        if pick is not None:
            fields = _pick_detection_record(pick)
            stamp = pick.header.stamp
            fields["stamp_sec"] = (
                float(stamp.sec) + 1e-9 * float(stamp.nanosec))
            fields["pca_source"] = str(getattr(pick, "pca_source", "") or "")
            gate = observation_gate(pick, auth.detect_reason)
            try:
                _xyz, _quat, scene_size = pickup_collision_aabb(pick)
            except ValueError as exc:
                fields["scene_aabb_error"] = str(exc)
        selected = None
        diag = self._last_auth_diag
        if isinstance(diag, dict):
            for read in diag.get("auth_reads") or []:
                for row in read.get("window") or []:
                    if row.get("selected"):
                        selected = row
        return authorized_pick_boundary(
            fields, authorized=bool(auth.authorized),
            detect_reason=auth.detect_reason,
            observation_gate_name=gate, selected_frame=selected,
            scene_size=scene_size)

    def _capture_pick_readiness_failure(self, trial, decision):
        """Save the gated frame, not a later image after a failed YOLO wait."""
        bundle = self.freeze_failure_bundle(decision)
        dump_dir = str(getattr(self._args, 'dump_dir', '') or '')
        if not dump_dir:
            return
        images, metadata, arrays = self.decoded_dump()
        metadata.update(capture_complete=bool(bundle.get('capture_complete')),
                        missing=list(bundle.get('missing') or []),
                        decision_record=decision,
                        replay_possible=bool(bundle.get('replay_possible')))
        dest = write_trial_dump(os.path.join(dump_dir, 'perception'),
            trial_to_dict(trial), images=images, extras=metadata, arrays=arrays)
        trial.extras['perception_failure_capture'] = {
            'directory': dest, 'capture_complete': metadata['capture_complete'],
            'missing': metadata['missing'], 'replay_possible': metadata['replay_possible']}

    def _pick_carry(self, index, trial, already_spawned=False):
        """Pick through vacuum-follow. Returns (pick_msg, box_msg) or None."""
        self._join_trace.clear()
        self._mask_trace.clear()
        self._last_auth_diag = None
        goto_ok, goto_msg, _ = self._home_arm()
        if not goto_ok:
            trial.fail_code = self.home_arm_code()
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
        command_size = size_eval.get("command_size") or []
        self._release_physics = {
            "model_name": str(size_eval.get("model_name") or ""),
            "command_size": [float(v) for v in command_size],
            "visual_id": str(size_eval.get("visual_id") or ""),
            "mass_kg": size_eval.get("mass_kg"),
        }
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
            decision = self._yolo_decision_record or _parse_json(
                self._seg_stats["payload"])
            trial.extras["seg_stats"] = decision
            trial.extras["decision_record"] = decision
            self._capture_pick_readiness_failure(trial, decision)
            return None
        if not self.wait_tracked_cargo(
                generation, self._args.geometry_timeout, spawn.box.id):
            trial.fail_code = tracker_wait_fail_code(
                _parse_json(self._filter_stats["payload"]),
                generation, spawn.box.id)
            return None
        auth = self.detect_until_authorized()
        trial.extras["pick_authorization"] = auth.to_dict()
        trial.extras["authorized_pick"] = self._authorized_pick_record(auth)
        boundary = self.perception_boundary_document()
        trial.extras["perception_boundary"] = boundary
        trial.extras["perception_boundary_summary"] = (
            boundary.get("classification") or {})
        current = self.call_srv(
            self._current, GetCurrentBox.Request(), timeout=10.0)
        if not auth.authorized:
            trial.fail_code = auth.decision.reason
            trial.extras["detect"] = auth.detect_reason or auth.decision.reason
            return None
        pick_msg = auth.box
        self._last_authorized_pick = pick_msg
        self._last_pick_detection = _pick_detection_record(pick_msg)
        if current is None or not current.success:
            trial.fail_code = "GT_UNAVAILABLE"
            return None
        place_err = self.on_detected_luggage(pick_msg, trial, spawn)
        if place_err:
            trial.fail_code = place_err
            return None
        if self._args.use_vacuum:
            sync_ok, sync_msg, sync_gen = self.sync_pickup_geometry(pick_msg)
            trial.extras["payload_sync"] = {
                "ok": sync_ok, "message": sync_msg, "generation": sync_gen}
            if not sync_ok:
                trial.fail_code = "PAYLOAD_SYNC_FAILED"
                return None
            scene_ok, scene_msg = self.add_scene_box(pick_msg)
            trial.extras["scene_add"] = scene_msg
            if not scene_ok:
                trial.fail_code = "SCENE_ADD_FAILED"
                return None
        if self._typed is not None:
            self._frame_pick_msg = pick_msg
            from luggage_description.frame_contracts import ContractError
            try:
                built = self._typed.build_pick(
                    request_id=self._frame_request_id("pick"),
                    scene_revision=self._frame_scene_revision,
                    pick_message=pick_msg)
            except ContractError as error:
                built = None
                trial.extras["build_pick"] = "%s: %s" % (error.code, error.field)
        else:
            req = BuildMotionSequence.Request()
            req.phase = "pick"
            req.pick = pick_msg
            built = self.call_srv(self._build, req, timeout=15.0)
        if built is None or not built.success:
            trial.fail_code = "BUILD_FAILED"
            trial.extras["build_pick"] = built.message if built else "timeout"
            return None
        segments = list(built.segments)
        motion_segments = list(getattr(built, "motion_segments", ()) or ())
        self._frame_motion_segments = motion_segments
        self._frame_motion_index = 0
        for segment_index, segment in enumerate(segments):
            if (self._typed is not None
                    and segment_index < len(motion_segments)):
                ok, message, _res = self._typed.plan_motion(
                    motion_segments[segment_index],
                    self._args.plan_timeout,
                    name="PlanMotionV2:%s" % segment.name)
            else:
                goal = PlanMotion.Goal()
                goal.segment = segment
                ok, message, _res = self.send_action(
                    self._plan, goal, timeout=self._args.plan_timeout,
                    name="PlanMotion:%s" % segment.name)
            if not ok:
                trial.fail_code = "PLAN_%s" % segment.name
                trial.extras["plan_%s" % segment.name] = message
                trial.extras["plan_failure"] = self._plan_failure_record(
                    segments, segment, message)
                return None
            if segment.name == "attach" and self._args.use_vacuum:
                vac_ok, vac_msg = self.vacuum_command(True, pick_msg=pick_msg)
                trial.vac_attach = vac_ok
                trial.extras["vac_attach"] = vac_msg
                if not vac_ok:
                    revision_dump = getattr(
                        self, "_last_vacuum_attach_evidence", None)
                    if revision_dump:
                        trial.extras["vacuum_attach_revisions"] = revision_dump
                    trial.fail_code = "VACUUM_ATTACH"
                    return None
                # [ST09-I S5.7-6 评审点 1] a new attach STARTS a cycle:
                # the previous cycle's exit proof is void from here —
                # it must never carry across a new carrying phase.
                self._start_new_cycle()
        if self._args.use_vacuum:
            state = dict(self._vacuum_state or {})
            if not (state.get("attached") and not state.get("fail_reason")):
                trial.fail_code = "VACUUM_FOLLOW"
                return None
        return pick_msg, current.box


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--diagnostic-selection-only", action="store_true",
        help="Capture the real post-pick selection response and stop before "
             "carry execution; diagnostic result is never a packing pass.")
    parser.add_argument("--n", type=int, default=3)
    parser.add_argument("--out", default=DEFAULT_OUT)
    parser.add_argument("--dump-dir", default="")
    parser.add_argument("--observe-pose", default="pickup_observe")
    parser.add_argument("--plan-timeout", type=float, default=60.0)
    parser.add_argument(
        "--select-budget-s", type=float, default=0.0,
        help="Functional-diagnostic Select budget override (s). 0 keeps "
             "the frozen n*60s service budget; >0 must equal the "
             "selector's diagnostic_route_budget_s so the client wait "
             "(budget + 30 s comm overhead) covers the real search.")
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
    parser.add_argument(
        "--placement-prior", action="store_true",
        help="[opt-in 2026-10-19] Measured placement-prior adoption "
             "(read-only sidecar; launch start_placement_prior:=true "
             "must match). OFF = the original flow with ZERO prior "
             "calls. The prior only reorders Select candidates and "
             "narrows the measured suction candidate BEFORE the grasp; "
             "full route certification is unchanged; failures fall "
             "back by name (never a stale prior).")
    parser.add_argument(
        "--release-mode", choices=("kinematic", "dynamic"),
        default="kinematic",
        help="[planning-verification 2026-10-19] kinematic: keep the "
             "carried body as a static obstacle at the DECLARED "
             "release end-state (no delete, no dynamic respawn, no "
             "free fall) — proves planning+state-chain feasibility, "
             "NOT contact physics/suction/landing accuracy. dynamic: "
             "the full physics release (r5 behavior, future "
             "contact-validation track).")
    parser.add_argument(
        "--release-dwell-s", type=float, default=2.0,
        help="Hold the certified retreat knot this long after the "
             "dynamic swap so the box falls clear of the panel before "
             "the retreat's first stroke (gate3_aprime_r5: 20 ms panel "
             "graze at 102.84-102.86 s, verify margin eroded to "
             "3.5 mm). 0 disables the dwell.")
    parser.add_argument("--scene-tf-config", default="")
    parser.add_argument(
        "--synthetic-size", default="",
        help="W,D,H metres for --dry-run / --payload none. Required there.")
    parser.add_argument(
        "--auth-max-attempts", type=int, default=11,
        help="Safety cap on non-hold detect samples. The wait itself "
             "is --auth-max-elapsed-sec (default 5 s).")
    parser.add_argument("--auth-max-elapsed-sec", type=float, default=5.0)
    parser.add_argument("--auth-wait-period-sec", type=float, default=0.5)
    args = parser.parse_args(argv)
    if ((args.plan_only or args.payload == "none")
            and _parse_synthetic_size(args.synthetic_size) is None):
        parser.error(
            "--synthetic-size W,D,H is required for --dry-run and "
            "--payload none")
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

    def _spin_with_crash_log():
        # [n3 functional, 2026-10-22] An exception escaping a callback kills
        # this thread (and the executor); as a daemon thread its traceback
        # gets truncated by interpreter teardown (n3_functional_20261022_
        # 130429 lost the frames exactly that way while the handoff settle
        # gate went evidence-blind). Persist the full traceback FIRST, then
        # re-raise. Observability only — no behavior change.
        try:
            executor.spin()
        except BaseException:
            with open(os.path.join(args.out, "executor_crash.txt"), "w",
                      encoding="utf-8") as handle:
                handle.write(traceback.format_exc())
            raise

    spinner = threading.Thread(target=_spin_with_crash_log, daemon=True)
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
