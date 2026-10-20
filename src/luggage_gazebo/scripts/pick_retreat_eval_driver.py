#!/usr/bin/env python3
"""Closed-loop pick/retreat eval: observe → detect → plan → execute → observe → clear.

State machine per docs/plans/closed_loop_eval_driver.md. ROS interfaces only;
no YOLO / RANSAC / MoveIt imports. Ctrl-C flushes completed trials.

Per trial: GoTo observe, clear leftover, spawn, wait until post-mask-filter
YOLO publishes a cargo box for that generation, wait tracker cargo, wait
until the depth blob on the platform matches this spawn's catalog AABB
(GetCurrentBox), then DetectLuggage vs that same GetCurrentBox. Overlay
green GT is the catalog AABB, not the mesh lid-band / depth silhouette.
observe, then ClearCurrentBox. Failure paths also return to observe before
clearing so perception epoch reset happens with the camera on the platform.

Three independent rates (see luggage_gazebo.eval_metrics.summarize):
  detect_pass_rate, plan_pass_rate, retreat_pass_rate.
Geometric pick accuracy is suction_contact_frame vs the measured box after
attach. ``--use-vacuum`` adds PlanningScene add + VacuumCommand around
attach/retreat. GT size mismatch is diagnostic unless ``--strict-gt``.
"""

from __future__ import division

import argparse
import collections
import json
import math
import os
import re
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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from luggage_msgs.action import PlanMotion
from luggage_msgs.msg import DetectionFrame, VacuumState
from luggage_msgs.srv import (
    BuildMotionSequence,
    ClearCurrentBox,
    DetectLuggage,
    GetCurrentBox,
    SpawnNextBox,
    SyncPickupBox,
    VacuumCommand,
)

from luggage_perception import ros_message_adapters as adapters
from luggage_planning.pick_authorization import (
    DEFAULT_MAX_ATTEMPTS,
    DEFAULT_MAX_ELAPSED_SEC,
    DEFAULT_WAIT_PERIOD_SEC,
    AuthorizationConfig,
    PickAuthorizationPolicy,
    run_authorization_loop,
)
from luggage_planning.pickup_collision import pickup_collision_aabb
from luggage_planning.ros_clock_wait import wait_event
from luggage_perception.cargo_instance_tracker import parse_current_box_payload
from luggage_perception.join_boundary_trace import classify_hold_boundary
from luggage_perception.detect_overlay import rotation_from_quaternion
from luggage_perception.eval.detection_accuracy import (
    BoxObservation,
    DetectionAccuracy,
)
from luggage_perception.eval.detection_gate_sampling import (
    FRAME_JOIN_AFTER_SPAWN_SEC,
    FRAME_JOIN_WAIT_SEC,
    annotate_overlay_boxes,
    annotate_yolo_detections,
    apply_dump_timestamp_banners,
    build_aligned_dump,
    is_gt_fallback,
    is_perception_estimate,
    perception_reason,
    pick_joined_stamp,
    stamp_sec_from_key,
    write_png,
)

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.isdir(os.path.join(_PKG_ROOT, "luggage_gazebo")):
    sys.path.insert(0, _PKG_ROOT)
from luggage_gazebo.eval_metrics import (  # noqa: E402
    DEFAULT_VISUAL_TOL_XY,
    DEFAULT_VISUAL_TOL_Z,
    TrialRecord,
    cargo_generation_ready,
    depth_to_camera_xyz,
    first_yolo_ready,
    label_aabb,
    nearest_catalog_id,
    points_aabb,
    raised_object_measure,
    spawn_visual_matches_gt,
    summarize,
    tracker_epoch_matches,
    tracker_wait_fail_code,
    transform_camera_xyz_to_world,
    trial_from_dict,
    trial_to_dict,
)
from luggage_gazebo.eval_dump_identity import (  # noqa: E402
    dump_seg_stats,
    dump_tf_stamp_from_color,
    freeze_dump_bundle,
)

def current_ros_domain():
    return os.environ.get("ROS_DOMAIN_ID", "") or "0"


def ros_domain_of_pid(pid):
    """Return a process's ROS_DOMAIN_ID, or None when it cannot be read."""
    try:
        with open("/proc/%s/environ" % pid, "rb") as handle:
            raw = handle.read()
    except (OSError, IOError):
        return None
    for item in raw.split(b"\0"):
        if item.startswith(b"ROS_DOMAIN_ID="):
            return item.split(b"=", 1)[1].decode("utf-8", "replace") or "0"
    return "0"


def count_procs_in_domain(pattern, domain):
    """Count matching processes that share this ROS domain.

    A stack on another `ROS_DOMAIN_ID` shares the machine but not the ROS
    graph, so it must not read as a residual participant here. A process whose
    environment cannot be read is counted, keeping the gate fail-closed.
    """
    try:
        out = subprocess.check_output(["pgrep", "-f", pattern], text=True)
    except (OSError, subprocess.CalledProcessError):
        return 0
    count = 0
    for token in out.split():
        found = ros_domain_of_pid(token.strip())
        if found is None or found == domain:
            count += 1
    return count


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


def _detection_frame_size(frame):
    """WDH stored on this frame, or None when the frame has no PCA box."""
    box = getattr(frame, "box", None)
    if box is None or not bool(getattr(frame, "pca_valid", False)):
        return None
    return [float(box.width), float(box.depth), float(box.height)]


def detection_frame_as_dict(msg):
    """Compact DetectionFrame for trial.json (no ROS types)."""
    if msg is None:
        return None
    stamp = msg.header.stamp
    yolo = []
    for box in list(msg.yolo or []):
        raw = getattr(box, "bbox", None)
        try:
            bbox = [int(v) for v in list(raw)[:4]] if raw is not None else []
        except (TypeError, ValueError):
            bbox = []
        if len(bbox) < 4:
            bbox = []
        yolo.append({
            "label": int(box.label),
            "prompt": str(box.prompt),
            "confidence": float(box.confidence),
            "bbox": bbox,
            "held": bool(box.held),
        })
    out = {
        "stamp": float(stamp.sec) + 1e-9 * float(stamp.nanosec),
        "frame_id": msg.header.frame_id,
        "yolo_optical_frame": str(msg.yolo_optical_frame),
        "frame_seq": int(msg.frame_seq),
        "generation": int(msg.generation),
        "instance_id": str(msg.instance_id),
        "yolo": yolo,
        "pca_valid": bool(msg.pca_valid),
        "pca_reason": str(msg.pca_reason),
        "pca_source": str(msg.pca_source),
        "size_wdh": _detection_frame_size(msg),
        "pca_confidence": float(msg.pca_confidence),
        "n_cargo_points": int(msg.n_cargo_points),
        "centroid": [
            float(msg.centroid.x),
            float(msg.centroid.y),
            float(msg.centroid.z),
        ],
        "box": None,
    }
    if msg.pca_valid:
        out["box"] = observation_from_detected(msg.box).__dict__
        out["box"]["id"] = str(getattr(msg.box, "id", "") or "")
    return out


def _parse_json(payload):
    if not payload:
        return None
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _stamp_key(msg):
    return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))


def _image_meta(msg):
    if msg is None:
        return None
    stamp = msg.header.stamp
    meta = {
        "stamp": float(stamp.sec) + 1e-9 * float(stamp.nanosec),
        "frame_id": msg.header.frame_id,
    }
    encoding = getattr(msg, "encoding", None)
    if encoding is not None:
        meta["encoding"] = encoding
    if hasattr(msg, "width"):
        meta["width"] = int(msg.width)
    if hasattr(msg, "height"):
        meta["height"] = int(msg.height)
    return meta


def _decode_dump_image(msg):
    """RGB or grey uint8 for PNG. Overlay is published as bgr8."""
    if msg is None:
        return None
    arr = adapters.image_array_from_msg(msg)
    if arr is None:
        return None
    if arr.ndim == 3 and (msg.encoding or "").lower() == "bgr8":
        return arr[:, :, ::-1].copy()
    return arr


class _StampBuffer(object):
    """Keep the last *maxlen* messages keyed by header stamp."""

    def __init__(self, maxlen=60):
        self._maxlen = int(maxlen)
        self._items = collections.OrderedDict()
        self._lock = threading.Lock()

    def push(self, msg):
        key = _stamp_key(msg)
        with self._lock:
            self._items[key] = msg
            self._items.move_to_end(key)
            while len(self._items) > self._maxlen:
                self._items.popitem(last=False)

    def snapshot(self):
        with self._lock:
            return dict(self._items)

    def nearest(self, stamp_sec, tol=0.08):
        """Return (msg, |dt|) for the closest stamp, or (None, None)."""
        snap = self.snapshot()
        if not snap:
            return None, None
        if stamp_sec is None:
            key = max(snap)
            return snap[key], None
        target = float(stamp_sec)
        best_msg = None
        best_dt = None
        for key, msg in snap.items():
            value = stamp_sec_from_key(key)
            if value is None:
                continue
            dt = abs(value - target)
            if best_dt is None or dt < best_dt:
                best_msg, best_dt = msg, dt
        if best_dt is None or best_dt > float(tol):
            return None, None
        return best_msg, best_dt


class _JsonStampBuffer(object):
    """Keep the last *maxlen* JSON dicts keyed by a float stamp."""

    def __init__(self, maxlen=30):
        self._maxlen = int(maxlen)
        self._items = collections.OrderedDict()
        self._lock = threading.Lock()

    def push(self, stamp, payload):
        if stamp is None or not isinstance(payload, dict):
            return
        try:
            key = float(stamp)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._items[key] = payload
            self._items.move_to_end(key)
            while len(self._items) > self._maxlen:
                self._items.popitem(last=False)

    def newest_first(self):
        with self._lock:
            return [payload for _key, payload in reversed(self._items.items())]

    def nearest(self, stamp_sec, tol=0.08):
        with self._lock:
            items = list(self._items.items())
        if not items:
            return None, None
        if stamp_sec is None:
            return items[-1][1], None
        target = float(stamp_sec)
        best_payload = None
        best_dt = None
        for key, payload in items:
            dt = abs(float(key) - target)
            if best_dt is None or dt < best_dt:
                best_payload, best_dt = payload, dt
        if best_dt is None or best_dt > float(tol):
            return None, None
        return best_payload, best_dt


def write_trial_dump(dump_dir, fields, images=None, extras=None, arrays=None,
                     exact_dir=False):
    """Write one trial's JSON + PNG frames. Returns the folder or None.

    ``exact_dir=True`` writes into ``dump_dir`` itself (pack-eval box_XX folders).
    """
    if not dump_dir:
        return None
    code = fields.get("fail_code") or "ok"
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", str(code)).strip("_")[:80]
    if not slug:
        slug = "ok"
    if exact_dir:
        dest = dump_dir
    else:
        dest = os.path.join(
            dump_dir, "trial_%02d_%s" % (int(fields.get("index") or 0), slug))
    os.makedirs(dest, exist_ok=True)
    payload = dict(fields)
    if extras:
        payload["frame_meta"] = extras
    with open(os.path.join(dest, "trial.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    for name, arr in (images or {}).items():
        if arr is None:
            continue
        write_png(os.path.join(dest, "%s.png" % name), arr)
    for name, arr in (arrays or {}).items():
        if arr is None:
            continue
        import numpy as np  # noqa: WPS433  dump path only
        np.save(os.path.join(dest, "%s.npy" % name), arr)
    return dest


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
        self._cargo = {"recv": None, "stamp": 0.0, "n_points": 0}
        self._diag = {"payload": None}
        self._support_stream = {}
        self._seg_stats = {"payload": None}
        self._filter_stats = {"payload": None}
        self._seg_stats_buf = _JsonStampBuffer(maxlen=30)
        self._filter_stats_buf = _JsonStampBuffer(maxlen=30)
        self._join_trace = collections.deque(maxlen=256)
        self._mask_trace = collections.deque(maxlen=256)
        self._last_auth_diag = None
        self._current_box_topic = {"payload": None}
        self._size_eval = {"payload": None}
        self._camera_info = None
        self._detection_frames = _StampBuffer(maxlen=30)
        self._buffers = None
        self._trial_snapshot = {}
        self._trial_join = {
            "aligned": False, "stamp_key": None, "cargo_matched": False,
        }
        self._yolo_decision_record = None
        self._frozen_bundle = None
        self._tf_buffer = Buffer()
        # Dedicated TF node so /tf does not starve camera_info on the driver
        # executor (early trials otherwise fail SPAWN_VISUAL_TF: no_camera_info).
        self._tf_node = Node("pick_eval_tf", use_global_arguments=False)
        self._tf_listener = TransformListener(
            self._tf_buffer, self._tf_node, spin_thread=True)
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
        cloud_hist = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        image_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
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
            String, "/luggage_detector/stream_stats_json",
            self._on_support_stream, latch, callback_group=self._group)
        self.create_subscription(
            String, "/semantic_segmenter/stats_json",
            self._on_seg_stats, latch, callback_group=self._group)
        self.create_subscription(
            String, "/semantic_point_filter/stats_json",
            self._on_filter_stats, latch, callback_group=self._group)
        trace_qos = QoSProfile(
            depth=100, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            String, "/semantic_point_filter/join_trace",
            self._on_join_trace, trace_qos, callback_group=self._group)
        self.create_subscription(
            String, "/semantic_segmenter/mask_trace",
            self._on_mask_trace, trace_qos, callback_group=self._group)
        self.create_subscription(
            String, "/luggage/current_box",
            lambda m: self._current_box_topic.__setitem__("payload", m.data),
            latch, callback_group=self._group)
        self.create_subscription(
            String, "/luggage/perception/size_eval/spawned",
            lambda m: self._size_eval.__setitem__("payload", m.data),
            latch, callback_group=self._group)
        self.create_subscription(
            PointCloud2, "/luggage/semantic/cargo_points",
            self._on_cloud, sensor, callback_group=self._group)
        self.create_subscription(
            PointCloud2, "/luggage/semantic/cargo_points",
            self._on_cargo, cloud_hist, callback_group=self._group)
        self.create_subscription(
            DetectionFrame, "/luggage/perception/detection_frame",
            lambda m: self._detection_frames.push(m),
            cloud_hist, callback_group=self._group)
        self._buffers = {"depth": _StampBuffer(), "cargo": _StampBuffer(maxlen=20)}
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/depth/image",
            lambda m: self._buffers["depth"].push(m),
            image_qos, callback_group=self._group)
        self.create_subscription(
            CameraInfo, "/luggage/preprocessed/camera/color/camera_info",
            self._on_camera_info, image_qos, callback_group=self._group)
        if args.dump_dir:
            for name, topic in (
                    ("color", "/luggage/preprocessed/camera/color/image"),
                    ("overlay", "/luggage/semantic/overlay"),
                    ("mask", "/luggage/semantic/mask")):
                self._buffers[name] = _StampBuffer()
                self.create_subscription(
                    Image, topic,
                    lambda m, stream=name: self._buffers[stream].push(m),
                    image_qos, callback_group=self._group)
            # cargo cloud is always buffered for AABB diagnostics.

        self._spawn = self.create_client(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box",
            callback_group=self._group)
        self._clear = self.create_client(
            ClearCurrentBox, "/pickup_box_spawner/clear_current_box",
            callback_group=self._group)
        self._current = self.create_client(
            GetCurrentBox, "/pickup_box_spawner/get_current_box",
            callback_group=self._group)
        self._sync = self.create_client(
            SyncPickupBox, "/pickup_box_spawner/sync_detected_pickup_box",
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
        # [ST09-I S5.7-6] the v1 GoToRobotPose action client is deleted:
        # goto_observe and the cargo exit ride the certified
        # GoToRobotPoseV2 service (the v1 action stays only for the
        # hardware pick chain).
        self._vacuum_state = {}
        self._vacuum_events = {}
        self._scene = None
        self._vacuum = None
        if getattr(args, "use_vacuum", False):
            from luggage_planning.planning_scene_client import (
                PlanningSceneClient,
            )
            self._scene = PlanningSceneClient(
                self, callback_group=self._group)
            self._vacuum = self.create_client(
                VacuumCommand, "/vacuum/command",
                callback_group=self._group)
            self.create_subscription(
                VacuumState, "/vacuum/state", self._on_vacuum_state, latch,
                callback_group=self._group)
            self.create_subscription(
                String, "/vacuum/events_json", self._on_vacuum_events, latch,
                callback_group=self._group)

        # ST09 F21/F22: typed v2 chain. --frame-runtime adopts the stack's
        # measured identity from the live graph heartbeat and routes this
        # driver's service/action calls through the shared typed adapter;
        # every wire call becomes v2 while trial bookkeeping keeps the v1
        # response copies. Off (default) is byte-identical legacy.
        self._typed = None
        self._frame_runtime_handle = None
        self._frame_scene_revision = ""
        self._frame_pick_msg = None
        self._frame_grasp = None
        self._frame_slot_record = None
        self._frame_motion_segments = []
        self._frame_motion_index = 0
        self._frame_request_seq = 0
        if getattr(args, "frame_runtime", False):
            from luggage_planning.typed_driver_adapter import (
                TypedDriverAdapter, adopt_runtime_identity)
            peer = str(getattr(args, "frame_peer", "") or "motion_occ_driver")
            runtime, _registry = adopt_runtime_identity(self, peer=peer)
            self._frame_runtime_handle = runtime
            self._frame_scene_revision = str(runtime.identity["scene_revision"])
            self._typed = TypedDriverAdapter(self, runtime,
                                             group=self._group)
            self.get_logger().info(
                "frame runtime adopted (peer=%s, scene=%s)"
                % (peer, self._frame_scene_revision[:10]))

    def _frame_request_id(self, prefix):
        self._frame_request_seq += 1
        return "%s-%d" % (prefix, self._frame_request_seq)

    def _typed_build_place(self, pick_msg):
        """BuildMotionSequenceV2 place legs from the adopted chain context.

        The slot record comes from the v2 placement/selection chain and the
        grasp was measured at attach; anything less fails closed rather
        than falling back to a hand-converted slot.
        """
        from luggage_description.frame_contracts import ContractError
        from luggage_planning.typed_driver_adapter import (
            container_transform_fields)
        slot_record = getattr(self, "_frame_slot_record", None)
        grasp = getattr(self, "_frame_grasp", None)
        if slot_record is None or grasp is None:
            return None, "TYPED_PLACE_CONTEXT_MISSING"
        grasp = dict(grasp)
        grasp["request_id"] = str(slot_record.get("request_id") or "")
        try:
            built = self._typed.build_place(
                slot=slot_record, grasp=grasp,
                box_wdh=[float(pick_msg.width), float(pick_msg.depth),
                         float(pick_msg.height)],
                container_transform=container_transform_fields(
                    self._scene_config))
        except ContractError as error:
            return None, "%s: %s" % (error.code, error.field)
        return built, ""

    def _frame_capture_grasp(self, pick_msg):
        """Measure the box-frame grasp transport at attach (TF + detection).

        The grasp stamp is the suction-contact TF stamp at contact. A
        zero stamp is left as zero so the selector can refuse it; it is
        not replaced with the clock.
        """
        # A confirmed attachment is frozen for this carrying phase. Combining
        # a later TCP with the pre-pick detection would invent a new grasp.
        if getattr(self, '_frame_attachment_confirmed', False):
            return
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "world", "suction_contact_frame", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0))
        except Exception as exc:  # noqa: BLE001 - grasp capture is best effort
            self.get_logger().warning("frame grasp capture TF failed: %s" % exc)
            return
        translation = tf_msg.transform.translation
        rotation = tf_msg.transform.rotation
        stamp = tf_msg.header.stamp
        tool_stamp_ns = int(stamp.sec) * 10 ** 9 + int(stamp.nanosec)
        from luggage_planning.typed_driver_adapter import grasp_from_attach
        box_id, generation = self._frame_box_identity()
        self._frame_grasp = grasp_from_attach(
            self._frame_request_id("grasp"), self._frame_scene_revision,
            [float(pick_msg.pose.position.x), float(pick_msg.pose.position.y),
             float(pick_msg.pose.position.z)],
            [float(pick_msg.pose.orientation.x),
             float(pick_msg.pose.orientation.y),
             float(pick_msg.pose.orientation.z),
             float(pick_msg.pose.orientation.w)],
            (float(translation.x), float(translation.y), float(translation.z)),
            (float(rotation.x), float(rotation.y),
             float(rotation.z), float(rotation.w)),
            stamp_ns=tool_stamp_ns)
        self._frame_grasp["box_id"] = str(box_id or "")
        self._frame_grasp["generation"] = int(generation or 0)
        self._frame_grasp_stamp_ns = int(tool_stamp_ns)

    def _frame_box_identity(self):
        """(box_id, generation) from the latched current_box payload."""
        box_id, generation = parse_current_box_payload(
            (self._current_box_topic.get("payload") if isinstance(
                getattr(self, "_current_box_topic", None), dict) else None)
            or "")
        return str(box_id or ""), int(generation or 0)

    def vacuum_command(self, enable, pick_msg=None):
        self._last_vacuum_attach_evidence = None
        pick_msg = pick_msg or getattr(self, "_frame_pick_msg", None)
        if self._typed is not None and pick_msg is not None:
            from luggage_planning.typed_driver_adapter import (
                detection_record_from_box)
            from luggage_planning.attachment_frame_adapters import (
                vacuum_attach_revision_dump)
            from luggage_description.frame_contracts import ContractError
            box_id, generation = self._frame_box_identity()
            request_id = self._frame_request_id("vacuum")
            box_pose = detection_record_from_box(
                pick_msg, request_id, self._frame_scene_revision)
            grasp = None
            if enable:
                self._frame_attachment_confirmed = False
                self._frame_grasp = None
                # Lock at contact: live TCP versus the still-on-platform box.
                self._frame_capture_grasp(pick_msg)
                grasp = getattr(self, "_frame_grasp", None)
            try:
                response = self._typed.vacuum_command(
                    enable=enable, request_id=request_id,
                    scene_revision=self._frame_scene_revision,
                    box_id=box_id, generation=generation,
                    measured_box=pick_msg, box_pose=box_pose,
                    grasp=grasp)
            except ContractError as error:
                return False, "%s: %s" % (error.code, error.field)
            if response is None:
                return False, "vacuum command timeout"
            if enable and response.success:
                if not getattr(response, 'has_measured_attachment', False):
                    self._frame_grasp = None
                    return False, 'MEASURED_ATTACHMENT_MISSING'
                from luggage_planning.hardware_place_planning import attachment_from_response
                from luggage_planning.typed_driver_adapter import grasp_record
                actual = attachment_from_response(response, request_id=request_id,
                    box_id=box_id, generation=generation, scene_revision=self._frame_scene_revision)
                self._frame_grasp = grasp_record(request_id, actual.scene_revision,
                    actual.box_from_tcp.translation, actual.box_from_tcp.quaternion_xyzw,
                    stamp_ns=actual.stamp_ns)
                self._frame_grasp.update(box_id=actual.box_id, generation=actual.generation)
                self._frame_grasp_stamp_ns = actual.stamp_ns
                self._frame_attachment_confirmed = True
            if not enable and response.success:
                self._frame_attachment_confirmed = False
                self._frame_grasp = None
            if not response.success:
                stamp = pick_msg.header.stamp
                self._last_vacuum_attach_evidence = vacuum_attach_revision_dump(
                    response.message,
                    live_scene_revision=self._frame_scene_revision,
                    fallback={
                        "request_scene_revision": str(
                            self._frame_scene_revision or ""),
                        "box_pose_scene_revision": str(
                            box_pose.get("scene_revision") or ""),
                        "tool_pose_scene_revision": "",
                        "bound_scene_revision": "",
                        "measured_stamp_ns": (
                            int(stamp.sec) * 10 ** 9 + int(stamp.nanosec)),
                        "box_id": str(box_id),
                        "generation": int(generation),
                    })
            return bool(response.success), str(response.message or "")
        if self._vacuum is None:
            return False, "no vacuum client"
        req = VacuumCommand.Request()
        req.enable = bool(enable)
        response = self.call_srv(self._vacuum, req, timeout=10.0)
        if response is None:
            return False, "vacuum command timeout"
        return bool(response.success), response.message or ""

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
        self._cargo["n_points"] = int(msg.width) * max(int(msg.height), 1)
        buf = (self._buffers or {}).get("cargo")
        if buf is not None:
            buf.push(msg)

    def _on_seg_stats(self, msg):
        self._seg_stats["payload"] = msg.data
        parsed = _parse_json(msg.data)
        if parsed is not None:
            self._seg_stats_buf.push(parsed.get("stamp"), parsed)

    def _on_join_trace(self, msg):
        parsed = _parse_json(msg.data)
        if isinstance(parsed, dict):
            self._join_trace.append(parsed)

    def _on_mask_trace(self, msg):
        parsed = _parse_json(msg.data)
        if isinstance(parsed, dict):
            self._mask_trace.append(parsed)

    def _on_filter_stats(self, msg):
        self._filter_stats["payload"] = msg.data
        parsed = _parse_json(msg.data)
        if parsed is not None:
            stamp = parsed.get("last_join_stamp")
            if stamp is None or float(stamp or 0) <= 0.0:
                stamp = parsed.get("last_cloud_stamp")
            if stamp is None or float(stamp or 0) <= 0.0:
                stamp = parsed.get("cloud_stamp")
            self._filter_stats_buf.push(stamp, parsed)

    def _on_support_stream(self, msg):
        parsed = _parse_json(msg.data)
        if isinstance(parsed, dict):
            self._support_stream = parsed

    def _support_stream_snapshot(self):
        stats = self._support_stream or {}
        timing = stats.get("timing_ms") or {}
        pipeline = timing.get("pipeline") or {}
        return {
            "support_reason": stats.get("support_reason"),
            "support_gate": stats.get("support_gate"),
            "support_valid": stats.get("support_valid"),
            "geometry_level": stats.get("geometry_level"),
            "n_cargo_points": stats.get("n_cargo_points"),
            "raw_lookup": stats.get("raw_lookup"),
            "top_input_points": pipeline.get("top_input_points"),
            "support_input_points": pipeline.get("support_input_points"),
            "support_candidate_points": pipeline.get(
                "support_candidate_points"),
            "support_inlier_points": pipeline.get("support_inlier_points"),
        }

    def _on_camera_info(self, msg):
        self._camera_info = msg

    def _on_vacuum_state(self, msg):
        self._vacuum_state = {
            "attached": bool(msg.attached),
            "vacuum_on": bool(msg.vacuum_on),
            "fail_reason": msg.fail_reason or "",
            "contact_distance": float(msg.contact_distance),
            "retention_margin": float(msg.retention_margin),
            "tilt_deg": float(msg.tilt_deg),
            # [ST09-I G0-1] receive-time freshness basis (the message
            # carries no stamp) — mirrors route_producer's selector-side
            # latched vacuum record.
            "received_ns": int(self.get_clock().now().nanoseconds),
        }

    def _on_vacuum_events(self, msg):
        parsed = _parse_json(msg.data)
        if parsed is not None:
            self._vacuum_events = parsed

    def call_srv(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=min(timeout, 15.0)):
            return None
        recorder = getattr(self, '_service_boundary_recorder', None)
        if recorder is not None:
            recorder(client.srv_name+':request', request)
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout):
            return None
        response = future.result()
        if recorder is not None:
            recorder(client.srv_name+':response', response)
        return response

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
        reached, wait_reason = wait_event(
            done, timeout, clock=self.get_clock())
        if not reached:
            try:
                handle.cancel_goal()
            except Exception:
                pass
            extra = " (%s)" % wait_reason if wait_reason else ""
            return False, "%s timeout%s" % (name, extra), None
        wrapped = result_future.result()
        result = wrapped.result
        ok = (wrapped.status == GoalStatus.STATUS_SUCCEEDED
              and bool(getattr(result, "success", False)))
        return ok, str(getattr(result, "message", "")), result

    def _proc_count(self, pattern):
        return count_procs_in_domain(pattern, current_ros_domain())

    def graph_error(self):
        # Process counts, not DDS names: Fast-DDS keeps ghost participants
        # for tens of seconds after SIGTERM, which would false-positive a
        # ros2-node-list uniqueness check without actually dual-feeding
        # the camera depth image.
        for label, pattern in DUPLICATE_PROCS:
            count = self._proc_count(pattern)
            if count > 1:
                return "duplicate process %s (x%d); residual graph" % (
                    label, count)
        names = set(self.get_node_names())
        required = list(REQUIRED_NODES)
        if self._args.use_vacuum:
            required.append("vacuum_controller")
        missing = [n for n in required if n not in names]
        if missing:
            return "missing nodes: %s" % ", ".join(missing)
        n_bridge = self._proc_count("ros_gz_bridge/parameter_bridge")
        if n_bridge > 3:
            return "duplicate process parameter_bridge (x%d); residual graph" % (
                n_bridge)
        return ""

    def _graph_diag_sample(self, t0):
        """[r8 bounded boot diagnostics 2026-10-20, recording only]

        One observation of THIS process's own graph view: full node
        names AND namespaces, self visibility, and the /vacuum/command
        SERVICE discovery state — distinguishing a node-name audit
        failure from actual communication unavailability. Never a fix.
        """
        try:
            names_ns = sorted(
                (ns, n) for n, ns in
                self.get_node_names_and_namespaces())
        except Exception as exc:  # noqa: BLE001 - diagnostics only
            names_ns = [("ERROR", repr(exc)[:120])]
        vacuum_service = None
        try:
            client = getattr(self, "_vacuum_cmd_diag_client", None)
            if client is None:
                from luggage_msgs.srv import VacuumCommand
                client = self.create_client(
                    VacuumCommand, "/vacuum/command")
                self._vacuum_cmd_diag_client = client
            vacuum_service = bool(client.service_is_ready())
        except Exception as exc:  # noqa: BLE001 - diagnostics only
            vacuum_service = "ERROR:%s" % repr(exc)[:80]
        return {
            "elapsed_s": round(time.monotonic() - t0, 2),
            "n_nodes": len(names_ns),
            "nodes_namespaced": ["%s%s" % (ns, n) for ns, n in names_ns],
            "self_visible": any(
                n == self.get_name() for _ns, n in names_ns),
            "vacuum_command_service_ready": vacuum_service,
        }

    def _dump_graph_diagnostics(self, verdict, samples, last_error=None):
        env = dict(os.environ)
        dds_env = {k: v for k, v in env.items() if (
            k.startswith("ROS_") or k.startswith("RMW_")
            or k.startswith("FASTRTPS") or k.startswith("FASTDDS")
            or k.startswith("CYCLONEDDS"))}
        record = {
            "verdict": verdict,
            "last_error": last_error,
            "required_nodes": list(REQUIRED_NODES) + (
                ["vacuum_controller"]
                if getattr(self._args, "use_vacuum", False) else []),
            "dds_env": dds_env,
            "executor_thread_alive": None,
            "samples": samples,
        }
        try:
            record["executor_thread_alive"] = bool(
                self._diag_spinner_thread.is_alive())
        except AttributeError:
            pass
        out_dir = str(getattr(self._args, "out", "") or "")
        try:
            json_path = os.path.join(out_dir, "graph_diagnostics.json")
            with open(json_path, "w", encoding="utf-8") as handle:
                json.dump(record, handle, indent=1)
            self.get_logger().info(
                "graph diagnostics: %s -> %s (%d samples)" % (
                    verdict, json_path, len(samples)))
        except (OSError, ValueError) as exc:
            self.get_logger().warning(
                "graph diagnostics dump failed: %s" % exc)
        return record

    def wait_graph(self, timeout=90.0):
        deadline = time.time() + timeout
        last = "not checked"
        samples = []
        t0 = time.monotonic()
        next_sample = t0
        while time.time() < deadline:
            err = self.graph_error()
            last = err
            if err.startswith("duplicate"):
                self._dump_graph_diagnostics("duplicate_process",
                                             samples, err)
                return err
            if not err:
                self._dump_graph_diagnostics("graph_ok", samples)
                return ""
            if time.monotonic() >= next_sample:
                samples.append(self._graph_diag_sample(t0))
                next_sample += 5.0     # one observation per 5 s
            time.sleep(1.0)
        self._dump_graph_diagnostics("graph_wait_failed", samples, last)
        # [r8 diagnostics] Bounded hold so an EXTERNAL probe can observe
        # the SAME moment (synchronous control, not a replacement for
        # this process's own evidence). No behavior change; the abort
        # still returns after the window.
        hold = float(getattr(self._args, "graph_diag_hold_sec", 30.0)
                     or 0.0)
        if hold > 0.0:
            self.get_logger().info(
                "holding %ss diagnostic window (external probe)" % hold)
            hold_until = time.monotonic() + hold
            while time.monotonic() < hold_until:
                time.sleep(1.0)
                samples.append(self._graph_diag_sample(t0))
            self._dump_graph_diagnostics(
                "graph_wait_failed_post_hold", samples, last)
        return last or "graph wait timeout"

    def goto_observe(self):
        """[ST09-I S5.7-6] Home to the named observe pose via V2.

        A current/here name is a CALLER-side decision to stay put (the
        service never falls back to it); a missing typed surface is a
        named refusal, not a v1 detour (non frame-runtime gazebo runs
        lose goto by design — GOTO_V2_UNAVAILABLE).
        """
        from luggage_planning.named_robot_poses import is_current_pose_name

        pose_name = str(self._args.observe_pose or "")
        if is_current_pose_name(pose_name):
            return True, "goto=%s (current)" % pose_name, None
        if getattr(self, "_typed", None) is None:
            return False, "GOTO_V2_UNAVAILABLE", None
        ok, message = self._goto_named_pose_v2(
            pose_name, timeout=self._args.goto_timeout)
        return ok, message or ("goto %s" % pose_name), None

    def _wait_map_quiescence(self, stats_client, seen_revision,
                             timeout=10.0):
        """[ST09-I G2] Boot-churn settle for the goto map gate.

        Polls the AUTHORITATIVE stats until the map revision stops
        advancing (two consecutive equal reads) — an observable
        quiescent state, not a blind sleep and not a producer-gate
        change: the fail-closed latest-map admission stays exactly as
        strict, the caller just re-plans against a settled map."""
        from luggage_msgs.srv import GetCargoMapStats

        deadline = time.monotonic() + float(timeout)
        last = int(seen_revision)
        while time.monotonic() < deadline:
            stats = self.call_srv(
                stats_client, GetCargoMapStats.Request(), timeout=3.0)
            if stats is None or not stats.success:
                return False
            revision = int(stats.map_revision)
            if revision == last:
                return True
            last = revision
            time.sleep(0.2)
        return False

    def _goto_named_pose_v2(self, pose_name, timeout=30.0):
        """[ST09-I S5.7-6] Certified motion to a named site pose.

        Mirrors the cargo driver's _execute_observation_route: the
        GoToRobotPoseV2 service validates the site-pose file against
        the live identity and returns COMPLETE certified joint legs
        (typed, executed in order with per-leg admission scopes) —
        never a pose spec handed to the planner. The certified route
        identity is stashed for the exit-proof bookkeeping.

        [ST09-I G2] MAP_STALE is retried (bounded) after an observable
        revision-quiescence wait: right after boot the readiness
        integrations are still committing between the caller's stats
        read and the producer's fresh fetch, and the first home goto
        must not fail the whole session on that churn.
        """
        from luggage_msgs.srv import GetCargoMapStats, GoToRobotPoseV2

        client = getattr(self, "_goto_route_client", None)
        if client is None:
            client = self.create_client(
                GoToRobotPoseV2,
                "/trajectory_selector/goto_robot_pose_v2",
                callback_group=self._group)
            self._goto_route_client = client
        if not client.wait_for_service(timeout_sec=5.0):
            return False, "GOTO_ROUTE_UNAVAILABLE"
        stats_client = getattr(self, "_goto_stats_client", None)
        if stats_client is None:
            stats_client = self.create_client(
                GetCargoMapStats, "/cargo_map/get_stats",
                callback_group=self._group)
            self._goto_stats_client = stats_client
        if not stats_client.wait_for_service(timeout_sec=5.0):
            return False, "MAP_STATUS_UNAVAILABLE"
        request = GoToRobotPoseV2.Request()
        request.request_id = self._frame_request_id("goto_%s" % pose_name)
        request.pose_name = str(pose_name)
        joints = (self._last_joint_state or {}).get("position") or []
        request.current_joint_state = [
            float(v) for v in list(joints)[:6]]
        request.scene_revision = str(self._frame_scene_revision or "")
        result = None
        for attempt in range(3):
            # a fresh request id per attempt: the frame runtime may
            # treat a repeated id as a replay, not a retry
            request.request_id = self._frame_request_id(
                "goto_%s" % pose_name
                + ("" if attempt == 0 else "_r%d" % attempt))
            stats = self.call_srv(
                stats_client, GetCargoMapStats.Request(), timeout=5.0)
            if stats is None or not stats.success:
                return False, "MAP_STATUS_UNAVAILABLE"
            request.latest_map_revision = int(stats.map_revision)
            result = self.call_srv(client, request, timeout=timeout)
            if result is None:
                return False, "GOTO_ROUTE_UNAVAILABLE"
            if result.success:
                break
            reason = str(result.reason_code or "GOTO_ROUTE_REJECTED")
            if reason != "MAP_STALE" or attempt == 2:
                detail = str(getattr(result, "detail", "") or "")
                if detail:
                    return False, "%s detail=%s" % (reason, detail[:500])
                return False, reason
            if not self._wait_map_quiescence(
                    stats_client, int(stats.map_revision)):
                return False, reason
        if not bool(getattr(result, "success", False)):
            return False, str(
                getattr(result, "reason_code", None)
                or "GOTO_ROUTE_REJECTED")
        if bool(result.already_there):
            self._last_goto_route = None
            return True, "already_there"
        admission = {
            "route_digest": str(result.route_digest or ""),
            "certification_id": str(result.certification_id or ""),
            "admission_map_revision": int(
                result.admission_map_revision or 0),
        }
        digests = [str(d) for d in (result.leg_digests or [])]
        states = [str(v) for v in (result.leg_load_states or [])]
        executed = 0
        for index, segment in enumerate(result.motion_segments or []):
            # [ST09-I S5.7-6] each leg claims ITS OWN certified scope
            # (named-pose routes certify under the unloaded index).
            leg_admission = dict(admission)
            if index < len(digests):
                leg_admission["leg_digest"] = digests[index]
                leg_admission["load_state"] = (
                    states[index] if index < len(states) else "unloaded")
            ok, message, _rec = self._typed.plan_motion(
                segment, self._args.plan_timeout,
                name="PlanMotionV2:goto_%s" % pose_name,
                admission=leg_admission)
            executed += 1
            if not ok:
                self._last_goto_route = None
                return False, "GOTO_LEG_FAILED:%s" % str(
                    message or segment.name)[:120]
        if executed == 0:
            self._last_goto_route = None
            return False, "GOTO_ROUTE_EMPTY"
        self._last_goto_route = {
            "request_id": str(request.request_id),
            "route_digest": admission["route_digest"],
            "certification_id": admission["certification_id"],
            "scene_revision": str(self._frame_scene_revision or ""),
        }
        return True, ""

    def wait_tracked_cargo(self, generation, timeout, spawn_id=None):
        """Wait for tracker cargo of *generation* with n_points > 0."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            stats = _parse_json(self._filter_stats["payload"])
            if cargo_generation_ready(stats, generation) and (
                    spawn_id is None
                    or tracker_epoch_matches(stats, generation, spawn_id)):
                return True
            time.sleep(0.05)
        return False

    def wait_yolo_boxes(self, generation, spawn_id, min_stamp, timeout):
        """Wait for post-mask-filter YOLO cargo boxes of this spawn.

        Scans the stats ring newest-first so a matching sample is not lost
        behind a stale latest latch. Returns the matching stats dict, or
        None on timeout (``_yolo_decision_record`` keeps the last checked).
        """
        self._yolo_decision_record = None
        last_checked = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            records = self._seg_stats_buf.newest_first()
            latch = _parse_json(self._seg_stats["payload"])
            combined = list(records)
            if latch is not None and latch not in combined:
                combined.append(latch)
            if combined:
                last_checked = combined[0]
            match = first_yolo_ready(
                combined, generation, expected_id=spawn_id,
                min_stamp=min_stamp)
            if match:
                self._yolo_decision_record = match
                return match
            time.sleep(0.05)
        self._yolo_decision_record = last_checked
        return None

    def overlay_projection_inputs(self):
        """World->optical extrinsics + pinhole K for the dumped colour frame."""
        info = self._camera_info
        if info is None or len(info.k) < 6:
            return None, None, "no_camera_info"
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return None, None, "bad_intrinsics"
        camera_frame = info.header.frame_id or "camera_depth_optical_frame"
        color = (self._trial_snapshot or {}).get("color")
        if color is not None and color.header.frame_id:
            camera_frame = color.header.frame_id
        tf_stamp = dump_tf_stamp_from_color(color)
        if tf_stamp is None:
            return None, None, "no_color_stamp"
        stamp = rclpy.time.Time(
            seconds=int(tf_stamp[0]), nanoseconds=int(tf_stamp[1]),
            clock_type=self.get_clock().clock_type)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                camera_frame, "world", stamp,
                rclpy.duration.Duration(seconds=0.5))
        except TransformException as exc:
            return None, None, "tf:%s" % exc
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rotation = rotation_from_quaternion([q.x, q.y, q.z, q.w])
        translation = [float(t.x), float(t.y), float(t.z)]
        return (rotation, translation), (fx, fy, cx, cy), None

    def optical_to_world(self):
        """Optical->world extrinsics + pinhole K for the live depth frame."""
        info = self._camera_info
        if info is None or len(info.k) < 6:
            return None, None, "no_camera_info"
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        if fx <= 1e-6 or fy <= 1e-6:
            return None, None, "bad_intrinsics"
        frames = []
        color = (self._trial_snapshot or {}).get("color")
        if color is not None and color.header.frame_id:
            frames.append(str(color.header.frame_id))
        if info.header.frame_id:
            frames.append(str(info.header.frame_id))
        frames.extend((
            "camera_color_optical_frame",
            "camera_depth_optical_frame",
        ))
        seen = set()
        last_err = "tf:no_frame"
        for camera_frame in frames:
            if not camera_frame or camera_frame in seen:
                continue
            seen.add(camera_frame)
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    "world", camera_frame, rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.5))
            except TransformException as exc:
                last_err = "tf:%s" % exc
                continue
            t = tf_msg.transform.translation
            q = tf_msg.transform.rotation
            rotation = rotation_from_quaternion([q.x, q.y, q.z, q.w])
            translation = [float(t.x), float(t.y), float(t.z)]
            return (rotation, translation), (fx, fy, cx, cy), None
        return None, None, last_err

    def latest_depth_m(self):
        buf = (self._buffers or {}).get("depth")
        if buf is None:
            return None
        snap = buf.snapshot()
        if not snap:
            return None
        millimetres = adapters.depth_array_from_msg(snap[max(snap)])
        if millimetres is None:
            return None
        return millimetres.astype("float32") * 0.001

    def wait_spawn_visual(self, gt, timeout, expected_class=None):
        """Wait until the platform depth blob matches this spawn's catalog AABB.

        This is the camera's rendered object, not YOLO bbox-fill / PCA.
        *gt* is GetCurrentBox (catalog width/depth/height). A leftover
        previous mesh must fail here, not at DETECT_GATE.
        """
        deadline = time.time() + float(timeout)
        last = {"reason": "no_depth"}
        platform_z = float(gt.z) - 0.5 * float(gt.height)
        gt_size = (float(gt.width), float(gt.depth), float(gt.height))
        info_deadline = time.time() + min(2.0, float(timeout))
        while self._camera_info is None and time.time() < info_deadline:
            time.sleep(0.05)
        while time.time() < deadline:
            depth_m = self.latest_depth_m()
            extrinsics, intrinsics, err = self.optical_to_world()
            if err or depth_m is None:
                last = {"reason": err or "no_depth", "gt": list(gt_size)}
                time.sleep(0.08)
                continue
            rotation, translation = extrinsics
            fx, fy, cx, cy = intrinsics
            cam = depth_to_camera_xyz(depth_m, fx, fy, cx, cy)
            world = transform_camera_xyz_to_world(cam, rotation, translation)
            measured = raised_object_measure(
                world, platform_z, (float(gt.x), float(gt.y)), roi_margin=0.5,
                max_height=float(gt.height) + self._args.tol_visual_z + 0.02)
            if measured is None:
                last = {
                    "reason": "no_blob",
                    "observed": None,
                    "center": None,
                    "n_points": None,
                    "gt": list(gt_size),
                    "obs_class": None,
                    "gt_class": expected_class or nearest_catalog_id(
                        gt_size, silhouette=False),
                }
                time.sleep(0.08)
                continue
            observed = (
                measured["width"], measured["depth"], measured["height"])
            last = {
                "reason": "size",
                "observed": [
                    measured["width"], measured["depth"], measured["height"]],
                "center": [measured["x"], measured["y"], measured["z"]],
                "n_points": measured["n"],
                "gt": list(gt_size),
                "obs_class": nearest_catalog_id(observed, silhouette=True),
                "gt_class": expected_class or nearest_catalog_id(
                    gt_size, silhouette=False),
            }
            if spawn_visual_matches_gt(
                    observed, gt_size,
                    tol_xy=self._args.tol_visual_xy,
                    tol_z=self._args.tol_visual_z,
                    expected_class=expected_class):
                last["reason"] = "ok"
                return True, last
            time.sleep(0.08)
        return False, last

    def snapshot_frames(self, timeout=FRAME_JOIN_WAIT_SEC, min_stamp_sec=None,
                        require_cargo=False):
        """Wait for one stamp-joined color+depth+overlay triplet, then freeze it."""
        if not self._buffers:
            self._trial_snapshot = {}
            self._trial_join = {
                "aligned": False, "stamp_key": None, "cargo_matched": False,
                "min_stamp_sec": min_stamp_sec,
            }
            return self._trial_snapshot
        deadline = time.time() + float(timeout)
        snaps = {}
        joined = None
        cargo_matched = False
        required = ("color", "depth", "overlay")
        cargo_required = required + ("cargo",)
        while True:
            snaps = {name: buf.snapshot() for name, buf in self._buffers.items()}
            if require_cargo:
                joined = pick_joined_stamp(
                    snaps, required=cargo_required, min_stamp_sec=min_stamp_sec)
                cargo_matched = joined is not None
                if joined is None:
                    joined = pick_joined_stamp(
                        snaps, required=required, min_stamp_sec=min_stamp_sec)
            else:
                joined = pick_joined_stamp(
                    snaps, required=required, min_stamp_sec=min_stamp_sec)
                cargo_key = pick_joined_stamp(
                    snaps, required=("color", "cargo"),
                    min_stamp_sec=min_stamp_sec)
                cargo_matched = (
                    joined is not None and cargo_key is not None
                    and cargo_key == joined)
            if joined is not None and (not require_cargo or cargo_matched):
                break
            if time.time() >= deadline:
                break
            time.sleep(0.02)
        names = ("color", "depth", "overlay", "mask", "cargo")
        if joined is not None:
            self._trial_snapshot = {
                name: (snaps.get(name) or {}).get(joined) for name in names
            }
            self._trial_join = {
                "aligned": bool(not require_cargo or cargo_matched),
                "stamp_key": joined,
                "cargo_matched": bool(
                    cargo_matched or (
                        joined in (snaps.get("cargo") or {}))),
                "min_stamp_sec": min_stamp_sec,
            }
            if require_cargo:
                self._trial_join["aligned"] = bool(cargo_matched)
                self._trial_join["cargo_matched"] = bool(cargo_matched)
        else:
            self._trial_snapshot = {}
            for name in names:
                buf = snaps.get(name) or {}
                key = max(buf) if buf else None
                self._trial_snapshot[name] = buf.get(key) if key else None
            self._trial_join = {
                "aligned": False,
                "stamp_key": None,
                "cargo_matched": False,
                "min_stamp_sec": min_stamp_sec,
            }
        return self._trial_snapshot

    def freeze_exact_stamp(self, stamp_sec, tol=1e-6):
        """Re-freeze the snapshot to the detector's cloud header stamp."""
        if stamp_sec is None or not self._buffers:
            return False
        snaps = {name: buf.snapshot() for name, buf in self._buffers.items()}
        target = float(stamp_sec)
        key = None
        for candidate in (snaps.get("color") or {}):
            value = stamp_sec_from_key(candidate)
            if value is not None and abs(value - target) <= float(tol):
                key = candidate
                break
        if key is None:
            return False
        if (key not in (snaps.get("overlay") or {})
                or key not in (snaps.get("depth") or {})):
            return False
        names = ("color", "depth", "overlay", "mask", "cargo")
        self._trial_snapshot = {
            name: (snaps.get(name) or {}).get(key) for name in names
        }
        self._trial_join = {
            "aligned": True,
            "stamp_key": key,
            "cargo_matched": key in (snaps.get("cargo") or {}),
            "min_stamp_sec": (self._trial_join or {}).get("min_stamp_sec"),
        }
        return True

    def wait_detect_frame(self, stamp_sec, timeout=4.0):
        if stamp_sec is None:
            return False
        deadline = time.time() + float(timeout)
        while True:
            if self.freeze_exact_stamp(stamp_sec):
                return True
            if time.time() >= deadline:
                return False
            time.sleep(0.02)

    def ros_now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def decoded_dump(self):
        """Stamp-joined RGB/depth arrays + metadata from the last snapshot."""
        snapshot = self._trial_snapshot or {}
        join = self._trial_join or {}
        color = _decode_dump_image(snapshot.get("color"))
        overlay = _decode_dump_image(snapshot.get("overlay"))
        depth_m = None
        depth_msg = snapshot.get("depth")
        if depth_msg is not None:
            millimetres = adapters.depth_array_from_msg(depth_msg)
            if millimetres is not None:
                depth_m = millimetres.astype("float32") * 0.001
        mask_labels = None
        mask_msg = snapshot.get("mask")
        if mask_msg is not None:
            labels = adapters.image_array_from_msg(mask_msg)
            if labels is not None:
                mask_labels = labels[:, :, 0] if labels.ndim == 3 else labels
        images, arrays, extras = build_aligned_dump(
            color, depth_m, overlay, mask_labels)
        extras["aligned"] = bool(join.get("aligned"))
        extras["cargo_matched"] = bool(join.get("cargo_matched"))
        stamp_key = join.get("stamp_key")
        extras["join_stamp_key"] = list(stamp_key) if stamp_key else None
        extras["join_stamp"] = (
            stamp_sec_from_key(stamp_key) if stamp_key else None)
        extras["min_stamp_sec"] = join.get("min_stamp_sec")
        extras["status"] = _parse_json(self._status["payload"])
        extras["seg_stats"] = dump_seg_stats(
            self._frozen_bundle, _parse_json(self._seg_stats["payload"]))
        extras["diag"] = _parse_json(self._diag["payload"])
        extras["filter_stats"] = _parse_json(self._filter_stats["payload"])
        for name in ("color", "depth", "overlay", "mask", "cargo"):
            extras[name] = _image_meta(snapshot.get(name))
        return images, extras, arrays

    def capture_perception_layers(self, stamp_sec=None):
        """YOLO / mask / cargo / PCA snapshot nearest to *stamp_sec*."""
        layers = {
            "stamp": stamp_sec,
            "detection_frame": None,
            "detection_frame_dt": None,
            "filter_stats": None,
            "filter_stats_dt": None,
            "seg_stats": None,
            "seg_stats_dt": None,
            "mask_cargo_aabb": None,
            "cargo_aabb": None,
            "dropped_yolo": [],
        }
        frame, dt = self._detection_frames.nearest(stamp_sec)
        if frame is not None:
            layers["detection_frame"] = detection_frame_as_dict(frame)
            layers["detection_frame_dt"] = dt
        if (self._frozen_bundle
                and self._frozen_bundle.get("decision_record") is not None):
            layers["seg_stats"] = self._frozen_bundle["decision_record"]
            layers["seg_stats_dt"] = 0.0
        else:
            seg, dt = self._seg_stats_buf.nearest(stamp_sec)
            if seg is None:
                seg = _parse_json(self._seg_stats["payload"])
                dt = None
            layers["seg_stats"] = seg
            layers["seg_stats_dt"] = dt
        seg = layers.get("seg_stats")
        if isinstance(seg, dict):
            layers["dropped_yolo"] = list(
                seg.get("detections_dropped_self_body") or [])
        filt, dt = self._filter_stats_buf.nearest(stamp_sec)
        if filt is None:
            filt = _parse_json(self._filter_stats["payload"])
            dt = None
        layers["filter_stats"] = filt
        layers["filter_stats_dt"] = dt
        snapshot = self._trial_snapshot or {}
        mask_msg = snapshot.get("mask")
        mask_labels = None
        if mask_msg is not None:
            labels = adapters.image_array_from_msg(mask_msg)
            if labels is not None:
                mask_labels = labels[:, :, 0] if labels.ndim == 3 else labels
        if mask_labels is not None:
            layers["mask_cargo_aabb"] = label_aabb(mask_labels, 2)
        cargo_msg = snapshot.get("cargo")
        if cargo_msg is None:
            buf = (self._buffers or {}).get("cargo")
            if buf is not None:
                cargo_msg, cargo_dt = buf.nearest(stamp_sec)
                layers["cargo_cloud_dt"] = cargo_dt
        if cargo_msg is not None:
            pts = adapters.cloud_points_from_msg(cargo_msg)
            aabb = points_aabb(pts) if pts is not None else None
            n_pts = 0 if pts is None else int(len(pts))
            layers["cargo_aabb"] = None if aabb is None else {
                "dx": aabb[0], "dy": aabb[1], "dz": aabb[2], "n": aabb[3],
            }
            layers["cargo_n_points"] = n_pts
            layers["cargo"] = _image_meta(cargo_msg)
        return layers

    def attach_perception_layers(self, fields, stamp_sec=None):
        extras = fields.setdefault("extras", {})
        if stamp_sec is None:
            stamp_sec = extras.get("detect_cloud_stamp")
        layers = self.capture_perception_layers(stamp_sec)
        extras["perception_layers"] = layers
        if extras.get("filter_stats") is None:
            extras["filter_stats"] = layers.get("filter_stats")
        if extras.get("seg_stats") is None:
            extras["seg_stats"] = layers.get("seg_stats")
        return layers

    def freeze_failure_bundle(self, decision_record):
        """Pin rings to the gated stats stamp. Does not wait for later frames."""
        snaps = {}
        if self._buffers:
            snaps = {name: buf.snapshot() for name, buf in self._buffers.items()}
        bundle = freeze_dump_bundle(decision_record, snaps)
        self._frozen_bundle = bundle
        join_key = bundle.get("join_key")
        names = ("color", "depth", "overlay", "mask", "cargo")
        if join_key is not None:
            self._trial_snapshot = {
                name: (snaps.get(name) or {}).get(join_key) for name in names
            }
            self._trial_join = {
                "aligned": True,
                "stamp_key": join_key,
                "cargo_matched": join_key in (snaps.get("cargo") or {}),
                "min_stamp_sec": (self._trial_join or {}).get("min_stamp_sec"),
            }
        else:
            self._trial_snapshot = {}
            self._trial_join = {
                "aligned": False, "stamp_key": None, "cargo_matched": False,
            }
        return bundle

    def maybe_dump(self, fields, stamp0):
        """Write color/depth/overlay/mask PNGs for this trial. Mutates extras."""
        dump_dir = getattr(self._args, "dump_dir", "") or ""
        if not dump_dir:
            return ""
        info_deadline = time.time() + 2.0
        while self._camera_info is None and time.time() < info_deadline:
            time.sleep(0.05)
        detect_stamp = (fields.get("extras") or {}).get("detect_cloud_stamp")
        if self._frozen_bundle is None:
            if detect_stamp is not None:
                if not self.wait_detect_frame(detect_stamp):
                    self.snapshot_frames(
                        timeout=FRAME_JOIN_AFTER_SPAWN_SEC,
                        min_stamp_sec=stamp0, require_cargo=True)
            else:
                self.snapshot_frames(
                    timeout=FRAME_JOIN_AFTER_SPAWN_SEC,
                    min_stamp_sec=stamp0, require_cargo=True)
        images, extras, arrays = self.decoded_dump()
        extrinsics, intrinsics, proj_err = self.overlay_projection_inputs()
        rotation = translation = None
        if extrinsics is not None:
            rotation, translation = extrinsics
        overlay, box_meta = annotate_overlay_boxes(
            images.get("overlay"),
            gt=(fields.get("extras") or {}).get("gt"),
            measured=(fields.get("extras") or {}).get("measured"),
            rotation=rotation,
            translation=translation,
            intrinsics=intrinsics)
        layers = self.attach_perception_layers(fields, detect_stamp)
        dropped = layers.get("dropped_yolo") or []
        if overlay is not None and dropped:
            overlay = annotate_yolo_detections(overlay, dropped)
        if overlay is not None:
            images["overlay"] = overlay
        if proj_err:
            box_meta["project_error"] = proj_err
        extras.update(box_meta)
        extras["perception_layers"] = layers
        extras["filter_stats"] = layers.get("filter_stats") or extras.get(
            "filter_stats")
        yolo_ready = (fields.get("extras") or {}).get("seg_stats_yolo_ready")
        if yolo_ready:
            extras["seg_stats_yolo_ready"] = yolo_ready
            extras["seg_stats_dump"] = extras.get("seg_stats")
        visual_info = (fields.get("extras") or {}).get("spawn_visual")
        if visual_info:
            extras["spawn_visual"] = visual_info
        if self._frozen_bundle:
            extras["capture_complete"] = bool(
                self._frozen_bundle.get("capture_complete"))
            extras["missing"] = list(self._frozen_bundle.get("missing") or [])
            extras["decision_record"] = self._frozen_bundle.get(
                "decision_record")
            extras["join_dt"] = self._frozen_bundle.get("join_dt")
            extras["replay_possible"] = bool(
                self._frozen_bundle.get("replay_possible"))
        dump_stamp = self.ros_now_sec()
        extras["dump_stamp"] = dump_stamp
        images, extras = apply_dump_timestamp_banners(
            images, extras,
            dump_stamp=dump_stamp,
            detect_stamp=detect_stamp)
        dest = write_trial_dump(
            dump_dir, fields, images=images, extras=extras, arrays=arrays)
        if dest:
            fields.setdefault("extras", {})["dump"] = dest
            print("  dumped %s" % dest, flush=True)
        return dest or ""

    def finish_trial(self, fields, t0, stamp0=None, dump=False, clear=False):
        self.attach_perception_layers(fields)
        if dump:
            self.maybe_dump(fields, stamp0)
        if clear:
            goto_ok, goto_msg, _ = self.goto_observe()
            if not goto_ok:
                fields.setdefault("extras", {})["clear_goto"] = goto_msg
            self.call_srv(self._clear, ClearCurrentBox.Request(), timeout=15.0)
        fields["wall_time_sec"] = time.time() - t0
        return TrialRecord(**fields)

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

    def _lookup_xyz_quat(self, target, source):
        """TF position + orientation. Raises ``TransformException``."""
        tf_msg = self._tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(),
            rclpy.duration.Duration(seconds=1.0))
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        return (
            (float(t.x), float(t.y), float(t.z)),
            (float(r.x), float(r.y), float(r.z), float(r.w)),
        )

    def add_scene_box(self, box):
        if self._scene is None:
            return False, "no planning scene client"
        try:
            xyz, quat, size = pickup_collision_aabb(box)
        except ValueError as exc:
            return False, str(exc)
        return self._scene.add_pickup_box(xyz, quat, size)

    def _auth_policy(self):
        args = self._args
        return PickAuthorizationPolicy(AuthorizationConfig(
            max_attempts=int(getattr(
                args, "auth_max_attempts", DEFAULT_MAX_ATTEMPTS)),
            max_elapsed_sec=float(getattr(
                args, "auth_max_elapsed_sec", DEFAULT_MAX_ELAPSED_SEC)),
            wait_period_sec=float(getattr(
                args, "auth_wait_period_sec", DEFAULT_WAIT_PERIOD_SEC)),
        ))

    def detect_until_authorized(self):
        """DetectLuggage until MEASURED_SUPPORT, else fail closed."""

        def _once():
            detect = self.call_srv(
                self._detect, DetectLuggage.Request(),
                timeout=self._args.detect_timeout)
            if detect is None:
                return False, None, "DETECT_TIMEOUT"
            if not detect.success or not detect.luggage:
                return False, None, detect.message or "MEASURED_NONE"
            return True, detect.luggage[0], detect.message or ""

        result = run_authorization_loop(
            _once, time.sleep, time.monotonic, self._auth_policy())
        self._last_auth_diag = self._wait_auth_reads(result.attempts)
        return result

    def _wait_auth_reads(self, attempts, timeout=0.5):
        """Latest diagnostics once this detect loop's reads have arrived."""
        needed = int(attempts or 0)
        deadline = time.time() + float(timeout)
        latest = None
        while True:
            latest = _parse_json(self._diag.get("payload"))
            reads = (latest or {}).get("auth_reads") or []
            if needed <= 0 or len(reads) >= needed or time.time() >= deadline:
                return latest
            time.sleep(0.02)

    def _detection_frame_rows(self):
        rows = []
        for key, msg in sorted(self._detection_frames.snapshot().items()):
            rows.append({
                "stamp": stamp_sec_from_key(key),
                "generation": int(msg.generation),
                "instance_id": str(msg.instance_id),
                "pca_source": str(msg.pca_source),
                "pca_valid": bool(msg.pca_valid),
                "pca_reason": str(msg.pca_reason),
                "n_cargo_points": int(msg.n_cargo_points),
                "support_valid": bool(msg.support_valid),
                "support_reason": str(msg.support_reason),
                "geometry_level": int(msg.geometry_level),
                "size_wdh": _detection_frame_size(msg),
            })
        return rows

    def perception_boundary_document(self):
        """Join, mask, and authorization-window records for this pick."""
        join_trace = list(self._join_trace)
        mask_trace = list(self._mask_trace)
        diag = self._last_auth_diag
        if not isinstance(diag, dict):
            diag = _parse_json(self._diag.get("payload"))
        reads = (diag or {}).get("auth_reads") or []
        return {
            "current_box": _parse_json(
                self._current_box_topic.get("payload")),
            "join_trace": join_trace,
            "mask_trace": mask_trace,
            "detection_frames": self._detection_frame_rows(),
            "diagnostics": diag,
            "classification": classify_hold_boundary(
                join_trace, auth_reads=reads, mask_trace=mask_trace),
        }

    def sync_pickup_geometry(self, pick_msg, timeout=5.0):
        """Backfill measured geometry onto the current box instance (CAS).

        expected_generation comes from the latched /luggage/current_box the
        driver already tracks, so the spawner refuses a sync that would
        leak onto a newer box. Vacuum attach geometry and the motion
        occupancy sweep payload read the measured record afterwards.
        Returns (ok, message, generation).
        """
        _box_id, generation = parse_current_box_payload(
            self._current_box_topic.get("payload") or "")
        request = SyncPickupBox.Request()
        request.box = pick_msg
        request.expected_generation = int(generation)
        response = self.call_srv(self._sync, request, timeout=timeout)
        if response is None:
            return False, "sync service timeout", generation
        return (bool(response.success), str(response.message or ""),
                int(response.generation))

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

    def gt_model_name(self, timeout=5.0):
        """GT model_name of the current instance, via the pull service.

        Eval-side path only: /luggage/current_box no longer carries GT
        fields (privilege boundary). Returns "" when unavailable.
        """
        response = self.call_srv(
            self._current, GetCurrentBox.Request(), timeout=timeout)
        if response is None or not response.success:
            return ""
        return str(response.model_name or "")

    def run_trial(self, index):
        t0 = time.time()
        self._yolo_decision_record = None
        self._frozen_bundle = None
        fields = {
            "index": index,
            "catalog_id": "",
            "visual_id": "",
            "detect_failure": "",
            "accuracy_ok": None,
            "accuracy_reason": "",
            "detect_usable": None,
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
            "vac_attach": None,
            "vac_follow": None,
            "fail_code": "",
            "spawn_to_yolo_sec": None,
            "spawn_to_visual_sec": None,
            "spawn_to_detect_sec": None,
            "extras": {},
        }
        goto_ok, goto_msg, _ = self.goto_observe()
        if not goto_ok:
            fields["fail_code"] = "GOTO_FAILED"
            fields["extras"]["goto"] = goto_msg
            return self.finish_trial(fields, t0, dump=True, clear=True)

        dirty = self.ensure_clean()
        if dirty:
            fields["fail_code"] = dirty.split(":")[0]
            fields["extras"]["clean"] = dirty
            return self.finish_trial(fields, t0)

        spawn = self.call_srv(
            self._spawn, SpawnNextBox.Request(), timeout=30.0)
        if spawn is None or not spawn.success:
            fields["fail_code"] = "SPAWN_FAILED"
            fields["extras"]["spawn"] = (
                spawn.message if spawn else "timeout")
            return self.finish_trial(fields, t0, dump=True, clear=True)

        spawn_stamp = self.ros_now_sec()
        fields["extras"]["spawn_stamp"] = spawn_stamp
        fields["extras"]["gt"] = observation_from_detected(spawn.box).__dict__
        fields["extras"]["spawn_id"] = spawn.box.id

        size_eval = _parse_json(self._size_eval["payload"]) or {}
        fields["catalog_id"] = str(size_eval.get("catalog_id") or "")
        fields["visual_id"] = str(size_eval.get("visual_id") or "")
        if not fields["catalog_id"] and spawn.box.id:
            parts = spawn.box.id.rsplit("_", 1)
            if len(parts) == 2:
                fields["catalog_id"] = parts[1]

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
        fields["extras"]["spawn_generation"] = generation

        snapshot = _parse_json(self._status["payload"]) or {}
        try:
            stamp0 = float(snapshot.get("primary_stamp") or 0.0)
        except (TypeError, ValueError):
            stamp0 = 0.0

        yolo_stats = self.wait_yolo_boxes(
            generation, spawn.box.id, spawn_stamp,
            self._args.geometry_timeout)
        if not yolo_stats:
            fields["fail_code"] = "YOLO_NOT_READY"
            decision = self._yolo_decision_record or _parse_json(
                self._seg_stats["payload"])
            fields["extras"]["seg_stats"] = decision
            fields["extras"]["decision_record"] = decision
            fields["extras"]["filter_stats"] = _parse_json(
                self._filter_stats["payload"])
            self.freeze_failure_bundle(decision)
            return self.finish_trial(
                fields, t0, stamp0=stamp0, dump=True, clear=True)
        try:
            yolo_stamp = float(yolo_stats.get("stamp"))
        except (TypeError, ValueError):
            yolo_stamp = self.ros_now_sec()
        fields["extras"]["yolo_ready_stamp"] = yolo_stamp
        fields["extras"]["seg_stats_yolo_ready"] = yolo_stats
        fields["spawn_to_yolo_sec"] = yolo_stamp - spawn_stamp

        if not self.wait_tracked_cargo(
                generation, self._args.geometry_timeout, spawn.box.id):
            fields["fail_code"] = tracker_wait_fail_code(
                _parse_json(self._filter_stats["payload"]),
                generation, spawn.box.id)
            fields["extras"]["filter_stats"] = _parse_json(
                self._filter_stats["payload"])
            fields["extras"]["seg_stats"] = _parse_json(
                self._seg_stats["payload"])
            return self.finish_trial(
                fields, t0, stamp0=stamp0, dump=True, clear=True)

        gt_spawn = observation_from_detected(spawn.box)
        visual_ok, visual_info = self.wait_spawn_visual(
            gt_spawn, self._args.geometry_timeout,
            expected_class=fields.get("catalog_id") or None)
        fields["extras"]["spawn_visual"] = visual_info
        fields["extras"]["spawn_visual"] = visual_info
        if visual_ok:
            fields["spawn_to_visual_sec"] = self.ros_now_sec() - spawn_stamp
        else:
            reason = str((visual_info or {}).get("reason") or "")
            if reason.startswith("tf:") or reason in (
                    "no_camera_info", "bad_intrinsics", "no_depth"):
                fields["fail_code"] = "SPAWN_VISUAL_TF"
                fields["extras"]["seg_stats"] = _parse_json(
                    self._seg_stats["payload"])
                fields["extras"]["filter_stats"] = _parse_json(
                    self._filter_stats["payload"])
                return self.finish_trial(
                    fields, t0, stamp0=stamp0, dump=True, clear=True)
            fields["fail_code"] = "SPAWN_VISUAL_MISMATCH"
            fields["extras"]["seg_stats"] = _parse_json(
                self._seg_stats["payload"])
            fields["extras"]["filter_stats"] = _parse_json(
                self._filter_stats["payload"])
            if getattr(self._args, "strict_gt", False):
                return self.finish_trial(
                    fields, t0, stamp0=stamp0, dump=True, clear=True)

        auth = self.detect_until_authorized()
        fields["extras"]["pick_authorization"] = auth.to_dict()
        current = self.call_srv(
            self._current, GetCurrentBox.Request(), timeout=10.0)
        diag = self._diag["payload"]
        diag_parsed = _parse_json(diag) or {}
        fields["extras"]["detect_cloud_stamp"] = diag_parsed.get("cloud_stamp")
        try:
            cloud_stamp = float(diag_parsed.get("cloud_stamp"))
        except (TypeError, ValueError):
            cloud_stamp = None
        if cloud_stamp is not None and cloud_stamp + 1e-9 >= spawn_stamp:
            fields["spawn_to_detect_sec"] = cloud_stamp - spawn_stamp
        if current is not None and current.success:
            fields["extras"]["gt"] = observation_from_detected(
                current.box).__dict__
        if not auth.authorized:
            fields["fail_code"] = auth.decision.reason
            fields["detect_failure"] = auth.detect_reason or auth.decision.reason
            fields["extras"]["detect_message"] = auth.detect_reason
            fields["extras"]["support_stream"] = self._support_stream_snapshot()
            fields["extras"]["backend"] = (
                (_parse_json(self._seg_stats["payload"]) or {}).get("backend"))
            return self.finish_trial(
                fields, t0, stamp0=stamp0, dump=True, clear=True)

        detect_message = auth.detect_reason
        pick_msg = auth.box
        fields["detect_failure"] = detect_message or ""
        fields["extras"]["detect_message"] = detect_message
        fields["extras"]["backend"] = (
            (_parse_json(self._seg_stats["payload"]) or {}).get("backend"))

        if current is None or not current.success:
            fields["fail_code"] = "GT_UNAVAILABLE"
            return self.finish_trial(
                fields, t0, stamp0=stamp0, dump=True, clear=True)

        if is_gt_fallback(detect_message, diag):
            fields["fail_code"] = "DETECT_GT_FALLBACK"
            fields["detect_failure"] = perception_reason(
                detect_message, diag) or "DETECT_GT_FALLBACK"
            return self.finish_trial(
                fields, t0, stamp0=stamp0, dump=True, clear=True)

        if not is_perception_estimate(
                True, detect_message, True, diag):
            fields["fail_code"] = detect_message or "MEASURED_NONE"
            fields["detect_failure"] = fields["fail_code"]
            return self.finish_trial(
                fields, t0, stamp0=stamp0, dump=True, clear=True)

        measured = observation_from_detected(pick_msg)
        gt = observation_from_detected(current.box)
        result = self._accuracy.compare(measured, gt)
        fields["detect_usable"] = True
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

        if not result.ok and not fields["fail_code"]:
            fields["fail_code"] = "DETECT_GATE:%s" % result.reason
        # Dump while the suitcase is still in the camera frame, before motion.
        self.maybe_dump(fields, stamp0)
        if not result.ok and getattr(self._args, "strict_gt", False):
            return self.finish_trial(fields, t0, stamp0=stamp0, clear=True)

        if self._args.use_vacuum:
            sync_ok, sync_msg, sync_gen = self.sync_pickup_geometry(pick_msg)
            fields["extras"]["payload_sync"] = {
                "ok": sync_ok, "message": sync_msg, "generation": sync_gen}
            if not sync_ok:
                fields["fail_code"] = "PAYLOAD_SYNC_FAILED"
                return self.finish_trial(fields, t0, stamp0=stamp0, clear=True)
            scene_ok, scene_msg = self.add_scene_box(pick_msg)
            fields["extras"]["scene_add"] = scene_msg
            if not scene_ok:
                fields["fail_code"] = "SCENE_ADD_FAILED"
                return self.finish_trial(fields, t0, stamp0=stamp0, clear=True)

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
                fields["extras"]["build"] = "%s: %s" % (error.code, error.field)
        else:
            req = BuildMotionSequence.Request()
            req.phase = "pick"
            req.pick = pick_msg
            built = self.call_srv(self._build, req, timeout=15.0)
        if built is None or not built.success:
            fields["fail_code"] = "BUILD_FAILED"
            fields["extras"]["build"] = (
                built.message if built else "timeout")
            return self.finish_trial(fields, t0, stamp0=stamp0, clear=True)

        segments = list(built.segments)
        motion_segments = list(getattr(built, "motion_segments", ()) or ())
        fields["segments_planned"] = len(segments)
        attach_z = None
        attach_xyz = None
        retreat_xyz = None
        failures = []
        succeeded = 0
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
                if getattr(self._args, "use_vacuum", False):
                    vac_ok, vac_msg = self.vacuum_command(True, pick_msg=pick_msg)
                    fields["vac_attach"] = vac_ok
                    fields["extras"]["vac_attach"] = vac_msg
                    if not vac_ok:
                        revision_dump = getattr(
                            self, "_last_vacuum_attach_evidence", None)
                        if revision_dump:
                            fields["extras"]["vacuum_attach_revisions"] = (
                                revision_dump)
                        failures.append(("vacuum_attach", vac_msg))
                        fields["fail_code"] = "VACUUM_ATTACH"
                        break
            if segment.name == "pick_retreat":
                retreat_xyz, tf_err = self.suction_xyz()
                if retreat_xyz is None:
                    fields["extras"]["retreat_tf"] = tf_err
        fields["segments_succeeded"] = succeeded
        fields["segment_failures"] = tuple(failures)

        if getattr(self._args, "use_vacuum", False) and fields.get("vac_attach"):
            state = dict(self._vacuum_state or {})
            fields["extras"]["vac_state"] = state
            events = self._vacuum_events if isinstance(
                self._vacuum_events, dict) else {}
            fields["extras"]["follow_skipped"] = events.get("follow_skipped")
            fields["vac_follow"] = bool(
                state.get("attached") and not state.get("fail_reason"))
            if not fields["vac_follow"] and not fields["fail_code"]:
                fields["fail_code"] = "VACUUM_FOLLOW"
            off_ok, off_msg = self.vacuum_command(False)
            fields["extras"]["vac_detach"] = off_msg
            if not off_ok and not fields["fail_code"]:
                fields["fail_code"] = "VACUUM_DETACH"
        elif getattr(self._args, "use_vacuum", False):
            if self._scene is not None:
                self._scene.detach_and_remove()

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

        goto_ok, goto_msg, _ = self.goto_observe()
        if not goto_ok:
            fields.setdefault("extras", {})["return_observe"] = goto_msg
            if not fields["fail_code"]:
                fields["fail_code"] = "GOTO_FAILED"
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
        "- Spawn-visual tols: xy=%.3f z=%.3f" % (
            extra.get("tol_visual_xy", DEFAULT_VISUAL_TOL_XY),
            extra.get("tol_visual_z", DEFAULT_VISUAL_TOL_Z)),
        "- Retreat check: expected ΔZ=%.3f m, tol=%.3f m" % (
            extra.get("expected_retreat_dz", 0.35),
            extra.get("retreat_tol", 0.08)),
        "",
        "## Rates (independent denominators)",
        "",
        "| Metric | Rate | n |",
        "|---|---|---|",
        "| Detect pass (vs GetCurrentBox catalog AABB, diagnostic) | %.1f%% | %s / %s |" % (
            100.0 * summary["detect_pass_rate"],
            summary["n_detect_ok"], summary["n_detect_compared"]),
        "| Detect usable (perception estimate) | %.1f%% | %s / %s |" % (
            100.0 * summary.get("detect_usable_rate", 0.0),
            summary.get("n_detect_usable", 0), summary.get("n", 0)),
        "| Plan pass (4 segments, among detect-usable) | %.1f%% | %s / %s |" % (
            100.0 * summary["plan_pass_rate"],
            summary["n_plan_ok"], summary.get("n_detect_usable",
                                              summary["n_detect_ok"])),
        "| Retreat height (among plan-pass) | %.1f%% | %s / %s |" % (
            100.0 * summary["retreat_pass_rate"],
            summary["n_retreat_ok"], summary["n_plan_ok"]),
        "| End-to-end pick (retreat-ok / N) | %.1f%% | %s / %s |" % (
            100.0 * summary["pick_pass_rate"],
            summary["n_retreat_ok"], summary["n"]),
        "| YOLO ready (post-mask cargo box) | %.1f%% | %s / %s |" % (
            100.0 * summary.get("yolo_ready_rate", 0.0),
            summary.get("n_yolo_ready", 0), summary.get("n", 0)),
        "| Spawn visual = GT (depth blob) | %.1f%% | %s / %s |" % (
            100.0 * summary.get("visual_ready_rate", 0.0),
            summary.get("n_visual_ready", 0), summary.get("n", 0)),
    ]
    if summary.get("n_vac_ran"):
        lines.extend([
            "| Vac attach | %.1f%% | %s / %s |" % (
                100.0 * summary.get("vac_attach_rate", 0.0),
                summary.get("n_vac_attach_ok", 0),
                summary.get("n_vac_ran", 0)),
            "| Vac follow | %.1f%% | %s / %s |" % (
                100.0 * summary.get("vac_follow_rate", 0.0),
                summary.get("n_vac_follow_ok", 0),
                summary.get("n_vac_follow_ran", 0)),
        ])
    lines.extend([
        "",
        "## Geometric pick (attach, no vacuum)",
        "",
        "Attach XY error vs measured box centre; Z vs measured box top.",
        "",
        "| | n | mean | std | p50 | p95 |",
        "|---|---|---|---|---|---|",
    ])
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
    lines.extend([
        "",
        "## Spawn → detect latency (sim time, s)",
        "",
        "Clock starts when `SpawnNextBox` returns. YOLO ready is the first "
        "post-mask-filter cargo box (`raw_cargo`) for that generation. "
        "Spawn visual is when the platform depth blob matches this spawn's "
        "catalog AABB (GetCurrentBox). DetectLuggage is scored against that "
        "same AABB. A leftover previous mesh fails at the visual gate, not "
        "DETECT_GATE. A size/iou miss does "
        "not abort the pick unless `--strict-gt`. "
        "Detect is `DetectLuggage` cloud stamp. `YOLO_NOT_READY`, "
        "`TRACKER_STALE`, `CARGO_NOT_READY`, and `SPAWN_VISUAL_TF` stay "
        "blocking. `SPAWN_VISUAL_MISMATCH` and `DETECT_GATE:*` are "
        "recorded but not blocking by default.",
        "",
        "| | n | mean | std | p50 | p95 |",
        "|---|---|---|---|---|---|",
    ])
    for key, label in (
            ("spawn_to_yolo_sec", "spawn → YOLO box"),
            ("spawn_to_visual_sec", "spawn → visual=GT"),
            ("spawn_to_detect_sec", "spawn → DetectLuggage")):
        slot = summary.get(key) or {}
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
                  "| i | catalog | visual | detect | usable | plan | retreat | "
                  "attach XY | ΔZ | fail |",
                  "|---|---|---|---|---|---|---|---|---|---|"])
    for rec in records:
        usable = (
            "yes" if rec.detect_usable else (
                "—" if rec.detect_usable is None else "no"))
        lines.append(
            "| %02d | %s | %s | %s | %s | %s/%s | %s | %s | %s | %s |" % (
                rec.index, rec.catalog_id or "?", rec.visual_id or "?",
                "ok" if rec.accuracy_ok else (
                    rec.detect_failure or rec.accuracy_reason or "—"),
                usable,
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
        "- `--use-vacuum` (default off) adds PlanningScene `pickup_box` before "
        "BuildMotionSequence, VacuumCommand after attach, and release after "
        "retreat. Geometric pick remains the n50_v2 baseline.",
        "- MoveIt planning scene contains the suitcase only when `--use-vacuum`.",
        "- `keep_camera_down` / `lock_wrist` are not implemented.",
        "- Detect overlay GT is GetCurrentBox catalog AABB at the spawn "
        "origin, not the mesh lid-band or the depth-blob silhouette.",
        "- Visual gate compares the platform depth blob to that AABB. "
        "`SPAWN_VISUAL_MISMATCH` is kept if later DETECT_GATE also fires.",
        "- `pickup_observe` camera at about (-1.0, 0, 1.9), optical +Z down, "
        "centred on the pickup platform so large (0.80 m) fits in the D435 "
        "FOV. Older (-0.8, 0, 1.7) clipped the image left.",
        "- `SPAWN_VISUAL_MISMATCH` and `DETECT_GATE:*` are diagnostic unless "
        "`--strict-gt`. They do not abort the pick.",
        "- `SPAWN_VISUAL_TF` is a camera TF lookup failure during the visual "
        "gate, not a leftover mesh.",
        "- `TRACKER_STALE` means YOLO already had a post-mask cargo box for "
        "this spawn, but the cargo tracker was still on clear / the previous "
        "generation. `CARGO_NOT_READY` is reserved for a matching epoch with "
        "zero cargo points.",
        "",
    ])
    return "\n".join(lines) + "\n"


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--n", "--trials", type=int, default=20, dest="n")
    parser.add_argument("--out", default=os.path.normpath(DEFAULT_OUT))
    parser.add_argument(
        "--dump-dir", default="",
        help="Write color/depth/overlay/mask PNGs per trial (empty = off).")
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
    parser.add_argument(
        "--tol-visual-xy", type=float, default=DEFAULT_VISUAL_TOL_XY,
        help="Spawn-visual vs GT width/depth (m). Wrong catalog class fails.")
    parser.add_argument(
        "--tol-visual-z", type=float, default=DEFAULT_VISUAL_TOL_Z,
        help="Spawn-visual vs GT height (m).")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-graph-check", action="store_true")
    parser.add_argument(
        "--graph-diag-hold-sec", type=float, default=30.0,
        help="[r8 diagnostics] Hold window after a failed graph wait so "
             "an external probe can observe the same moment; 0 "
             "disables. Recording only.")
    parser.add_argument(
        "--strict-gt", action="store_true",
        help="Abort on DETECT_GATE and SPAWN_VISUAL_MISMATCH (old behavior).")
    parser.add_argument(
        "--use-vacuum", action="store_true",
        help="Add pickup_box to PlanningScene and call /vacuum/command.")
    parser.add_argument("--auth-max-attempts", type=int,
                        default=DEFAULT_MAX_ATTEMPTS)
    parser.add_argument("--auth-max-elapsed-sec", type=float,
                        default=DEFAULT_MAX_ELAPSED_SEC)
    parser.add_argument("--auth-wait-period-sec", type=float,
                        default=DEFAULT_WAIT_PERIOD_SEC)
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
        "tol_visual_xy": args.tol_visual_xy,
        "tol_visual_z": args.tol_visual_z,
        "strict_gt": bool(getattr(args, "strict_gt", False)),
        "use_vacuum": bool(getattr(args, "use_vacuum", False)),
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
        "dump_dir": args.dump_dir,
        "expected_retreat_dz": args.expected_retreat_dz,
        "retreat_tol": args.retreat_tol,
        "tol_xy": args.tol_xy, "tol_z": args.tol_z,
        "tol_size": args.tol_size, "tol_yaw": args.tol_yaw,
        "tol_iou": args.tol_iou,
        "tol_visual_xy": args.tol_visual_xy,
        "tol_visual_z": args.tol_visual_z,
        "strict_gt": bool(args.strict_gt),
        "use_vacuum": bool(args.use_vacuum),
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
        listener = getattr(node, "_tf_listener", None)
        tf_exec = getattr(listener, "executor", None) if listener else None
        if tf_exec is not None:
            tf_exec.shutdown()
        tf_node = getattr(node, "_tf_node", None)
        node.destroy_node()
        if tf_node is not None:
            tf_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
