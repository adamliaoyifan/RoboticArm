#!/usr/bin/env python3
"""Todo 2 sampling driver: DetectLuggage vs GetCurrentBox, N trials.

One-off evidence collector per docs/plans/closed_loop_detection_gt_gate.md
(sampling script lives here, not in a package, until todo 4's eval driver
absorbs it). Authority for the report is DetectionAccuracy + this driver;
the detector's own evaluation_compare_gt stays off.

Flow per trial: SpawnNextBox -> wait live geometry_ok -> DetectLuggage ->
GetCurrentBox -> DetectionAccuracy.compare -> ClearCurrentBox. The arm is
sent to pickup_observe once before the loop (FJT, M3 not ready); it never
moves during sampling, so the per-trial goto is a no-op and is skipped.

Writes one JSON line per trial to the output file as it goes (Ctrl-C safe),
then a summary JSON next to it.

Failed trials dump stamp-joined color + depth + overlay (+ colorized mask)
under ``--fail-dump-dir``. Join key is the image header stamp: preprocessor
color/depth share primary_stamp; overlay/mask inherit that RGB stamp.
Detect waits until that same stamp is on ``/luggage/semantic/cargo_points``
so the measurement is the same camera frame as the dumped RGB/overlay.
color.png and overlay.png burn raw/overlay/dump stamps, MATCH/MISMATCH,
infer latency, and cargo/detect stamps. overlay.png also keeps the YOLO 2D
boxes and projected 3D OBBs: green = GetCurrentBox (GT), cyan = DetectLuggage.

GT fallback (success=True + GT box) is recorded as a detect failure, never
compared. geometry_ok must come from a new observation (fresh cloud + newer
primary_stamp), not a TRANSIENT_LOCAL latch or the preprocessor's 1 Hz
status timer. The joined triplet must also be newer than that pre-spawn
stamp; a leftover buffer frame is FRAME_NOT_ALIGNED, not a gate result.
"""

import argparse
import collections
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from luggage_msgs.srv import (
    ClearCurrentBox, DetectLuggage, GetCurrentBox, SpawnNextBox,
)

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "src", "luggage_perception"))
from luggage_perception import ros_message_adapters as adapters  # noqa: E402
from luggage_perception.eval.detection_accuracy import (  # noqa: E402
    BoxObservation, DetectionAccuracy,
)
from luggage_perception.eval.detection_gate_sampling import (  # noqa: E402
    FRAME_JOIN_AFTER_SPAWN_SEC,
    FRAME_JOIN_WAIT_SEC,
    FRESH_SEC,
    annotate_overlay_boxes,
    apply_dump_timestamp_banners,
    build_aligned_dump,
    dump_failure_bundle,
    format_trial_line,
    is_gt_fallback,
    is_perception_estimate,
    perception_reason,
    pick_joined_stamp,
    stamp_sec_from_key,
    summarize_trial_records,
    wait_ready,
)
from luggage_perception.detect_overlay import (  # noqa: E402
    rotation_from_quaternion,
)

JOINTS = ["elfin_joint1", "elfin_joint2", "elfin_joint3",
          "elfin_joint4", "elfin_joint5", "elfin_joint6"]
PICKUP_OBSERVE = [1.8806, -1.7736, -1.0491, 4.4439, 1.7721, 1.8473]
POSE_NAME = "pickup_observe"

OUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..",
    "docs", "status", "evidence", "detection_gt_gate")


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
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


def _parse_status(payload):
    if not payload:
        return None
    try:
        data = json.loads(payload)
    except ValueError:
        return None
    return data if isinstance(data, dict) else None


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


def _stamp_key(msg):
    return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))


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


def _parse_json_payload(payload):
    if not payload:
        return None
    if isinstance(payload, dict):
        return payload
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


class GateRun(Node):

    def __init__(self, n_trials, fresh_sec=FRESH_SEC):
        super().__init__(
            "detection_gt_gate_run",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ])
        self._group = ReentrantCallbackGroup()
        self._n = n_trials
        self._fresh_sec = float(fresh_sec)
        self._accuracy = DetectionAccuracy()

        self._diag = {"payload": None}
        self._status = {"payload": None, "recv": None}
        self._seg_stats = {"payload": None}
        self._cloud = {"recv": None}
        self._buffers = {
            "color": _StampBuffer(),
            "depth": _StampBuffer(),
            "overlay": _StampBuffer(),
            "mask": _StampBuffer(),
            "cargo": _StampBuffer(maxlen=20),
        }
        self._trial_snapshot = {}
        self._trial_join = {
            "aligned": False, "stamp_key": None, "cargo_matched": False,
        }
        self._camera_info = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        tqos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, "/luggage_detector/diagnostics_json",
            lambda m: self._diag.__setitem__("payload", m.data), tqos,
            callback_group=self._group)
        self.create_subscription(
            String, "/luggage/preprocessed/status",
            self._on_status, tqos,
            callback_group=self._group)
        self.create_subscription(
            String, "/semantic_segmenter/stats_json",
            lambda m: self._seg_stats.__setitem__("payload", m.data), tqos,
            callback_group=self._group)
        # Volatile sensor QoS: a new cloud means the preprocessor emitted,
        # unlike status which is latched and also republished by a 1 Hz timer.
        # Image dumps need depth>=5 to match preprocessor BEST_EFFORT pubs;
        # depth=1 stalls one stream while others keep moving.
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            PointCloud2, "/luggage/preprocessed/camera/depth/points",
            self._on_cloud, sensor_qos,
            callback_group=self._group)
        for name, topic in (
                ("color", "/luggage/preprocessed/camera/color/image"),
                ("depth", "/luggage/preprocessed/camera/depth/image"),
                ("overlay", "/luggage/semantic/overlay"),
                ("mask", "/luggage/semantic/mask")):
            self.create_subscription(
                Image, topic,
                lambda m, stream=name: self._buffers[stream].push(m),
                image_qos,
                callback_group=self._group)
        self.create_subscription(
            PointCloud2, "/luggage/semantic/cargo_points",
            lambda m: self._buffers["cargo"].push(m),
            image_qos,
            callback_group=self._group)
        self.create_subscription(
            CameraInfo, "/luggage/preprocessed/camera/color/camera_info",
            self._on_camera_info, image_qos,
            callback_group=self._group)

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
        self._fjt = ActionClient(
            self, FollowJointTrajectory,
            "/elfin_arm_controller/follow_joint_trajectory",
            callback_group=self._group)

    def _on_status(self, msg):
        self._status["payload"] = msg.data
        self._status["recv"] = time.time()

    def _on_cloud(self, _msg):
        self._cloud["recv"] = time.time()

    def _on_camera_info(self, msg):
        self._camera_info = msg

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
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                camera_frame, "world", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5))
        except TransformException as exc:
            return None, None, "tf:%s" % exc
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rotation = rotation_from_quaternion([q.x, q.y, q.z, q.w])
        translation = [float(t.x), float(t.y), float(t.z)]
        return (rotation, translation), (fx, fy, cx, cy), None

    def snapshot_frames(self, timeout=FRAME_JOIN_WAIT_SEC, min_stamp_sec=None,
                        require_cargo=False):
        """Wait for one stamp-joined color+depth+overlay triplet, then freeze it.

        Overlay/mask inherit the preprocessor RGB stamp, so a matching
        header.stamp is the same camera frame. *min_stamp_sec* drops leftovers
        from before this trial's spawn. When *require_cargo* is set, alignment
        also needs cargo_points on that same stamp so DetectLuggage cannot
        reuse the previous box's cloud.
        """
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

    def wait_aligned_frames(self, min_stamp_sec,
                            timeout=FRAME_JOIN_AFTER_SPAWN_SEC):
        """Block until a post-spawn RGB/overlay/cargo triplet is frozen."""
        self.snapshot_frames(
            timeout=timeout, min_stamp_sec=min_stamp_sec, require_cargo=True)
        join = self._trial_join or {}
        return bool(join.get("aligned") and join.get("cargo_matched")
                    and join.get("stamp_key") is not None)

    def freeze_exact_stamp(self, stamp_sec, tol=1e-6):
        """Re-freeze the snapshot to the detector's cloud header stamp."""
        if stamp_sec is None:
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

    def wait_detect_frame(self, stamp_sec, timeout=2.0):
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

    def decoded_failure_dump(self):
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
        extras["status"] = _parse_status(self._status["payload"])
        extras["seg_stats"] = _parse_json_payload(self._seg_stats["payload"])
        extras["diag"] = _parse_json_payload(self._diag["payload"])
        for name in ("color", "depth", "overlay", "mask", "cargo"):
            extras[name] = _image_meta(snapshot.get(name))
        return images, extras, arrays

    # -- helpers --------------------------------------------------------

    def call(self, client, request, timeout=20.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout):
            return None
        return future.result()

    def wait_geometry_ok(self, timeout=15.0):
        """Wait for a live stable observation, not a latched last-good status."""
        wait_started = time.time()
        snapshot = _parse_status(self._status["payload"]) or {}
        try:
            stamp0 = float(snapshot.get("primary_stamp") or 0.0)
        except (TypeError, ValueError):
            stamp0 = 0.0
        deadline = wait_started + timeout
        while time.time() < deadline:
            now = time.time()
            if wait_ready(
                    status_data=_parse_status(self._status["payload"]),
                    cloud_recv=self._cloud["recv"],
                    stamp_at_start=stamp0,
                    wait_started=wait_started,
                    now=now,
                    fresh_sec=self._fresh_sec):
                return True
            time.sleep(0.05)
        return False

    def goto_pickup_observe(self, duration=4.0):
        if not self._fjt.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("FJT action server unavailable")
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = list(JOINTS)
        point = JointTrajectoryPoint()
        point.positions = list(PICKUP_OBSERVE)
        point.velocities = [0.0] * 6
        sec = int(duration)
        point.time_from_start = Duration(
            sec=sec, nanosec=int((duration - sec) * 1e9))
        traj.points = [point]
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)
        event = threading.Event()
        future = self._fjt.send_goal_async(goal)
        future.add_done_callback(lambda _f: event.set())
        event.wait(20.0)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError("pickup_observe goal rejected")
        result_event = threading.Event()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _f: result_event.set())
        result_event.wait(duration + 60.0)
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError("pickup_observe status=%s" % result.status)

    # -- one trial ------------------------------------------------------

    def run_trial(self, index):
        record = {"index": index, "t_start": time.time()}
        snapshot = _parse_status(self._status["payload"]) or {}
        try:
            stamp0 = float(snapshot.get("primary_stamp") or 0.0)
        except (TypeError, ValueError):
            stamp0 = 0.0
        spawn = self.call(self._spawn, SpawnNextBox.Request())
        if spawn is None or not spawn.success:
            record["failure"] = "SPAWN_FAILED:%s" % (
                spawn.message if spawn else "timeout")
            self.snapshot_frames(min_stamp_sec=stamp0, require_cargo=True)
            return record
        record["spawn_id"] = spawn.box.id
        record["stamp0"] = stamp0

        if not self.wait_geometry_ok():
            record["failure"] = "GEOMETRY_NOT_STABLE"
            self.snapshot_frames(
                timeout=FRAME_JOIN_AFTER_SPAWN_SEC,
                min_stamp_sec=stamp0, require_cargo=True)
            self.call(self._clear, ClearCurrentBox.Request())
            return record

        if not self.wait_aligned_frames(min_stamp_sec=stamp0):
            record["failure"] = "FRAME_NOT_ALIGNED"
            record["join"] = dict(self._trial_join or {})
            self.call(self._clear, ClearCurrentBox.Request())
            return record

        detect = self.call(self._detect, DetectLuggage.Request(), timeout=30.0)
        current = self.call(self._current, GetCurrentBox.Request())
        self.call(self._clear, ClearCurrentBox.Request())
        diag = _parse_json_payload(self._diag["payload"]) or {}
        detect_stamp = diag.get("cloud_stamp")
        if detect_stamp is not None:
            self.wait_detect_frame(detect_stamp)
        record["join"] = dict(self._trial_join or {})
        record["aligned"] = bool((self._trial_join or {}).get("aligned"))
        record["detect_cloud_stamp"] = detect_stamp

        if detect is None:
            record["failure"] = "DETECT_TIMEOUT"
            return record
        record["detect_message"] = detect.message
        record["diag"] = self._diag["payload"]
        if record.get("detect_cloud_stamp") is None:
            diag = _parse_json_payload(self._diag["payload"]) or {}
            record["detect_cloud_stamp"] = diag.get("cloud_stamp")

        if current is None or not current.success:
            record["failure"] = "GT_UNAVAILABLE"
            return record

        gt = observation_from_detected(current.box)
        record["gt"] = gt.__dict__

        if is_gt_fallback(detect.message, self._diag["payload"]):
            record["detect_failure"] = "DETECT_GT_FALLBACK"
            record["perception_reason"] = perception_reason(
                detect.message, self._diag["payload"])
            record["measured"] = None
            record["result"] = None
            record["t_end"] = time.time()
            return record

        if not detect.success:
            record["detect_failure"] = detect.message
            record["measured"] = None
            record["result"] = None
            record["t_end"] = time.time()
            return record

        if detect.luggage and is_perception_estimate(
                detect.success, detect.message, True, self._diag["payload"]):
            measured = observation_from_detected(detect.luggage[0])
            record["measured"] = measured.__dict__
            result = self._accuracy.compare(measured, gt)
            record["result"] = result.__dict__
        else:
            record["measured"] = None
            record["result"] = None
            record["detect_failure"] = detect.message or "MEASURED_NONE"
        record["t_end"] = time.time()
        return record


def load_trial_records(trials_path):
    records = []
    with open(trials_path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def write_summary(summary_path, config, records):
    summary = summarize_trial_records(records)
    if "segmenter_backend" in config:
        summary["segmenter_backend"] = config["segmenter_backend"]
    payload = dict(config)
    payload["summary"] = summary
    with open(summary_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
    return summary


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "n_trials", nargs="?", type=int, default=20,
        help="Number of spawn/detect trials (default 20)")
    parser.add_argument(
        "--summarize-only", action="store_true",
        help="Rebuild summary.json from an existing trials.jsonl; no ROS")
    parser.add_argument(
        "--trials", default=os.path.join(OUT_DIR, "trials.jsonl"),
        help="JSONL path for --summarize-only or live output")
    parser.add_argument(
        "--fresh-sec", type=float, default=FRESH_SEC,
        help="Max age of a new cloud/status observation (seconds)")
    parser.add_argument(
        "--fail-dump-dir",
        default=os.path.join(OUT_DIR, "failures"),
        help="Folder for failed-trial stamp-joined color/depth/overlay + trial.json")
    parser.add_argument(
        "--no-fail-dump", action="store_true",
        help="Do not write per-failure frame dumps")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    os.makedirs(OUT_DIR, exist_ok=True)
    trials_path = args.trials
    summary_path = os.path.join(os.path.dirname(trials_path), "summary.json")

    if args.summarize_only:
        records = load_trial_records(trials_path)
        config = {
            "pose": POSE_NAME,
            "n_trials": len(records),
            "note": "summarize-only; records may predate driver fixes",
        }
        summary = write_summary(summary_path, config, records)
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
        return 0

    rclpy.init()
    node = GateRun(args.n_trials, fresh_sec=args.fresh_sec)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    config = {
        "pose": POSE_NAME,
        "n_trials": args.n_trials,
        "fresh_sec": args.fresh_sec,
        "gates": {
            "tol_xy": 0.03, "tol_z": 0.02, "tol_size": 0.05,
            "tol_yaw": 0.15, "tol_iou": 0.60, "min_aspect": 1.15,
        },
        "pass_rate_target": 0.90,
        "t_start": time.time(),
        "fail_dump_dir": None if args.no_fail_dump else args.fail_dump_dir,
    }
    # Record the live backend so a silent stub fallback cannot fake the run.
    stats_deadline = time.time() + 5.0
    while time.time() < stats_deadline and not node._seg_stats["payload"]:
        time.sleep(0.1)
    if node._seg_stats["payload"]:
        try:
            config["segmenter_backend"] = json.loads(
                node._seg_stats["payload"]).get("backend")
        except ValueError:
            config["segmenter_backend"] = "unparseable"
    else:
        config["segmenter_backend"] = "NO_STATS (segmenter not running?)"
    with open(summary_path, "w", encoding="utf-8") as handle:
        json.dump(config, handle, indent=2, sort_keys=True)

    try:
        node.goto_pickup_observe()
        dump_root = None if args.no_fail_dump else args.fail_dump_dir
        if dump_root:
            os.makedirs(dump_root, exist_ok=True)
        with open(trials_path, "w", encoding="utf-8") as handle:
            for i in range(args.n_trials):
                try:
                    record = node.run_trial(i)
                except Exception as exc:  # noqa: BLE001 - keep sampling
                    record = {"index": i, "failure": "EXC:%s" % exc}
                    node.snapshot_frames()
                handle.write(json.dumps(record, sort_keys=True) + "\n")
                handle.flush()
                line = format_trial_line(i, record)
                if not args.no_fail_dump:
                    images, extras, arrays = node.decoded_failure_dump()
                    extrinsics, intrinsics, proj_err = (
                        node.overlay_projection_inputs())
                    rotation = translation = None
                    if extrinsics is not None:
                        rotation, translation = extrinsics
                    overlay, box_meta = annotate_overlay_boxes(
                        images.get("overlay"),
                        gt=record.get("gt"),
                        measured=record.get("measured"),
                        rotation=rotation,
                        translation=translation,
                        intrinsics=intrinsics)
                    if overlay is not None:
                        images["overlay"] = overlay
                    if proj_err:
                        box_meta["project_error"] = proj_err
                    extras.update(box_meta)
                    dump_stamp = node.ros_now_sec()
                    extras["dump_stamp"] = dump_stamp
                    images, extras = apply_dump_timestamp_banners(
                        images, extras,
                        dump_stamp=dump_stamp,
                        detect_stamp=record.get("detect_cloud_stamp"))
                    dumped = dump_failure_bundle(
                        args.fail_dump_dir, record,
                        images=images, extras=extras, arrays=arrays)
                    if dumped:
                        line = "%s dumped=%s" % (line, dumped)
                print(line, flush=True)
        records = load_trial_records(trials_path)
        summary = write_summary(summary_path, config, records)
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
