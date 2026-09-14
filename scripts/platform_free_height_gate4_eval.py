#!/usr/bin/env python3
"""Gate 4 eval: compare online DetectionFrame vs GetCurrentBox (eval-only).

Does not feed GT into the detector. Writes JSONL + summary under --out.

PF-R4: scoring semantics live in ``luggage_perception.eval.gate4_scoring``
(pure, unit-tested). This file is only the ROS collection harness:

- expected instance identity comes from the eval-side spawn response,
  never from the detector output being tested;
- warmup/settled separation, coverage gating (sizes/XY/yaw/trial counts),
  active-window Hz from observed inter-frame intervals, and the fail-
  closed raw-only negative-control verdict all live in the module;
- evidence records the exact launch parameters and code revision.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
import tf2_ros

from luggage_msgs.msg import DetectionFrame
from luggage_msgs.srv import GetCurrentBox, SpawnNextBox
from luggage_perception import ros_message_adapters as adapters
from luggage_perception.eval import gate4_scoring as scoring
from luggage_perception.eval.detection_gate_sampling import (
    build_aligned_dump,
    pick_joined_stamp,
    project_box_observation,
    stamp_sec_from_key,
)
from luggage_perception.eval.gate4_dump import (
    StampBuffer,
    crop_workspace_xy,
    deproject_labelled_clouds,
    extract_top_ransac,
    model_pose_from_gz,
    parse_gz_pose_info,
    select_dump_stamps,
    trial_folder_name,
    trial_is_failure,
    write_index,
    write_pca_replay_dir,
    write_snapshot_dir,
)
from luggage_perception.eval.sim_texture import (
    CATALOG_SIZES,
    aabb_from_span,
    c1_g6_gate,
    catalog_size_from_box_id,
    classify_trial,
    dump_capture_health,
    edge_strip_accepted,
    model_name_from_spawn_message,
    spawn_is_flip,
    write_dump_manifest,
)
from luggage_perception.sensor_preprocessor import transform_points

WARMUP_FRAMES = 5  # support-stability window after an instance change
GAP_SEC = 2.0      # orchestration gap threshold for active-window Hz
# Eval-only YOLO floor for SIM_TEXTURE_LOW_CONFIDENCE proof. Production
# predict(conf=0.2) never emits the 0.07-0.11 GT matches.
EVAL_YOLO_CONF = 0.01

_PCA_CLOUD_KEYS = (
    "cargo_world",
    "mask_cargo_world",
    "depth_all_workspace",
    "filter_obstacle_world",
)


def _pca_replay_clouds(clouds):
    out = {}
    for key in _PCA_CLOUD_KEYS:
        payload = (clouds or {}).get(key)
        if not payload:
            continue
        out[key] = payload.get("points")
    return out


def _stamp_sec(header):
    return float(header.stamp.sec) + 1e-9 * float(header.stamp.nanosec)


def _yaw_from_quat(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))


def _stamp_key(msg):
    return (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))


def _decode_dump_image(msg):
    if msg is None:
        return None
    arr = adapters.image_array_from_msg(msg)
    if arr is None:
        return None
    if arr.ndim == 3 and (msg.encoding or "").lower() == "bgr8":
        return arr[:, :, ::-1].copy()
    return np.asarray(arr).copy()


def _decode_depth_m(msg):
    if msg is None:
        return None
    encoding = (msg.encoding or "").lower()
    if encoding in ("32fc1", "32fc"):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)
        return np.asarray(arr, dtype=np.float32).copy()
    millimetres = adapters.depth_array_from_msg(msg)
    if millimetres is None:
        return None
    return np.asarray(millimetres, dtype=np.float32) * 0.001


def _json_payload(msg):
    if msg is None or not getattr(msg, "data", None):
        return None
    try:
        data = json.loads(msg.data)
    except (TypeError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def _camera_info_dict(msg):
    if msg is None:
        return None
    return {
        "frame_id": msg.header.frame_id,
        "stamp": float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec),
        "width": int(msg.width),
        "height": int(msg.height),
        "k": [float(v) for v in msg.k],
        "d": [float(v) for v in msg.d],
        "p": [float(v) for v in msg.p],
    }


def _quat_matrix(tx, ty, tz, qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy), tx],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx), ty],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy), tz],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=np.float64)


class Gate4Eval(Node):
    def __init__(self, dump_enabled=False):
        overrides = []
        if dump_enabled:
            overrides.append(
                Parameter("use_sim_time", Parameter.Type.BOOL, True))
        super().__init__(
            "platform_free_height_gate4_eval",
            parameter_overrides=overrides)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._frames = []
        self._dump_enabled = bool(dump_enabled)
        self.create_subscription(
            DetectionFrame, "/luggage/perception/detection_frame",
            self._on_frame, qos)
        self._get = self.create_client(
            GetCurrentBox, "/pickup_box_spawner/get_current_box")
        self._spawn = self.create_client(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box")
        self._buffers = {}
        self._json_latest = {}
        self._camera_info = None
        self._tf_buffer = None
        self._trial_samples = []
        self._eval_yolo = None
        self._eval_yolo_prompts = []
        self._eval_yolo_labels = []
        if self._dump_enabled:
            self._enable_dump_subs()

    def _ensure_eval_yolo(self):
        """Lazy eval-only YOLO-World copy. Never published."""
        if self._eval_yolo is not None:
            return self._eval_yolo
        import yaml
        from ament_index_python.packages import get_package_share_directory
        from luggage_perception.semantic_segmenter import _setup_clip_vendor
        from ultralytics import YOLOWorld

        _setup_clip_vendor()
        share = get_package_share_directory("luggage_perception")
        cfg_path = os.path.join(share, "config", "semantic_segmenter.yaml")
        params = yaml.safe_load(open(cfg_path))["semantic_segmenter"][
            "ros__parameters"]
        prompts = [str(p) for p in (params.get("prompts") or [])]
        labels = [int(v) for v in (params.get("class_mapping_labels") or [])]
        model_name = os.path.join(share, "models", "yolov8s-world.pt")
        if not os.path.isfile(model_name):
            model_name = str(params.get("model_name") or "yolov8s-world.pt")
        model = YOLOWorld(model_name)
        if prompts:
            model.set_classes(prompts)
        self._eval_yolo = model
        self._eval_yolo_prompts = prompts
        self._eval_yolo_labels = labels
        return model

    def _eval_low_conf_proposals(self, rgb):
        """Cargo proposals at EVAL_YOLO_CONF. Empty on any load/predict miss."""
        if rgb is None:
            return []
        try:
            model = self._ensure_eval_yolo()
            results = model.predict(
                np.asarray(rgb), conf=EVAL_YOLO_CONF, device="cuda",
                verbose=False)
        except Exception as exc:  # noqa: BLE001 - waiver proof, never crash
            self.get_logger().warning("eval low-conf YOLO failed: %s" % exc)
            return []
        if not results:
            return []
        result = results[0]
        boxes = getattr(result.boxes, "xyxy", None)
        classes = getattr(result.boxes, "cls", None)
        confs = getattr(result.boxes, "conf", None)
        if boxes is None or classes is None or len(boxes) == 0:
            return []
        boxes = boxes.cpu().numpy()
        classes = classes.cpu().numpy()
        confs = (
            confs.cpu().numpy() if confs is not None
            else np.zeros(len(boxes)))
        height, width = np.asarray(rgb).shape[:2]
        prompts = self._eval_yolo_prompts
        labels = self._eval_yolo_labels
        out = []
        for idx in range(len(boxes)):
            cls_idx = int(classes[idx])
            prompt = prompts[cls_idx] if 0 <= cls_idx < len(prompts) else ""
            label = labels[cls_idx] if 0 <= cls_idx < len(labels) else 2
            if int(label) != 2:
                continue
            x1, y1, x2, y2 = boxes[idx]
            bbox = [
                max(0, int(round(x1))), max(0, int(round(y1))),
                min(width, int(round(x2))), min(height, int(round(y2))),
            ]
            if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
                continue
            out.append({
                "label": 2,
                "prompt": prompt,
                "confidence": float(confs[idx]),
                "bbox": bbox,
                "accepted": False,
                "accept_reason": "eval_low_conf",
                "eval_only": True,
            })
        return out

    def _on_frame(self, msg):
        self._frames.append((time.monotonic(), msg))

    def _enable_dump_subs(self):
        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        latch_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._buffers = {
            "color": StampBuffer(maxlen=80),
            "depth": StampBuffer(maxlen=80),
            "overlay": StampBuffer(maxlen=80),
            "mask": StampBuffer(maxlen=80),
            "instance_mask": StampBuffer(maxlen=40),
            "cargo": StampBuffer(maxlen=40),
            "obstacle": StampBuffer(maxlen=40),
            "depth_pts": StampBuffer(maxlen=40),
        }
        self._trial_samples = []
        for name, topic in (
                ("color", "/luggage/preprocessed/camera/color/image"),
                ("depth", "/luggage/preprocessed/camera/depth/image"),
                ("overlay", "/luggage/semantic/overlay"),
                ("mask", "/luggage/semantic/mask"),
                ("instance_mask", "/luggage/semantic/instance_mask")):
            self.create_subscription(
                Image, topic,
                lambda m, stream=name: self._buffers[stream].push(m),
                image_qos)
        for name, topic in (
                ("cargo", "/luggage/semantic/cargo_points"),
                ("obstacle", "/luggage/semantic/obstacle_points"),
                ("depth_pts", "/luggage/preprocessed/camera/depth/points")):
            self.create_subscription(
                PointCloud2, topic,
                lambda m, stream=name: self._buffers[stream].push(m),
                image_qos)
        self.create_subscription(
            CameraInfo, "/luggage/preprocessed/camera/color/camera_info",
            self._on_camera_info, image_qos)
        for key, topic in (
                ("stream_stats", "/luggage_detector/stream_stats_json"),
                ("diagnostics", "/luggage_detector/diagnostics_json"),
                ("seg_stats", "/semantic_segmenter/stats_json"),
                ("filter_stats", "/semantic_point_filter/stats_json"),
                ("prep_status", "/luggage/preprocessed/status"),
                ("current_box", "/luggage/current_box")):
            self.create_subscription(
                String, topic,
                lambda m, name=key: self._json_latest.__setitem__(
                    name, _json_payload(m)),
                latch_qos)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    def _on_camera_info(self, msg):
        self._camera_info = msg

    def _call(self, client, req, timeout=20.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                return None
        return fut.result()

    def spawn_next(self):
        return self._call(self._spawn, SpawnNextBox.Request(), timeout=60.0)

    def get_gt(self):
        return self._call(self._get, GetCurrentBox.Request())

    def collect(self, duration_sec, record_dump_samples=False,
                reset_samples=True):
        t_end = time.monotonic() + duration_sec
        start_n = len(self._frames)
        if record_dump_samples and reset_samples:
            self._trial_samples = []
            next_sample = time.monotonic()
        elif record_dump_samples:
            next_sample = time.monotonic()
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if record_dump_samples and time.monotonic() >= next_sample:
                captured = self._capture_decoded_sample()
                if captured is not None:
                    self._trial_samples.append(captured)
                next_sample += 0.5
        return list(self._frames[start_n:])

    def capture_gz_pose(self, world_name, model_name):
        topic = "/world/%s/pose/info" % world_name
        try:
            proc = subprocess.run(
                ["ign", "topic", "-e", "-t", topic, "--num", "1"],
                capture_output=True, text=True, timeout=3.0, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return "", {"error": str(exc), "model": None}
        raw = proc.stdout or ""
        parsed = parse_gz_pose_info(raw)
        return raw, {
            "model_name": model_name,
            "model": model_pose_from_gz(parsed, model_name),
            "n_entities": len(parsed),
        }

    def _tf_to_world(self, points, frame_id):
        if self._tf_buffer is None or points is None or not frame_id:
            return None, "tf unavailable"
        pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                "world", str(frame_id), Time(),
                timeout=Duration(seconds=0.25))
        except Exception as exc:  # noqa: BLE001 - dump must not crash scoring
            return None, str(exc)
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        matrix = _quat_matrix(
            float(t.x), float(t.y), float(t.z),
            float(r.x), float(r.y), float(r.z), float(r.w))
        return transform_points(pts, matrix), None

    def _world_to_optical(self, optical_frame):
        """Return (R, t) mapping world points into the optical frame."""
        if self._tf_buffer is None or not optical_frame:
            return None, None, "tf unavailable"
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                str(optical_frame), "world", Time(),
                timeout=Duration(seconds=0.25))
        except Exception as exc:  # noqa: BLE001 - dump must not crash scoring
            return None, None, str(exc)
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        matrix = _quat_matrix(
            float(t.x), float(t.y), float(t.z),
            float(r.x), float(r.y), float(r.z), float(r.w))
        return matrix[:3, :3], matrix[:3, 3], None

    def _nearest_msg(self, snaps, name, key):
        items = snaps.get(name) or {}
        if not items:
            return None
        if key in items:
            return items[key]
        return items[max(items)]

    def _workspace_pair(self):
        return getattr(self, "_workspace", ([-1.0, 0.0], [0.5, 0.5]))

    def _put_cloud(self, clouds, extras, name, points, frame_id):
        if points is None:
            points = np.zeros((0, 3), dtype=np.float64)
        clouds[name] = {"points": points, "frame_id": str(frame_id or "")}
        world, err = self._tf_to_world(points, frame_id)
        extras.setdefault("tf_errors", {})[name] = err
        if world is None:
            return
        clouds[name + "_world"] = {"points": world, "frame_id": "world"}
        center, half = self._workspace_pair()
        clouds[name + "_workspace"] = {
            "points": crop_workspace_xy(world, center, half),
            "frame_id": "world",
        }

    def _snapshot_from_key(self, key, snaps=None):
        if snaps is None:
            snaps = {name: buf.snapshot() for name, buf in self._buffers.items()}
        color = _decode_dump_image((snaps.get("color") or {}).get(key))
        overlay = _decode_dump_image(self._nearest_msg(snaps, "overlay", key))
        depth_m = _decode_depth_m((snaps.get("depth") or {}).get(key))
        mask_msg = self._nearest_msg(snaps, "mask", key)
        mask_labels = None
        if mask_msg is not None:
            labels = adapters.image_array_from_msg(mask_msg)
            if labels is not None:
                mask_labels = labels[:, :, 0] if labels.ndim == 3 else labels
                mask_labels = np.asarray(mask_labels).copy()
        images, arrays, extras = build_aligned_dump(
            color, depth_m, overlay, mask_labels)
        inst_msg = self._nearest_msg(snaps, "instance_mask", key)
        if inst_msg is not None:
            inst = adapters.image_array_from_msg(inst_msg)
            if inst is not None:
                inst = inst[:, :, 0] if inst.ndim == 3 else inst
                images["instance_mask"] = np.asarray(inst).copy()
                arrays["instance_mask"] = np.asarray(inst).copy()
        cargo_msg = (snaps.get("cargo") or {}).get(key)
        cargo_matched = cargo_msg is not None
        if cargo_msg is None:
            cargo_msg = self._nearest_msg(snaps, "cargo", key)
        cargo_pts = (
            adapters.cloud_points_from_msg(cargo_msg)
            if cargo_msg is not None else None)
        cargo_frame = (
            cargo_msg.header.frame_id if cargo_msg is not None else "")
        extras["join_stamp"] = stamp_sec_from_key(key)
        extras["join_stamp_key"] = list(key)
        extras["camera_info"] = _camera_info_dict(self._camera_info)
        extras["stream_stats"] = self._json_latest.get("stream_stats")
        extras["diagnostics"] = self._json_latest.get("diagnostics")
        extras["seg_stats"] = self._json_latest.get("seg_stats")
        extras["filter_stats"] = self._json_latest.get("filter_stats")
        extras["prep_status"] = self._json_latest.get("prep_status")
        extras["current_box"] = self._json_latest.get("current_box")
        extras["color_frame_id"] = (
            (snaps.get("color") or {}).get(key).header.frame_id
            if (snaps.get("color") or {}).get(key) is not None else None)
        extras["cargo_frame_id"] = cargo_frame
        extras["cargo_matched"] = cargo_matched
        if cargo_msg is not None and not cargo_matched:
            extras["cargo_fallback_stamp"] = (
                float(cargo_msg.header.stamp.sec)
                + 1e-9 * float(cargo_msg.header.stamp.nanosec))
        clouds = {}
        optical = extras.get("color_frame_id") or cargo_frame or (
            "camera_color_optical_frame")
        self._put_cloud(clouds, extras, "cargo_camera", cargo_pts, cargo_frame)
        for stream, name in (
                ("obstacle", "filter_obstacle"),
                ("depth_pts", "depth_topic")):
            msg = self._nearest_msg(snaps, stream, key)
            pts = (
                adapters.cloud_points_from_msg(msg)
                if msg is not None else None)
            frame = msg.header.frame_id if msg is not None else optical
            extras["%s_matched" % stream] = bool(
                msg is not None and _stamp_key(msg) == key)
            self._put_cloud(clouds, extras, name, pts, frame)
        rebuilt = deproject_labelled_clouds(
            (arrays or {}).get("depth"),
            (arrays or {}).get("mask_labels"),
            extras.get("camera_info"),
            stride=2)
        extras["mask_hist"] = rebuilt.get("mask_hist")
        extras["deproject"] = rebuilt.get("stats")
        for name, pts in (rebuilt.get("clouds") or {}).items():
            self._put_cloud(clouds, extras, name, pts, optical)
        extras["tf_error"] = (extras.get("tf_errors") or {}).get("cargo_camera")
        return images, arrays, extras, clouds

    def _capture_decoded_sample(self):
        if not self._buffers:
            return None
        snaps = {name: buf.snapshot() for name, buf in self._buffers.items()}
        key = pick_joined_stamp(snaps, required=("color", "depth", "overlay"))
        if key is None:
            key = pick_joined_stamp(snaps, required=("color", "depth"))
        if key is None:
            color_keys = snaps.get("color") or {}
            if not color_keys:
                return None
            key = max(color_keys)
        images, arrays, extras, clouds = self._snapshot_from_key(key, snaps)
        extras["captured_monotonic"] = time.monotonic()
        return {
            "images": images, "arrays": arrays, "extras": extras,
            "clouds": clouds, "stamp_key": key,
        }

    def write_trial_dump(self, dump_root, trial, recovery, box_id, rows,
                         min_stamp_sec, workspace, world_name,
                         dump_pass_trials=True, eval_low_conf=False):
        if not self._dump_enabled or not dump_root:
            return None
        failed = trial_is_failure(recovery, rows)
        if not failed and not dump_pass_trials:
            return None
        folder = trial_folder_name(trial, recovery, box_id, rows)
        dest = Path(dump_root) / folder
        dest.mkdir(parents=True, exist_ok=True)
        samples = list(self._trial_samples or [])
        snapshot_dirs = []
        last_reason = ""
        last_n_cargo = None
        if samples:
            idxs = select_dump_stamps(
                list(range(len(samples))), count=3)
            labels = (
                ("early", "mid", "late") if len(idxs) >= 3
                else (("late",) if len(idxs) == 1
                      else ("early", "late")[:len(idxs)]))
            eval_dets = []
            if eval_low_conf:
                late = samples[idxs[-1]]
                late_stats = ((late.get("extras") or {}).get("seg_stats") or {})
                if int(late_stats.get("accepted_cargo_count") or 0) == 0:
                    eval_dets = self._eval_low_conf_proposals(
                        (late.get("images") or {}).get("color"))
                    late.setdefault("extras", {})[
                        "eval_low_conf_detections"] = eval_dets
            for label, idx in zip(labels, idxs):
                sample = samples[idx]
                extras = dict(sample.get("extras") or {})
                extras["trial"] = trial
                extras["box_id"] = box_id
                extras["failed"] = failed
                extras["recovery"] = recovery
                extras["workspace_center_xy"] = list(workspace[0])
                extras["workspace_half_extents"] = list(workspace[1])
                extras["scoring_tail"] = (rows or [])[-1] if rows else None
                extras["sample_index"] = idx
                extras["n_samples"] = len(samples)
                if extras.get("scoring_tail"):
                    last_reason = extras["scoring_tail"].get("pca_reason") or last_reason
                    last_n_cargo = extras["scoring_tail"].get("n_cargo_points")
                stats = extras.get("stream_stats") or {}
                if last_n_cargo is None:
                    last_n_cargo = stats.get("n_cargo_points")
                    last_reason = stats.get("pca_reason") or last_reason
                extras["pca_replay"] = write_pca_replay_dir(
                    str(dest / label / "pca_replay"),
                    _pca_replay_clouds(sample.get("clouds")),
                    workspace=workspace)
                if eval_dets:
                    extras["eval_low_conf_detections"] = eval_dets
                write_snapshot_dir(
                    str(dest / label),
                    images=sample.get("images"),
                    arrays=sample.get("arrays"),
                    extras=extras,
                    clouds=sample.get("clouds"))
                snapshot_dirs.append(label)
        else:
            snaps = {name: buf.snapshot() for name, buf in self._buffers.items()}
            key = pick_joined_stamp(
                snaps, required=("color", "depth", "overlay"),
                min_stamp_sec=min_stamp_sec)
            if key is None:
                key = pick_joined_stamp(
                    snaps, required=("color", "depth"),
                    min_stamp_sec=min_stamp_sec)
            keys = [key] if key is not None else []
            labels = (
                ("early", "mid", "late") if len(keys) >= 3
                else (("late",) if len(keys) == 1
                      else ("early", "late")[:len(keys)]))
            for label, stamp_key in zip(labels, keys):
                images, arrays, extras, clouds = self._snapshot_from_key(
                    stamp_key)
                extras["trial"] = trial
                extras["box_id"] = box_id
                extras["failed"] = failed
                extras["recovery"] = recovery
                extras["workspace_center_xy"] = list(workspace[0])
                extras["workspace_half_extents"] = list(workspace[1])
                extras["scoring_tail"] = (rows or [])[-1] if rows else None
                if extras.get("scoring_tail"):
                    last_reason = extras["scoring_tail"].get("pca_reason") or ""
                    last_n_cargo = extras["scoring_tail"].get("n_cargo_points")
                extras["pca_replay"] = write_pca_replay_dir(
                    str(dest / label / "pca_replay"),
                    _pca_replay_clouds(clouds),
                    workspace=workspace)
                write_snapshot_dir(
                    str(dest / label), images=images, arrays=arrays,
                    extras=extras, clouds=clouds)
                snapshot_dirs.append(label)
        gz_raw, gz_json = self.capture_gz_pose(world_name, box_id)
        (dest / "gz_pose.txt").write_text(gz_raw or "")
        (dest / "gz_pose.json").write_text(
            json.dumps(gz_json, indent=2, default=str) + "\n")
        trial_rec = {
            "trial": trial,
            "box_id": box_id,
            "failed": failed,
            "recovery": recovery,
            "snapshots": snapshot_dirs,
            "pca_reason": last_reason,
            "n_cargo_points": last_n_cargo,
            "gz_pose": gz_json.get("model"),
        }
        (dest / "trial.json").write_text(
            json.dumps(trial_rec, indent=2, default=str) + "\n")
        with (dest / "scores.jsonl").open("w") as handle:
            for row in rows or []:
                handle.write(json.dumps(row) + "\n")
        health = dump_capture_health(str(dest))
        write_dump_manifest(str(dest), {
            "trial": trial,
            "box_id": box_id,
            "failed": failed,
            "recovery": recovery,
            "capture_complete": health.get("capture_complete"),
            "replay_possible": health.get("replay_possible"),
            "missing": health.get("missing"),
            "snapshots": snapshot_dirs,
        })
        return {
            "trial": trial,
            "folder": folder,
            "failed": failed,
            "box_id": box_id,
            "pca_reason": last_reason,
            "n_cargo_points": last_n_cargo,
            "t_first_valid_sec": recovery.get("t_first_valid_sec"),
            "t_first_full3d_sec": recovery.get("t_first_full3d_sec"),
            "snapshots": snapshot_dirs,
            "dump_dir": str(dest),
            "dump_health": health,
        }


def _row(frame, gt, trial, box_id, monotonic_sec=None):
    """One DetectionFrame -> scoring row (estimate + reference + errors)."""
    box = frame.box
    gt_box = gt.box if gt is not None and gt.success else None
    top_z = (float(box.top_surface_pose.position.z)
             if box.top_surface_valid else None)
    gt_top = gt_support = None
    if gt_box is not None:
        gt_top = float(gt_box.pose.position.z) + 0.5 * float(gt_box.height)
        gt_support = (float(gt_box.pose.position.z)
                      - 0.5 * float(gt_box.height))
    full = bool(box.height_valid) and int(frame.geometry_level) == 1
    support_z = float(frame.support_z) if full else None

    def _err(est, ref):
        return None if est is None or ref is None else abs(est - ref)

    return {
        "trial": trial,
        "box_id": box_id,
        "instance_id": str(frame.instance_id),
        "generation": int(frame.generation),
        "stamp_sec": _stamp_sec(frame.header),
        "frame_id": frame.header.frame_id,
        "pca_valid": bool(frame.pca_valid),
        "pca_reason": str(frame.pca_reason),
        "pca_source": str(frame.pca_source),
        "n_cargo_points": int(frame.n_cargo_points),
        "geometry_level": int(frame.geometry_level),
        "support_valid": bool(frame.support_valid),
        "support_reason": str(frame.support_reason),
        "support_z": support_z,
        "top_surface_valid": bool(box.top_surface_valid),
        "height_valid": bool(box.height_valid),
        "height_source": int(box.height_source),
        "est_top_z": top_z,
        "est_height": float(box.height),
        "est_width": float(box.width),
        "est_depth": float(box.depth),
        "est_xy": [float(box.pose.position.x), float(box.pose.position.y)],
        "gt_top_z": gt_top,
        "gt_support_z": gt_support,
        "gt_height": None if gt_box is None else float(gt_box.height),
        "gt_width": None if gt_box is None else float(gt_box.width),
        "gt_depth": None if gt_box is None else float(gt_box.depth),
        "gt_xy": (None if gt_box is None else
                  [float(gt_box.pose.position.x),
                   float(gt_box.pose.position.y)]),
        "err_top_m": _err(top_z, gt_top),
        "err_support_m": _err(support_z, gt_support),
        "err_height_m": (
            _err(float(box.height), float(gt_box.height))
            if full and gt_box is not None else None),
        "err_xy_m": (
            None if gt_box is None else math.hypot(
                float(box.pose.position.x) - float(gt_box.pose.position.x),
                float(box.pose.position.y) - float(gt_box.pose.position.y))),
        "err_width_m": (
            None if gt_box is None else abs(
                float(box.width) - float(gt_box.width))),
        "err_depth_m": (
            None if gt_box is None else abs(
                float(box.depth) - float(gt_box.depth))),
        "false_measured_height": bool(
            box.height_valid and int(frame.geometry_level) == 1
            and not frame.support_valid),
        "monotonic_sec": monotonic_sec,
    }


def _gt_fields(gt):
    if gt is None or not getattr(gt, "success", False):
        return None
    box = gt.box
    return {
        "gt_top_z": float(box.pose.position.z) + 0.5 * float(box.height),
        "gt_xy": [float(box.pose.position.x), float(box.pose.position.y)],
        "gt_width": float(box.width),
        "gt_depth": float(box.depth),
        "gt_height": float(box.height),
        "obs": {
            "x": float(box.pose.position.x),
            "y": float(box.pose.position.y),
            "z": float(box.pose.position.z),
            "yaw": _yaw_from_quat(box.pose.orientation),
            "width": float(box.width),
            "depth": float(box.depth),
            "height": float(box.height),
        },
    }


def _detections_from_sample(sample):
    extras = (sample or {}).get("extras") or {}
    stats = extras.get("seg_stats") or {}
    dets = list(stats.get("detections") or [])
    if not dets:
        dets = list(extras.get("detections") or [])
    dets.extend(extras.get("eval_low_conf_detections") or [])
    return dets


def _merge_sample_detections(samples):
    """Union of live + eval-only boxes across early/mid/late snapshots."""
    out = []
    seen = set()
    for sample in samples or []:
        for det in _detections_from_sample(sample):
            bbox = tuple(int(v) for v in (det.get("bbox") or [])[:4])
            key = (
                bbox,
                round(float(det.get("confidence") or 0.0), 4),
                int(det.get("label") or -1),
            )
            if key in seen:
                continue
            seen.add(key)
            out.append(det)
    return out


def _intrinsics_xy(camera_info):
    if not camera_info:
        return None
    k = camera_info.get("k") if isinstance(camera_info, dict) else None
    if k is None or len(k) < 6:
        return None
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if min(fx, fy) <= 1e-9:
        return None
    return (fx, fy, cx, cy)


def _write_exclude_generations(path, generations):
    if not path:
        return
    dest = Path(path)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(sorted(set(int(g) for g in generations))) + "\n")


def _classify_trial_inputs(node, gt, recovery, settled, dumped, spawn_message,
                           workspace):
    samples = list(node._trial_samples or [])
    sample = samples[-1] if samples else None
    extras = (sample or {}).get("extras") or {}
    detections = _merge_sample_detections(samples)
    current = extras.get("current_box") or {}
    if isinstance(current, str):
        try:
            current = json.loads(current)
        except (TypeError, ValueError):
            current = {}
    gt_info = _gt_fields(gt)
    gt_bbox = None
    optical = extras.get("color_frame_id") or "camera_color_optical_frame"
    rotation, translation, _tf_err = node._world_to_optical(optical)
    intr = _intrinsics_xy(extras.get("camera_info"))
    if gt_info and rotation is not None and intr is not None:
        proj = project_box_observation(
            gt_info["obs"], rotation, translation, intr)
        if proj is not None:
            gt_bbox = aabb_from_span(proj.get("span"))
    clouds = (sample or {}).get("clouds") or {}
    depth_ws = ((clouds.get("depth_all_workspace") or clouds.get(
        "depth_all_world") or {}).get("points"))
    ransac = None
    if depth_ws is not None:
        ransac = extract_top_ransac(depth_ws, workspace=workspace)
        ransac = {k: v for k, v in ransac.items()
                  if k not in ("inliers", "outliers", "voxel")}
    n_cargo = 0
    for row in settled or []:
        try:
            n_cargo = max(n_cargo, int(row.get("n_cargo_points") or 0))
        except (TypeError, ValueError):
            pass
    if dumped and dumped.get("n_cargo_points"):
        try:
            n_cargo = max(n_cargo, int(dumped.get("n_cargo_points") or 0))
        except (TypeError, ValueError):
            pass
    n_accepted = sum(
        1 for det in detections
        if det.get("accepted") and int(det.get("label", -1)) == 2)
    generation = 0
    try:
        generation = int(current.get("generation") or 0)
    except (TypeError, ValueError):
        generation = 0
    health = (dumped or {}).get("dump_health") or {}
    if dumped and dumped.get("dump_dir") and not health:
        health = dump_capture_health(dumped.get("dump_dir"))
    return {
        "visual_kind": str(current.get("visual_kind") or "mesh"),
        "gt_bbox": gt_bbox,
        "image_wh": (640, 480),
        "detections": detections,
        "raw_depth_ransac": ransac,
        "gt": gt_info,
        "n_accepted_cargo": n_accepted,
        "n_cargo_points": n_cargo,
        "dump_health": health,
        "recovery": recovery,
        "settled": settled,
        "spawn_message": spawn_message,
        "generation": generation,
        "edge_fp_accepted": bool(edge_strip_accepted(detections)),
    }


def _revision_info():
    """Reproducible identity of the code under test (evidence contract)."""
    info = {}
    try:
        info["git_commit"] = subprocess.run(
            ["git", "rev-parse", "HEAD"], capture_output=True, text=True,
            timeout=5, check=True).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"], capture_output=True,
            text=True, timeout=5, check=True).stdout.splitlines()
        info["git_dirty_files"] = len([l for l in dirty if l.strip()])
    except Exception:  # noqa: BLE001 - evidence must record, never crash
        info["git_commit"] = None
    return info


def _matrix_coverage(trials_gt):
    """Deterministic-pose coverage from the eval-side GT record.

    Sizes, XY offsets, yaw values, and per-size trial counts. Placement
    variation comes from the spawner/eval side only; this reports what
    was actually covered so thin matrices fail the coverage gate.

    The size matrix counts CATALOG tiers, not observable dimensions: the
    spawner randomly picks one of two suitcase meshes per tier, so the
    observable GT size splits into two variants per tier and would
    wrongly fail the per-size coverage rule.
    """
    from luggage_description.box_catalog_utils import (
        box_catalog_entries, load_box_catalog)
    from luggage_description.scene_tf_config_utils import (
        load_scene_tf_config, resolve_scene_tf_config_path)
    try:
        tiers = [tuple(float(v) for v in e["size"])
                 for e in box_catalog_entries(load_box_catalog(
                     scene_config=load_scene_tf_config(
                         resolve_scene_tf_config_path())))]
    except Exception:  # noqa: BLE001 - catalog is advisory here
        tiers = []

    def _tier_of(w, d, h):
        if not tiers:
            return (round(w, 3), round(d, 3), round(h, 3))
        return min(tiers, key=lambda t: (
            abs(t[0] - w) + abs(t[1] - d) + abs(t[2] - h)))

    sizes = {}
    offsets = set()
    yaws = set()
    for g in trials_gt:
        if not g or g.get("gt_width") is None:
            continue
        key = _tier_of(g["gt_width"], g["gt_depth"], g["gt_height"])
        sizes[key] = sizes.get(key, 0) + 1
        offsets.add((round(g["gt_xy"][0], 2), round(g["gt_xy"][1], 2)))
        yaws.add(round(math.degrees(g["gt_yaw"]), 0))
    return {
        "sizes": [list(k) for k in sorted(sizes)],
        "n_sizes": len(sizes),
        "trials_per_size": {"x".join(str(v) for v in k): n
                            for k, n in sorted(sizes.items())},
        "xy_offsets": [list(o) for o in sorted(offsets)],
        "n_xy_offsets": len(offsets),
        "yaw_deg": sorted(yaws),
        "n_yaws": len(yaws),
        "n_trials": sum(sizes.values()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--settle-sec", type=float, default=4.0)
    parser.add_argument("--warmup-frames", type=int, default=WARMUP_FRAMES)
    parser.add_argument("--gap-sec", type=float, default=GAP_SEC)
    parser.add_argument(
        "--negative-control-raw-only", action="store_true",
        help="fail-closed control: every frame must be invalid with "
             "DETECT_CARGO_SEGMENTATION_REQUIRED and zero valid outputs")
    parser.add_argument("--min-sizes", type=int, default=3)
    parser.add_argument("--min-xy-offsets", type=int, default=3)
    parser.add_argument("--min-yaws", type=int, default=3)
    parser.add_argument("--min-trials-per-size", type=int, default=10)
    parser.add_argument("--launch-params", default="",
                        help="exact launch argument string, recorded as-is")
    parser.add_argument(
        "--dump-dir", default="",
        help="eval-only: write RGB/depth/mask/cloud dumps per trial; "
             "failed trials also write reconstructed depth/mask clouds "
             "and pca_replay RANSAC/PCA intermediates. Empty defaults to "
             "<out>/dumps")
    parser.add_argument(
        "--no-dump", action="store_true",
        help="skip camera/cloud dumps even when --out is set")
    parser.add_argument("--world-name", default="airport_loading")
    parser.add_argument(
        "--workspace-center", nargs=2, type=float, default=[-1.0, 0.0],
        metavar=("X", "Y"))
    parser.add_argument(
        "--workspace-half", nargs=2, type=float, default=[0.5, 0.5],
        metavar=("HX", "HY"))
    parser.add_argument(
        "--c1-g6", action="store_true",
        help="PF-R10 generation-6 trial-level C1: 4/6 normal pass with "
             "size mix, at most two SIM_TEXTURE_LOW_CONFIDENCE waivers, "
             "spawn-flip replacements, and C2 sampling extension")
    parser.add_argument(
        "--c2-extend-trials", type=int, default=-1,
        help="extra unscored trials after C1 so labelled RSS coverage "
             "can reach two occurrences per size. -1 means 6 when "
             "--c1-g6, else 0")
    parser.add_argument(
        "--c2-exclude-file", default="",
        help="JSON list of current_box generations excluded from C2 RSS "
             "fits (texture-waived trials)")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    dump_root = None
    if not args.no_dump:
        dump_root = Path(args.dump_dir) if args.dump_dir else (out / "dumps")
    if dump_root is not None:
        dump_root.mkdir(parents=True, exist_ok=True)
    workspace = (list(args.workspace_center), list(args.workspace_half))

    rclpy.init()
    node = Gate4Eval(dump_enabled=dump_root is not None)
    node._workspace = workspace
    all_rows = []
    all_stamps = []
    trials = []
    trials_gt = []
    stale_frames_total = 0
    t_run_start = time.monotonic()
    spawn_failures = 0
    trial_recoveries = []
    dump_records = []
    classified_trials = []
    c2_exclude = []
    dump_index = 0
    scored_c1 = 0
    c2_extend = (
        args.c2_extend_trials if args.c2_extend_trials >= 0
        else (6 if args.c1_g6 else 0))
    exclude_path = args.c2_exclude_file or (
        str(out / "c2_exclude_generations.json") if args.c1_g6 else "")
    if exclude_path:
        _write_exclude_generations(exclude_path, [])
    try:
        # Let the stream settle before trial 0 (arm/observe/camera
        # warmup after launch): a first trial started mid-warmup fails
        # all of its frames.
        settle_deadline = time.monotonic() + 20.0
        while time.monotonic() < settle_deadline:
            before = len(node._frames)
            node.collect(1.0)
            if before > 0 and len(node._frames) - before >= 2:
                break

        def execute_one(score_c1):
            nonlocal dump_index, spawn_failures, stale_frames_total
            trial = dump_index
            dump_index += 1
            n0 = len(node._frames)
            spawn = node.spawn_next()
            t_placed = time.monotonic()
            spawn_message = (
                None if spawn is None else getattr(spawn, "message", None))
            if spawn is None or not spawn.success:
                rec = {
                    "trial": trial, "spawn_ok": False,
                    "n_settled": 0,
                    "t_first_valid_sec": None,
                    "t_first_full3d_sec": None,
                }
                box_id = model_name_from_spawn_message(spawn_message)
                if args.c1_g6 and spawn_is_flip(spawn_message):
                    rec["trial_class"] = "infrastructure_invalid"
                    rec["class"] = "infrastructure_invalid"
                else:
                    spawn_failures += 1
                    rec["trial_class"] = "fail"
                all_rows.append({
                    "trial": trial, "spawn_ok": False,
                    "message": spawn_message,
                    "trial_class": rec.get("trial_class"),
                })
                try:
                    dumped = node.write_trial_dump(
                        dump_root, trial, rec, box_id or spawn_message,
                        [], None, workspace, args.world_name)
                except Exception as exc:  # noqa: BLE001
                    node.get_logger().error("trial dump failed: %s" % exc)
                    dumped = None
                if dumped:
                    dumped["trial_class"] = rec.get("trial_class")
                    dump_records.append(dumped)
                payload = {
                    "trial": trial,
                    "box_id": box_id,
                    "size": catalog_size_from_box_id(box_id),
                    "trial_class": rec.get("trial_class"),
                    "reasons": (
                        ["spawn_flip"] if rec.get("trial_class")
                        == "infrastructure_invalid" else ["spawn_fail"]),
                    "recovery": rec,
                    "warmup": [],
                    "settled": [],
                    "generation": 0,
                    "score_c1": bool(score_c1),
                }
                if rec.get("trial_class") != "infrastructure_invalid":
                    trial_recoveries.append(rec)
                return payload
            node.collect(0.5, record_dump_samples=node._dump_enabled,
                         reset_samples=True)
            gt = node.get_gt()
            expected_instance = (
                spawn.box.id or
                (gt.box.id if gt and gt.success else None))
            samples = node.collect(
                args.settle_sec,
                record_dump_samples=node._dump_enabled,
                reset_samples=False)
            stamps = [_stamp_sec(fr.header) for _t, fr in samples]
            if score_c1:
                all_stamps.extend(stamps)
            rows = [
                _row(fr, gt, trial, expected_instance or "unknown",
                     monotonic_sec=t)
                for t, fr in samples]
            all_rows.extend(rows)
            owned, stale_n = scoring.filter_expected_instance(
                rows, expected_instance)
            if score_c1:
                stale_frames_total += stale_n
            warmup, settled = scoring.split_warmup(
                owned, warmup_frames=args.warmup_frames)
            recovery_rows = []
            for t, fr in node._frames[n0:]:
                recovery_rows.append({
                    "instance_id": str(fr.instance_id),
                    "top_surface_valid": bool(fr.box.top_surface_valid),
                    "height_valid": bool(fr.box.height_valid),
                    "geometry_level": int(fr.geometry_level),
                    "height_source": int(fr.box.height_source),
                    "monotonic_sec": t,
                })
            recovery_owned, _stale = scoring.filter_expected_instance(
                recovery_rows, expected_instance)
            t_valid, t_full = scoring.recovery_times(
                recovery_owned, t_placed)
            recovery = {
                "trial": trial, "spawn_ok": True,
                "n_settled": len(settled),
                "t_first_valid_sec": t_valid,
                "t_first_full3d_sec": t_full,
            }
            try:
                dumped = node.write_trial_dump(
                    dump_root, trial, recovery, expected_instance,
                    owned, min(stamps) if stamps else None,
                    workspace, args.world_name,
                    eval_low_conf=bool(score_c1))
            except Exception as exc:  # noqa: BLE001
                node.get_logger().error("trial dump failed: %s" % exc)
                dumped = None
            if dumped:
                dump_records.append(dumped)
            inputs = _classify_trial_inputs(
                node, gt, recovery, settled, dumped, spawn_message,
                workspace)
            if args.c1_g6:
                classified = classify_trial(dict(
                    inputs, box_id=expected_instance,
                    size=catalog_size_from_box_id(expected_instance)))
                recovery["trial_class"] = classified["trial_class"]
                recovery["class"] = classified["trial_class"]
                if dumped is not None:
                    dumped["trial_class"] = classified["trial_class"]
                    dumped["reasons"] = classified.get("reasons")
            else:
                classified = {"trial_class": (
                    "fail" if trial_is_failure(recovery, settled)
                    else "normal_pass"), "reasons": []}
            payload = {
                "trial": trial,
                "box_id": expected_instance,
                "instance_id": expected_instance,
                "size": catalog_size_from_box_id(expected_instance),
                "trial_class": classified.get("trial_class"),
                "reasons": classified.get("reasons") or [],
                "texture_proof": classified.get("texture_proof"),
                "recovery": recovery,
                "warmup": warmup,
                "settled": settled,
                "generation": inputs.get("generation") or 0,
                "score_c1": bool(score_c1),
            }
            if dumped and dumped.get("dump_dir"):
                dest = Path(dumped["dump_dir"])
                dest.mkdir(parents=True, exist_ok=True)
                (dest / "detections.json").write_text(
                    json.dumps(inputs.get("detections") or [], indent=2) + "\n")
                (dest / "iou.json").write_text(json.dumps({
                    "gt_bbox": inputs.get("gt_bbox"),
                    "trial_class": payload["trial_class"],
                    "texture_proof": classified.get("texture_proof"),
                }, indent=2, default=str) + "\n")
            if score_c1:
                trial_recoveries.append(recovery)
                trials.append({
                    "trial": trial, "box_id": expected_instance,
                    "instance_id": expected_instance,
                    "warmup": warmup, "settled": settled,
                    "trial_class": payload["trial_class"],
                })
                trials_gt.append({
                    "gt_width": (float(gt.box.width)
                                 if gt and gt.success else None),
                    "gt_depth": (float(gt.box.depth)
                                 if gt and gt.success else None),
                    "gt_height": (float(gt.box.height)
                                  if gt and gt.success else None),
                    "gt_xy": ([float(gt.box.pose.position.x),
                               float(gt.box.pose.position.y)]
                              if gt and gt.success else None),
                    "gt_yaw": (_yaw_from_quat(gt.box.pose.orientation)
                               if gt and gt.success else None),
                })
            if not samples:
                all_rows.append({
                    "trial": trial, "box_id": expected_instance,
                    "spawn_ok": True,
                    "top_surface_valid": False, "height_valid": False,
                    "message": "no detection_frame during settle window",
                })
            return payload

        max_attempts = args.trials * 3 if args.c1_g6 else args.trials
        attempts = 0
        size_ok = {name: 0 for name in CATALOG_SIZES}

        def _exclude_if_unlabelled(payload):
            if payload.get("trial_class") == "normal_pass":
                size = catalog_size_from_box_id(
                    payload.get("size") or payload.get("box_id"))
                if size in size_ok:
                    size_ok[size] += 1
                return
            gen = int(payload.get("generation") or 0)
            if gen:
                c2_exclude.append(gen)
                _write_exclude_generations(exclude_path, c2_exclude)

        if args.c1_g6:
            # One unscored catalog cycle so scored C1 starts on the next
            # carryon,standard,large pair with a warm YOLO epoch.
            for _prime in range(len(CATALOG_SIZES)):
                _exclude_if_unlabelled(execute_one(score_c1=False))
        while scored_c1 < args.trials and attempts < max_attempts:
            attempts += 1
            payload = execute_one(score_c1=True)
            if args.c1_g6 and payload.get("trial_class") == (
                    "infrastructure_invalid"):
                classified_trials.append(payload)
                continue
            scored_c1 += 1
            classified_trials.append(payload)
            if payload.get("trial_class") == "SIM_TEXTURE_LOW_CONFIDENCE":
                c2_exclude.append(int(payload.get("generation") or 0))
                _write_exclude_generations(exclude_path, c2_exclude)
            elif payload.get("trial_class") == "normal_pass":
                size = catalog_size_from_box_id(
                    payload.get("size") or payload.get("box_id"))
                if size in size_ok:
                    size_ok[size] += 1
        extra = 0
        max_extra = max(c2_extend, 24 if args.c1_g6 else c2_extend)
        while extra < max_extra:
            if extra >= c2_extend and all(
                    size_ok[name] >= 2 for name in CATALOG_SIZES):
                break
            payload = execute_one(score_c1=False)
            extra += 1
            _exclude_if_unlabelled(payload)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    run_sec = time.monotonic() - t_run_start

    jsonl = out / "frames.jsonl"
    with jsonl.open("w") as fh:
        for row in all_rows:
            fh.write(json.dumps(row) + "\n")

    summary = {
        "mode": ("raw_negative_control"
                 if args.negative_control_raw_only else "semantic"),
        "trials_requested": args.trials,
        "spawn_failures": spawn_failures,
        "stale_instance_frames": stale_frames_total,
        "warmup_frames_config": args.warmup_frames,
        "gap_sec": args.gap_sec,
        "launch_params": args.launch_params,
        "revision": _revision_info(),
        "active_output_hz": scoring.active_window_hz(
            all_stamps, gap_sec=args.gap_sec),
        # End-to-end trial-cycle rate includes orchestration gaps and is
        # reported separately from the active-window rate.
        "trial_cycle_hz": (args.trials / run_sec) if run_sec > 0 else None,
        "trial_cycle_sec_mean": (
            run_sec / args.trials if args.trials else None),
        "matrix_coverage": _matrix_coverage(trials_gt),
        "trial_recoveries": trial_recoveries,
        "c1_g6": bool(args.c1_g6),
        "c2_exclude_generations": c2_exclude,
        "c2_exclude_file": exclude_path,
        "classified_trials": [
            {
                "trial": t.get("trial"),
                "box_id": t.get("box_id"),
                "size": t.get("size"),
                "trial_class": t.get("trial_class"),
                "reasons": t.get("reasons"),
                "generation": t.get("generation"),
                "score_c1": t.get("score_c1"),
                "recovery": t.get("recovery"),
            }
            for t in classified_trials
        ],
    }
    if args.negative_control_raw_only:
        settled_rows = [r for t in trials for r in t["settled"]]
        summary.update(scoring.negative_control_verdict(settled_rows))
        summary["gate4_pass"] = bool(
            summary["negative_control_pass"]
            and summary["active_output_hz"] is not None)
    else:
        summary.update(scoring.aggregate(trials))
        if args.c1_g6:
            scored = [t for t in classified_trials if t.get("score_c1")]
            g6 = c1_g6_gate(
                scored,
                active_output_hz=scoring.active_window_hz(
                    [row.get("stamp_sec")
                     for t in scored
                     if t.get("trial_class") == "normal_pass"
                     for row in (t.get("settled") or [])
                     if row.get("stamp_sec") is not None],
                    gap_sec=args.gap_sec),
                diagnostic_summary=summary)
            summary.update(g6)
            summary["gate4_pass"] = bool(g6.get("c1_g6_pass"))
            summary["gate4_failures"] = list(g6.get("c1_g6_failures") or [])
        else:
            summary = scoring.gate_pass(summary)
            summary["coverage_failures"] = scoring.coverage_gate(
                summary["matrix_coverage"],
                min_sizes=args.min_sizes,
                min_xy_offsets=args.min_xy_offsets,
                min_yaws=args.min_yaws,
                min_trials=args.trials,
                min_trials_per_size=args.min_trials_per_size)
            if summary["coverage_failures"]:
                summary["gate4_pass"] = False
                summary["gate4_failures"] = (
                    list(summary.get("gate4_failures", []))
                    + summary["coverage_failures"])
            extra = scoring.placement_recovery_gate(
                spawn_failures,
                trial_recoveries,
                failed_count=summary["categories"]["failed"])
            if extra:
                summary["gate4_pass"] = False
                summary["gate4_failures"] = (
                    list(summary.get("gate4_failures", [])) + extra)
    if dump_root is not None:
        summary["dump_dir"] = str(dump_root)
        summary["dump_trials"] = dump_records
        write_index(str(dump_root), dump_records, extra={
            "out": str(out),
            "workspace_center_xy": workspace[0],
            "workspace_half_extents": workspace[1],
        })
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["gate4_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
