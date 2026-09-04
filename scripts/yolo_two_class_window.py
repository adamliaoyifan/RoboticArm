#!/usr/bin/env python3
"""YOLO two-class 10s window: loafbrr vs vintage hit rate + stamps.

Assumes sim_world is already up with semantic chain and mesh pickup visuals::

    ROS_DOMAIN_ID=0 ros2 launch luggage_gazebo sim_world.launch.py \\
      gui:=false use_rviz:=false use_semantic:=true \\
      visual_kind:=mesh size_mode:=catalog yaw_mode:=continuous \\
      yaw_range:=0.0,0.0 sequence_ids:=carryon,standard \\
      observe_pose_name:=pickup_observe \\
      semantic_require_backend:=bbox_fill

Then::

    python3 scripts/yolo_two_class_window.py --window-sec 10

Per visual: SpawnNextBox (settle is inside the spawner) -> 0.5 s -> record
every unique ``image_stamp`` for ``window_sec`` wall seconds. Hit = raw YOLO
cargo after self-body, not a temporal hold. Writes jsonl + summary.json under
``docs/status/evidence/yolo_two_class_window/<visual>/``.
"""

from __future__ import division

import argparse
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from luggage_msgs.srv import ClearCurrentBox, GetCurrentBox, SpawnNextBox

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "src", "luggage_perception"))
from luggage_perception.detect_overlay import (  # noqa: E402
    project_detection,
    rotation_from_quaternion,
)
from luggage_perception.eval.yolo_window_stats import (  # noqa: E402
    GT_IOU_THRESH,
    aabb_from_uv,
    annotate_gt,
    summarize_window,
)

OUT_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..",
    "docs", "status", "evidence", "yolo_two_class_window")

CATALOG_VISUAL = {
    "carryon": "loafbrr",
    "standard": "vintage",
    "large": "loafbrr",
    "suitcase_loafbrr": "loafbrr",
    "suitcase_vintage": "vintage",
}


def _parse_json(payload):
    if not payload:
        return None
    try:
        return json.loads(payload)
    except ValueError:
        return None


class YoloWindowNode(Node):

    def __init__(self):
        super().__init__("yolo_two_class_window")
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._stats = None
        self._overlay = None
        self._camera_info = None
        self._size_eval = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        stats_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        image_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            String, "/semantic_segmenter/stats_json",
            self._on_stats, stats_qos, callback_group=self._group)
        self.create_subscription(
            String, "/luggage/perception/size_eval/spawned",
            self._on_size_eval, stats_qos, callback_group=self._group)
        self.create_subscription(
            Image, "/luggage/semantic/overlay",
            self._on_overlay, image_qos, callback_group=self._group)
        self.create_subscription(
            CameraInfo, "/luggage/preprocessed/camera/color/camera_info",
            self._on_camera_info, image_qos, callback_group=self._group)

        self._spawn = self.create_client(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box",
            callback_group=self._group)
        self._clear = self.create_client(
            ClearCurrentBox, "/pickup_box_spawner/clear_current_box",
            callback_group=self._group)
        self._current = self.create_client(
            GetCurrentBox, "/pickup_box_spawner/get_current_box",
            callback_group=self._group)

    def _on_stats(self, msg):
        data = _parse_json(msg.data)
        if data is None:
            return
        with self._lock:
            self._stats = data

    def _on_size_eval(self, msg):
        data = _parse_json(msg.data)
        if data is None:
            return
        with self._lock:
            self._size_eval = data

    def _on_overlay(self, msg):
        with self._lock:
            self._overlay = msg

    def _on_camera_info(self, msg):
        self._camera_info = msg

    def snapshot_stats(self):
        with self._lock:
            return dict(self._stats) if self._stats else None

    def snapshot_overlay(self):
        with self._lock:
            return self._overlay

    def snapshot_size_eval(self):
        with self._lock:
            return dict(self._size_eval) if self._size_eval else None

    def wait_service(self, client, name, timeout=30.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("%s not available" % name)

    def call(self, client, request, timeout=30.0):
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: event.set())
        if not event.wait(timeout=timeout):
            raise RuntimeError("service call timed out")
        return future.result()

    def wait_stats(self, timeout=60.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.snapshot_stats() is not None:
                return
            time.sleep(0.1)
        raise RuntimeError("no /semantic_segmenter/stats_json yet")

    def projection_inputs(self):
        info = self._camera_info
        if info is None or len(info.k) < 6:
            return None, None, None, "no_camera_info"
        fx, fy, cx, cy = (
            float(info.k[0]), float(info.k[4]),
            float(info.k[2]), float(info.k[5]))
        if fx <= 1e-6 or fy <= 1e-6:
            return None, None, None, "bad_intrinsics"
        camera_frame = info.header.frame_id or "camera_depth_optical_frame"
        try:
            tf_msg = self._tf_buffer.lookup_transform(
                camera_frame, "world", rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.5))
        except TransformException as exc:
            return None, None, None, "tf:%s" % exc
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        rotation = rotation_from_quaternion([q.x, q.y, q.z, q.w])
        translation = [float(t.x), float(t.y), float(t.z)]
        image_size = (int(info.width), int(info.height))
        return (rotation, translation), (fx, fy, cx, cy), image_size, None

    def gt_bbox(self):
        resp = self.call(self._current, GetCurrentBox.Request())
        if resp is None or not resp.success:
            return None, "no_current_box"
        extra, k, image_size, err = self.projection_inputs()
        if err:
            return None, err
        rotation, translation = extra
        box = resp.box
        pos = [box.pose.position.x, box.pose.position.y, box.pose.position.z]
        quat = [
            box.pose.orientation.x, box.pose.orientation.y,
            box.pose.orientation.z, box.pose.orientation.w,
        ]
        size = [box.width, box.depth, box.height]
        _centre, corner_uv, corner_valid, _ok = project_detection(
            pos, quat, size, rotation, translation, k)
        aabb = aabb_from_uv(corner_uv, corner_valid, image_size)
        if aabb is None:
            return None, "empty_projection"
        return aabb, None

    def visual_name(self, fallback):
        eval_rec = self.snapshot_size_eval() or {}
        for key in (eval_rec.get("visual_id"), eval_rec.get("catalog_id")):
            name = CATALOG_VISUAL.get(str(key or ""))
            if name:
                return name
        model = str(eval_rec.get("model_name") or "")
        for token in reversed(model.split("_")):
            name = CATALOG_VISUAL.get(token)
            if name:
                return name
        return fallback


def _save_overlay(msg, path):
    if msg is None:
        return False
    try:
        import cv2
    except ImportError:
        return False
    channels = 3 if "8" in str(msg.encoding) else 1
    if channels != 3:
        return False
    image = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        image = image.reshape((int(msg.height), int(msg.width), 3))
    except ValueError:
        return False
    os.makedirs(os.path.dirname(path), exist_ok=True)
    return bool(cv2.imwrite(path, image))


def collect_window(node, window_sec, extra_settle, save_overlays, out_dir):
    time.sleep(float(extra_settle))
    gt_bbox, gt_err = node.gt_bbox()
    seen = set()
    frames = []
    saved_hit = False
    saved_miss = False
    deadline = time.time() + float(window_sec)
    while time.time() < deadline:
        rec = node.snapshot_stats()
        if rec is None:
            time.sleep(0.02)
            continue
        stamp = rec.get("image_stamp", rec.get("stamp"))
        if stamp is None or stamp in seen:
            time.sleep(0.02)
            continue
        seen.add(stamp)
        rec = dict(rec)
        rec["image_stamp"] = stamp
        rec = annotate_gt(rec, gt_bbox, thresh=GT_IOU_THRESH)
        rec["gt_proj_error"] = gt_err
        frames.append(rec)
        if save_overlays:
            overlay = node.snapshot_overlay()
            if rec.get("raw_cargo") and not saved_hit:
                saved_hit = _save_overlay(
                    overlay, os.path.join(out_dir, "first_hit_overlay.png"))
            if (not rec.get("raw_cargo")) and not saved_miss:
                saved_miss = _save_overlay(
                    overlay, os.path.join(out_dir, "first_miss_overlay.png"))
        time.sleep(0.02)
    return frames, gt_err


def run_cells(node, window_sec, extra_settle, save_overlays, out_root, n_cells):
    node.wait_service(node._spawn, "spawn_next_box")
    node.wait_service(node._clear, "clear_current_box")
    node.wait_service(node._current, "get_current_box")
    node.wait_stats()
    summaries = []
    for index in range(int(n_cells)):
        node.call(node._clear, ClearCurrentBox.Request())
        spawned = node.call(node._spawn, SpawnNextBox.Request())
        if spawned is None or not spawned.success:
            raise RuntimeError(
                "spawn failed: %s" % getattr(spawned, "message", spawned))
        want = spawned.box.id
        deadline = time.time() + 2.0
        while time.time() < deadline:
            ev = node.snapshot_size_eval() or {}
            if ev.get("model_name") == want:
                break
            time.sleep(0.05)
        visual = node.visual_name("cell_%02d" % index)
        out_dir = os.path.join(out_root, visual)
        os.makedirs(out_dir, exist_ok=True)
        frames, gt_err = collect_window(
            node, window_sec, extra_settle, save_overlays, out_dir)
        jsonl_path = os.path.join(out_dir, "trials.jsonl")
        with open(jsonl_path, "w", encoding="utf-8") as handle:
            for rec in frames:
                handle.write(json.dumps(rec, sort_keys=True) + "\n")
        summary = summarize_window(frames)
        summary["visual"] = visual
        summary["gt_proj_error"] = gt_err
        summary["window_sec"] = float(window_sec)
        summary["n_hit_ratio"] = "%d/%d" % (
            summary["n_raw_hit"], summary["n_frames"])
        eval_rec = node.snapshot_size_eval() or {}
        summary["catalog_id"] = eval_rec.get("catalog_id")
        summary["visual_id"] = eval_rec.get("visual_id")
        summary["visual_kind"] = eval_rec.get("visual_kind")
        summary["spawn_size"] = [
            eval_rec.get("width"), eval_rec.get("depth"), eval_rec.get("height")]
        with open(os.path.join(out_dir, "summary.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
        print(
            "%s: %s hit_rate=%.3f n_gt_aligned=%d n_fp=%d n_held_rescue=%d "
            "max_miss=%s inferred_hz=%s"
            % (visual, summary["n_hit_ratio"], summary["hit_rate"],
               summary["n_gt_aligned"], summary["n_false_positive"],
               summary["n_held_rescue"],
               (summary.get("miss_streaks") or {}).get("max"),
               ("%.2f" % summary["inferred_hz"]
                if summary.get("inferred_hz") else "?")))
        for hint in summary.get("postproc_hints") or []:
            print("  hint: %s" % hint)
        summaries.append(summary)
        node.call(node._clear, ClearCurrentBox.Request())
    combined = {
        "window_sec": float(window_sec),
        "cells": summaries,
    }
    os.makedirs(out_root, exist_ok=True)
    with open(os.path.join(out_root, "summary.json"), "w",
              encoding="utf-8") as handle:
        json.dump(combined, handle, indent=2, sort_keys=True)
    return combined


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window-sec", type=float, default=10.0)
    parser.add_argument("--extra-settle-sec", type=float, default=0.5)
    parser.add_argument("--n-cells", type=int, default=2,
                        help="SpawnNextBox count (carryon then standard).")
    parser.add_argument("--out-dir", default=OUT_DIR)
    parser.add_argument("--save-overlays", action="store_true", default=True)
    parser.add_argument("--no-save-overlays", action="store_false",
                        dest="save_overlays")
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = YoloWindowNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        run_cells(
            node, args.window_sec, args.extra_settle_sec,
            args.save_overlays, os.path.abspath(args.out_dir), args.n_cells)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
