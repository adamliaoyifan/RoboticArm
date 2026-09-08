#!/usr/bin/env python3
"""PF-R8 A4 accepted-detection recall measurement (capture + score).

The measurement definition is constant across the pre-change baseline and
the post-change run: THIS script classifies every cargo detection with the
production predicate (``evaluate_detection_acceptance``) using live
camera TF at the message stamp and the configured static workspace
geometry, regardless of which stack produced the detections. The stack
under test is selected by which install you source before launching.

Live capture (needs a running accepted-profile stack on the same domain):

    python3 pf_r8_recall_capture.py --out <dir> --trials 15 --trial-sec 10

    # baseline (pre-PF-R8 detector): launch the stack from the gen3
    # worktree install; post-change: launch from the PF-R8 install.

Offline scoring (deterministic, re-runnable):

    python3 pf_r8_recall_capture.py --score <dir>/raw.jsonl

Settled frames: frames carrying a non-empty instance_id at least
``--warmup`` (default 5, gate4 WARMUP_FRAMES) frames after an instance
change. Reported metrics:

- ``recall_raw``: settled frames with >=1 accepted in-region cargo
  detection / settled frames (A4-2 numerator without the hold).
- ``recall_gate_bridged``: same numerator after replaying the repaired
  DetectionTemporalGate over the stream; held frames count as accepted
  (the hold's designed bridge; A4-1's k<=2 derivation).
- ``max_miss_run``: longest run of consecutive settled frames with no
  accepted detection, pre-gate (A4-1 bar: <= 2).
- ``production_agreement``: fraction of cargo detections where the live
  segmenter's own annotation (post-change stacks only) matches this
  script's classification.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

DEFAULT_WORKSPACE = {
    "center_xy": [-1.0, 0.0],
    "half_xy": [0.5, 0.5],
    "plane_z": 0.86,
    "margin": 0.15,
    "world_frame": "world",
}
DEFAULT_INTRINSICS = {"fx": 337.22194822727283, "fy": 337.22194822727283,
                      "cx": 320.0, "cy": 240.0}


# --------------------------------------------------------------------------
# Offline scoring (pure; unit-testable without ROS)
# --------------------------------------------------------------------------

def score_rows(rows, warmup=5, window=5, min_ratio=0.5):
    """Score captured rows; returns the summary dict (see module docstring)."""
    from luggage_perception.detection_temporal_gate import DetectionTemporalGate

    settled = []
    last_key = None
    since_change = None
    for row in rows:
        key = (row.get("generation"), row.get("instance_id"))
        if key != last_key:
            last_key = key
            since_change = 0
        else:
            since_change += 1
        row["settled"] = bool(row.get("instance_id")) and since_change >= warmup
        # Raw recall counts only live detector hits; held boxes are the
        # gate's bridge and are scored separately via the gate replay.
        row["accepted_present"] = any(
            d.get("accepted") and not d.get("held")
            for d in row.get("detections", [])
            if int(d.get("label", -1)) == 2)
        if row["settled"]:
            settled.append(row)

    # Gate replay over the FULL stream (instance epochs reset the gate the
    # way the live node does on /luggage/current_box changes). No RGB was
    # recorded, so every frame feeds a constant image = the documented
    # no-scene-change signal; epoch resets carry the spawn-change duty.
    gate = DetectionTemporalGate(window_size=window, min_positive_ratio=min_ratio)
    agreement = [0, 0]
    last_epoch = None
    for row in rows:
        epoch = (row.get("generation"), row.get("instance_id"))
        if epoch != last_epoch:
            gate.reset()
            last_epoch = epoch
        dets = [dict(d) for d in row.get("detections", [])]
        for det in dets:
            if ("production_accepted" in det
                    and int(det.get("label", -1)) == 2):
                agreement[0] += int(
                    bool(det["production_accepted"]) == bool(det["accepted"]))
                agreement[1] += 1
        labels = np.zeros((int(row.get("image_height") or 480),
                           int(row.get("image_width") or 640)), dtype=np.uint8)
        rgb = np.full(
            (int(row.get("image_height") or 480),
             int(row.get("image_width") or 640), 3), 128, dtype=np.uint8)
        _, _, stats = gate.apply(labels, dets, rgb)
        row["gate_held"] = bool(stats["held"])
        row["gate_accepted"] = bool(stats["accepted_cargo"])

    n_settled = len(settled)
    n_accepted = sum(1 for r in settled if r["accepted_present"])
    n_gate = sum(1 for r in settled
                 if r["accepted_present"] or r["gate_held"])
    max_run = 0
    run = 0
    for r in settled:
        if r["accepted_present"]:
            run = 0
        else:
            run += 1
            max_run = max(max_run, run)
    return {
        "frames": len(rows),
        "settled_frames": n_settled,
        "recall_raw": (n_accepted / n_settled) if n_settled else None,
        "recall_gate_bridged": (n_gate / n_settled) if n_settled else None,
        "max_miss_run": max_run,
        "production_agreement": (
            agreement[0] / agreement[1]) if agreement[1] else None,
        "production_matched": agreement[1],
    }


# --------------------------------------------------------------------------
# Live capture
# --------------------------------------------------------------------------

def capture(out_dir, trials, trial_sec, warmup):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from luggage_msgs.msg import DetectionFrame, YoloDetections
    from luggage_msgs.srv import SpawnNextBox
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    from luggage_perception.semantic_segmenter import (
        WorkspaceAcceptanceContext,
        evaluate_detection_acceptance,
    )
    from luggage_perception.wrist_self_body import (
        matrix_from_translation_quaternion)

    ws = DEFAULT_WORKSPACE

    class Capture(Node):
        def __init__(self):
            super().__init__("pf_r8_recall_capture")
            self.rows = []
            self.stats_recent = []
            self.frame_reasons = []
            self.tf_buffer = Buffer()
            # The TF listener floods a single-threaded executor and starves
            # the RELIABLE stats subscription; give every subscription its
            # own (mutually exclusive, per-callback) group and use a
            # multi-threaded executor.
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            self._g_yolo = MutuallyExclusiveCallbackGroup()
            self._g_stats = MutuallyExclusiveCallbackGroup()
            self._g_frame = MutuallyExclusiveCallbackGroup()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(
                YoloDetections, "/luggage/semantic/yolo_detections",
                self._on_yolo, qos, callback_group=self._g_yolo)
            stats_qos = QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                String, "/semantic_segmenter/stats_json",
                self._on_stats, stats_qos, callback_group=self._g_stats)
            self.create_subscription(
                DetectionFrame, "/luggage/perception/detection_frame",
                self._on_frame, QoSProfile(
                    depth=20, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self._g_frame)
            self.spawn = self.create_client(
                SpawnNextBox, "/pickup_box_spawner/spawn_next_box")

        def _on_stats(self, msg):
            try:
                rec = json.loads(msg.data)
            except ValueError:
                return
            self.stats_recent.append(rec)
            if len(self.stats_recent) > 40:
                self.stats_recent.pop(0)
            # The stats record for stamp X is published right after the
            # yolo message for X, so at yolo-callback time it is not there
            # yet. Retro-annotate the already-recorded row instead.
            try:
                stamp = float(rec.get("mask_stamp", -1e18))
            except (TypeError, ValueError):
                return
            for row in reversed(self.rows[-8:]):
                if abs(row["stamp"] - stamp) <= 2e-3:
                    self._attach_production(row, rec)
                    break

        @staticmethod
        def _attach_production(row, rec):
            prod = {}
            for det in rec.get("detections", []):
                if det.get("bbox"):
                    prod[tuple(int(v) for v in det["bbox"])] = det
            for det in row.get("detections", []):
                pdet = prod.get(tuple(det["bbox"]))
                if pdet is not None and "accepted" in pdet:
                    det["production_accepted"] = bool(pdet["accepted"])

        def _stats_for(self, stamp):
            """Nearest segmenter stats record within 2 ms (stamp round-trip)."""
            best, best_d = None, 2e-3
            for rec in self.stats_recent:
                d = abs(float(rec.get("mask_stamp", -1e18)) - stamp)
                if d <= best_d:
                    best, best_d = rec, d
            return best

        def _on_frame(self, msg):
            self.frame_reasons.append({
                "stamp": msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec,
                "pca_reason": str(msg.pca_reason or ""),
                "pca_valid": bool(msg.pca_valid),
                "geometry_level": int(msg.geometry_level),
                "n_cargo_points": int(msg.n_cargo_points),
            })

        def _on_yolo(self, msg):
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            frame_id = msg.header.frame_id or "camera_depth_optical_frame"
            ctx = None
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    ws["world_frame"], frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp))
                trans = tf_msg.transform.translation
                rot = tf_msg.transform.rotation
                mat = matrix_from_translation_quaternion(
                    (trans.x, trans.y, trans.z),
                    (rot.x, rot.y, rot.z, rot.w))
                ctx = WorkspaceAcceptanceContext(
                    fx=DEFAULT_INTRINSICS["fx"], fy=DEFAULT_INTRINSICS["fy"],
                    cx=DEFAULT_INTRINSICS["cx"], cy=DEFAULT_INTRINSICS["cy"],
                    plane_z=ws["plane_z"],
                    center_xy=tuple(ws["center_xy"]),
                    half_xy=tuple(ws["half_xy"]),
                    margin=ws["margin"],
                    optical_to_world=tuple(
                        tuple(float(mat[r][c]) for c in range(4))
                        for r in range(3)))
            except TransformException:
                ctx = None
            dets = []
            prod = {}
            stats_rec = self._stats_for(stamp)
            if stats_rec:
                for det in stats_rec.get("detections", []):
                    if det.get("bbox"):
                        prod[tuple(int(v) for v in det["bbox"])] = det
            for box in msg.detections:
                accepted, reason = evaluate_detection_acceptance(
                    list(box.bbox), ctx)
                det = {
                    "label": int(box.label),
                    "conf": float(box.confidence),
                    "bbox": [int(v) for v in box.bbox],
                    "held": bool(box.held),
                    "accepted": bool(accepted),
                    "reason": reason,
                }
                pdet = prod.get(tuple(det["bbox"]))
                if pdet is not None and "accepted" in pdet:
                    det["production_accepted"] = bool(pdet["accepted"])
                dets.append(det)
            self.rows.append({
                "stamp": stamp,
                "frame_id": frame_id,
                "generation": int(msg.generation),
                "instance_id": str(msg.instance_id),
                "image_width": int(msg.image_width),
                "image_height": int(msg.image_height),
                "tf_ok": ctx is not None,
                "detections": dets,
            })

    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    node = Capture()
    if not node.spawn.wait_for_service(timeout_sec=30.0):
        print("spawn service unavailable", file=sys.stderr)
        rclpy.shutdown()
        return 2
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    t_end = time.monotonic() + trials * trial_sec + 8.0
    next_spawn = time.monotonic() + 2.0
    spawned = 0
    while time.monotonic() < t_end:
        if spawned < trials and time.monotonic() >= next_spawn:
            node.spawn.call_async(SpawnNextBox.Request())
            spawned += 1
            next_spawn += trial_sec
        ex.spin_once(timeout_sec=0.2)
    raw_path = os.path.join(out_dir, "raw.jsonl")
    with open(raw_path, "w", encoding="utf-8") as handle:
        for row in node.rows:
            handle.write(json.dumps(row) + "\n")
    frames_path = os.path.join(out_dir, "detection_frames.json")
    with open(frames_path, "w", encoding="utf-8") as handle:
        json.dump(node.frame_reasons, handle)
    node.destroy_node()
    rclpy.shutdown()
    print("captured %d yolo rows, %d detection frames -> %s"
          % (len(node.rows), len(node.frame_reasons), raw_path))
    summary = score_rows(node.rows, warmup=warmup)
    summary["no_cargo_frames"] = sum(
        1 for f in node.frame_reasons if f["n_cargo_points"] == 0)
    summary_path = os.path.join(out_dir, "summary.json")
    with open(summary_path, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1)
    print(json.dumps(summary, indent=1))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", help="output directory for raw.jsonl/summary.json")
    ap.add_argument("--trials", type=int, default=15)
    ap.add_argument("--trial-sec", type=float, default=10.0)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--score", help="score an existing raw.jsonl and exit")
    args = ap.parse_args()
    if args.score:
        with open(args.score, encoding="utf-8") as handle:
            rows = [json.loads(line) for line in handle if line.strip()]
        rows = [r for r in rows if isinstance(r, dict)]
        print(json.dumps(score_rows(rows, warmup=args.warmup), indent=1))
        return 0
    if not args.out:
        ap.error("--out or --score required")
    return capture(args.out, args.trials, args.trial_sec, args.warmup)


if __name__ == "__main__":
    sys.exit(main())
