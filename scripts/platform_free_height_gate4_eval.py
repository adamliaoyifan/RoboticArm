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
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from luggage_msgs.msg import DetectionFrame
from luggage_msgs.srv import GetCurrentBox, SpawnNextBox
from luggage_perception.eval import gate4_scoring as scoring

WARMUP_FRAMES = 5  # support-stability window after an instance change
GAP_SEC = 2.0      # orchestration gap threshold for active-window Hz


def _stamp_sec(header):
    return float(header.stamp.sec) + 1e-9 * float(header.stamp.nanosec)


def _yaw_from_quat(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))


class Gate4Eval(Node):
    def __init__(self):
        super().__init__("platform_free_height_gate4_eval")
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._frames = []
        self.create_subscription(
            DetectionFrame, "/luggage/perception/detection_frame",
            self._on_frame, qos)
        self._get = self.create_client(
            GetCurrentBox, "/pickup_box_spawner/get_current_box")
        self._spawn = self.create_client(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box")

    def _on_frame(self, msg):
        self._frames.append((time.monotonic(), msg))

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
        return self._call(self._spawn, SpawnNextBox.Request())

    def get_gt(self):
        return self._call(self._get, GetCurrentBox.Request())

    def collect(self, duration_sec):
        t_end = time.monotonic() + duration_sec
        start_n = len(self._frames)
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return [m for _, m in self._frames[start_n:]]


def _row(frame, gt, trial, box_id):
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
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Gate4Eval()
    all_rows = []
    all_stamps = []
    trials = []
    trials_gt = []
    stale_frames_total = 0
    t_run_start = time.monotonic()
    spawn_failures = 0
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
        for trial in range(args.trials):
            spawn = node.spawn_next()
            if spawn is None or not spawn.success:
                spawn_failures += 1
                all_rows.append({
                    "trial": trial, "spawn_ok": False,
                    "message": None if spawn is None else spawn.message,
                })
                continue
            time.sleep(0.5)
            gt = node.get_gt()
            # Expected identity comes from the eval-side spawn response
            # (GT), never from the detector output being tested: if only
            # stale frames arrive after a spawn they must be counted as
            # stale, not adopted as the new instance.
            expected_instance = (
                spawn.box.id or
                (gt.box.id if gt and gt.success else None))
            frames = node.collect(args.settle_sec)
            stamps = [_stamp_sec(fr.header) for fr in frames]
            all_stamps.extend(stamps)
            rows = [_row(fr, gt, trial, expected_instance or "unknown")
                    for fr in frames]
            all_rows.extend(rows)
            owned, stale_n = scoring.filter_expected_instance(
                rows, expected_instance)
            stale_frames_total += stale_n
            warmup, settled = scoring.split_warmup(
                owned, warmup_frames=args.warmup_frames)
            trials.append({
                "trial": trial, "box_id": expected_instance,
                "instance_id": expected_instance,
                "warmup": warmup, "settled": settled,
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
            if not frames:
                all_rows.append({
                    "trial": trial, "box_id": expected_instance,
                    "spawn_ok": True,
                    "top_surface_valid": False, "height_valid": False,
                    "message": "no detection_frame during settle window",
                })
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
    }
    if args.negative_control_raw_only:
        settled_rows = [r for t in trials for r in t["settled"]]
        summary.update(scoring.negative_control_verdict(settled_rows))
        summary["gate4_pass"] = bool(
            summary["negative_control_pass"]
            and summary["active_output_hz"] is not None)
    else:
        summary.update(scoring.aggregate(trials))
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
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0 if summary["gate4_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
