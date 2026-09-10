#!/usr/bin/env python3
"""PF-R9 g2 D1 complete-path baseline probe.

Subscribes the full accepted subscriber graph plus the g2 target surface:
raw colour, preprocessed colour, preprocessed depth (the aligned-depth
reader generation 2's filter will become), preprocessed cloud (current
graph), preprocessor status (1 Hz, carries d1_stage_ms and cumulative
d1_bytes_total), and filter stats.

Records, over the scored window:
- arrival-based emission counters (unique stamps per topic);
- raw->pre latency for colour and depth;
- the node-reported per-stage timing (receive->view, core, queue wait,
  output construction, per-product publish) and byte counters as deltas.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String


def _pct(values, q):
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered))) - 1))
    return ordered[idx]


class D1Probe(Node):
    def __init__(self):
        super().__init__("pf_r9_g2_d1_probe")
        groups = [MutuallyExclusiveCallbackGroup() for _ in range(6)]
        be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        rel = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        tl = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.raw_rgb = {}
        self.pre_rgb = {}
        self.pre_depth = {}
        self.pre_cloud = set()
        self.status_first = None
        self.status_last = None
        self.filter_first = None
        self.filter_last = None
        self.create_subscription(
            Image, "/camera/color/image_raw",
            self._on("raw_rgb"), be, callback_group=groups[0])
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/color/image",
            self._on("pre_rgb"), be, callback_group=groups[1])
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/depth/image",
            self._on("pre_depth"), be, callback_group=groups[2])
        self.create_subscription(
            PointCloud2, "/luggage/preprocessed/camera/depth/points",
            self._on_set("pre_cloud"), be, callback_group=groups[3])
        self.create_subscription(
            String, "/luggage/preprocessed/status",
            self._status, tl, callback_group=groups[4])
        self.create_subscription(
            String, "/semantic_point_filter/stats_json",
            self._filter, tl, callback_group=groups[5])

    def _on(self, key):
        def cb(msg):
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            getattr(self, key).setdefault(stamp, time.monotonic())
        return cb

    def _on_set(self, key):
        def cb(msg):
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            getattr(self, key).add(stamp)
        return cb

    def _status(self, msg):
        try:
            rec = json.loads(msg.data)
        except ValueError:
            return
        rec["_wall"] = time.monotonic()
        if rec.get("d1_stage_ms") or rec.get("d1_bytes_total"):
            if self.status_first is None:
                self.status_first = rec
            self.status_last = rec

    def _filter(self, msg):
        try:
            rec = json.loads(msg.data)
        except ValueError:
            return
        rec["_wall"] = time.monotonic()
        if self.filter_first is None:
            self.filter_first = rec
        self.filter_last = rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=125.0)
    ap.add_argument("--warmup", type=float, default=15.0)
    args = ap.parse_args()
    rclpy.init()
    probe = D1Probe()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(probe)
    t0 = time.monotonic()
    window_start = t0 + args.warmup
    window_end = t0 + args.warmup + args.duration
    while time.monotonic() < window_end:
        ex.spin_once(timeout_sec=0.2)

    def in_win(d):
        return {k: v for k, v in d.items() if window_start <= v <= window_end}

    raw = in_win(probe.raw_rgb)
    pre = in_win(probe.pre_rgb)
    pre_d = in_win(probe.pre_depth)
    lags_rgb = [max(0.0, t - probe.raw_rgb[s]) for s, t in pre.items()
                if s in probe.raw_rgb]
    lags_depth = [max(0.0, t - probe.raw_rgb[s]) for s, t in pre_d.items()
                  if s in probe.raw_rgb]
    out = {
        "duration_sec": args.duration,
        "raw_rgb_unique": len(raw),
        "pre_rgb_unique": len(pre),
        "pre_depth_unique": len(pre_d),
        "pre_cloud_unique": len(probe.pre_cloud),
        "emission_over_rgb": len(pre) / len(raw) if raw else None,
        "raw_to_pre_rgb_sec": {
            "n": len(lags_rgb), "p50": _pct(lags_rgb, 0.5),
            "p95": _pct(lags_rgb, 0.95), "max": _pct(lags_rgb, 1.0)},
        "raw_to_pre_depth_sec": {
            "n": len(lags_depth), "p50": _pct(lags_depth, 0.5),
            "p95": _pct(lags_depth, 0.95), "max": _pct(lags_depth, 1.0)},
        "d1_stage_ms_final": (
            probe.status_last.get("d1_stage_ms")
            if probe.status_last else None),
        "d1_bytes_delta": None,
        "filter_counters": None,
    }
    if probe.status_first and probe.status_last:
        b0 = probe.status_first.get("d1_bytes_total") or {}
        b1 = probe.status_last.get("d1_bytes_total") or {}
        out["d1_bytes_delta"] = {
            k: (b1.get(k, 0) - b0.get(k, 0)) for k in sorted(set(b0) | set(b1))}
    if probe.filter_first and probe.filter_last:
        f0, f1 = probe.filter_first, probe.filter_last
        out["filter_counters"] = {
            k: (f1.get(k, 0) - f0.get(k, 0))
            for k in ("cloud", "mask", "joined", "stale_cloud_dropped",
                      "stale_mask_dropped")
            if isinstance(f1.get(k), (int, float))}
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=1)
    print(json.dumps(out, indent=1))
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
