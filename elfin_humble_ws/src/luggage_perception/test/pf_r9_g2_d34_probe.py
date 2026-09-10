#!/usr/bin/env python3
"""PF-R9 g2 D3/D4 probe: depth-primary throughput and join quality.

Common scored window: >= 120 s after 15 s warmup at pickup_observe, full
accepted subscriber graph.

- D3 emitted / unique colour header stamps >= 0.80; raw colour receipt to
  preprocessed colour p50 <= 60 ms (report p95, max, rates, emitted bytes,
  queue drops, payload copy count — from the preprocessor status).
- D4 valid paired depth / emitted >= 0.95 (emissions carrying aligned
  depth on the preprocessed depth topic); filter exact joins / received
  depth >= 0.95; (stale_depth_dropped + stale_mask_dropped) /
  (depth + mask) < 0.05; misses/evictions by named counter.
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
from sensor_msgs.msg import Image
from std_msgs.msg import String


def _pct(values, q):
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered))) - 1))
    return ordered[idx]


class D34Probe(Node):
    def __init__(self):
        super().__init__("pf_r9_g2_d34_probe")
        groups = [MutuallyExclusiveCallbackGroup() for _ in range(5)]
        # All sensor-surface subs need transport depth >=10: at ~28 Hz a
        # depth-2 BEST_EFFORT queue measurably drops frames and deflates
        # the D3/D4 ratios (the preprocessor's own counters show zero
        # depth skips on the same window).
        be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        tl = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.raw_rgb = {}
        self.pre_rgb = {}
        self.pre_depth = {}
        self.prep_first = None
        self.prep_last = None
        self.filter_first = None
        self.filter_last = None
        self.emit_status_total = 0
        self.emit_status_depth_ok = 0
        self.create_subscription(
            Image, "/camera/color/image_raw",
            self._on("raw_rgb"), be, callback_group=groups[0])
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/color/image",
            self._on("pre_rgb"), be, callback_group=groups[1])
        # The depth product publishes RELIABLE (mandatory geometry input);
        # a depth-2 BEST_EFFORT probe sub undercounts it (measured 0.91
        # against the preprocessor's own zero-skip counters).
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/depth/image",
            self._on("pre_depth"), be, callback_group=groups[2])
        self.create_subscription(
            String, "/luggage/preprocessed/status",
            self._prep_status, tl, callback_group=groups[3])
        self.create_subscription(
            String, "/semantic_point_filter/stats_json",
            self._rec("filter"), tl, callback_group=groups[4])

    def _on(self, key):
        def cb(msg):
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            getattr(self, key).setdefault(stamp, time.monotonic())
        return cb

    def _rec(self, key):
        def cb(msg):
            try:
                rec = json.loads(msg.data)
            except ValueError:
                return
            rec["_wall"] = time.monotonic()
            pair = (self.filter_first, self.filter_last)
            if pair[0] is None:
                self.filter_first = rec
            self.filter_last = rec
        return cb

    def _prep_status(self, msg):
        """First-party D4 metric: per-emission records carry flags."""
        try:
            rec = json.loads(msg.data)
        except ValueError:
            return
        rec["_wall"] = time.monotonic()
        if self.prep_first is None:
            self.prep_first = rec
        self.prep_last = rec
        flags = rec.get("flags")
        if isinstance(flags, dict) and "depth_ok" in flags:
            now = time.monotonic()
            # Counted on the scored window only (main sets boundaries).
            if getattr(self, "_window_start", 0) <= now <= getattr(
                    self, "_window_end", float("inf")):
                self.emit_status_total += 1
                self.emit_status_depth_ok += int(bool(flags["depth_ok"]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=125.0)
    ap.add_argument("--warmup", type=float, default=15.0)
    args = ap.parse_args()
    rclpy.init()
    probe = D34Probe()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=5)
    ex.add_node(probe)
    t0 = time.monotonic()
    window_start = t0 + args.warmup
    window_end = t0 + args.warmup + args.duration
    probe._window_start = window_start
    probe._window_end = window_end
    while time.monotonic() < window_end:
        ex.spin_once(timeout_sec=0.2)

    def in_win(d):
        return {k: v for k, v in d.items() if window_start <= v <= window_end}

    raw = in_win(probe.raw_rgb)
    pre = in_win(probe.pre_rgb)
    pre_d = in_win(probe.pre_depth)
    lags = [max(0.0, t - probe.raw_rgb[s]) for s, t in pre.items()
            if s in probe.raw_rgb]
    out = {
        "duration_sec": args.duration,
        "raw_rgb_unique": len(raw),
        "pre_rgb_unique": len(pre),
        "pre_depth_unique": len(pre_d),
        "d3_emission_over_rgb": (
            len(pre) / len(raw)) if raw else None,
        "d3_raw_to_pre_rgb_sec": {
            "n": len(lags), "p50": _pct(lags, 0.5),
            "p95": _pct(lags, 0.95), "max": _pct(lags, 1.0)},
        "d4_paired_depth_over_emitted": (
            probe.emit_status_depth_ok / probe.emit_status_total)
            if probe.emit_status_total else None,
        "d4_emit_status_total": probe.emit_status_total,
        "d4_probe_topic_crosscheck": (
            len(set(pre_d) & set(pre)) / len(pre)) if pre else None,
    }
    p0, p1 = probe.prep_first, probe.prep_last
    if p0 and p1:
        keys = ("depth_wait_skips", "info_wait_skips",
                "acquisition_mismatches", "exact_lookup_misses",
                "payload_materialisations", "rollback_events",
                "emit_queue_drops")
        out["prep_counters"] = {
            k: (p1.get(k, 0) - p0.get(k, 0)) for k in keys}
        out["prep_last"] = {
            k: p1.get(k) for k in (
                "payload_bytes_buffered", "emit_queue_depth",
                "camera_epoch", "d1_stage_ms")}
        out["prep_buffers_last"] = p1.get("buffers")
        out["prep_evictions_last"] = p1.get("evictions")
    f0, f1 = probe.filter_first, probe.filter_last
    if f0 and f1:
        def delta(k):
            try:
                return int(f1.get(k, 0)) - int(f0.get(k, 0))
            except (TypeError, ValueError):
                return None
        depth = delta("depth")
        mask = delta("mask")
        joined = delta("joined")
        out["d4_filter"] = {
            "depth_delta": depth,
            "mask_delta": mask,
            "joined_delta": joined,
            "stale_depth_dropped_delta": delta("stale_depth_dropped"),
            "stale_mask_dropped_delta": delta("stale_mask_dropped"),
            "exact_join_joined_over_depth": (
                joined / depth) if depth else None,
            "stale_drop_ratio": (
                ((delta("stale_depth_dropped") or 0)
                 + (delta("stale_mask_dropped") or 0))
                / max(1, (depth or 0) + (mask or 0))),
        }
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=1)
    print(json.dumps(out, indent=1))
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
