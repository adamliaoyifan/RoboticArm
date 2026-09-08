#!/usr/bin/env python3
"""PF-R9 B3/B4 probe: preprocessor emission throughput and join quality.

Common scored window: >= 120 s at pickup_observe with the first 15 s
discarded as warmup (plan B3/B4 shared window). All rates and ratios are
computed on that window:

- B3 emitted-observation rate: unique primary stamps on
  /luggage/preprocessed/camera/color/image over the window, divided by the
  unique RGB header stamps on /camera/color/image_raw received over the
  same window (denominator = unique stamps, not callbacks); bar >= 0.8.
- B3 latency: raw_img -> pre_rgb per stamp = wall arrival of the
  preprocessed RGB minus wall arrival of the raw RGB with the same header
  stamp (both wall-monotonic; same clock domain); bar p50 <= 60 ms.
- B4 cloud_ok: unique primary stamps that also appeared on
  /luggage/preprocessed/camera/depth/points / emitted; bar >= 0.95.
- B4 filter exact-join and stale-drop ratios from
  /semantic_point_filter/stats_json cumulative counters, scored as
  counter deltas across the window: joined/cloud >= 0.95 and
  (stale_cloud_dropped + stale_mask_dropped) / (cloud + mask) < 0.05.
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


class B34Probe(Node):
    def __init__(self):
        super().__init__("pf_r9_b34_probe")
        groups = [MutuallyExclusiveCallbackGroup() for _ in range(4)]
        self.raw_rgb = {}      # header stamp -> arrival wall
        self.pre_rgb = {}      # header stamp -> arrival wall
        self.pre_cloud = set()  # header stamps
        self.stats_first = None
        self.stats_last = None
        be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        stats_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Image, "/camera/color/image_raw",
            self._on_raw, be, callback_group=groups[0])
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/color/image",
            self._on_pre, be, callback_group=groups[1])
        self.create_subscription(
            PointCloud2, "/luggage/preprocessed/camera/depth/points",
            self._on_cloud, be, callback_group=groups[2])
        self.create_subscription(
            String, "/semantic_point_filter/stats_json",
            self._on_stats, stats_qos, callback_group=groups[3])

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def _on_raw(self, msg):
        self.raw_rgb.setdefault(self._stamp(msg), time.monotonic())

    def _on_pre(self, msg):
        self.pre_rgb.setdefault(self._stamp(msg), time.monotonic())

    def _on_cloud(self, msg):
        self.pre_cloud.add(self._stamp(msg))

    def _on_stats(self, msg):
        try:
            rec = json.loads(msg.data)
        except ValueError:
            return
        if self.stats_first is None:
            self.stats_first = rec
        self.stats_last = rec


def score(probe, window_start, window_end, duration):
    raw_in_window = {
        s: t for s, t in probe.raw_rgb.items()
        if window_start <= t <= window_end}
    pre_in_window = {
        s: t for s, t in probe.pre_rgb.items()
        if window_start <= t <= window_end}
    lags = []
    for stamp, pre_arrival in pre_in_window.items():
        raw_arrival = probe.raw_rgb.get(stamp)
        if raw_arrival is not None:
            lags.append(max(0.0, pre_arrival - raw_arrival))
    emitted = set(pre_in_window)
    with_cloud = {s for s in emitted if s in probe.pre_cloud}
    out = {
        "duration_sec": duration,
        "rgb_unique_in_window": len(raw_in_window),
        "emitted_in_window": len(emitted),
        "b3_emission_over_rgb": (
            len(emitted) / len(raw_in_window)) if raw_in_window else None,
        "b3_raw_to_pre_rgb_sec": {
            "n": len(lags), "p50": _pct(lags, 0.5),
            "p95": _pct(lags, 0.95), "max": _pct(lags, 1.0)},
        "b4_cloud_ok_fraction": (
            len(with_cloud) / len(emitted)) if emitted else None,
    }
    first, last = probe.stats_first, probe.stats_last
    if first is not None and last is not None:
        def delta(key):
            try:
                return int(last.get(key, 0)) - int(first.get(key, 0))
            except (TypeError, ValueError):
                return None
        cloud = delta("cloud")
        mask = delta("mask")
        joined = delta("joined")
        stale = None
        if cloud is not None and mask is not None:
            stale_c = delta("stale_cloud_dropped") or 0
            stale_m = delta("stale_mask_dropped") or 0
            stale = (stale_c + stale_m) / max(1, cloud + mask)
            out["b4_filter"] = {
                "cloud_delta": cloud,
                "mask_delta": mask,
                "joined_delta": joined,
                "stale_cloud_dropped_delta": stale_c,
                "stale_mask_dropped_delta": stale_m,
                "exact_join_joined_over_cloud": (
                    joined / cloud) if cloud else None,
                "stale_drop_ratio": stale,
            }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=125.0)
    ap.add_argument("--warmup", type=float, default=15.0)
    args = ap.parse_args()
    rclpy.init()
    probe = B34Probe()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(probe)
    t0 = time.monotonic()
    window_start = t0 + args.warmup
    window_end = t0 + args.warmup + args.duration
    while time.monotonic() < window_end:
        ex.spin_once(timeout_sec=0.2)
    summary = score(probe, window_start, window_end, args.duration)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1)
    print(json.dumps(summary, indent=1))
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
