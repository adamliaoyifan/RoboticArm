#!/usr/bin/env python3
"""PF-R9 B1: per-topic period + matched-header receipt-lag probe.

Records, for /camera/color/image_raw and /camera/depth/points:

- header-stamp period series (sim /clock domain, from message headers);
- arrival-time period series (wall clock, monotonic receipt);
- the matched-header cloud-vs-RGB receipt lag: for every header stamp
  present on BOTH topics, |cloud_arrival_wall - rgb_arrival_wall| plus the
  signed value, p50/p95/max;
- the fraction of RGB header stamps with no same-stamp cloud at all
  (unmatched fraction);
- delivery under BEST_EFFORT vs RELIABLE (one subscription each, so the
  bridge's RELIABLE behaviour and a BEST_EFFORT alternative are measured
  side by side — the node comment records that BEST_EFFORT "barely
  delivers" on this RMW; this measures it).

Clock domains: header stamps are sim /camera time (domain: sim /clock);
arrival times are time.monotonic() on this host (domain: wall). No latency
mixes the two domains; the receipt LAG subtracts two wall-clock receipt
times of co-stamped messages.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2


def _pct(values, q):
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered))) - 1))
    return ordered[idx]


class PeriodProbe(Node):
    def __init__(self):
        super().__init__("pf_r9_period_probe")
        groups = [MutuallyExclusiveCallbackGroup() for _ in range(4)]
        be = ReliabilityPolicy.BEST_EFFORT
        rel = ReliabilityPolicy.RELIABLE
        self.streams = {}
        from functools import partial
        for key, msg_type, topic, reliability, group in (
            ("rgb_be", Image, "/camera/color/image_raw", be, groups[0]),
            ("rgb_rel", Image, "/camera/color/image_raw", rel, groups[1]),
            ("cloud_be", PointCloud2, "/camera/depth/points", be, groups[2]),
            ("cloud_rel", PointCloud2, "/camera/depth/points", rel, groups[3]),
        ):
            rec = {"stamps": [], "arrivals": []}
            self.streams[key] = rec
            qos = QoSProfile(depth=1, reliability=reliability)
            self.create_subscription(
                msg_type, topic, partial(self._on, rec),
                qos, callback_group=group)

    def _on(self, rec, msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        rec["stamps"].append(stamp)
        rec["arrivals"].append(time.monotonic())


def summarize(probe, duration):
    out = {"duration_sec": duration, "streams": {}}
    for key, rec in probe.streams.items():
        stamps, arrivals = rec["stamps"], rec["arrivals"]
        stamp_deltas = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
        arrival_deltas = [b - a for a, b in zip(arrivals, arrivals[1:])
                          if b > a]
        out["streams"][key] = {
            "count": len(stamps),
            "unique_stamps": len(set(stamps)),
            "header_period_sec": {
                "p50": _pct(stamp_deltas, 0.5),
                "p95": _pct(stamp_deltas, 0.95),
                "max": _pct(stamp_deltas, 1.0),
                "mean": (sum(stamp_deltas) / len(stamp_deltas))
                if stamp_deltas else None,
            },
            "arrival_period_sec": {
                "p50": _pct(arrival_deltas, 0.5),
                "p95": _pct(arrival_deltas, 0.95),
                "max": _pct(arrival_deltas, 1.0),
                "mean": (sum(arrival_deltas) / len(arrival_deltas))
                if arrival_deltas else None,
            },
        }
    # Matched-header receipt lag: rgb_rel vs cloud_rel (the live path).
    rgb = probe.streams["rgb_rel"]
    cloud = probe.streams["cloud_rel"]
    rgb_by_stamp = {}
    for stamp, arrival in zip(rgb["stamps"], rgb["arrivals"]):
        rgb_by_stamp.setdefault(stamp, arrival)
    lags = []
    signed = []
    matched = 0
    for stamp, arrival in zip(cloud["stamps"], cloud["arrivals"]):
        rgb_arrival = rgb_by_stamp.get(stamp)
        if rgb_arrival is None:
            continue
        matched += 1
        delta = arrival - rgb_arrival
        signed.append(delta)
        lags.append(abs(delta))
    unmatched = len(rgb_by_stamp) - matched
    out["matched_header"] = {
        "clock_domain": "receipt lag = wall(monotonic) cloud arrival minus "
                        "wall(monotonic) rgb arrival of the same header "
                        "stamp; header stamps themselves are sim /clock",
        "rgb_unique": len(rgb_by_stamp),
        "cloud_unique": len(set(cloud["stamps"])),
        "matched": matched,
        "unmatched_rgb": unmatched,
        "unmatched_fraction": (
            unmatched / len(rgb_by_stamp)) if rgb_by_stamp else None,
        "abs_receipt_lag_sec": {
            "p50": _pct(lags, 0.5), "p95": _pct(lags, 0.95),
            "max": _pct(lags, 1.0)},
        "signed_cloud_minus_rgb_sec": {
            "p50": _pct(signed, 0.5), "p95": _pct(signed, 0.95),
            "max": _pct(signed, 1.0)},
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=130.0)
    ap.add_argument("--warmup", type=float, default=15.0)
    args = ap.parse_args()
    rclpy.init()
    probe = PeriodProbe()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(probe)
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.warmup + args.duration:
        ex.spin_once(timeout_sec=0.2)
    # Drop the warmup prefix from every stream before summarizing.
    cut = time.monotonic() - args.duration
    for rec in probe.streams.values():
        pairs = [(s, a) for s, a in zip(rec["stamps"], rec["arrivals"])
                 if a >= cut]
        rec["stamps"] = [p[0] for p in pairs]
        rec["arrivals"] = [p[1] for p in pairs]
    summary = summarize(probe, args.duration)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=1)
    print(json.dumps(summary, indent=1))
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
