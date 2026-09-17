#!/usr/bin/env python3
"""Independent Livox monitor. Does not join scans to RGB-D stamps."""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from luggage_perception.eval.dsim_livox_metrics import (
    intensity_from_structured,
    summarize_scan,
    summarize_window,
    xyz_from_structured,
)


_FLOAT32 = 7
_UINT8 = 1
_UINT16 = 4
_FLOAT64 = 8


def _field_size(datatype):
    return {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}.get(int(datatype), 4)


def decode_livox_msg(msg):
    names = {f.name: f for f in msg.fields}
    n = int(msg.width) * max(1, int(msg.height))
    if "x" not in names or n <= 0:
        return np.zeros((0, 3), dtype=np.float64), None, n
    xyz = xyz_from_structured(
        msg.data, int(msg.point_step), n,
        x_off=int(names["x"].offset),
        y_off=int(names["y"].offset) if "y" in names else 4,
        z_off=int(names["z"].offset) if "z" in names else 8)
    intensity = None
    for key in ("intensity", "i", "reflectivity"):
        if key in names:
            field = names[key]
            intensity = intensity_from_structured(
                msg.data, int(msg.point_step), n,
                offset=int(field.offset),
                field_size=_field_size(field.datatype))
            break
    return xyz, intensity, n


class LivoxMonitor(Node):
    def __init__(self):
        super().__init__("dsim_livox_monitor")
        be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.scans = []
        self._n = 0
        self._t0 = time.monotonic()
        self.create_subscription(PointCloud2, "/livox/lidar", self._on, be)

    def _on(self, msg):
        xyz, intensity, n = decode_livox_msg(msg)
        self._n += 1
        rec = summarize_scan(
            xyz, intensity=intensity,
            frame_id=msg.header.frame_id, n_raw=n,
            heavy=(self._n % 20 == 0))
        rec["t"] = time.monotonic() - self._t0
        rec["stamp"] = [int(msg.header.stamp.sec), int(msg.header.stamp.nanosec)]
        rec["width"] = int(msg.width)
        rec["height"] = int(msg.height)
        rec["point_step"] = int(msg.point_step)
        rec["fields"] = [f.name for f in msg.fields]
        self.scans.append(rec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--warmup", type=float, default=15.0)
    ap.add_argument("--duration", type=float, default=120.0)
    args = ap.parse_args()
    rclpy.init()
    node = LivoxMonitor()
    end = args.warmup + args.duration + 1.0
    t_start = time.monotonic()
    while time.monotonic() - t_start < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    t0, t1 = args.warmup, args.warmup + args.duration
    window = [s for s in node.scans if t0 <= s["t"] <= t1]
    payload = {
        "warmup_s": args.warmup,
        "duration_s": args.duration,
        "n_total": len(node.scans),
        "window": summarize_window(window, [s["t"] for s in window]),
        "last": window[-1] if window else None,
    }
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
    print(json.dumps({"out": args.out, "window": payload["window"]}, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
