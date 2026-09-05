#!/usr/bin/env python3
"""PF-R6 stage-performance probe: per-stage P50/P95/max + active rate.

Subscribes every stage boundary of the accepted semantic path and computes
interval-based rates plus cross-stage stamp deltas on ONE clock domain
(the ROS receipt monotonic wall clock; ROS header stamps are only used as
join keys). Also samples detector RSS. Runs for --duration seconds and
prints a JSON summary.
"""
import argparse
import json
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from luggage_msgs.msg import DetectionFrame, YoloDetections
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String


def pct(sorted_vals, p):
    if not sorted_vals:
        return None
    return sorted_vals[min(len(sorted_vals) - 1,
                           int(round((p / 100.0) * (len(sorted_vals) - 1))))]


def stats(vals):
    s = sorted(vals)
    return {"n": len(s), "p50": pct(s, 50), "p95": pct(s, 95),
            "max": s[-1] if s else None}


def rate(stamps, gap):
    if len(stamps) < 2:
        return None
    st = sorted(stamps)
    iv = span = 0.0
    wi = 0
    start = prev = st[0]
    for t in st[1:]:
        if t - prev > gap:
            span += prev - start
            iv += wi
            wi = 0
            start = t
        else:
            wi += 1
        prev = t
    span += prev - start
    iv += wi
    return round(iv / span, 3) if span > 0 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--gap", type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    n = Node("pf_r6_probe")
    be = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
    ev = {"raw_img": [], "pre_rgb": [], "depth_pts": [], "cargo": [],
          "yolo": [], "frame": [], "diag": []}
    stamps = defaultdict(dict)  # topic -> {(s,ns): receipt_mono}
    rss = []
    cargo_pts = []

    def mk(topic):
        def cb(msg):
            t = time.monotonic()
            h = msg.header.stamp
            ev[topic].append(t)
            stamps[topic][(h.sec, h.nanosec)] = t
            if topic == "cargo":
                cargo_pts.append(len(msg.data) // (msg.point_step or 16))
        return cb

    n.create_subscription(Image, "/camera/color/image_raw", mk("raw_img"), be)
    n.create_subscription(
        Image, "/luggage/preprocessed/camera/color/image", mk("pre_rgb"), be)
    n.create_subscription(
        PointCloud2, "/luggage/preprocessed/camera/depth/points",
        mk("depth_pts"), be)
    n.create_subscription(
        PointCloud2, "/luggage/semantic/cargo_points", mk("cargo"), be)
    n.create_subscription(
        YoloDetections, "/luggage/semantic/yolo_detections", mk("yolo"), be)

    def fcb(msg):
        t = time.monotonic()
        h = msg.header.stamp
        ev["frame"].append(t)
        stamps["frame"][(h.sec, h.nanosec)] = t

    n.create_subscription(DetectionFrame,
                          "/luggage/perception/detection_frame", fcb, be)

    def dcb(msg):
        ev["diag"].append(time.monotonic())

    n.create_subscription(String, "/luggage_detector/diagnostics_json",
                          dcb, be)

    import os
    pid = None
    try:
        import subprocess
        out = subprocess.run(
            ["pgrep", "-f", "lib/luggage_perception/luggage_detector_node"],
            capture_output=True, text=True).stdout.split()
        pid = int(out[0]) if out else None
    except Exception:
        pid = None

    t0 = time.monotonic()
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(n, timeout_sec=0.2)
        if pid and int(time.monotonic() - t0) % 5 == 0:
            try:
                with open("/proc/%d/status" % pid) as fh:
                    for line in fh:
                        if line.startswith("VmRSS:"):
                            rss.append(int(line.split()[1]))
                            break
            except OSError:
                pass
    n.destroy_node()
    rclpy.shutdown()

    # cross-stage stamp-to-receipt deltas (join keys matched exactly)
    def delta(a, b):
        out = []
        for k, t in stamps[a].items():
            if k in stamps[b]:
                out.append((stamps[b][k] - t) * 1000.0)
        return out

    report = {
        "duration_s": args.duration,
        "rates_hz": {k: rate(v, args.gap) for k, v in ev.items()},
        "counts": {k: len(v) for k, v in ev.items()},
        "stage_delta_ms": {
            "raw_img->pre_rgb": stats(delta("raw_img", "pre_rgb")),
            "pre_rgb->yolo": stats(delta("pre_rgb", "yolo")),
            "depth_pts->cargo": stats(delta("depth_pts", "cargo")),
            "yolo->frame": stats(delta("yolo", "frame")),
            "raw_img->frame_e2e": stats(delta("raw_img", "frame")),
        },
        "cargo_points": stats(cargo_pts) if cargo_pts else None,
        "detector_rss_kb": {"first": rss[0] if rss else None,
                            "last": rss[-1] if rss else None,
                            "samples": len(rss)},
        "pid": pid,
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
