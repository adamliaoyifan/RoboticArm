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
from collections import Counter, defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from luggage_msgs.msg import DetectionFrame, YoloDetections
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String

from luggage_perception.cargo_instance_tracker import parse_current_box_payload
from luggage_perception.pf_r6_metrics import (
    active_rate,
    first_latency_summary,
    stamp_delta_ms,
    stamp_match_summary,
    stats,
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--gap", type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    n = Node("pf_r6_probe")
    be = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
    ev = {"raw_img": [], "pre_rgb": [], "depth_pts": [], "cargo": [],
          "yolo": [], "frame": [], "diag": [], "filter_stats": [],
          "stream_stats": [], "current_box": []}
    stamps = defaultdict(dict)  # topic -> {(s,ns): receipt_mono}
    rss = []
    cargo_pts = []
    filter_stage_timing = defaultdict(list)
    stream_timing = defaultdict(list)
    valid_stream_timing = defaultdict(list)
    pca_reasons = Counter()
    latest_filter_stats = {}
    latest_stream_stats = {}
    latest_valid_stream_stats = {}
    epoch_events = {}

    def flatten(prefix, value, out):
        if isinstance(value, dict):
            for key, child in value.items():
                name = "%s.%s" % (prefix, key) if prefix else str(key)
                flatten(name, child, out)
            return
        try:
            number = float(value)
        except (TypeError, ValueError):
            return
        out[prefix].append(number)

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
        generation = int(getattr(msg, "generation", 0) or 0)
        rec = epoch_events.get(generation)
        if rec is not None:
            if int(getattr(msg, "geometry_level", 0)) == 0:
                rec.setdefault("first_top_only", t)
            if int(getattr(msg, "geometry_level", 0)) == 1:
                rec.setdefault("first_full_3d", t)

    n.create_subscription(DetectionFrame,
                          "/luggage/perception/detection_frame", fcb, be)

    def dcb(msg):
        ev["diag"].append(time.monotonic())

    n.create_subscription(String, "/luggage_detector/diagnostics_json",
                          dcb, be)

    def json_cb(kind):
        def cb(msg):
            nonlocal latest_filter_stats, latest_stream_stats
            nonlocal latest_valid_stream_stats
            ev[kind].append(time.monotonic())
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict):
                return
            if kind == "filter_stats":
                latest_filter_stats = payload
                flatten("", payload.get("stage_ms") or {},
                        filter_stage_timing)
            else:
                latest_stream_stats = payload
                pca_reasons[str(payload.get("pca_reason", ""))] += 1
                timing = payload.get("timing_ms") or {}
                flatten("", timing, stream_timing)
                if payload.get("pca_valid"):
                    latest_valid_stream_stats = payload
                    flatten("", timing, valid_stream_timing)
        return cb

    n.create_subscription(String, "/semantic_point_filter/stats_json",
                          json_cb("filter_stats"), be)
    n.create_subscription(String, "/luggage_detector/stream_stats_json",
                          json_cb("stream_stats"), be)

    def box_cb(msg):
        t = time.monotonic()
        ev["current_box"].append(t)
        _box_id, generation = parse_current_box_payload(msg.data)
        epoch_events[generation] = {"start": t}

    n.create_subscription(String, "/luggage/current_box", box_cb, be)

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

    report = {
        "duration_s": args.duration,
        "rates_hz": {k: active_rate(v, args.gap) for k, v in ev.items()},
        "counts": {k: len(v) for k, v in ev.items()},
        "stage_delta_ms": {
            "raw_img->pre_rgb": stats(
                stamp_delta_ms(stamps, "raw_img", "pre_rgb")),
            "pre_rgb->yolo": stats(
                stamp_delta_ms(stamps, "pre_rgb", "yolo")),
            "depth_pts->cargo": stats(
                stamp_delta_ms(stamps, "depth_pts", "cargo")),
            "yolo->frame": stats(
                stamp_delta_ms(stamps, "yolo", "frame")),
            "raw_img->frame_e2e": stats(
                stamp_delta_ms(stamps, "raw_img", "frame")),
        },
        "stamp_matches": {
            "pre_rgb/yolo": stamp_match_summary(stamps, "pre_rgb", "yolo"),
            "depth_pts/cargo": stamp_match_summary(
                stamps, "depth_pts", "cargo"),
            "yolo/frame": stamp_match_summary(stamps, "yolo", "frame"),
        },
        "first_latency_after_box_ms": first_latency_summary(epoch_events),
        "latest_filter_stats": latest_filter_stats,
        "latest_stream_stats": latest_stream_stats,
        "latest_valid_stream_stats": latest_valid_stream_stats,
        "pca_reasons": dict(pca_reasons),
        "filter_stage_timing_ms": {
            k: stats(v) for k, v in sorted(filter_stage_timing.items())
        },
        "stream_timing_ms": {
            k: stats(v) for k, v in sorted(stream_timing.items())
        },
        "valid_stream_timing_ms": {
            k: stats(v) for k, v in sorted(valid_stream_timing.items())
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
