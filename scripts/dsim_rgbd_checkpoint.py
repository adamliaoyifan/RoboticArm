#!/usr/bin/env python3
"""DSIM RGBD graph/timing checkpoint. Does not launch Gazebo."""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


TOPICS = {
    "color": ("/camera/color/image_raw", Image),
    "depth_m": ("/camera/depth/image_meters", Image),
    "depth_mm": ("/camera/depth/image_raw", Image),
    "color_info": ("/camera/color/camera_info", CameraInfo),
    "depth_info": ("/camera/depth/camera_info", CameraInfo),
    "pre_color": ("/luggage/preprocessed/camera/color/image", Image),
    "pre_depth": ("/luggage/preprocessed/camera/depth/image", Image),
    "pre_color_info": ("/luggage/preprocessed/camera/color/camera_info", CameraInfo),
    "pre_depth_info": ("/luggage/preprocessed/camera/depth/camera_info", CameraInfo),
    "cam_points": ("/camera/depth/points", PointCloud2),
    "pre_cam_points": ("/luggage/preprocessed/camera/depth/points", PointCloud2),
}

K_TARGET = {
    "fx": 323.1775,
    "fy": 322.8994,
    "cx": 317.7526,
    "cy": 178.0294,
}


def _stamp_key(stamp):
    return (int(stamp.sec), int(stamp.nanosec))


def _k_from_info(msg):
    k = list(msg.k)
    return {
        "fx": float(k[0]) if len(k) > 0 else None,
        "fy": float(k[4]) if len(k) > 4 else None,
        "cx": float(k[2]) if len(k) > 2 else None,
        "cy": float(k[5]) if len(k) > 5 else None,
        "frame_id": msg.header.frame_id,
        "width": int(msg.width),
        "height": int(msg.height),
    }


class Checkpoint(Node):
    def __init__(self):
        super().__init__("dsim_rgbd_checkpoint")
        be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.records = {key: [] for key in TOPICS}
        self._t0 = time.monotonic()
        for key, (topic, typ) in TOPICS.items():
            self.create_subscription(
                typ, topic, self._make_cb(key, typ), be)

    def _make_cb(self, key, typ):
        def cb(msg):
            now = time.monotonic() - self._t0
            rec = {
                "t": now,
                "stamp": _stamp_key(msg.header.stamp),
                "frame_id": msg.header.frame_id,
            }
            if typ is Image:
                rec.update({
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": msg.encoding,
                    "step": int(msg.step),
                    "data_len": len(msg.data),
                    "is_bigendian": int(msg.is_bigendian),
                })
                if msg.encoding in ("32FC1", "TYPE_32FC1") and msg.data:
                    import struct
                    rec["sample_f32"] = struct.unpack_from("<f", msg.data, 0)[0]
                if msg.encoding in ("16UC1", "mono16") and len(msg.data) >= 2:
                    rec["sample_u16"] = int.from_bytes(msg.data[:2], "little")
            elif typ is CameraInfo:
                rec.update(_k_from_info(msg))
            elif typ is PointCloud2:
                rec.update({
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "point_step": int(msg.point_step),
                })
            self.records[key].append(rec)
        return cb


def _rate(recs, t0, t1):
    n = sum(1 for r in recs if t0 <= r["t"] <= t1)
    dt = max(1e-6, t1 - t0)
    return n / dt, n


def _graph_dump():
    out = {}
    for cmd, key in (
        (["ros2", "topic", "list", "-t"], "topic_list"),
        (["ros2", "node", "list"], "node_list"),
    ):
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        out[key] = {
            "returncode": proc.returncode,
            "stdout": proc.stdout,
            "stderr": proc.stderr,
        }
    pc_topics = []
    for line in out["topic_list"]["stdout"].splitlines():
        if "PointCloud2" in line:
            pc_topics.append(line.strip())
    details = {}
    for line in pc_topics:
        topic = line.split()[0]
        info = subprocess.run(
            ["ros2", "topic", "info", "-v", topic],
            capture_output=True, text=True, timeout=20)
        details[topic] = info.stdout
    out["pointcloud2_topics"] = pc_topics
    out["pointcloud2_info"] = details
    clock = subprocess.run(
        ["ros2", "topic", "info", "/clock"],
        capture_output=True, text=True, timeout=15)
    out["clock_info"] = clock.stdout
    return out


def _identity_stats(records, window):
    t0, t1 = window
    groups = defaultdict(dict)
    for key in ("color", "depth_mm", "color_info", "depth_info"):
        for rec in records[key]:
            if not (t0 <= rec["t"] <= t1):
                continue
            groups[tuple(rec["stamp"])][key] = rec
    n = 0
    exact = 0
    for recs in groups.values():
        if "color" not in recs:
            continue
        n += 1
        need = ("color", "depth_mm", "color_info", "depth_info")
        if any(k not in recs for k in need):
            continue
        c = recs["color"]
        ok = True
        for k in need[1:]:
            r = recs[k]
            if r["stamp"] != c["stamp"]:
                ok = False
            if r.get("width") not in (None, c["width"]):
                ok = False
            if r.get("height") not in (None, c["height"]):
                ok = False
            if r["frame_id"] != c["frame_id"]:
                ok = False
        if ok:
            exact += 1
    return {
        "color_acquisitions": n,
        "four_product_exact": exact,
        "four_product_exact_ratio": (exact / n) if n else 0.0,
    }


def _k_error(sample):
    if not sample:
        return None
    err = {}
    for key, target in K_TARGET.items():
        val = sample.get(key)
        if val is None:
            err[key] = None
            continue
        err[key] = abs(val - target) / target
    return err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--warmup", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=30.0)
    args = ap.parse_args()
    rclpy.init()
    node = Checkpoint()
    end = args.warmup + args.duration + 2.0
    t_start = time.monotonic()
    while time.monotonic() - t_start < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    window = (args.warmup, args.warmup + args.duration)
    rates = {}
    for key, recs in node.records.items():
        hz, n = _rate(recs, *window)
        last = recs[-1] if recs else None
        rates[key] = {"hz": hz, "n": n, "total": len(recs), "last": last}
    graph = _graph_dump()
    identity = _identity_stats(node.records, window)
    sample_info = rates["depth_info"]["last"] or rates["color_info"]["last"]
    payload = {
        "warmup_s": args.warmup,
        "duration_s": args.duration,
        "window": list(window),
        "rates": rates,
        "identity": identity,
        "k_target": K_TARGET,
        "k_rel_error": _k_error(sample_info),
        "graph": graph,
        "counts": {k: len(v) for k, v in node.records.items()},
    }
    with open(args.out, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, default=str)
    node.destroy_node()
    rclpy.shutdown()
    color_hz = rates["color"]["hz"]
    depth_hz = rates["depth_mm"]["hz"]
    print(json.dumps({
        "color_hz": color_hz,
        "depth_mm_hz": depth_hz,
        "identity": identity,
        "k_rel_error": payload["k_rel_error"],
        "pc2": graph["pointcloud2_topics"],
        "out": args.out,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
