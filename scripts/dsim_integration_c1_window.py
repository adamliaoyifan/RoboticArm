#!/usr/bin/env python3
"""DSI-C0/C1 scored window: RGB-D identity, joins, detector hits, independent Livox.

Does not launch Gazebo. Same 15 s warmup + 120 s window for camera and Livox.
Livox is recorded independently and is never exact-paired with RGB-D.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import time
from collections import defaultdict, deque

import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dsim_livox_monitor import decode_livox_msg  # noqa: E402
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String

from luggage_msgs.srv import SpawnNextBox
from luggage_perception.eval.dsim_c1_scoring import score_c1
from luggage_perception.eval.dsim_livox_metrics import summarize_scan, summarize_window


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _load_checkpoint():
    path = os.path.join(ROOT, "scripts", "dsim_rgbd_checkpoint.py")
    spec = importlib.util.spec_from_file_location("dsim_rgbd_checkpoint", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _stamp_key(stamp):
    return (int(stamp.sec), int(stamp.nanosec))


class C1Window(Node):
    def __init__(self):
        super().__init__("dsim_integration_c1_window")
        be = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        rel = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST)
        tl = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._t0 = time.monotonic()
        self.records = defaultdict(list)
        self.raw_rgb = {}
        self.pre_rgb = {}
        self.pre_depth = {}
        self.prep_first = None
        self.prep_last = None
        self.filter_first = None
        self.filter_last = None
        self.emit_status_total = 0
        self.emit_status_depth_ok = 0
        self.detector_stats = []
        self.livox_scans = []
        self.image_ring = deque(maxlen=60)
        self._window_start = 0.0
        self._window_end = float("inf")
        groups = [MutuallyExclusiveCallbackGroup() for _ in range(8)]
        ck = _load_checkpoint()
        self._ck = ck
        for key, (topic, typ) in ck.TOPICS.items():
            if typ is PointCloud2:
                continue
            qos = rel if key in ("depth_mm", "pre_depth") else be
            self.create_subscription(
                typ, topic, self._make_rec(key, typ), qos,
                callback_group=groups[0])
        self.create_subscription(
            PointCloud2, "/livox/lidar", self._on_livox, be,
            callback_group=groups[1])
        self.create_subscription(
            Image, "/camera/color/image_raw",
            self._on_stamp("raw_rgb"), be, callback_group=groups[2])
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/color/image",
            self._on_stamp("pre_rgb"), be, callback_group=groups[3])
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/depth/image",
            self._on_stamp("pre_depth"), rel, callback_group=groups[4])
        self.create_subscription(
            String, "/luggage/preprocessed/status",
            self._prep_status, tl, callback_group=groups[5])
        self.create_subscription(
            String, "/semantic_point_filter/stats_json",
            self._filter_status, tl, callback_group=groups[6])
        self.create_subscription(
            String, "/luggage_detector/stream_stats_json",
            self._detector_status, tl, callback_group=groups[7])
        self._spawn = self.create_client(
            SpawnNextBox, "/pickup_box_spawner/spawn_next_box")

    def now(self):
        return time.monotonic() - self._t0

    def _make_rec(self, key, typ):
        def cb(msg):
            rec = {
                "t": self.now(),
                "stamp": _stamp_key(msg.header.stamp),
                "frame_id": msg.header.frame_id,
            }
            if typ is Image:
                rec.update({
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": msg.encoding,
                })
                if key in ("color", "depth_mm"):
                    self.image_ring.append({
                        "key": key,
                        "t": rec["t"],
                        "stamp": rec["stamp"],
                        "width": rec["width"],
                        "height": rec["height"],
                        "encoding": msg.encoding,
                        "data": bytes(msg.data[: min(len(msg.data), 16)]),
                        "data_len": len(msg.data),
                    })
            elif typ is CameraInfo:
                rec.update(self._ck._k_from_info(msg))
            self.records[key].append(rec)
        return cb

    def _on_stamp(self, key):
        def cb(msg):
            stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            getattr(self, key).setdefault(stamp, time.monotonic())
        return cb

    def _on_livox(self, msg):
        xyz, intensity, n = decode_livox_msg(msg)
        rec = summarize_scan(
            xyz, intensity=intensity, frame_id=msg.header.frame_id, n_raw=n)
        rec["t"] = self.now()
        rec["stamp"] = _stamp_key(msg.header.stamp)
        rec["width"] = int(msg.width)
        rec["height"] = int(msg.height)
        self.livox_scans.append(rec)

    def _prep_status(self, msg):
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
            if self._window_start <= now <= self._window_end:
                self.emit_status_total += 1
                self.emit_status_depth_ok += int(bool(flags["depth_ok"]))

    def _filter_status(self, msg):
        try:
            rec = json.loads(msg.data)
        except ValueError:
            return
        rec["_wall"] = time.monotonic()
        if self.filter_first is None:
            self.filter_first = rec
        self.filter_last = rec

    def _detector_status(self, msg):
        try:
            rec = json.loads(msg.data)
        except ValueError:
            return
        rec["_wall"] = time.monotonic()
        if self._window_start <= rec["_wall"] <= self._window_end:
            self.detector_stats.append(rec)

    def spawn_once(self, timeout=20.0):
        deadline = time.monotonic() + timeout
        while not self._spawn.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > deadline:
                return {"ok": False, "reason": "no_spawn_service"}
        future = self._spawn.call_async(SpawnNextBox.Request())
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return {"ok": False, "reason": "spawn_timeout"}
        resp = future.result()
        return {
            "ok": bool(resp.success),
            "reason": "" if resp.success else str(resp.message),
            "box_id": str(getattr(resp.box, "id", "") or ""),
        }


def _in_win(mapping, start, end):
    return {k: v for k, v in mapping.items() if start <= v <= end}


def _pct(values, q):
    if not values:
        return None
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(np.ceil(q * len(ordered))) - 1))
    return ordered[idx]


def build_d34(node, start, end):
    raw = _in_win(node.raw_rgb, start, end)
    pre = _in_win(node.pre_rgb, start, end)
    pre_d = _in_win(node.pre_depth, start, end)
    lags = [max(0.0, t - node.raw_rgb[s]) for s, t in pre.items()
            if s in node.raw_rgb]
    out = {
        "raw_rgb_unique": len(raw),
        "pre_rgb_unique": len(pre),
        "pre_depth_unique": len(pre_d),
        "d3_emission_over_rgb": (len(pre) / len(raw)) if raw else None,
        "d3_raw_to_pre_rgb_sec": {
            "n": len(lags), "p50": _pct(lags, 0.5),
            "p95": _pct(lags, 0.95), "max": _pct(lags, 1.0)},
        "d4_paired_depth_over_emitted": (
            node.emit_status_depth_ok / node.emit_status_total)
            if node.emit_status_total else None,
        "d4_emit_status_total": node.emit_status_total,
        "d4_probe_topic_crosscheck": (
            len(set(pre_d) & set(pre)) / len(pre)) if pre else None,
    }
    p0, p1 = node.prep_first, node.prep_last
    if p0 and p1:
        keys = ("depth_wait_skips", "info_wait_skips",
                "acquisition_mismatches", "exact_lookup_misses",
                "payload_materialisations", "rollback_events",
                "emit_queue_drops")
        out["prep_counters"] = {
            k: (p1.get(k, 0) - p0.get(k, 0)) for k in keys}
    f0, f1 = node.filter_first, node.filter_last
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
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--warmup", type=float, default=15.0)
    ap.add_argument("--duration", type=float, default=120.0)
    ap.add_argument("--spawn", action="store_true", default=True)
    ap.add_argument("--no-spawn", dest="spawn", action="store_false")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rclpy.init()
    node = C1Window()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    spawn_rec = {"ok": True, "reason": "skipped"}
    if args.spawn:
        # Drive the executor so the spawn response can arrive.
        deadline = time.monotonic() + 25.0
        spawn_rec = {"ok": False, "reason": "spawn_pending"}
        while time.monotonic() < deadline and not node._spawn.service_is_ready():
            ex.spin_once(timeout_sec=0.1)
        future = node._spawn.call_async(SpawnNextBox.Request())
        while time.monotonic() < deadline and not future.done():
            ex.spin_once(timeout_sec=0.1)
        if future.done():
            resp = future.result()
            spawn_rec = {
                "ok": bool(resp.success),
                "reason": "" if resp.success else str(resp.message),
                "box_id": str(getattr(resp.box, "id", "") or ""),
            }
        else:
            spawn_rec = {"ok": False, "reason": "spawn_timeout"}

    t0 = time.monotonic()
    window_start = t0 + args.warmup
    window_end = t0 + args.warmup + args.duration
    node._window_start = window_start
    node._window_end = window_end
    while time.monotonic() < window_end:
        ex.spin_once(timeout_sec=0.1)

    ck = node._ck
    local_t0 = args.warmup
    local_t1 = args.warmup + args.duration
    window = (local_t0, local_t1)
    rates = {}
    for key, recs in node.records.items():
        hz, n = ck._rate(recs, *window)
        last = recs[-1] if recs else None
        rates[key] = {"hz": hz, "n": n, "total": len(recs), "last": last}
    graph = ck._graph_dump()
    identity = ck._identity_stats(node.records, window)
    d34 = build_d34(node, window_start, window_end)
    livox_window = [s for s in node.livox_scans if local_t0 <= s["t"] <= local_t1]
    # Keep Livox payloads bounded: drop bulky per-scan arrays.
    livox_slim = []
    for rec in livox_window:
        slim = dict(rec)
        slim.pop("nn_spacing_m", None)
        livox_slim.append({
            "t": slim["t"],
            "stamp": slim["stamp"],
            "frame_id": slim.get("frame_id"),
            "n_raw": slim.get("n_raw"),
            "finite_ratio": slim.get("finite_ratio"),
            "range": slim.get("range"),
            "dominant_plane": slim.get("dominant_plane"),
            "intensity": slim.get("intensity"),
            "deskewed": slim.get("deskewed"),
            "configured_grid": slim.get("configured_grid"),
            "width": slim.get("width"),
            "height": slim.get("height"),
        })
    payload = {
        "warmup_s": args.warmup,
        "duration_s": args.duration,
        "spawn": spawn_rec,
        "rates": rates,
        "identity": identity,
        "d34": d34,
        "detector_stream_stats": [
            {
                "stamp": rec.get("stamp"),
                "raw_lookup": rec.get("raw_lookup"),
                "raw_counts": rec.get("raw_counts"),
                "n_cargo_points": rec.get("n_cargo_points"),
                "support_valid": rec.get("support_valid"),
            }
            for rec in node.detector_stats
        ],
        "pointcloud2_topics": graph.get("pointcloud2_topics") or [],
        "clock_info": graph.get("clock_info") or "",
        "graph": {
            "node_list": (graph.get("node_list") or {}).get("stdout"),
            "topic_list": (graph.get("topic_list") or {}).get("stdout"),
            "pointcloud2_topics": graph.get("pointcloud2_topics"),
            "clock_info": graph.get("clock_info"),
        },
    }
    verdict = score_c1(payload)
    livox_summary = summarize_window(
        livox_window, [s["t"] for s in livox_window])
    t2 = None
    if not verdict["pass"]:
        t2_dir = os.path.join(args.out, "fail_t2")
        os.makedirs(t2_dir, exist_ok=True)
        ring = list(node.image_ring)
        latest_t = ring[-1]["t"] if ring else 0.0
        kept = [r for r in ring if r["t"] >= latest_t - 2.0]
        t2_path = os.path.join(t2_dir, "rgbd_ring.json")
        with open(t2_path, "w", encoding="utf-8") as handle:
            json.dump(kept, handle, indent=1)
        t2 = {"rgbd_ring": t2_path, "n": len(kept)}
        if livox_window:
            last = dict(livox_window[-1])
            last.pop("nn_spacing_m", None)
            livox_fail = os.path.join(t2_dir, "livox_last_scan.json")
            with open(livox_fail, "w", encoding="utf-8") as handle:
                json.dump(last, handle, indent=1)
    out = {
        "c1_verdict": verdict,
        "livox": livox_summary,
        "livox_scans_n": len(livox_window),
        "t2": t2,
        "payload_identity": identity,
        "d34": d34,
        "spawn": spawn_rec,
        "rates": {k: {"hz": v["hz"], "n": v["n"]} for k, v in rates.items()},
    }
    with open(os.path.join(args.out, "c1_verdict.json"), "w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=2, default=str)
    with open(os.path.join(args.out, "c1_payload.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=1, default=str)
    with open(os.path.join(args.out, "c0_graph.json"), "w", encoding="utf-8") as handle:
        json.dump(payload["graph"], handle, indent=1, default=str)
    with open(os.path.join(args.out, "livox_window.json"), "w", encoding="utf-8") as handle:
        json.dump({"summary": livox_summary, "n": len(livox_slim)}, handle, indent=2)
    print(json.dumps({
        "c1_pass": verdict["pass"],
        "failures": verdict["failures"],
        "livox": livox_summary,
        "out": args.out,
    }, indent=2, default=str))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
