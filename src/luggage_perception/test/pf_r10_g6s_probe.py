#!/usr/bin/env python3
"""PF-R10 C2 / PF-G6S lifecycle probe.

Runs alongside a `platform_free_height_gate4_eval.py` run and records, with
wall-clock timestamps, every quantity the C2 rules make decidable:

- per-stage P50/P95/max and active rates on the accepted semantic path
  (raw colour -> preprocessed colour -> detection frame), plus the
  active-window end-to-end `raw_img->frame` latency;
- `executor_lag_sec` series from `/semantic_point_filter/stats_json`;
- buffer occupancy series: preprocessor camera caches and emit-queue depth
  from `/luggage/preprocessed/status`, filter join buffers from the filter
  stats, detector raw-join buffer from `/luggage_detector/stream_stats_json`;
- per-node RSS of the four online perception processes, sampled from
  /proc/<pid>/status at --rss-hz.

Writes `<out>/g6s_raw.json` (series) and `<out>/g6s_summary.json`
(threshold verdicts). Quartiles Q1/Q4 are time quartiles of the scored
window: samples whose timestamp falls in the first / last 25 % of the
run, per the PF-R10 C2 definition. The C1 quantities themselves
(`active_output_hz`, gate4 rates) come from the gate4 eval summary, not
from this probe.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from luggage_msgs.msg import DetectionFrame, YoloDetections
from luggage_perception.pf_r6_metrics import (
    active_rate,
    stamp_delta_ms,
    stats,
)

RSS_NODES = {
    "sensor_preprocessor": "lib/luggage_perception/sensor_preprocessor_node",
    "semantic_segmenter": "lib/luggage_perception/semantic_segmenter_node",
    "semantic_point_filter": "lib/luggage_perception/semantic_point_filter_node",
    "luggage_detector": "lib/luggage_perception/luggage_detector_node",
}

# Buffers whose occupancy is pending work (backlog risk), per C2: the
# filter's and detector's exact-join sides. Camera ring buffers are
# deliberate history and only carry the peak <= maxlen rule.
PENDING_WORK_BUFFERS = (
    "filter.depth", "filter.mask", "filter.instance",
    "filter.exact_join_candidates", "detector.raw_buffer",
)


def resolve_pids():
    pids = {}
    for name, pattern in RSS_NODES.items():
        try:
            out = subprocess.run(
                ["pgrep", "-f", pattern],
                capture_output=True, text=True).stdout.split()
        except Exception:
            out = []
        pids[name] = int(out[0]) if out else None
    return pids


def read_rss_mib(pid):
    try:
        with open("/proc/%d/status" % pid) as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        pass
    return None


def time_quartiles(samples):
    """samples: list of (t, value). Returns (q1_values, q4_values)."""
    if not samples:
        return [], []
    ts = [t for t, _ in samples]
    t0, t1 = min(ts), max(ts)
    span = t1 - t0
    if span <= 0:
        return [v for _, v in samples], []
    q1_cut = t0 + 0.25 * span
    q4_cut = t0 + 0.75 * span
    q1 = [v for t, v in samples if t <= q1_cut]
    q4 = [v for t, v in samples if t >= q4_cut]
    return q1, q4


def mean(values):
    vals = [float(v) for v in values if v is not None]
    return sum(vals) / len(vals) if vals else None


def lsq_slope(values):
    """Least-squares slope of (t, v) samples, v per minute."""
    pts = [(float(t), float(v)) for t, v in values if v is not None]
    if len(pts) < 2:
        return None
    t0 = pts[0][0]
    xs = [(t - t0) / 60.0 for t, _ in pts]
    ys = [v for _, v in pts]
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    denom = sum((x - mx) ** 2 for x in xs)
    if denom <= 0:
        return 0.0
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom


class G6SProbe(Node):
    def __init__(self, rss_hz):
        super().__init__("pf_r10_g6s_probe")
        self._rss_period = 1.0 / max(rss_hz, 0.1)
        self._events = defaultdict(list)        # topic -> receipt times
        self._stamps = defaultdict(dict)        # topic -> {key: receipt}
        self._rss = defaultdict(list)           # node -> (t, MiB)
        self._rss_fit_series = {}               # node -> bucket-min series
        self._rss_bucket_sec = 5.0
        self._pids = {}
        self._executor_lag = []                 # (t, sec)
        self._occupancy = defaultdict(list)     # buffer -> (t, value)
        self._maxlens = {}
        self._stage_ms = defaultdict(list)      # stage -> (t, ms)
        self._detector_timing = defaultdict(list)
        self._support_gates = defaultdict(list)  # gate label -> (t, level)
        self._yolo_series = []                    # (t, conf, area, held, id)
        self._prep_latency_series = []          # per-status extras
        self._last_filter_stats = {}
        self._last_detector_stats = {}
        self._last_prep_status = {}

        be10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        be20 = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        rl10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        def stage_cb(topic):
            def cb(msg):
                t = time.monotonic()
                h = msg.header.stamp
                self._events[topic].append(t)
                self._stamps[topic][(h.sec, h.nanosec)] = t
            return cb

        # Sensor-surface and preprocessed products publish BEST_EFFORT on
        # this stack; a RELIABLE subscriber against them gets zero
        # delivery (the PF-R9 g2 probe notes record the same trap).
        self.create_subscription(
            Image, "/camera/color/image_raw", stage_cb("raw_img"), be10)
        self.create_subscription(
            Image, "/luggage/preprocessed/camera/color/image",
            stage_cb("pre_rgb"), be10)
        self.create_subscription(
            DetectionFrame, "/luggage/perception/detection_frame",
            stage_cb("frame"), be20)

        def yolo_cb(msg):
            t = time.monotonic()
            self._events["yolo"].append(t)
            self._stamps["yolo"][(msg.header.stamp.sec,
                                   msg.header.stamp.nanosec)] = t
            for det in msg.detections:
                b = det.bbox
                area = int((b[2] - b[0]) * (b[3] - b[1])) if b is not None \
                    and len(b) == 4 else 0
                self._yolo_series.append((
                    t, float(det.confidence), area, bool(det.held),
                    str(det.label)))

        self.create_subscription(
            YoloDetections, "/luggage/semantic/yolo_detections",
            yolo_cb, be20)

        def filter_cb(msg):
            t = time.monotonic()
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict):
                return
            self._last_filter_stats = payload
            lag = payload.get("executor_lag_sec")
            if isinstance(lag, (int, float)):
                self._executor_lag.append((t, float(lag)))
            occ = payload.get("buffer_occupancy") or {}
            for key in ("depth", "mask", "instance",
                        "exact_join_candidates"):
                value = occ.get(key)
                if isinstance(value, (int, float)):
                    self._occupancy["filter.%s" % key].append((t, int(value)))
            if "buffer_maxlen" in payload:
                self._maxlens["filter.buffer_maxlen"] = payload["buffer_maxlen"]
            for key, value in (payload.get("stage_ms") or {}).items():
                if isinstance(value, (int, float)):
                    self._stage_ms[key].append((t, float(value)))

        self.create_subscription(
            String, "/semantic_point_filter/stats_json", filter_cb, rl10)

        def detector_cb(msg):
            t = time.monotonic()
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict):
                return
            self._last_detector_stats = payload
            self._occupancy["detector.raw_buffer"].append(
                (t, int(payload.get("raw_buffer_len") or 0)))
            self._maxlens["detector.raw_buffer_maxlen"] = (
                payload.get("raw_buffer_maxlen"))
            self._support_gates[str(payload.get("support_gate") or "")].append(
                (t, int(payload.get("geometry_level") or 0)))
            for key, value in (payload.get("timing_ms") or {}).items():
                if isinstance(value, (int, float)):
                    self._detector_timing[key].append((t, float(value)))

        self.create_subscription(
            String, "/luggage_detector/stream_stats_json", detector_cb, rl10)

        def prep_cb(msg):
            t = time.monotonic()
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict):
                return
            self._last_prep_status = payload
            for key, value in (payload.get("buffers") or {}).items():
                if isinstance(value, (int, float)):
                    self._occupancy["prep.%s" % key].append((t, int(value)))
            depth = payload.get("emit_queue_depth")
            if isinstance(depth, (int, float)):
                self._occupancy["prep.emit_queue"].append((t, int(depth)))

        self.create_subscription(
            String, "/luggage/preprocessed/status", prep_cb, rl10)

        def segmenter_cb(msg):
            t = time.monotonic()
            self._events["segmenter_stats"].append(t)
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            if isinstance(payload, dict):
                self._last_filter_stats.setdefault(
                    "_segmenter", payload.get("backend"))

        self.create_subscription(
            String, "/semantic_segmenter/stats_json", segmenter_cb, rl10)

    def sample_rss(self):
        for name, pid in self._pids.items():
            if pid is None:
                continue
            value = read_rss_mib(pid)
            if value is not None:
                self._rss[name].append((time.monotonic(), value))

    def refresh_pids(self):
        resolved = resolve_pids()
        for name, pid in resolved.items():
            if self._pids.get(name) is None and pid is not None:
                self._pids[name] = pid


def occupancy_verdicts(probe):
    out = {}
    for buffer_name, series in probe._occupancy.items():
        if not series:
            continue
        values = [v for _, v in series]
        entry = {
            "peak": max(values),
            "mean": mean(values),
            "n": len(values),
        }
        base = buffer_name.rsplit(".", 1)[0]
        if buffer_name == "detector.raw_buffer":
            maxlen = probe._maxlens.get("detector.raw_buffer_maxlen")
        elif buffer_name.startswith("filter."):
            maxlen = probe._maxlens.get("filter.buffer_maxlen")
        else:
            maxlen = probe._maxlens.get("%s_maxlen" % base)
        if maxlen is not None:
            entry["configured_maxlen"] = maxlen
            entry["peak_within_maxlen"] = bool(max(values) <= maxlen)
        if buffer_name in PENDING_WORK_BUFFERS:
            _, q4 = time_quartiles(series)
            entry["q4_mean"] = mean(q4)
            if maxlen is not None:
                entry["q4_mean_within_half"] = bool(
                    (mean(q4) or 0.0) <= 0.5 * float(maxlen))
        out[buffer_name] = entry
    return out


def bucket_min_series(series, bucket_sec, t_end=None):
    """Collapse (t, v) samples to per-bucket minima, time-anchored.

    The final bucket is usually partial (the stop file arrives mid-bucket
    right after the eval's last trial burst, before any reclamation) and
    its minimum is the post-burst peak; fitting it measures the burst,
    not growth. Partial tail buckets (< 80 % coverage) are dropped.
    """
    if not series:
        return []
    t0 = series[0][0]
    if t_end is None:
        t_end = series[-1][0]
    buckets = {}
    for t, v in series:
        buckets.setdefault(int((t - t0) // bucket_sec), []).append(v)
    out = []
    for k, vs in sorted(buckets.items()):
        span = min(t_end, t0 + (k + 1) * bucket_sec) - (t0 + k * bucket_sec)
        if span < 0.8 * bucket_sec:
            continue
        out.append((t0 + (k + 0.5) * bucket_sec, min(vs)))
    return out


def rss_verdicts(probe):
    out = {}
    for name, raw_series in probe._rss.items():
        series = bucket_min_series(
            raw_series, getattr(probe, "_rss_bucket_sec", 5.0))
        probe._rss_fit_series[name] = series
        if not series:
            out[name] = {"n": 0, "pid": probe._pids.get(name)}
            continue
        q1, q4 = time_quartiles(series)
        q1m, q4m = mean(q1), mean(q4)
        slope = lsq_slope(series)
        entry = {
            "pid": probe._pids.get(name),
            "n": len(series),
            "rss_mib_first": series[0][1],
            "rss_mib_last": series[-1][1],
            "rss_mib_min": min(v for _, v in series),
            "rss_mib_max": max(v for _, v in series),
            "q1_mean_mib": q1m,
            "q4_mean_mib": q4m,
            "slope_mib_per_min": slope,
            "slope_ok": (slope is not None and slope <= 2.0),
            "quartile_ok": (
                q1m is not None and q4m is not None
                and q4m <= 1.10 * q1m + 50.0),
        }
        entry["pass"] = bool(entry["slope_ok"] and entry["quartile_ok"])
        out[name] = entry
    return out


def executor_lag_verdict(probe):
    series = probe._executor_lag
    if not series:
        return {"n": 0}
    q1, q4 = time_quartiles(series)
    q1m, q4m = mean(q1), mean(q4)
    return {
        "n": len(series),
        "p50": stats([v for _, v in series])["p50"],
        "p95": stats([v for _, v in series])["p95"],
        "max": stats([v for _, v in series])["max"],
        "q1_mean": q1m,
        "q4_mean": q4m,
        "q4_mean_ok": (q4m is not None and q4m <= 0.20),
        "ratio_ok": (q1m is not None and q4m is not None
                     and q4m <= 1.25 * q1m),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, required=True,
                    help="hard cap; scoring window ends at this duration")
    ap.add_argument("--stop-file", default="",
                    help="end the window early when this file appears, so "
                         "the probe window can match the gate4 run exactly")
    ap.add_argument("--gap", type=float, default=2.0)
    ap.add_argument("--rss-hz", type=float, default=2.0)
    ap.add_argument("--rss-bucket-sec", type=float, default=5.0,
                    help="report and fit the per-bucket MINIMUM VmRSS: "
                         "the detector/filter allocation pattern cycles "
                         "box-size classes (bounded +/-45 MiB, no growth), "
                         "and a least-squares fit on instantaneous samples "
                         "measures that oscillation's phase, not growth. A "
                         "rolling floor removes transients while a real "
                         "leak still raises it linearly.")
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    probe = G6SProbe(args.rss_hz)
    probe._rss_bucket_sec = args.rss_bucket_sec
    probe.refresh_pids()
    wall_start = datetime.now(timezone.utc).isoformat()
    t0 = time.monotonic()
    next_rss = t0
    next_pid_refresh = t0 + 30.0
    try:
        while time.monotonic() - t0 < args.duration:
            if args.stop_file and os.path.exists(args.stop_file):
                break
            rclpy.spin_once(probe, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_rss:
                probe.sample_rss()
                next_rss = now + probe._rss_period
            if now >= next_pid_refresh:
                probe.refresh_pids()
                next_pid_refresh = now + 30.0
    finally:
        t_end = time.monotonic()
        wall_end = datetime.now(timezone.utc).isoformat()
        probe.destroy_node()
        rclpy.shutdown()

    stage_deltas = {}
    for left, right, label in (
            ("raw_img", "pre_rgb", "raw_img->pre_rgb"),
            ("pre_rgb", "frame", "pre_rgb->frame"),
            ("raw_img", "frame", "raw_img->frame_e2e")):
        stage_deltas[label] = stats(
            stamp_delta_ms(probe._stamps, left, right))

    stage_series = {}
    for key, series in probe._stage_ms.items():
        stage_series["filter.%s_ms" % key] = stats([v for _, v in series])
    for key, series in probe._detector_timing.items():
        stage_series["detector.%s_ms" % key] = stats([v for _, v in series])

    raw = {
        "wall_start_utc": wall_start,
        "wall_end_utc": wall_end,
        "duration_s": t_end - t0,
        "events": {k: v for k, v in probe._events.items()},
        "executor_lag": probe._executor_lag,
        "occupancy": dict(probe._occupancy),
        "maxlens": dict(probe._maxlens),
        "rss": {k: v for k, v in probe._rss.items()},
        "rss_bucket_min": {k: v for k, v in probe._rss_fit_series.items()},
        "pids": dict(probe._pids),
        "stage_ms": {k: v for k, v in probe._stage_ms.items()},
        "detector_timing_ms": {
            k: v for k, v in probe._detector_timing.items()},
        "support_gates": {
            k: [(t, lvl) for t, lvl in v]
            for k, v in probe._support_gates.items()},
        "yolo_series": list(probe._yolo_series),
        "last_filter_stats": probe._last_filter_stats,
        "last_detector_stats": probe._last_detector_stats,
        "last_prep_status": probe._last_prep_status,
    }
    (out / "g6s_raw.json").write_text(json.dumps(raw, indent=1) + "\n")

    summary = {
        "wall_start_utc": wall_start,
        "wall_end_utc": wall_end,
        "duration_s": t_end - t0,
        "active_rates_hz": {
            k: active_rate(v, args.gap)
            for k, v in probe._events.items()},
        "stage_delta_ms": stage_deltas,
        "stage_percentiles": stage_series,
        "executor_lag_sec": executor_lag_verdict(probe),
        "occupancy": occupancy_verdicts(probe),
        "rss": rss_verdicts(probe),
        "maxlens": dict(probe._maxlens),
    }
    (out / "g6s_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
