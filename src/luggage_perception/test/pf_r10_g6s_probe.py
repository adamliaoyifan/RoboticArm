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
  /proc/<pid>/status at --rss-hz, each labelled with current-box generation,
  luggage size, and scored occurrence.

Writes `<out>/g6s_raw.json` (series) and `<out>/g6s_summary.json`
(threshold verdicts). Raw RSS first/last/min/max, Q1/Q4, and mixed-size
slope are diagnostics only. The C2 growth gate is the size-controlled
time coefficient ``beta`` on settled labelled samples. The C1 quantities
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

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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

CATALOG_SIZE_IDS = ("carryon", "standard", "large")
RSS_GROWTH_BETA_LIMIT = 2.0
MIN_OCCURRENCES_PER_SIZE = 2
MIN_SAMPLES_PER_OCCURRENCE = 10


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


def parse_box_dict(payload):
    """Return a dict from a ``/luggage/current_box`` JSON payload."""
    if not payload:
        return {}
    if isinstance(payload, dict):
        return payload if payload else {}
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def luggage_size_label(payload):
    """Catalog size id, or a rounded WxDxH string for a non-empty box."""
    data = parse_box_dict(payload)
    box_id = str(data.get("id") or data.get("model_name") or "")
    if not box_id:
        return ""
    lower = box_id.lower()
    for name in CATALOG_SIZE_IDS:
        if lower == name or lower.endswith("_" + name):
            return name
    width, depth, height = data.get("width"), data.get("depth"), data.get("height")
    if all(isinstance(v, (int, float)) for v in (width, depth, height)):
        return "%.3fx%.3fx%.3f" % (float(width), float(depth), float(height))
    return box_id


class CurrentBoxLabeler:
    """Track generation, size, and per-size scored occurrence index."""

    def __init__(self):
        self.generation = 0
        self.size = ""
        self.occurrence = 0
        self.occurrences = {}

    def update(self, payload):
        data = parse_box_dict(payload)
        try:
            generation = int(data.get("generation") or 0)
        except (TypeError, ValueError):
            generation = 0
        size = luggage_size_label(data)
        if generation != self.generation or size != self.size:
            self.generation = generation
            self.size = size
            if size:
                self.occurrences[size] = int(self.occurrences.get(size) or 0) + 1
                self.occurrence = self.occurrences[size]
            else:
                self.occurrence = 0
        return self.snapshot()

    def snapshot(self):
        return {
            "generation": int(self.generation),
            "size": str(self.size or ""),
            "occurrence": (int(self.occurrence) if self.size else None),
        }


def labelled_rss_row(stamp_sec, rss_mib, label):
    return {
        "t": float(stamp_sec),
        "rss_mib": float(rss_mib),
        "generation": int(label.get("generation") or 0),
        "size": str(label.get("size") or ""),
        "occurrence": label.get("occurrence"),
    }


def occurrence_coverage(rows):
    """Count labelled samples per (size, occurrence). Empty epochs omitted."""
    counts = {}
    for row in rows:
        size = row.get("size") or ""
        occ = row.get("occurrence")
        if not size or occ is None:
            continue
        key = (str(size), int(occ))
        counts[key] = counts.get(key, 0) + 1
    return counts


def coverage_scorable(rows, min_occurrences=MIN_OCCURRENCES_PER_SIZE,
                      min_samples=MIN_SAMPLES_PER_OCCURRENCE):
    counts = occurrence_coverage(rows)
    sizes = sorted({size for size, _occ in counts})
    if not sizes:
        return False, "no_labelled_size_samples", {}
    per_size = {}
    for size in sizes:
        qualified = [
            occ for (s, occ), n in counts.items()
            if s == size and n >= min_samples]
        per_size[size] = {
            "qualified_occurrences": len(qualified),
            "occurrences": sorted(
                occ for s, occ in counts if s == size),
        }
        if len(qualified) < min_occurrences:
            return False, "size_%s_needs_%d_occurrences_of_%d_samples" % (
                size, min_occurrences, min_samples), per_size
    return True, "", per_size


def warmup_cutoff_sec(rows):
    """First time every observed size has appeared at least once."""
    first = {}
    for row in rows:
        size = row.get("size") or ""
        if not size:
            continue
        t = float(row["t"])
        if size not in first or t < first[size]:
            first[size] = t
    if not first:
        return None
    return max(first.values())


def fit_samples(rows, occupancy_ready_t=None):
    """Settled labelled samples after every size has been exercised once."""
    cutoff = warmup_cutoff_sec(rows)
    if cutoff is None:
        return []
    if occupancy_ready_t is not None:
        cutoff = max(cutoff, float(occupancy_ready_t))
    out = []
    for row in rows:
        if (row.get("size")
                and row.get("occurrence") is not None
                and float(row["t"]) >= cutoff):
            out.append(row)
    return out


def fit_size_adjusted_beta(rows):
    """OLS: RSS = intercept + size FE + beta * elapsed_minutes."""
    if len(rows) < 4:
        return {
            "scorable": False,
            "reason": "too_few_fit_samples",
            "n_fit": len(rows),
            "beta_mib_per_min": None,
        }
    sizes = sorted({row["size"] for row in rows})
    t0 = float(rows[0]["t"])
    design = []
    observed = []
    for row in rows:
        line = [1.0]
        for size in sizes[1:]:
            line.append(1.0 if row["size"] == size else 0.0)
        line.append((float(row["t"]) - t0) / 60.0)
        design.append(line)
        observed.append(float(row["rss_mib"]))
    matrix = np.asarray(design, dtype=np.float64)
    y = np.asarray(observed, dtype=np.float64)
    coef, _resid, rank, _sv = np.linalg.lstsq(matrix, y, rcond=None)
    if int(rank) < matrix.shape[1]:
        return {
            "scorable": False,
            "reason": "rank_deficient_design",
            "n_fit": len(rows),
            "beta_mib_per_min": None,
            "rank": int(rank),
        }
    effects = {sizes[0]: 0.0}
    for i, size in enumerate(sizes[1:]):
        effects[size] = float(coef[1 + i])
    return {
        "scorable": True,
        "reason": "",
        "n_fit": len(rows),
        "beta_mib_per_min": float(coef[-1]),
        "intercept_mib": float(coef[0]),
        "size_effects_mib": effects,
        "sizes": sizes,
        "rank": int(rank),
    }


class G6SProbe(Node):
    def __init__(self, rss_hz):
        super().__init__("pf_r10_g6s_probe")
        self._rss_period = 1.0 / max(rss_hz, 0.1)
        self._events = defaultdict(list)        # topic -> receipt times
        self._stamps = defaultdict(dict)        # topic -> {key: receipt}
        self._rss = defaultdict(list)           # node -> labelled sample dicts
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
        self._box_labeler = CurrentBoxLabeler()

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

        box_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def current_box_cb(msg):
            self._box_labeler.update(msg.data)

        self.create_subscription(
            String, "/luggage/current_box", current_box_cb, box_qos)

    def sample_rss(self):
        label = self._box_labeler.snapshot()
        now = time.monotonic()
        for name, pid in self._pids.items():
            if pid is None:
                continue
            value = read_rss_mib(pid)
            if value is not None:
                self._rss[name].append(labelled_rss_row(now, value, label))

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


def rss_pairs(rows):
    pairs = []
    for row in rows:
        if isinstance(row, dict):
            pairs.append((float(row["t"]), float(row["rss_mib"])))
        else:
            pairs.append((float(row[0]), float(row[1])))
    return pairs


def occupancy_ready_t(probe):
    times = []
    for series in probe._occupancy.values():
        times.extend(t for t, _value in series)
    return min(times) if times else None


def rss_verdicts(probe):
    out = {}
    ready_t = occupancy_ready_t(probe)
    for name, raw_series in probe._rss.items():
        pairs = rss_pairs(raw_series)
        series = bucket_min_series(
            pairs, getattr(probe, "_rss_bucket_sec", 5.0))
        probe._rss_fit_series[name] = series
        if not pairs:
            out[name] = {"n": 0, "pid": probe._pids.get(name)}
            continue
        q1, q4 = time_quartiles(series if series else pairs)
        q1m, q4m = mean(q1), mean(q4)
        slope = lsq_slope(series if series else pairs)
        labelled = [row for row in raw_series if isinstance(row, dict)]
        coverage_ok, coverage_reason, per_size = coverage_scorable(labelled)
        settled = fit_samples(labelled, occupancy_ready_t=ready_t)
        adjusted = fit_size_adjusted_beta(settled) if settled else {
            "scorable": False,
            "reason": "no_settled_labelled_samples",
            "n_fit": 0,
            "beta_mib_per_min": None,
        }
        scorable = bool(coverage_ok and adjusted.get("scorable"))
        reason = coverage_reason or adjusted.get("reason") or ""
        beta = adjusted.get("beta_mib_per_min")
        beta_ok = bool(
            scorable and beta is not None and beta <= RSS_GROWTH_BETA_LIMIT)
        entry = {
            "pid": probe._pids.get(name),
            "n": len(series) if series else len(pairs),
            "n_labelled": len(labelled),
            "rss_mib_first": pairs[0][1],
            "rss_mib_last": pairs[-1][1],
            "rss_mib_min": min(v for _, v in pairs),
            "rss_mib_max": max(v for _, v in pairs),
            "q1_mean_mib": q1m,
            "q4_mean_mib": q4m,
            "slope_mib_per_min": slope,
            "raw_slope_is_diagnostic": True,
            "quartile_is_diagnostic": True,
            "coverage_ok": coverage_ok,
            "coverage_reason": coverage_reason,
            "coverage_per_size": per_size,
            "warmup_cutoff_sec": warmup_cutoff_sec(labelled),
            "occupancy_ready_sec": ready_t,
            "n_fit": adjusted.get("n_fit", 0),
            "beta_mib_per_min": beta,
            "beta_ok": beta_ok,
            "size_effects_mib": adjusted.get("size_effects_mib"),
            "intercept_mib": adjusted.get("intercept_mib"),
            "growth_scorable": scorable,
            "unscorable_reason": ("" if scorable else reason),
        }
        entry["pass"] = bool(beta_ok)
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
