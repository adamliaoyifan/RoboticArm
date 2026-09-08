#!/usr/bin/env python3
"""Pure helpers for PF-R6 performance instrumentation."""

from __future__ import division


def pct(sorted_vals, percentile):
    if not sorted_vals:
        return None
    idx = int(round((float(percentile) / 100.0) * (len(sorted_vals) - 1)))
    return sorted_vals[min(len(sorted_vals) - 1, max(0, idx))]


def stats(values, digits=3):
    vals = sorted(float(v) for v in values)
    if not vals:
        return {"n": 0, "p50": None, "p95": None, "max": None}
    return {
        "n": len(vals),
        "p50": round(pct(vals, 50), digits),
        "p95": round(pct(vals, 95), digits),
        "max": round(vals[-1], digits),
    }


def active_rate(stamps, max_gap_sec, digits=3):
    if len(stamps) < 2:
        return None
    seq = sorted(float(t) for t in stamps)
    intervals = 0
    active_span = 0.0
    start = prev = seq[0]
    for t in seq[1:]:
        if t - prev > float(max_gap_sec):
            active_span += prev - start
            start = t
        else:
            intervals += 1
        prev = t
    active_span += prev - start
    if active_span <= 0.0:
        return None
    return round(float(intervals) / active_span, digits)


def stamp_delta_ms(stamps_by_topic, left, right):
    out = []
    left_stamps = stamps_by_topic.get(left, {})
    right_stamps = stamps_by_topic.get(right, {})
    for key, t0 in left_stamps.items():
        if key in right_stamps:
            out.append((float(right_stamps[key]) - float(t0)) * 1000.0)
    return out


def stamp_match_summary(stamps_by_topic, left, right):
    left_keys = set(stamps_by_topic.get(left, {}))
    right_keys = set(stamps_by_topic.get(right, {}))
    matched = left_keys & right_keys
    return {
        "left": left,
        "right": right,
        "left_count": len(left_keys),
        "right_count": len(right_keys),
        "matched": len(matched),
        "left_only": len(left_keys - right_keys),
        "right_only": len(right_keys - left_keys),
    }


def first_latency_summary(events_by_generation, digits=3):
    out = {}
    for generation, record in sorted(events_by_generation.items()):
        start = record.get("start")
        top = record.get("first_top_only")
        full = record.get("first_full_3d")
        out[str(generation)] = {
            "first_top_only_ms": (
                round((top - start) * 1000.0, digits)
                if start is not None and top is not None else None),
            "first_full_3d_ms": (
                round((full - start) * 1000.0, digits)
                if start is not None and full is not None else None),
        }
    return out
