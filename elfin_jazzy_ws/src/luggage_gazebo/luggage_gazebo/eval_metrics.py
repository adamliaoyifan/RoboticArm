#!/usr/bin/env python3
"""Aggregate pick/retreat eval trials. Numpy/stdlib only, no ROS.

Three independent rates (docs/plans/closed_loop_eval_driver.md):

  detect_pass_rate  — DetectionAccuracy.ok among trials that compared
  plan_pass_rate    — four PlanMotion successes among detect-pass trials
  retreat_pass_rate — retreat height check among plan-pass trials

Pick accuracy (geometric, no vacuum): suction XY/Z vs the measured box
after attach. Stability is the sample std of those errors among trials
where attach ran.
"""

from __future__ import division

import math
from collections import Counter
from dataclasses import asdict, dataclass, field, fields
from typing import Optional, Tuple


def _pct(values, q):
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    idx = (len(ordered) - 1) * (float(q) / 100.0)
    lo = int(math.floor(idx))
    hi = int(math.ceil(idx))
    if lo == hi:
        return ordered[lo]
    frac = idx - lo
    return ordered[lo] * (1.0 - frac) + ordered[hi] * frac


def _mean(values):
    if not values:
        return None
    return sum(values) / float(len(values))


def _std(values):
    if len(values) < 2:
        return 0.0 if values else None
    mean = _mean(values)
    var = sum((v - mean) ** 2 for v in values) / float(len(values) - 1)
    return math.sqrt(var)


@dataclass(frozen=True)
class TrialRecord:
    index: int
    catalog_id: str = ""
    visual_id: str = ""
    detect_failure: str = ""
    accuracy_ok: Optional[bool] = None
    accuracy_reason: str = ""
    err_xy: Optional[float] = None
    err_z: Optional[float] = None
    err_width: Optional[float] = None
    err_depth: Optional[float] = None
    err_height: Optional[float] = None
    iou: Optional[float] = None
    segments_planned: int = 0
    segments_succeeded: int = 0
    segment_failures: Tuple[Tuple[str, str], ...] = ()
    attach_xy_err: Optional[float] = None
    attach_z_err: Optional[float] = None
    attach_xy_gt: Optional[float] = None
    attach_z_gt: Optional[float] = None
    retreat_delta_z: Optional[float] = None
    retreat_ok: Optional[bool] = None
    wall_time_sec: float = 0.0
    fail_code: str = ""
    extras: dict = field(default_factory=dict)

    def detect_compared(self):
        return self.accuracy_ok is not None

    def detect_passed(self):
        return bool(self.accuracy_ok)

    def plan_passed(self):
        return (
            self.detect_passed()
            and self.segments_planned >= 4
            and self.segments_succeeded >= self.segments_planned
            and not self.segment_failures
        )

    def retreat_passed(self):
        return self.plan_passed() and bool(self.retreat_ok)


def summarize(records, expected_retreat_dz=0.35):
    """Fold TrialRecords into the three rates plus pick accuracy/stability."""
    records = list(records or [])
    n = len(records)
    compared = [r for r in records if r.detect_compared()]
    detect_ok = [r for r in compared if r.detect_passed()]
    plan_ok = [r for r in detect_ok if r.plan_passed()]
    retreat_ok = [r for r in plan_ok if r.retreat_passed()]
    attach_xy = [r.attach_xy_err for r in records if r.attach_xy_err is not None]
    attach_z = [abs(r.attach_z_err) for r in records if r.attach_z_err is not None]
    attach_xy_gt = [r.attach_xy_gt for r in records if r.attach_xy_gt is not None]
    attach_z_gt = [abs(r.attach_z_gt) for r in records if r.attach_z_gt is not None]
    retreat_dz = [r.retreat_delta_z for r in records if r.retreat_delta_z is not None]
    fail_codes = Counter(r.fail_code for r in records if r.fail_code)
    seg_fails = Counter()
    for rec in records:
        for name, _msg in rec.segment_failures:
            seg_fails[name] += 1
    by_catalog = {}
    for rec in records:
        key = rec.catalog_id or "unknown"
        slot = by_catalog.setdefault(key, {"n": 0, "detect_ok": 0, "plan_ok": 0, "retreat_ok": 0})
        slot["n"] += 1
        if rec.detect_passed():
            slot["detect_ok"] += 1
        if rec.plan_passed():
            slot["plan_ok"] += 1
        if rec.retreat_passed():
            slot["retreat_ok"] += 1
    by_visual = {}
    for rec in records:
        key = rec.visual_id or "unknown"
        slot = by_visual.setdefault(key, {"n": 0, "detect_ok": 0, "plan_ok": 0, "retreat_ok": 0})
        slot["n"] += 1
        if rec.detect_passed():
            slot["detect_ok"] += 1
        if rec.plan_passed():
            slot["plan_ok"] += 1
        if rec.retreat_passed():
            slot["retreat_ok"] += 1

    def _rate(num, den):
        return (float(num) / float(den)) if den else 0.0

    return {
        "n": n,
        "expected_retreat_dz": float(expected_retreat_dz),
        "detect_pass_rate": _rate(len(detect_ok), len(compared)),
        "n_detect_compared": len(compared),
        "plan_pass_rate": _rate(len(plan_ok), len(detect_ok)),
        "n_detect_ok": len(detect_ok),
        "retreat_pass_rate": _rate(len(retreat_ok), len(plan_ok)),
        "n_plan_ok": len(plan_ok),
        "n_retreat_ok": len(retreat_ok),
        "pick_pass_rate": _rate(len(retreat_ok), n),
        "fail_codes": dict(fail_codes),
        "segment_failures": dict(seg_fails),
        "by_catalog": by_catalog,
        "by_visual": by_visual,
        "attach_xy": {
            "n": len(attach_xy),
            "mean": _mean(attach_xy),
            "std": _std(attach_xy),
            "p50": _pct(attach_xy, 50),
            "p95": _pct(attach_xy, 95),
        },
        "attach_z_abs": {
            "n": len(attach_z),
            "mean": _mean(attach_z),
            "std": _std(attach_z),
            "p50": _pct(attach_z, 50),
            "p95": _pct(attach_z, 95),
        },
        "attach_xy_gt": {
            "n": len(attach_xy_gt),
            "mean": _mean(attach_xy_gt),
            "std": _std(attach_xy_gt),
            "p50": _pct(attach_xy_gt, 50),
            "p95": _pct(attach_xy_gt, 95),
        },
        "attach_z_gt_abs": {
            "n": len(attach_z_gt),
            "mean": _mean(attach_z_gt),
            "std": _std(attach_z_gt),
            "p50": _pct(attach_z_gt, 50),
            "p95": _pct(attach_z_gt, 95),
        },
        "retreat_delta_z": {
            "n": len(retreat_dz),
            "mean": _mean(retreat_dz),
            "std": _std(retreat_dz),
            "p50": _pct(retreat_dz, 50),
            "p95": _pct(retreat_dz, 95),
        },
        "detect_err_xy": {
            "p50": _pct([r.err_xy for r in compared if r.err_xy is not None], 50),
            "p95": _pct([r.err_xy for r in compared if r.err_xy is not None], 95),
        },
    }


def trial_to_dict(record):
    payload = asdict(record)
    payload["segment_failures"] = [list(item) for item in record.segment_failures]
    return payload


def trial_from_dict(payload):
    """Rebuild a TrialRecord from JSON (jsonl round-trip)."""
    data = dict(payload or {})
    segs = data.get("segment_failures") or ()
    data["segment_failures"] = tuple(
        (str(a), str(b)) for a, b in (tuple(item) for item in segs)
    )
    allowed = {item.name for item in fields(TrialRecord)}
    kwargs = {k: v for k, v in data.items() if k in allowed}
    return TrialRecord(**kwargs)
