#!/usr/bin/env python3
"""Gate 4 scoring semantics for the platform-free height eval (PF-R4).

Pure functions, no ROS: the ROS harness in
``scripts/platform_free_height_gate4_eval.py`` collects rows; this module
owns the semantics that decide pass/fail.

Rules (docs/plans/platform_free_height_remediation.md PF-R4):

- rows are filtered by the trial's ``instance_id``/``generation``: stale
  frames from a previous box never score;
- warmup (the detector's support-stability window after an instance
  change) is reported separately and excluded from settled scoring by
  the documented ``warmup_frames`` rule only;
- TOP_ONLY / FULL_3D / prior / failed frames are reported separately and
  are never merged into one detection-success rate;
- a required metric with zero samples FAILS the gate (absence of
  evidence cannot pass an accuracy limit);
- the active-window output rate excludes orchestration gaps
  (spawn/delete) by splitting the stamp sequence on gaps larger than
  ``gap_sec``; the end-to-end trial-cycle rate is reported separately.
"""

from __future__ import division

import math

# Gate 4 limits (docs/plans/platform_free_height_test_plan.md).
GATE4_LIMITS = {
    "top_surface_rate": 0.95,
    "full3d_rate": 0.95,
    "top_z_p95_m": 0.015,
    "top_z_max_m": 0.025,
    "support_z_p95_m": 0.015,
    "support_z_max_m": 0.025,
    "height_p95_m": 0.025,
    "height_max_m": 0.040,
    "xy_p95_m": 0.030,
    "width_depth_p95_m": 0.050,
    "false_measured_height": 0,
}

GEOMETRY_TOP_ONLY = 0
GEOMETRY_FULL_3D = 1
HEIGHT_SOURCE_CATALOG_PRIOR = 3


def filter_instance(rows, instance_id, generation):
    """Keep only rows belonging to the given instance/generation."""
    if instance_id is None and generation is None:
        return list(rows)
    return [
        r for r in rows
        if r.get("instance_id") == instance_id
        and r.get("generation") == generation
    ]


def filter_expected_instance(rows, expected_instance_id):
    """Keep rows matching the *eval-side* expected instance (PF-R4 rework).

    ``expected_instance_id`` comes from the spawn/current-box GT
    response, never from detector output: stale frames arriving after a
    spawn must stay stale (they do not become the selected instance).
    Returns ``(owned_rows, stale_count)``.
    """
    if expected_instance_id is None:
        return list(rows), 0
    owned = [r for r in rows
             if r.get("instance_id") == expected_instance_id]
    return owned, len(rows) - len(owned)


def split_warmup(rows, warmup_frames=5):
    """Split a trial's chronologically sorted rows into (warmup, settled).

    The first ``warmup_frames`` rows after an instance change are the
    detector's support-stability warmup: TOP_ONLY there is design
    behavior, not a geometry failure. They are reported but excluded
    from settled scoring.
    """
    warmup = list(rows[:max(0, int(warmup_frames))])
    settled = list(rows[max(0, int(warmup_frames)):])
    return warmup, settled


def _percentile(sorted_vals, p):
    return sorted_vals[min(len(sorted_vals) - 1,
                           int(round((p / 100.0) * (len(sorted_vals) - 1))))]


def stats(vals):
    vals = [float(v) for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    s = sorted(vals)
    return {
        "count": len(s),
        "p50": _percentile(s, 50),
        "p95": _percentile(s, 95),
        "max": s[-1],
    }


def active_window_hz(stamps_sec, gap_sec=2.0):
    """Output rate over active windows only (PF-G4H, PF-R4 rework).

    Splits the chronological stamp sequence on gaps larger than
    ``gap_sec`` (spawn/delete orchestration) and computes the rate from
    observed inter-frame intervals: a window of k frames contributes
    k-1 intervals over its first-to-last span, so a window of 10 frames
    at 5 Hz yields exactly 5 Hz (frame-count/span would report 5.56 Hz).
    Returns ``None`` when the sequence has no complete interval.
    """
    if len(stamps_sec) < 2:
        return None
    ordered = sorted(float(t) for t in stamps_sec)
    intervals = 0
    span = 0.0
    window_intervals = 0
    window_start = ordered[0]
    prev = ordered[0]
    for t in ordered[1:]:
        if (t - prev) > float(gap_sec):
            span += prev - window_start
            intervals += window_intervals
            window_intervals = 0
            window_start = t
        else:
            window_intervals += 1
        prev = t
    span += prev - window_start
    intervals += window_intervals
    if intervals <= 0 or span <= 0.0:
        return None
    return intervals / span


def coverage_gate(coverage, min_sizes=3, min_xy_offsets=3, min_yaws=3,
                  min_trials=30, min_trials_per_size=10):
    """Gate 4 deterministic-pose coverage (PF-R4 rework).

    ``coverage`` comes from the eval-side GT record (sizes, XY offsets,
    yaw values, per-size trial counts) — placement variation belongs to
    the spawner/eval side and is never an online algorithm input.
    Returns a list of failure strings; empty means the matrix is
    sufficient.
    """
    failures = []
    if coverage.get("n_sizes", 0) < min_sizes:
        failures.append("size coverage %d < %d" % (
            coverage.get("n_sizes", 0), min_sizes))
    if coverage.get("n_xy_offsets", 0) < min_xy_offsets:
        failures.append("xy offset coverage %d < %d" % (
            coverage.get("n_xy_offsets", 0), min_xy_offsets))
    if coverage.get("n_yaws", 0) < min_yaws:
        failures.append("yaw coverage %d < %d" % (
            coverage.get("n_yaws", 0), min_yaws))
    if coverage.get("n_trials", 0) < min_trials:
        failures.append("trials %d < %d" % (
            coverage.get("n_trials", 0), min_trials))
    per_size = coverage.get("trials_per_size") or {}
    thin = {k: v for k, v in per_size.items() if v < min_trials_per_size}
    if thin:
        failures.append(
            "trials per size below %d for %s" % (
                min_trials_per_size,
                sorted(thin)[:4]))
    return failures


#: pca_reason that must dominate every frame in the raw-only negative
#: control (PF-R2 fail-closed contract).
CARGO_SEGMENTATION_REQUIRED = "DETECT_CARGO_SEGMENTATION_REQUIRED"


def negative_control_verdict(settled_rows):
    """Raw-only fail-closed verdict (PF-R4 rework).

    The control PASSES only when the detector produced frames and every
    one of them failed with the explicit segmentation reason — zero
    valid tops, zero measured heights. Any valid output (or any other
    failure mode) fails the control.
    """
    problems = []
    if not settled_rows:
        problems.append("no frames collected in negative control")
    valid_top = [r for r in settled_rows if r.get("top_surface_valid")]
    if valid_top:
        problems.append("%d frames with valid top (must be zero)" % (
            len(valid_top),))
    measured = [r for r in settled_rows
                if r.get("height_valid") or r.get("geometry_level") == 1]
    if measured:
        problems.append("%d frames with full/measured geometry (must be "
                        "zero)" % (len(measured),))
    other_reason = [
        r for r in settled_rows
        if r.get("pca_reason") != CARGO_SEGMENTATION_REQUIRED]
    if other_reason:
        problems.append("%d frames without %s (got e.g. %r)" % (
            len(other_reason), CARGO_SEGMENTATION_REQUIRED,
            other_reason[0].get("pca_reason")))
    return {"negative_control_pass": not problems,
            "negative_control_failures": problems}


def _category(row):
    if not row.get("top_surface_valid"):
        return "failed"
    if row.get("height_valid") and row.get("geometry_level") == GEOMETRY_FULL_3D:
        return "full3d"
    if row.get("height_source") == HEIGHT_SOURCE_CATALOG_PRIOR:
        return "prior"
    return "top_only"


def aggregate(trials, require_full_geometry=True):
    """Aggregate scored trials -> summary dict with gate decision inputs.

    ``trials`` is a list of dicts ``{"warmup": [...], "settled": [...],
    "instance_id": ..., "cycle_sec": ...}``. Only settled rows score.
    """
    warmup_rows = [r for t in trials for r in t["warmup"]]
    settled_rows = [r for t in trials for r in t["settled"]]
    categories = {
        "warmup_frames": len(warmup_rows),
        "settled_frames": len(settled_rows),
        "top_only": sum(1 for r in settled_rows
                        if _category(r) == "top_only"),
        "full3d": sum(1 for r in settled_rows
                      if _category(r) == "full3d"),
        "prior": sum(1 for r in settled_rows
                     if _category(r) == "prior"),
        "failed": sum(1 for r in settled_rows
                      if _category(r) == "failed"),
    }
    top_valid = [r for r in settled_rows if r.get("top_surface_valid")]
    full3d = [r for r in settled_rows if _category(r) == "full3d"]

    summary = {
        "n_settled": len(settled_rows),
        "categories": categories,
        "top_surface_rate": (
            len(top_valid) / len(settled_rows)) if settled_rows else 0.0,
        # Full-geometry rate is measured among frames whose top was
        # valid (support coverage had something to work with).
        "full3d_rate": (len(full3d) / len(top_valid)) if top_valid else 0.0,
        "top_z_err_m": stats(
            [r.get("err_top_m") for r in top_valid]),
        "support_z_err_m": stats(
            [r.get("err_support_m") for r in full3d]),
        "height_err_m": stats(
            [r.get("err_height_m") for r in full3d]),
        "xy_err_m": stats(
            [r.get("err_xy_m") for r in top_valid]),
        "width_err_m": stats(
            [r.get("err_width_m") for r in top_valid]),
        "depth_err_m": stats(
            [r.get("err_depth_m") for r in top_valid]),
        "false_measured_height": sum(
            1 for r in settled_rows if r.get("false_measured_height")),
        "limits": dict(GATE4_LIMITS),
        "require_full_geometry": bool(require_full_geometry),
    }
    return summary


def gate_pass(summary):
    """Apply Gate 4 limits. Zero-sample required metrics fail (PF-R4)."""
    lim = summary["limits"]
    failures = []
    if summary["n_settled"] <= 0:
        failures.append("no settled frames")
    if summary["top_surface_rate"] < lim["top_surface_rate"]:
        failures.append("top_surface_rate %.3f < %.3f" % (
            summary["top_surface_rate"], lim["top_surface_rate"]))
    if summary["false_measured_height"] > lim["false_measured_height"]:
        failures.append("false_measured_height %d > 0" % (
            summary["false_measured_height"]))

    # Required metrics: a None p95/max means zero samples. Top Z and XY
    # are required whenever any settled frame exists; support/height are
    # required when full geometry is required (semantic auto mode).
    required = [
        ("top_z_err_m", summary["top_z_err_m"],
         [("p95", lim["top_z_p95_m"]), ("max", lim["top_z_max_m"])]),
        ("xy_err_m", summary["xy_err_m"],
         [("p95", lim["xy_p95_m"])]),
        ("width_err_m", summary["width_err_m"],
         [("p95", lim["width_depth_p95_m"])]),
        ("depth_err_m", summary["depth_err_m"],
         [("p95", lim["width_depth_p95_m"])]),
    ]
    if summary["require_full_geometry"]:
        required.extend([
            ("support_z_err_m", summary["support_z_err_m"],
             [("p95", lim["support_z_p95_m"]), ("max", lim["support_z_max_m"])]),
            ("height_err_m", summary["height_err_m"],
             [("p95", lim["height_p95_m"]), ("max", lim["height_max_m"])]),
        ])
        if summary["categories"]["full3d"] <= 0:
            failures.append("zero FULL_3D frames with full geometry required")
        if summary["full3d_rate"] < lim["full3d_rate"]:
            failures.append("full3d_rate %.3f < %.3f" % (
                summary["full3d_rate"], lim["full3d_rate"]))
    for name, stat, checks in required:
        for key, limit in checks:
            value = stat.get(key)
            if value is None:
                failures.append("%s %s has no samples (required)" % (
                    name, key))
            elif value > limit:
                failures.append("%s %s %.4f > %.4f" % (
                    name, key, value, limit))
    summary = dict(summary)
    summary["gate4_failures"] = failures
    summary["gate4_pass"] = not failures
    return summary
