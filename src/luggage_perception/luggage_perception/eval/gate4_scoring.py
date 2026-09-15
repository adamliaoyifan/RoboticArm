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


# Post-spawn drain before the scored cursor (PF-R7 G3 / Gate4). ROS stamp
# first; monotonic receipt is the fallback when stamps are missing.
SCORE_DRAIN_SEC = 0.4
SCORE_WARMUP_FRAMES = 5
STEADY_WINDOW_SEC = 8.0
SUPPORT_WINDOW_SIZE = 5
STEADY_START_SUPPORT_READY = "support-window-ready"
WALL_WATCHDOG_AFTER_PROPOSAL_SEC = 12.0
OUTPUT_HZ_MIN = 4.0
STAMP_EPS_SEC = 1e-9


def row_time_sec(row):
    """Barrier clock: ROS header stamp, else receipt monotonic."""
    if row is None:
        return None
    for key in ("stamp_sec", "monotonic_sec", "receipt_monotonic_sec"):
        value = row.get(key)
        if value is None:
            continue
        try:
            return float(value)
        except (TypeError, ValueError):
            continue
    return None


def row_matches_barrier(row, instance_id, generation):
    """True when the frame belongs to the scored instance *and* generation."""
    if instance_id is not None and row.get("instance_id") != instance_id:
        return False
    if generation is None:
        return True
    value = row.get("generation")
    if value is None:
        return False
    try:
        return int(value) == int(generation)
    except (TypeError, ValueError):
        return False


def infer_expected_generation(rows, instance_id):
    """Newest generation among frames already tagged with ``instance_id``."""
    gens = []
    for row in rows or []:
        if instance_id is not None and row.get("instance_id") != instance_id:
            continue
        value = row.get("generation")
        if value is None:
            continue
        try:
            gens.append(int(value))
        except (TypeError, ValueError):
            continue
    if not gens:
        return None
    return max(gens)


def quarantine_record(row, reason, region):
    return {
        "instance_id": row.get("instance_id"),
        "generation": row.get("generation"),
        "stamp_sec": row.get("stamp_sec"),
        "monotonic_sec": row.get("monotonic_sec"),
        "receipt_monotonic_sec": row.get("receipt_monotonic_sec") or row.get(
            "monotonic_sec"),
        "reason": reason,
        "region": region,
        "support_reason": row.get("support_reason"),
        "geometry_level": row.get("geometry_level"),
        "support_inliers": row.get("support_inliers"),
        "support_side_coverage": row.get("support_side_coverage"),
        "support_residual": row.get("support_residual"),
        "support_confidence": row.get("support_confidence"),
        "support_n_candidates": row.get("support_n_candidates"),
        "pca_source": row.get("pca_source"),
        "pca_raw_source": row.get("pca_raw_source") or row.get("pca_source"),
        "support_sample_admitted": row.get("support_sample_admitted"),
        "support_window_count": row.get("support_window_count"),
        "support_window_size": row.get("support_window_size"),
    }


def _as_bool(value):
    if value is None or value == "":
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("true", "1", "yes"):
        return True
    if text in ("false", "0", "no"):
        return False
    return None


def _as_int(value):
    if value is None or value == "":
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def reconstruct_support_admitted(row):
    """Match production: only a finite local-support fit enters history."""
    flagged = _as_bool(row.get("support_sample_admitted"))
    if flagged is not None:
        return flagged
    if str(row.get("pca_source") or "") != "measure":
        return False
    if not row.get("top_surface_valid"):
        return False
    reason = str(row.get("support_reason") or "")
    inliers = _as_int(row.get("support_inliers")) or 0
    if reason == "ok":
        return True
    if reason == "DETECT_SUPPORT_UNSTABLE" and inliers > 0:
        return True
    return False


def annotate_support_history(rows, instance_id, generation=None,
                             window_size=SUPPORT_WINDOW_SIZE):
    """Persist per-row history diagnostics without changing production.

    If every row already has ``support_window_count``, keep those live
    values. Otherwise reconstruct a zeroed window in chronological order.
    """
    rows = list(rows or [])
    window_size = max(1, int(window_size))
    have_live = True
    for row in rows:
        if not row_matches_barrier(row, instance_id, generation):
            continue
        if _as_int(row.get("support_window_count")) is None:
            have_live = False
            break
    count = 0
    for row in rows:
        if not row_matches_barrier(row, instance_id, generation):
            row["support_sample_admitted"] = False
            if _as_int(row.get("support_window_count")) is None:
                row["support_window_count"] = count
            if _as_int(row.get("support_window_size")) is None:
                row["support_window_size"] = window_size
            continue
        admitted = reconstruct_support_admitted(row)
        reason = str(row.get("support_reason") or "")
        inliers = _as_int(row.get("support_inliers")) or 0
        # geometry_not_settled publishes UNSTABLE with no inliers and
        # clears production history.
        if (not admitted and reason == "DETECT_SUPPORT_UNSTABLE"
                and inliers <= 0 and str(row.get("pca_source") or "") == "measure"):
            count = 0
        if admitted and not have_live:
            count = min(window_size, count + 1)
        if not have_live:
            row["support_window_count"] = count
        elif _as_int(row.get("support_window_count")) is not None:
            count = _as_int(row["support_window_count"])
        row["support_sample_admitted"] = bool(admitted)
        row["support_window_size"] = (
            _as_int(row.get("support_window_size")) or window_size)
        row["support_history_instance_id"] = (
            row.get("support_history_instance_id") or row.get("instance_id"))
        row["support_history_generation"] = (
            _as_int(row.get("support_history_generation"))
            if row.get("support_history_generation") is not None
            else row.get("generation"))
    return rows


def first_support_ready_stamp(rows, instance_id, generation=None,
                              window_size=SUPPORT_WINDOW_SIZE):
    """ROS stamp of the first *readiness-eligible* 5/5 history row.

    Occupancy leftover from a previous box (PF-R10 epoch carry) is not
    ready. The row must also be admitted into the filter. Independent of
    whether that row is FULL_3D.
    """
    window_size = int(window_size)
    for row in rows or []:
        if not row_matches_barrier(row, instance_id, generation):
            continue
        if not reconstruct_support_admitted(row):
            continue
        count = _as_int(row.get("support_window_count"))
        size = _as_int(row.get("support_window_size")) or window_size
        if count is None:
            continue
        if count == size == window_size:
            stamp = row_time_sec(row)
            if stamp is None or not math.isfinite(float(stamp)):
                return None, row
            return float(stamp), row
    return None, None


def split_steady_windows(rows, instance_id, generation=None,
                         drain_rows=None, window_sec=STEADY_WINDOW_SEC,
                         window_size=SUPPORT_WINDOW_SIZE,
                         clock_end_sec=None):
    """G4: quarantine drain, start at 5/5 history, score [t, t+8)."""
    bound = bind_collected_windows(
        drain_rows or [], rows, instance_id, generation=generation)
    owned = list(bound["owned_recovery"])
    annotate_support_history(owned, instance_id, bound["generation"],
                             window_size=window_size)
    t_steady, ready_row = first_support_ready_stamp(
        owned, instance_id, bound["generation"], window_size=window_size)
    window_sec = float(window_sec)
    transition = []
    settled = []
    missing_stamp = False
    for row in owned:
        stamp = row_time_sec(row)
        if stamp is None or not math.isfinite(float(stamp)):
            missing_stamp = True
            continue
        if t_steady is None or stamp < t_steady - STAMP_EPS_SEC:
            transition.append(row)
        elif stamp < t_steady + window_sec - STAMP_EPS_SEC:
            settled.append(row)
    last_stamp = None
    for row in owned:
        stamp = row_time_sec(row)
        if stamp is None:
            continue
        last_stamp = stamp if last_stamp is None else max(last_stamp, stamp)
    if clock_end_sec is None:
        clock_end_sec = last_stamp
    window_complete = (
        t_steady is not None
        and clock_end_sec is not None
        and math.isfinite(float(clock_end_sec))
        and float(clock_end_sec) + STAMP_EPS_SEC >= t_steady + window_sec
        and not missing_stamp)
    stale_scored = 0
    for row in settled:
        if not row_matches_barrier(row, instance_id, bound["generation"]):
            stale_scored += 1
            bound["quarantine"].append(quarantine_record(
                row, "stale_scored_or_fused", "scored"))
    bound.update({
        "score_mode": STEADY_START_SUPPORT_READY,
        "transition": transition,
        "warmup": [],
        "settled": settled,
        "t_steady": t_steady,
        "t_steady_end": None if t_steady is None else t_steady + window_sec,
        "ready_row": ready_row,
        "window_sec": window_sec,
        "window_complete": window_complete,
        "clock_end_sec": clock_end_sec,
        "missing_stamp": missing_stamp,
        "n_transition": len(transition),
        "n_settled": len(settled),
        "stale_scored_or_fused": stale_scored,
    })
    return bound


def split_score_windows(rows, instance_id, generation=None,
                        drain_sec=SCORE_DRAIN_SEC,
                        warmup_frames=SCORE_WARMUP_FRAMES):
    """Split spawn-aligned rows into drain, recovery, and scored windows.

    ``rows`` must be chronological. The drain uses the first row's ROS
    stamp (fallback: monotonic) plus ``drain_sec``. Owned drain frames
    are quarantined as unscored, not as safety failures. Stale frames
    observed before the barrier, and stale frames dropped after it, are
    diagnostics. Only a stale frame admitted into warmup/settled counts
    as ``stale_scored_or_fused``.
    """
    rows = list(rows or [])
    empty = {
        "drain": [],
        "owned_recovery": [],
        "owned_scored": [],
        "warmup": [],
        "settled": [],
        "stale_pre_barrier_observed": 0,
        "stale_post_barrier_dropped": 0,
        "stale_scored_or_fused": 0,
        "quarantine": [],
        "instance_id": instance_id,
        "generation": generation,
        "t0_sec": None,
    }
    if not rows:
        return empty
    if generation is None:
        generation = infer_expected_generation(rows, instance_id)
        empty["generation"] = generation
    t0 = row_time_sec(rows[0])
    empty["t0_sec"] = t0
    drain = []
    post = []
    for row in rows:
        stamp = row_time_sec(row)
        if (
            t0 is not None and stamp is not None
            and (stamp - t0) < float(drain_sec)
        ):
            drain.append(row)
        else:
            post.append(row)
    quarantine = []
    stale_pre = 0
    for row in drain:
        if row_matches_barrier(row, instance_id, generation):
            quarantine.append(quarantine_record(
                row, "post_spawn_drain", "drain"))
        else:
            stale_pre += 1
            quarantine.append(quarantine_record(
                row, "stale_pre_barrier_observed", "drain"))
    stale_post = 0
    owned_scored = []
    for row in post:
        if row_matches_barrier(row, instance_id, generation):
            owned_scored.append(row)
        else:
            stale_post += 1
            quarantine.append(quarantine_record(
                row, "stale_post_barrier_dropped", "score"))
    warmup, settled = split_warmup(owned_scored, warmup_frames=warmup_frames)
    owned_recovery = [
        row for row in rows
        if row_matches_barrier(row, instance_id, generation)]
    stale_scored = 0
    for row in list(warmup) + list(settled):
        if not row_matches_barrier(row, instance_id, generation):
            stale_scored += 1
            quarantine.append(quarantine_record(
                row, "stale_scored_or_fused", "scored"))
    empty.update({
        "drain": drain,
        "owned_recovery": owned_recovery,
        "owned_scored": owned_scored,
        "warmup": warmup,
        "settled": settled,
        "stale_pre_barrier_observed": stale_pre,
        "stale_post_barrier_dropped": stale_post,
        "stale_scored_or_fused": stale_scored,
        "quarantine": quarantine,
        "generation": generation,
    })
    return empty


def bind_collected_windows(drain_rows, score_rows, instance_id,
                           generation=None,
                           warmup_frames=SCORE_WARMUP_FRAMES):
    """Gate4 live order: drain collection is already excluded from score rows."""
    drain_rows = list(drain_rows or [])
    score_rows = list(score_rows or [])
    if generation is None:
        generation = infer_expected_generation(
            drain_rows + score_rows, instance_id)
    quarantine = []
    stale_pre = 0
    owned_drain = []
    for row in drain_rows:
        if row_matches_barrier(row, instance_id, generation):
            owned_drain.append(row)
            quarantine.append(quarantine_record(
                row, "post_spawn_drain", "drain"))
        else:
            stale_pre += 1
            quarantine.append(quarantine_record(
                row, "stale_pre_barrier_observed", "drain"))
    stale_post = 0
    owned_scored = []
    for row in score_rows:
        if row_matches_barrier(row, instance_id, generation):
            owned_scored.append(row)
        else:
            stale_post += 1
            quarantine.append(quarantine_record(
                row, "stale_post_barrier_dropped", "score"))
    warmup, settled = split_warmup(owned_scored, warmup_frames=warmup_frames)
    stale_scored = 0
    for row in list(warmup) + list(settled):
        if not row_matches_barrier(row, instance_id, generation):
            stale_scored += 1
            quarantine.append(quarantine_record(
                row, "stale_scored_or_fused", "scored"))
    return {
        "drain": drain_rows,
        "owned_recovery": owned_drain + owned_scored,
        "owned_scored": owned_scored,
        "warmup": warmup,
        "settled": settled,
        "stale_pre_barrier_observed": stale_pre,
        "stale_post_barrier_dropped": stale_post,
        "stale_scored_or_fused": stale_scored,
        "quarantine": quarantine,
        "instance_id": instance_id,
        "generation": generation,
        "t0_sec": row_time_sec(drain_rows[0]) if drain_rows else row_time_sec(
            score_rows[0] if score_rows else None),
    }


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


RECOVERY_LIMIT_SEC = 1.4
MIN_SETTLED_PER_TRIAL = 30


def recovery_times(owned_rows, t0_monotonic):
    """Untrimmed time to first valid top and first FULL_3D.

    ``owned_rows`` are chronological detector rows for the expected
    instance, including warmup. ``t0_monotonic`` is the spawn-return
    clock. Negative deltas (frames that arrived during spawn) count as
    0.0. Missing timestamps are skipped.
    """
    t_valid = None
    t_full = None
    t0 = float(t0_monotonic)
    for row in owned_rows:
        stamp = row.get("monotonic_sec")
        if stamp is None:
            continue
        dt = max(0.0, float(stamp) - t0)
        if t_valid is None and row.get("top_surface_valid"):
            t_valid = dt
        if t_full is None and _category(row) == "full3d":
            t_full = dt
        if t_valid is not None and t_full is not None:
            break
    return t_valid, t_full


def placement_recovery_gate(spawn_failures, trial_recoveries,
                            failed_count=0,
                            recovery_limit_sec=RECOVERY_LIMIT_SEC,
                            min_settled=MIN_SETTLED_PER_TRIAL):
    """PF-R10 closeout extras on top of Gate 4 limits.

    Failed placements must fail the run (they are not dropped from
    scoring). Recovery uses the untrimmed owned series. C1 also requires
    zero settled ``failed`` frames.
    """
    failures = []
    if int(spawn_failures) > 0:
        failures.append("spawn_failures %d > 0" % int(spawn_failures))
    if int(failed_count) > 0:
        failures.append("failed %d > 0" % int(failed_count))
    for rec in trial_recoveries:
        trial = rec.get("trial")
        n_settled = int(rec.get("n_settled") or 0)
        t_valid = rec.get("t_first_valid_sec")
        t_full = rec.get("t_first_full3d_sec")
        if n_settled < int(min_settled):
            failures.append("trial %s settled %d < %d" % (
                trial, n_settled, int(min_settled)))
        if t_valid is None or float(t_valid) > float(recovery_limit_sec):
            failures.append(
                "trial %s t_first_valid %s > %.1f s" % (
                    trial, t_valid, float(recovery_limit_sec)))
        if t_full is None or float(t_full) > float(recovery_limit_sec):
            failures.append(
                "trial %s t_first_full3d %s > %.1f s" % (
                    trial, t_full, float(recovery_limit_sec)))
    return failures
