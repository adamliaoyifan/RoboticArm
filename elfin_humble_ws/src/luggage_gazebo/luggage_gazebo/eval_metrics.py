#!/usr/bin/env python3
"""Aggregate pick/retreat eval trials. Numpy/stdlib only, no ROS.

Three independent rates (docs/plans/closed_loop_eval_driver.md):

  detect_pass_rate  — DetectionAccuracy.ok among trials that compared
  plan_pass_rate    — four PlanMotion successes among detect-pass trials
  retreat_pass_rate — retreat height check among plan-pass trials

Geometry vs GT is only compared after (1) post-mask-filter YOLO published a
cargo box for that spawn and (2) the depth blob on the pickup platform
matches this spawn's catalog AABB (GetCurrentBox). DetectLuggage is then
scored against that AABB. A leftover previous mesh must fail at the visual
gate, not as ``DETECT_GATE``. Spawn→YOLO / visual / DetectLuggage latencies
are summarized separately (performance, not a pass/fail gate).

Pick accuracy (geometric, no vacuum): suction XY/Z vs the measured box
after attach. Stability is the sample std of those errors among trials
where attach ran.
"""

from __future__ import division

import math
from collections import Counter
from dataclasses import asdict, dataclass, field, fields
from typing import Optional, Tuple

import numpy as np

# Observe-centered pick loop. ClearCurrentBox only after the arm is back
# at observe — never immediately after pick_retreat while the camera is high.
OBSERVE_LOOP_PHASES = (
    "goto_observe",
    "ensure_clean",
    "spawn",
    "wait_yolo_boxes",
    "wait_tracked_cargo",
    "wait_spawn_visual",
    "detect",
    "plan_execute",
    "goto_observe",
    "clear_box",
)

# Must match semantic_segmenter.LABEL_CARGO. Kept numeric so this module
# stays numpy/stdlib (no ROS, no perception import).
_LABEL_CARGO = 2

# Catalog width steps are 0.10–0.15 m. 8 cm rejects the wrong size class
# (n20 leftover mesh) without treating a few-cm silhouette vs this spawn's
# catalog AABB as a spawn failure.
DEFAULT_VISUAL_TOL_XY = 0.08
DEFAULT_VISUAL_TOL_Z = 0.06
_VISUAL_MIN_POINTS = 200
_VISUAL_PERCENTILE = (2.0, 98.0)
# Class fallback only: blob vs catalog AABB scaled to a typical lid-band
# fill. Primary visual match is blob vs this spawn's catalog AABB.
_SILHOUETTE_FILL = 0.86
_CATALOG_SIZES = (
    ("carryon", (0.55, 0.40, 0.25)),
    ("standard", (0.70, 0.45, 0.28)),
    ("large", (0.80, 0.50, 0.32)),
)


def cargo_generation_ready(stats, expected_generation):
    """True when tracker stats belong to *expected_generation* and have points."""
    if not isinstance(stats, dict):
        return False
    try:
        gen = int(stats.get("generation") or 0)
        expected = int(expected_generation)
    except (TypeError, ValueError):
        return False
    raw = stats.get("last_cargo_n_points")
    if raw is None:
        raw = stats.get("n_points")
    try:
        n_points = int(raw if raw is not None else 0)
    except (TypeError, ValueError):
        return False
    if n_points < 0:
        n_points = 0
    return gen == expected and n_points > 0


def yolo_boxes_ready(seg_stats, expected_generation, expected_id=None,
                     min_stamp=None):
    """True when post-mask-filter YOLO published a cargo box for this spawn.

    *raw_cargo* is after self-body / row-band drop and before temporal hold,
    so a held box from the previous suitcase cannot unblock the wait.
    *min_stamp* rejects stats whose image stamp is older than spawn.
    """
    if not isinstance(seg_stats, dict):
        return False
    try:
        gen = int(seg_stats.get("generation") or 0)
        expected = int(expected_generation)
    except (TypeError, ValueError):
        return False
    if gen != expected:
        return False
    if expected_id is not None:
        got_id = str(seg_stats.get("instance_id") or "")
        if got_id != str(expected_id):
            return False
    if min_stamp is not None:
        try:
            stamp = float(seg_stats.get("stamp"))
        except (TypeError, ValueError):
            return False
        if stamp + 1e-9 < float(min_stamp):
            return False
    if bool(seg_stats.get("raw_cargo")):
        return True
    n_cargo = 0
    for det in seg_stats.get("detections") or []:
        if not isinstance(det, dict) or det.get("held"):
            continue
        try:
            label = int(det.get("label", -1))
        except (TypeError, ValueError):
            continue
        if label == _LABEL_CARGO:
            n_cargo += 1
    return n_cargo > 0


def depth_to_camera_xyz(depth_m, fx, fy, cx, cy, max_depth=2.5, stride=2):
    """Unproject a depth image to optical-frame XYZ (Nx3)."""
    depth = np.asarray(depth_m, dtype=np.float64)
    if depth.ndim != 2 or fx <= 1e-9 or fy <= 1e-9:
        return np.zeros((0, 3), dtype=np.float64)
    step = max(1, int(stride))
    z = depth[::step, ::step]
    h, w = z.shape
    vs = np.arange(0, depth.shape[0], step, dtype=np.float64)[:h]
    us = np.arange(0, depth.shape[1], step, dtype=np.float64)[:w]
    uu, vv = np.meshgrid(us, vs)
    valid = np.isfinite(z) & (z > 0.05) & (z < float(max_depth))
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64)
    zz = z[valid]
    xx = (uu[valid] - float(cx)) * zz / float(fx)
    yy = (vv[valid] - float(cy)) * zz / float(fy)
    return np.column_stack((xx, yy, zz))


def transform_camera_xyz_to_world(points_cam, rotation, translation):
    """Apply T_world_camera (rotation 3x3, translation length-3)."""
    pts = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    rot = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    origin = np.asarray(translation, dtype=np.float64).reshape(3)
    return pts.dot(rot.T) + origin


def raised_object_measure(points_world, platform_z, roi_center_xy,
                          roi_margin=0.5, min_height=0.03,
                          min_points=_VISUAL_MIN_POINTS,
                          percentile=_VISUAL_PERCENTILE, max_height=None):
    """Visible top of a raised object on the pickup platform.

    Returns a dict ``width, depth, height, n, x, y, z`` or None. XY is the
    percentile AABB (ignores arm fingers / speckle); ``x,y`` is that AABB
    centre; ``z`` is platform + half height. Points above
    ``platform_z + max_height`` (suction panel in the ROI) are dropped when
    *max_height* is set. This is the camera's rendered object, not YOLO /
    PCA and not the catalog AABB.
    """
    pts = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
    if pts.size == 0:
        return None
    cx, cy = float(roi_center_xy[0]), float(roi_center_xy[1])
    margin = float(roi_margin)
    plat = float(platform_z)
    keep = (
        (pts[:, 0] >= cx - margin) & (pts[:, 0] <= cx + margin)
        & (pts[:, 1] >= cy - margin) & (pts[:, 1] <= cy + margin)
        & (pts[:, 2] >= plat + float(min_height))
    )
    if max_height is not None:
        keep = keep & (pts[:, 2] <= plat + float(max_height))
    blob = pts[keep]
    if len(blob) < int(min_points):
        return None
    lo, hi = percentile
    x_lo, x_hi = np.percentile(blob[:, 0], (lo, hi))
    y_lo, y_hi = np.percentile(blob[:, 1], (lo, hi))
    z_hi = float(np.percentile(blob[:, 2], hi))
    width = float(x_hi - x_lo)
    depth = float(y_hi - y_lo)
    height = max(0.01, z_hi - plat)
    return {
        "width": width,
        "depth": depth,
        "height": height,
        "n": int(len(blob)),
        "x": 0.5 * (float(x_lo) + float(x_hi)),
        "y": 0.5 * (float(y_lo) + float(y_hi)),
        "z": plat + 0.5 * height,
    }


def raised_object_size(points_world, platform_z, roi_center_xy, roi_margin=0.5,
                       min_height=0.03, min_points=_VISUAL_MIN_POINTS,
                       percentile=_VISUAL_PERCENTILE, max_height=None):
    """XY AABB + height of points standing on the pickup platform.

    Returns ``(width, depth, height, n)`` or None when the blob is empty.
    """
    measured = raised_object_measure(
        points_world, platform_z, roi_center_xy, roi_margin=roi_margin,
        min_height=min_height, min_points=min_points, percentile=percentile,
        max_height=max_height)
    if measured is None:
        return None
    return (
        measured["width"], measured["depth"], measured["height"],
        measured["n"])


def _sorted_xy(size):
    return tuple(sorted((abs(float(size[0])), abs(float(size[1])))))


def nearest_catalog_id(size, silhouette=False, fill=_SILHOUETTE_FILL):
    """Nearest catalog class by XY (+ height). *silhouette* scales templates."""
    if size is None or len(size) < 3:
        return None
    o_xy = _sorted_xy(size)
    o_h = abs(float(size[2]))
    best_id = None
    best_d = None
    for cid, gt in _CATALOG_SIZES:
        g_xy = _sorted_xy(gt)
        if silhouette:
            g_xy = (g_xy[0] * float(fill), g_xy[1] * float(fill))
        dxy = math.hypot(o_xy[0] - g_xy[0], o_xy[1] - g_xy[1])
        dz = abs(o_h - float(gt[2]))
        dist = dxy + 0.35 * dz
        if best_d is None or dist < best_d:
            best_id, best_d = cid, dist
    return best_id


def spawn_visual_matches_gt(observed_size, gt_size,
                            tol_xy=DEFAULT_VISUAL_TOL_XY,
                            tol_z=DEFAULT_VISUAL_TOL_Z,
                            expected_class=None):
    """True when the depth blob is this spawn's visible top, not leftovers.

    *observed_size* / *gt_size* are ``(width, depth, height)``. *gt_size*
    should be the spawned catalog AABB (GetCurrentBox), not the mesh
    lid-band. Width/depth are compared without axis order (90° yaw
    swaps them).

    Two paths: an AABB-close match against that size, or the same
    catalog class (*expected_class*, else nearest AABB class of *gt_size*)
    when the blob is a silhouette of a different visual of the same tier.
    """
    if observed_size is None or gt_size is None:
        return False
    if len(observed_size) < 3 or len(gt_size) < 3:
        return False
    obs_xy = _sorted_xy(observed_size)
    gt_xy = _sorted_xy(gt_size)
    xy_ok = (
        abs(obs_xy[0] - gt_xy[0]) <= float(tol_xy)
        and abs(obs_xy[1] - gt_xy[1]) <= float(tol_xy))
    z_ok = abs(float(observed_size[2]) - float(gt_size[2])) <= float(tol_z)
    if xy_ok and z_ok:
        return True
    obs_id = nearest_catalog_id(observed_size, silhouette=True)
    gt_id = (
        str(expected_class) if expected_class
        else nearest_catalog_id(gt_size, silhouette=False))
    return bool(obs_id) and bool(gt_id) and obs_id == gt_id


def tracker_epoch_matches(stats, expected_generation, expected_id=None):
    """True when filter stats are on this spawn's generation and id.

    An empty ``instance_id`` after clear is a stale epoch even if
    ``generation`` has not yet incremented to the new spawn.
    """
    if not isinstance(stats, dict):
        return False
    try:
        gen = int(stats.get("generation") or 0)
        expected = int(expected_generation)
    except (TypeError, ValueError):
        return False
    if gen != expected:
        return False
    if expected_id is None:
        return True
    got_id = str(stats.get("instance_id") or "")
    return got_id == str(expected_id)


def tracker_wait_fail_code(stats, expected_generation, expected_id=None):
    """Classify wait_tracked_cargo timeout after YOLO was already ready.

    ``CARGO_NOT_READY``: this spawn's epoch is live but the cargo cloud is
    empty. ``TRACKER_STALE``: filter is still on clear / the previous box.
    """
    if tracker_epoch_matches(stats, expected_generation, expected_id):
        return "CARGO_NOT_READY"
    return "TRACKER_STALE"


def label_aabb(mask, label):
    """Pixel AABB of ``mask == label``, or None when empty."""
    arr = np.asarray(mask)
    if arr.ndim != 2 or arr.size == 0:
        return None
    ys, xs = np.where(arr == int(label))
    if len(xs) == 0:
        return None
    u_min = int(xs.min())
    u_max = int(xs.max())
    v_min = int(ys.min())
    v_max = int(ys.max())
    return {
        "u_min": u_min, "u_max": u_max,
        "v_min": v_min, "v_max": v_max,
        "width_px": u_max - u_min + 1,
        "height_px": v_max - v_min + 1,
        "n": int(len(xs)),
    }


def points_aabb(points, percentile=_VISUAL_PERCENTILE):
    """Percentile AABB of Nx3 points: (dx, dy, dz, n) or None."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if len(pts) < 4:
        return None
    lo, hi = percentile
    spans = []
    for axis in range(3):
        a, b = np.percentile(pts[:, axis], (lo, hi))
        spans.append(float(b - a))
    return (spans[0], spans[1], spans[2], int(len(pts)))


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
    detect_usable: Optional[bool] = None
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
    vac_attach: Optional[bool] = None
    vac_follow: Optional[bool] = None
    wall_time_sec: float = 0.0
    spawn_to_yolo_sec: Optional[float] = None
    spawn_to_visual_sec: Optional[float] = None
    spawn_to_detect_sec: Optional[float] = None
    fail_code: str = ""
    extras: dict = field(default_factory=dict)

    def detect_compared(self):
        return self.accuracy_ok is not None

    def detect_passed(self):
        return bool(self.accuracy_ok)

    def perception_ready(self):
        """True when DetectLuggage returned a perception estimate.

        Falls back to ``detect_passed()`` for jsonl written before
        ``detect_usable`` existed.
        """
        if self.detect_usable is not None:
            return bool(self.detect_usable)
        return self.detect_passed()

    def plan_passed(self):
        return (
            self.perception_ready()
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
    usable = [r for r in records if r.perception_ready()]
    plan_ok = [r for r in usable if r.plan_passed()]
    retreat_ok = [r for r in plan_ok if r.retreat_passed()]
    attach_xy = [r.attach_xy_err for r in records if r.attach_xy_err is not None]
    attach_z = [abs(r.attach_z_err) for r in records if r.attach_z_err is not None]
    attach_xy_gt = [r.attach_xy_gt for r in records if r.attach_xy_gt is not None]
    attach_z_gt = [abs(r.attach_z_gt) for r in records if r.attach_z_gt is not None]
    retreat_dz = [r.retreat_delta_z for r in records if r.retreat_delta_z is not None]
    spawn_to_yolo = [
        r.spawn_to_yolo_sec for r in records
        if r.spawn_to_yolo_sec is not None]
    spawn_to_visual = [
        r.spawn_to_visual_sec for r in records
        if r.spawn_to_visual_sec is not None]
    spawn_to_detect = [
        r.spawn_to_detect_sec for r in records
        if r.spawn_to_detect_sec is not None]
    fail_codes = Counter(r.fail_code for r in records if r.fail_code)
    seg_fails = Counter()
    for rec in records:
        for name, _msg in rec.segment_failures:
            seg_fails[name] += 1
    by_catalog = {}
    for rec in records:
        key = rec.catalog_id or "unknown"
        slot = by_catalog.setdefault(key, {
            "n": 0, "detect_ok": 0, "detect_usable": 0,
            "plan_ok": 0, "retreat_ok": 0})
        slot["n"] += 1
        if rec.detect_passed():
            slot["detect_ok"] += 1
        if rec.perception_ready():
            slot["detect_usable"] += 1
        if rec.plan_passed():
            slot["plan_ok"] += 1
        if rec.retreat_passed():
            slot["retreat_ok"] += 1
    by_visual = {}
    for rec in records:
        key = rec.visual_id or "unknown"
        slot = by_visual.setdefault(key, {
            "n": 0, "detect_ok": 0, "detect_usable": 0,
            "plan_ok": 0, "retreat_ok": 0})
        slot["n"] += 1
        if rec.detect_passed():
            slot["detect_ok"] += 1
        if rec.perception_ready():
            slot["detect_usable"] += 1
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
        "detect_usable_rate": _rate(len(usable), n),
        "n_detect_usable": len(usable),
        "plan_pass_rate": _rate(len(plan_ok), len(usable)),
        "n_detect_ok": len(detect_ok),
        "retreat_pass_rate": _rate(len(retreat_ok), len(plan_ok)),
        "n_plan_ok": len(plan_ok),
        "n_retreat_ok": len(retreat_ok),
        "pick_pass_rate": _rate(len(retreat_ok), n),
        "n_vac_ran": len([r for r in records if r.vac_attach is not None]),
        "n_vac_attach_ok": len([r for r in records if r.vac_attach]),
        "n_vac_follow_ran": len([r for r in records if r.vac_follow is not None]),
        "n_vac_follow_ok": len([r for r in records if r.vac_follow]),
        "vac_attach_rate": _rate(
            len([r for r in records if r.vac_attach]),
            len([r for r in records if r.vac_attach is not None])),
        "vac_follow_rate": _rate(
            len([r for r in records if r.vac_follow]),
            len([r for r in records if r.vac_follow is not None])),
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
        "n_yolo_ready": len(spawn_to_yolo),
        "yolo_ready_rate": _rate(len(spawn_to_yolo), n),
        "n_visual_ready": len(spawn_to_visual),
        "visual_ready_rate": _rate(len(spawn_to_visual), n),
        "spawn_to_yolo_sec": {
            "n": len(spawn_to_yolo),
            "mean": _mean(spawn_to_yolo),
            "std": _std(spawn_to_yolo),
            "p50": _pct(spawn_to_yolo, 50),
            "p95": _pct(spawn_to_yolo, 95),
        },
        "spawn_to_visual_sec": {
            "n": len(spawn_to_visual),
            "mean": _mean(spawn_to_visual),
            "std": _std(spawn_to_visual),
            "p50": _pct(spawn_to_visual, 50),
            "p95": _pct(spawn_to_visual, 95),
        },
        "spawn_to_detect_sec": {
            "n": len(spawn_to_detect),
            "mean": _mean(spawn_to_detect),
            "std": _std(spawn_to_detect),
            "p50": _pct(spawn_to_detect, 50),
            "p95": _pct(spawn_to_detect, 95),
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
