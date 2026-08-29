"""ROS-free stats for a YOLO hit-rate window (no ROS, no YOLO).

Eval-only (``luggage_perception.eval``). Turns a list of per-frame records
into hit rate, miss/hit streaks, latency percentiles, and post-processing
hints. The live driver only records; this module is the authority for the
summary JSON.
"""

from __future__ import division

from collections import Counter

import numpy as np

from luggage_perception.detection_temporal_gate import bbox_iou
from luggage_perception.semantic_segmenter import LABEL_CARGO

GT_IOU_THRESH = 0.3
TEMPORAL_WINDOW_FRAMES = 5
TEMPORAL_MIN_POSITIVE_RATIO = 0.5
TEMPORAL_BBOX_IOU_RESET = 0.3


def aabb_from_uv(uv, valid=None, image_size=None):
    """Axis-aligned pixel box from projected corners. None if empty.

    ``uv`` may be a list or an (N, 2) ndarray (``project_detection`` output).
    Do not boolean-test the array: ``uv or []`` raises ValueError.
    """
    if uv is None:
        return None
    pts = np.asarray(uv, dtype=np.float64)
    if pts.size == 0:
        return None
    pts = pts.reshape(-1, 2)
    if valid is not None:
        mask = np.asarray(valid, dtype=bool).reshape(-1)
        if mask.shape[0] == pts.shape[0]:
            pts = pts[mask]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if pts.shape[0] < 2:
        return None
    x1, y1 = float(pts[:, 0].min()), float(pts[:, 1].min())
    x2, y2 = float(pts[:, 0].max()), float(pts[:, 1].max())
    if image_size is not None and len(image_size) >= 2:
        width, height = int(image_size[0]), int(image_size[1])
        x1 = max(0.0, min(float(width), x1))
        x2 = max(0.0, min(float(width), x2))
        y1 = max(0.0, min(float(height), y1))
        y2 = max(0.0, min(float(height), y2))
    if x2 <= x1 or y2 <= y1:
        return None
    return [int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))]


def best_raw_cargo_det(detections):
    """Largest-area cargo detection that is not a temporal hold."""
    best = None
    best_area = 0.0
    for det in detections or []:
        if int(det.get("label", -1)) != LABEL_CARGO:
            continue
        if det.get("held"):
            continue
        bbox = det.get("bbox")
        if bbox is None or len(bbox) < 4:
            continue
        area = max(0.0, float(bbox[2]) - float(bbox[0])) * max(
            0.0, float(bbox[3]) - float(bbox[1]))
        if area > best_area:
            best_area = area
            best = det
    return best


def best_raw_cargo_bbox(detections):
    det = best_raw_cargo_det(detections)
    if det is None:
        return None
    return [int(round(float(v))) for v in det["bbox"][:4]]


def annotate_gt(frame, gt_bbox, thresh=GT_IOU_THRESH):
    """Set gt_iou / gt_aligned on a frame dict. Returns the same dict."""
    record = dict(frame)
    meas = best_raw_cargo_bbox(record.get("detections"))
    if gt_bbox is None or meas is None:
        record["gt_iou"] = None
        record["gt_aligned"] = False
        return record
    iou = float(bbox_iou(meas, gt_bbox))
    record["gt_iou"] = iou
    record["gt_aligned"] = bool(iou >= float(thresh))
    return record


def run_lengths(flags):
    """[(value, length), ...] for consecutive equal bools."""
    runs = []
    if not flags:
        return runs
    current = bool(flags[0])
    length = 1
    for flag in flags[1:]:
        flag = bool(flag)
        if flag == current:
            length += 1
            continue
        runs.append((current, length))
        current = flag
        length = 1
    runs.append((current, length))
    return runs


def percentile(values, p):
    """Linear percentile. *p* is 0-100. None when empty."""
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * (float(p) / 100.0)
    lo = int(rank)
    hi = min(lo + 1, len(ordered) - 1)
    if lo == hi:
        return ordered[lo]
    frac = rank - lo
    return ordered[lo] + (ordered[hi] - ordered[lo]) * frac


def _streak_stats(lengths):
    if not lengths:
        return {
            "n": 0, "max": 0, "mean": 0.0, "p50": None, "p95": None,
        }
    return {
        "n": len(lengths),
        "max": int(max(lengths)),
        "mean": float(sum(lengths)) / float(len(lengths)),
        "p50": percentile(lengths, 50),
        "p95": percentile(lengths, 95),
    }


def postproc_hints(summary, window_frames=TEMPORAL_WINDOW_FRAMES,
                   iou_reset=TEMPORAL_BBOX_IOU_RESET):
    """Short notes for temporal-gate knobs. Does not change yaml."""
    hints = []
    n_frames = int(summary.get("n_frames") or 0)
    hit_rate = float(summary.get("hit_rate") or 0.0)
    miss = summary.get("miss_streaks") or {}
    max_miss = int(miss.get("max") or 0)
    if n_frames > 0 and hit_rate < 0.3 and max_miss >= max(1, int(0.8 * n_frames)):
        hints.append(
            "hit_rate is low and the longest miss streak covers the window; "
            "temporal hold cannot invent a box")
    if hit_rate >= 0.8 and max_miss <= 2 and n_frames >= window_frames:
        hints.append(
            "max miss streak <= 2 with high hit_rate; temporal_window_frames=%d "
            "may be larger than needed" % int(window_frames))
    n_raw = int(summary.get("n_raw_hit") or 0)
    n_fp = int(summary.get("n_false_positive") or 0)
    dropped = float(summary.get("mean_dropped_self_body") or 0.0)
    if n_raw > 0 and n_fp >= max(2, int(0.3 * n_raw)) and dropped >= 0.5:
        hints.append(
            "false positives with self-body drops; the panel, not the "
            "temporal window, is the main miss/FP source")
    iou_p05 = summary.get("adjacent_bbox_iou_p05")
    if iou_p05 is not None and 0.15 <= float(iou_p05) <= (float(iou_reset) + 0.05):
        hints.append(
            "adjacent-hit bbox IoU p05=%.3f is near temporal_bbox_iou_reset=%.2f; "
            "a jittery box may reset the window" % (float(iou_p05), float(iou_reset)))
    if not hints:
        hints.append("no automatic postproc change suggested from this window")
    return hints


def summarize_window(frames, gt_iou_thresh=GT_IOU_THRESH,
                     window_frames=TEMPORAL_WINDOW_FRAMES):
    """Build the per-visual summary dict from jsonl records."""
    records = list(frames or [])
    n = len(records)
    hits = [bool(r.get("raw_cargo")) for r in records]
    n_raw = sum(1 for h in hits if h)
    n_aligned = sum(1 for r in records if r.get("gt_aligned"))
    n_fp = sum(
        1 for r in records
        if r.get("raw_cargo") and r.get("gt_iou") is not None
        and not r.get("gt_aligned"))
    n_held_rescue = sum(
        1 for r in records if r.get("held") and not r.get("raw_cargo"))
    hit_rate = (float(n_raw) / float(n)) if n else 0.0

    miss_lengths = [length for flag, length in run_lengths(hits) if not flag]
    hit_lengths = [length for flag, length in run_lengths(hits) if flag]

    infer = [float(r["infer_ms"]) for r in records
             if r.get("infer_ms") is not None]
    lag_sim = []
    lag_wall = []
    for record in records:
        image = record.get("image_stamp")
        detect_sim = record.get("detect_sim_stamp")
        if image is not None and detect_sim is not None:
            lag_sim.append(float(detect_sim) - float(image))
        recv = record.get("recv_wall_sec")
        detect_wall = record.get("detect_wall_sec")
        if recv is not None and detect_wall is not None:
            lag_wall.append(float(detect_wall) - float(recv))

    confs = []
    prompts = Counter()
    prev_bbox = None
    adjacent_iou = []
    for record in records:
        if not record.get("raw_cargo"):
            prev_bbox = None
            continue
        det = best_raw_cargo_det(record.get("detections"))
        bbox = best_raw_cargo_bbox(record.get("detections"))
        if det is not None:
            confs.append(float(det.get("confidence", 0.0) or 0.0))
            prompts[str(det.get("prompt") or "?")] += 1
        if prev_bbox is not None and bbox is not None:
            adjacent_iou.append(float(bbox_iou(prev_bbox, bbox)))
        prev_bbox = bbox

    stamps = [float(r["image_stamp"]) for r in records
              if r.get("image_stamp") is not None]
    duration = (stamps[-1] - stamps[0]) if len(stamps) >= 2 else 0.0
    n_flip = 0
    for i in range(1, len(hits)):
        if hits[i] != hits[i - 1]:
            n_flip += 1
    flicker_hz = (float(n_flip) / duration) if duration > 1e-9 else None
    inferred_hz = (
        float(n - 1) / duration if n >= 2 and duration > 1e-9 else None)

    time_to_first = None
    for record in records:
        if record.get("raw_cargo") and stamps:
            try:
                time_to_first = float(record["image_stamp"]) - stamps[0]
            except (TypeError, ValueError, KeyError):
                time_to_first = None
            break

    dropped = [int(r.get("n_dropped_self_body") or 0) for r in records]
    mean_dropped = (
        float(sum(dropped)) / float(len(dropped)) if dropped else 0.0)

    hit_stamps = [
        {
            "image_stamp": r.get("image_stamp"),
            "detect_sim_stamp": r.get("detect_sim_stamp"),
            "detect_wall_sec": r.get("detect_wall_sec"),
            "lag_sim": (
                None if r.get("image_stamp") is None
                or r.get("detect_sim_stamp") is None
                else float(r["detect_sim_stamp"]) - float(r["image_stamp"])),
        }
        for r in records if r.get("raw_cargo")
    ]

    summary = {
        "n_frames": n,
        "n_raw_hit": n_raw,
        "hit_rate": hit_rate,
        "n_gt_aligned": n_aligned,
        "n_false_positive": n_fp,
        "n_held_rescue": n_held_rescue,
        "gt_iou_thresh": float(gt_iou_thresh),
        "miss_streaks": _streak_stats(miss_lengths),
        "hit_streaks": _streak_stats(hit_lengths),
        "adjacent_bbox_iou_p05": percentile(adjacent_iou, 5),
        "adjacent_bbox_iou_p50": percentile(adjacent_iou, 50),
        "confidence_p50": percentile(confs, 50),
        "confidence_p95": percentile(confs, 95),
        "prompt_counts": dict(prompts),
        "infer_ms_p50": percentile(infer, 50),
        "infer_ms_p95": percentile(infer, 95),
        "lag_sim_p50": percentile(lag_sim, 50),
        "lag_sim_p95": percentile(lag_sim, 95),
        "lag_wall_p50": percentile(lag_wall, 50),
        "lag_wall_p95": percentile(lag_wall, 95),
        "time_to_first_raw_cargo_sec": time_to_first,
        "flicker_hz": flicker_hz,
        "inferred_hz": inferred_hz,
        "duration_sec": duration,
        "mean_dropped_self_body": mean_dropped,
        "n_hit_stamp_rows": len(hit_stamps),
        "hit_stamps": hit_stamps,
        "temporal_window_frames": int(window_frames),
        "temporal_min_positive_ratio": TEMPORAL_MIN_POSITIVE_RATIO,
        "temporal_bbox_iou_reset": TEMPORAL_BBOX_IOU_RESET,
    }
    summary["postproc_hints"] = postproc_hints(
        summary, window_frames=window_frames)
    return summary
