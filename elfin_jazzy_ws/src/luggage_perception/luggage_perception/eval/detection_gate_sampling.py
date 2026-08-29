#!/usr/bin/env python3
"""Sampling-driver rules for DetectLuggage vs GetCurrentBox. No ROS.

Eval-only: imported by ``scripts/detection_gt_gate_run.py``, not by the
live detector. The Todo 2 driver used to treat GT fallback as a real
measurement and to treat a latched /luggage/preprocessed/status as "arm is
stable now". Both produce fake passes. These helpers are the gate for what
counts.
"""

from __future__ import division

import json
import math
import os
import re
import struct
import zlib
from collections import Counter

import numpy as np

from luggage_perception.eval.detection_accuracy import (
    BoxObservation, DetectionAccuracy,
)
from luggage_perception.detect_overlay import (
    COLOR_PERCEPTION_BGR,
    draw_detection_overlay,
    draw_timestamp_banner,
    project_detection,
    timestamp_banner_lines,
)

FRESH_SEC = 2.0
GT_FALLBACK_TOKEN = "gt fallback"
_BOX_FIELDS = ("x", "y", "z", "yaw", "width", "depth", "height")

# RGB legend for dumped mask.png (cargo red). Pixel value in the raw
# mono8 mask is the integer label, 0-4, which looks black in an image viewer.
MASK_VIZ_RGB = {
    0: (40, 40, 40),       # background
    1: (180, 180, 180),    # container_wall
    2: (220, 0, 0),        # cargo
    3: (0, 220, 220),      # robot_arm
    4: (255, 140, 0),      # unknown_object
}
MASK_LABEL_NAMES = {
    0: "background",
    1: "container_wall",
    2: "cargo",
    3: "robot_arm",
    4: "unknown_object",
}
MASK_MEANING = (
    "HxW uint8 class id used to route points: "
    "0=background, 1=container_wall, 2=cargo, "
    "3=robot_arm, 4=unknown_object. "
    "semantic_point_filter keeps cargo (2). "
    "mask.png is a colorized preview; raw ids 0-4 look black."
)
FRAME_JOIN_WAIT_SEC = 1.0
# Post-spawn dump/detect must wait for overlay/cargo newer than the pre-spawn
# primary_stamp. 1s is enough to join in-buffer frames, not to get a new one.
FRAME_JOIN_AFTER_SPAWN_SEC = 5.0

# Dump overlay: green = GetCurrentBox, cyan = DetectLuggage 3D OBB.
# YOLO 2D boxes stay as the segmenter drew them.
COLOR_GT_BOX_BGR = (0, 255, 0)
COLOR_MEASURED_BOX_BGR = COLOR_PERCEPTION_BGR
OVERLAY_BOX_LEGEND = {
    "gt": "green 3D OBB (GetCurrentBox)",
    "measured": "cyan 3D OBB (DetectLuggage)",
}


def is_gt_fallback_message(message):
    return GT_FALLBACK_TOKEN in (message or "").lower()


def diag_as_dict(diag):
    """diagnostics_json payload: dict, JSON string, or None."""
    if diag is None or diag == "":
        return {}
    if isinstance(diag, dict):
        return diag
    try:
        data = json.loads(diag)
    except (TypeError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def is_gt_fallback_diag(diag):
    return diag_as_dict(diag).get("source") == "gt_fallback"


def is_gt_fallback(message, diag=None):
    return is_gt_fallback_message(message) or is_gt_fallback_diag(diag)


def perception_reason(message, diag=None):
    """Underlying detector reason; for GT fallback this is STALE/NO_CLOUD/…"""
    data = diag_as_dict(diag)
    reason = data.get("reason")
    if reason and reason not in ("ok", "not_run"):
        return str(reason)
    if is_gt_fallback(message, diag):
        return "DETECT_GT_FALLBACK"
    return None


def is_perception_estimate(success, message, has_luggage, diag=None):
    if is_gt_fallback(message, diag):
        return False
    return bool(success) and bool(has_luggage)


def status_is_stable(data):
    if not isinstance(data, dict):
        return False
    flags = data.get("flags") or {}
    gate = data.get("motion_gate") or {}
    return bool(flags.get("geometry_ok")) and gate.get("state") == "stable"


def receipt_is_fresh(recv_time, now, wait_started, max_age=FRESH_SEC):
    if recv_time is None:
        return False
    if recv_time < wait_started:
        return False
    return (now - recv_time) <= float(max_age)


def primary_stamp_is_new(current_stamp, stamp_at_wait_start):
    """True when status carries a newer observation than at wait start.

    The preprocessor 1 Hz timer republishes the last flags without a new
    camera frame. primary_stamp only advances on a real emit.
    """
    try:
        current = float(current_stamp)
        baseline = float(stamp_at_wait_start)
    except (TypeError, ValueError):
        return False
    return current > baseline + 1e-6


def wait_ready(status_data, cloud_recv, stamp_at_start, wait_started, now,
               fresh_sec=FRESH_SEC):
    """True when geometry_ok is from a live observation, not a latch.

    Requires a point cloud received after *wait_started* (and within
    *fresh_sec*) plus a status primary_stamp newer than the snapshot taken
    when the wait began. Status recv_time alone is not enough: TRANSIENT_LOCAL
    delivers the last latch on subscribe, and the 1 Hz status timer keeps
    that latch looking fresh.
    """
    if not status_is_stable(status_data):
        return False
    if not receipt_is_fresh(cloud_recv, now, wait_started, fresh_sec):
        return False
    return primary_stamp_is_new(
        (status_data or {}).get("primary_stamp"), stamp_at_start)


def observation_from_dict(payload):
    return BoxObservation(**{k: payload[k] for k in _BOX_FIELDS})


def trial_has_perception_result(record):
    if not record or record.get("failure"):
        return False
    if not record.get("result") or not record.get("measured") or not record.get("gt"):
        return False
    return is_perception_estimate(
        success=True,
        message=record.get("detect_message"),
        has_luggage=True,
        diag=record.get("diag"),
    )


def trial_failure_code(record):
    if not record:
        return "UNKNOWN"
    if record.get("failure"):
        return str(record["failure"])
    if is_gt_fallback(record.get("detect_message"), record.get("diag")):
        reason = perception_reason(record.get("detect_message"), record.get("diag"))
        if reason and reason != "DETECT_GT_FALLBACK":
            return "DETECT_GT_FALLBACK:%s" % reason
        return "DETECT_GT_FALLBACK"
    if record.get("detect_failure"):
        return str(record["detect_failure"])
    if record.get("result") is None:
        return "MEASURED_NONE"
    return None


def format_trial_line(index, record):
    code = trial_failure_code(record)
    if code:
        return "trial %02d: %s" % (index, code)
    result = record["result"]
    return "trial %02d: ok=%s iou=%.3f err_xy=%.3f" % (
        index, result["ok"], result["iou"], result["err_xy"])


def trial_should_dump_failure(record):
    """True when the trial is not a passing perception-vs-GT gate."""
    if trial_failure_code(record):
        return True
    result = (record or {}).get("result")
    if isinstance(result, dict) and result.get("ok") is False:
        return True
    return False


def failure_dump_stem(record):
    """Directory name under the dump root: trial_03_gate_size."""
    index = int((record or {}).get("index") or 0)
    code = trial_failure_code(record)
    if not code:
        reason = ((record or {}).get("result") or {}).get("reason") or "fail"
        code = "gate_%s" % reason
    slug = re.sub(r"[^A-Za-z0-9._-]+", "_", str(code)).strip("_")[:80]
    if not slug:
        slug = "fail"
    return "trial_%02d_%s" % (index, slug)


def write_png(path, arr):
    """Write HxW (grey) or HxWx3 (RGB) uint8 as a PNG. No cv2."""
    arr = np.asarray(arr)
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    if arr.ndim == 2:
        arr = arr[:, :, None]
    if arr.ndim != 3 or arr.shape[2] not in (1, 3):
        raise ValueError("write_png expects HxW or HxWx3, got %r" % (arr.shape,))
    height, width, channels = arr.shape
    color_type = 2 if channels == 3 else 0
    raw = b"".join(b"\x00" + arr[row].tobytes() for row in range(height))

    def _chunk(tag, data):
        body = tag + data
        return (struct.pack(">I", len(data)) + body
                + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF))

    png = (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(
            ">IIBBBBB", width, height, 8, color_type, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(raw, 6))
        + _chunk(b"IEND", b"")
    )
    with open(path, "wb") as handle:
        handle.write(png)


def stamp_key_from_sec(stamp_sec):
    """Round-trip a float stamp to the (sec, nsec) header key."""
    stamp_sec = float(stamp_sec)
    whole = int(stamp_sec)
    nsec = int(round((stamp_sec - whole) * 1e9))
    if nsec >= 1000000000:
        whole += 1
        nsec -= 1000000000
    if nsec < 0:
        whole -= 1
        nsec += 1000000000
    return (whole, nsec)


def stamp_sec_from_key(key):
    """(sec, nsec) header key -> float seconds."""
    if key is None:
        return None
    return float(key[0]) + 1e-9 * float(key[1])


def pick_joined_stamp(buffers, required=("color", "depth", "overlay"),
                      min_stamp_sec=None):
    """Newest stamp present on every required stream.

    *buffers* maps stream name -> {stamp_key: payload}. Overlay/mask inherit
    the RGB header stamp, and the preprocessor emits color+depth with the
    same primary_stamp, so a hit is one camera frame.

    *min_stamp_sec*, when set, drops keys at or before that stamp so a dump
    cannot reuse a pre-spawn triplet that is still in the keep-last buffer.
    """
    if not buffers:
        return None
    keys = None
    for name in required:
        present = set((buffers.get(name) or {}).keys())
        keys = present if keys is None else keys.intersection(present)
        if not keys:
            return None
    if min_stamp_sec is not None:
        floor = float(min_stamp_sec) + 1e-6
        keys = {key for key in keys if stamp_sec_from_key(key) > floor}
        if not keys:
            return None
    return max(keys)


def apply_dump_timestamp_banners(images, extras, dump_stamp=None,
                                 cargo_stamp=None, infer_ms=None, pub_ms=None,
                                 detect_stamp=None):
    """Draw raw/overlay/dump stamps on color.png and overlay.png copies."""
    extras = dict(extras or {})
    color_meta = extras.get("color") or {}
    overlay_meta = extras.get("overlay") or {}
    cargo_meta = extras.get("cargo") or {}
    if cargo_stamp is None:
        cargo_stamp = cargo_meta.get("stamp")
    infer_from_stats = infer_ms
    if infer_from_stats is None:
        stats = extras.get("seg_stats") or {}
        join_stamp = extras.get("join_stamp")
        stats_stamp = stats.get("stamp")
        infer_ok = True
        if join_stamp is not None and stats_stamp is not None:
            try:
                infer_ok = abs(float(join_stamp) - float(stats_stamp)) <= 1e-3
            except (TypeError, ValueError):
                infer_ok = False
        if infer_ok:
            infer_from_stats = stats.get("inference_ms")
    lines, meta = timestamp_banner_lines(
        color_meta.get("stamp"), overlay_meta.get("stamp"),
        dump_stamp=dump_stamp, infer_ms=infer_from_stats,
        cargo_stamp=cargo_stamp, pub_ms=pub_ms, detect_stamp=detect_stamp)
    color_arr = (images or {}).get("color")
    overlay_arr = (images or {}).get("overlay")
    if color_arr is not None and overlay_arr is not None:
        try:
            meta["same_hw"] = tuple(color_arr.shape[:2]) == tuple(overlay_arr.shape[:2])
        except (TypeError, AttributeError):
            meta["same_hw"] = False
    else:
        meta["same_hw"] = False
    extras["stamp_check"] = meta
    extras["aligned"] = (
        bool(meta.get("aligned"))
        and bool(extras.get("aligned"))
        and bool(meta.get("same_hw")))
    if cargo_stamp is not None:
        extras["aligned"] = extras["aligned"] and bool(meta.get("cargo_matched"))
    if detect_stamp is not None:
        extras["aligned"] = extras["aligned"] and bool(meta.get("detect_matched"))
    try:
        import cv2  # noqa: F401,WPS433
    except ImportError:
        extras["stamp_check"]["banner"] = "no_cv2"
        return images, extras
    out = dict(images or {})
    banner_ok = bool(meta.get("aligned")) and bool(meta.get("same_hw", True))
    if cargo_stamp is not None:
        banner_ok = banner_ok and bool(meta.get("cargo_matched"))
    if detect_stamp is not None:
        banner_ok = banner_ok and bool(meta.get("detect_matched"))
    for name in ("color", "overlay"):
        arr = out.get(name)
        if arr is None:
            continue
        out[name] = draw_timestamp_banner(arr, lines, aligned=banner_ok)
    return out, extras


def colorize_mask_rgb(label_map):
    """HxWx3 RGB preview of a uint8 label map (0-4). Raw mask is unreadable."""
    labels = np.asarray(label_map)
    if labels.ndim != 2:
        raise ValueError("label_map must be HxW, got %r" % (labels.shape,))
    out = np.zeros((labels.shape[0], labels.shape[1], 3), dtype=np.uint8)
    for label_id, rgb in MASK_VIZ_RGB.items():
        out[labels == int(label_id)] = rgb
    return out


def depth_vis_uint8(depth_m, max_m=2.5):
    """0..max_m metres -> grey PNG; non-finite -> 0."""
    dep = np.asarray(depth_m, dtype=np.float32)
    vis = np.clip(np.nan_to_num(dep, nan=0.0, posinf=0.0, neginf=0.0), 0.0, max_m)
    return (vis / float(max_m) * 255.0).astype(np.uint8)


def build_aligned_dump(color_rgb, depth_m, overlay_rgb, mask_labels=None):
    """PNG arrays + depth.npy payload for one stamp-joined camera frame."""
    images = {}
    arrays = {}
    if color_rgb is not None:
        images["color"] = np.asarray(color_rgb)
    if overlay_rgb is not None:
        images["overlay"] = np.asarray(overlay_rgb)
    if depth_m is not None:
        depth_m = np.asarray(depth_m, dtype=np.float32)
        images["depth"] = depth_vis_uint8(depth_m)
        arrays["depth"] = depth_m
    if mask_labels is not None:
        labels = np.asarray(mask_labels)
        images["mask"] = colorize_mask_rgb(labels)
        arrays["mask_labels"] = labels
    extras = {
        "mask_meaning": MASK_MEANING,
        "mask_legend": dict(MASK_LABEL_NAMES),
        "mask_viz_rgb": {str(k): list(v) for k, v in MASK_VIZ_RGB.items()},
        "overlay_box_legend": dict(OVERLAY_BOX_LEGEND),
    }
    return images, arrays, extras


def quat_xyzw_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def pose_size_from_observation(obs):
    """BoxObservation or dict -> (position, quat_xyzw, size) or None."""
    if obs is None:
        return None
    if hasattr(obs, "x"):
        fields = {
            "x": obs.x, "y": obs.y, "z": obs.z, "yaw": obs.yaw,
            "width": obs.width, "depth": obs.depth, "height": obs.height,
        }
    elif isinstance(obs, dict):
        try:
            fields = {k: obs[k] for k in _BOX_FIELDS}
        except KeyError:
            return None
    else:
        return None
    try:
        position = (
            float(fields["x"]), float(fields["y"]), float(fields["z"]))
        size = (
            float(fields["width"]), float(fields["depth"]),
            float(fields["height"]))
        yaw = float(fields["yaw"])
    except (TypeError, ValueError):
        return None
    return position, quat_xyzw_from_yaw(yaw), size


def projected_uv_span(corner_uv, corner_valid):
    """Axis-aligned pixel span of the projected OBB corners."""
    corners = np.asarray(corner_uv, dtype=np.float64).reshape(-1, 2)
    valid = np.asarray(corner_valid, dtype=bool).reshape(-1)
    if corners.shape[0] == 0 or valid.shape[0] != corners.shape[0]:
        return None
    finite = valid & np.isfinite(corners).all(axis=1)
    if not np.any(finite):
        return None
    pts = corners[finite]
    u_min = float(pts[:, 0].min())
    u_max = float(pts[:, 0].max())
    v_min = float(pts[:, 1].min())
    v_max = float(pts[:, 1].max())
    return {
        "u_min": u_min, "u_max": u_max,
        "v_min": v_min, "v_max": v_max,
        "width_px": u_max - u_min,
        "height_px": v_max - v_min,
    }


def project_box_observation(obs, rotation, translation, intrinsics):
    """World-frame box -> centre/corners in pixels. None if obs is unusable."""
    pose = pose_size_from_observation(obs)
    if pose is None:
        return None
    position, quat, size = pose
    centre, corner_uv, corner_valid, centre_ok = project_detection(
        position, quat, size, rotation, translation, intrinsics)
    span = projected_uv_span(corner_uv, corner_valid)
    return {
        "centre_uv": (
            None if centre is None
            else [float(centre[0]), float(centre[1])]),
        "centre_valid": bool(centre_ok),
        "corner_uv": np.asarray(corner_uv, dtype=np.float64),
        "corner_valid": np.asarray(corner_valid, dtype=bool),
        "span": span,
        "size": [float(v) for v in size],
    }


def _box_label(prefix, size):
    return "%s %.2fx%.2fx%.2f" % (prefix, size[0], size[1], size[2])


def annotate_overlay_boxes(
        overlay_rgb, gt, measured, rotation, translation, intrinsics):
    """Draw GT (green) and measured (cyan) 3D OBBs onto an RGB overlay.

    Returns ``(rgb, meta)``. On missing inputs or no cv2, returns the original
    image and ``boxes_projected=False``. Geometry is still recorded in *meta*
    when the projection itself succeeds.
    """
    meta = {
        "boxes_projected": False,
        "overlay_box_legend": dict(OVERLAY_BOX_LEGEND),
    }
    if overlay_rgb is None:
        meta["project_error"] = "no_overlay"
        return overlay_rgb, meta
    if rotation is None or translation is None or intrinsics is None:
        meta["project_error"] = "no_camera"
        return overlay_rgb, meta

    rgb = np.asarray(overlay_rgb)
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        meta["project_error"] = "overlay_not_rgb"
        return overlay_rgb, meta

    projections = {}
    if gt is not None:
        projections["gt"] = project_box_observation(
            gt, rotation, translation, intrinsics)
    if measured is not None:
        projections["measured"] = project_box_observation(
            measured, rotation, translation, intrinsics)

    for name, proj in projections.items():
        if proj is None:
            continue
        meta.setdefault("overlay_boxes", {})[name] = {
            "centre_uv": proj["centre_uv"],
            "centre_valid": proj["centre_valid"],
            "span": proj["span"],
            "size": proj["size"],
        }

    try:
        import cv2  # noqa: F401,WPS433  lazy: dump path only
    except ImportError:
        meta["project_error"] = "no_cv2"
        return overlay_rgb, meta

    bgr = rgb[:, :, ::-1].copy()
    label_y = 8
    for name, prefix, color in (
            ("gt", "GT", COLOR_GT_BOX_BGR),
            ("measured", "meas", COLOR_MEASURED_BOX_BGR)):
        proj = projections.get(name)
        if not proj:
            continue
        bgr = draw_detection_overlay(
            bgr, proj["centre_uv"], proj["corner_uv"], proj["corner_valid"],
            _box_label(prefix, proj["size"]), color,
            label_origin=(8, label_y))
        label_y += 28
        meta["boxes_projected"] = True

    if not meta["boxes_projected"]:
        meta["project_error"] = "no_boxes"
        return overlay_rgb, meta
    return bgr[:, :, ::-1], meta


def dump_failure_bundle(dump_dir, record, images=None, extras=None, arrays=None):
    """Write one failed trial's JSON + PNG frames. Returns the folder or None."""
    if not dump_dir or not trial_should_dump_failure(record):
        return None
    dest = os.path.join(dump_dir, failure_dump_stem(record))
    os.makedirs(dest, exist_ok=True)
    payload = dict(record)
    if extras:
        payload["frame_meta"] = extras
    with open(os.path.join(dest, "trial.json"), "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, default=str)
        handle.write("\n")
    for name, arr in (images or {}).items():
        if arr is None:
            continue
        write_png(os.path.join(dest, "%s.png" % name), arr)
    for name, arr in (arrays or {}).items():
        if arr is None:
            continue
        np.save(os.path.join(dest, "%s.npy" % name), arr)
    return dest


def summarize_trial_records(records, accuracy=None):
    """Build the gate summary. GT fallback never enters n_with_result."""
    accuracy = accuracy or DetectionAccuracy()
    parsed = [rec for rec in records if trial_has_perception_result(rec)]
    results = [
        accuracy.compare(
            observation_from_dict(rec["measured"]),
            observation_from_dict(rec["gt"]),
        )
        for rec in parsed
    ]
    summary = accuracy.summarize(results)
    codes = Counter(
        trial_failure_code(rec) or "ok" for rec in records)
    summary["n_trials"] = len(records)
    summary["n_with_result"] = len(parsed)
    summary["n_gt_fallback"] = sum(
        1 for rec in records
        if is_gt_fallback(rec.get("detect_message"), rec.get("diag")))
    summary["failure_counts"] = dict(sorted(codes.items()))
    return summary
