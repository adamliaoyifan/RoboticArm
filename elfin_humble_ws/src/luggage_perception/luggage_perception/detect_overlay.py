#!/usr/bin/env python3
"""Pickup-box detection overlay core (no ROS, no rospy).

Turns one detection result (world-frame centre + orientation + extents) into
the two things a reviewer needs to judge it at a glance:

  - an oriented bounding box + centre projected into the colour image, and
  - the 3-D corner set the RViz marker draws.

The detector estimates the box in the world frame while the debug image is in
pixels, so the only non-trivial step is the projection. It uses the pinhole
model, taking the extrinsic from TF (supplied by the caller as R, t) and the
intrinsics from the ``camera_info`` that belongs to the image being drawn on.
That keeps the module honest for both the simulated and the real camera --
neither the frame nor the intrinsics are hardcoded here.

All math is plain numpy so this is unit-testable without a roscore, and cv2 is
imported lazily inside the drawing helpers so the geometry can be tested in a
bare environment.
"""

from __future__ import division

from collections import namedtuple

import numpy as np

# Corner/edge order matches scene_manager's pickup-box wireframe so the 2-D
# overlay and the 3-D marker describe the same box in the same order.
_CORNER_SIGNS = (
    (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
    (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
)

OBB_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)

# BGR. Perception is cyan, the GT fallback orange, so a run that quietly fell
# back to ground truth cannot be mistaken for a real measurement.
COLOR_PERCEPTION_BGR = (255, 255, 0)
COLOR_GT_FALLBACK_BGR = (0, 165, 255)
COLOR_FAILURE_BGR = (0, 0, 255)

_MIN_PROJECTION_Z = 1e-6

DetectionRecord = namedtuple(
    "DetectionRecord",
    ["success", "source", "reason", "confidence", "position", "quat", "size"],
)


def parse_detection_record(record):
    """Normalise a detector diagnostics dict into a DetectionRecord.

    Returns ``None`` when the payload is not a usable detection record at all.
    A well-formed failure (``success`` false, no ``detected`` block) parses
    into a record with ``position``/``quat``/``size`` set to ``None`` -- the
    caller needs that to clear a stale box rather than keep showing it.
    """
    if not isinstance(record, dict):
        return None
    if "success" not in record:
        return None

    success = bool(record.get("success", False))
    detected = record.get("detected")
    position = quat = size = None
    if isinstance(detected, dict):
        try:
            position = np.asarray(detected["position"], dtype=np.float64)
            quat = np.asarray(detected["orientation"], dtype=np.float64)
            size = np.asarray(detected["size"], dtype=np.float64)
        except (KeyError, TypeError, ValueError):
            position = quat = size = None
        else:
            if position.shape != (3,) or quat.shape != (4,) or size.shape != (3,):
                position = quat = size = None

    if success and position is None:
        # Reported success without a usable pose: treat as a failure so the
        # overlay clears instead of freezing on the previous box.
        success = False

    return DetectionRecord(
        success=success,
        source=str(record.get("source", "unknown")),
        reason=str(record.get("reason", "")),
        confidence=float(record.get("confidence", 0.0) or 0.0),
        position=position,
        quat=quat,
        size=size,
    )


def rotation_from_quaternion(quat_xyzw):
    """Rotation matrix from an (x, y, z, w) quaternion."""
    q = np.asarray(quat_xyzw, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = q / norm
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def obb_corners_world(position, quat_xyzw, size):
    """Eight OBB corners in the frame ``position`` is expressed in.

    ``size`` is (width, depth, height) -- the same order the detector and the
    ``DetectedLuggage`` message use -- and ``position`` is the geometric
    centre, so the corners are centre +- R * (w/2, d/2, h/2).
    """
    centre = np.asarray(position, dtype=np.float64).reshape(3)
    half = np.asarray(size, dtype=np.float64).reshape(3) * 0.5
    rot = rotation_from_quaternion(quat_xyzw)
    local = np.asarray(_CORNER_SIGNS, dtype=np.float64) * half
    return local.dot(rot.T) + centre


def transform_points(points, rotation, translation):
    """Apply p_out = R * p_in + t to an (N, 3) array."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    rot = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trans = np.asarray(translation, dtype=np.float64).reshape(3)
    return pts.dot(rot.T) + trans


def project_points(points_camera, intrinsics):
    """Project (N, 3) optical-frame points to pixels with a pinhole model.

    ``intrinsics`` is (fx, fy, cx, cy). Returns ``(uv, valid)`` where ``uv`` is
    an (N, 2) float array and ``valid`` is an (N,) bool mask that is False for
    points at or behind the image plane. Invalid rows are NaN so a caller that
    ignores the mask produces nothing drawable instead of a plausible-looking
    pixel behind the camera.
    """
    pts = np.asarray(points_camera, dtype=np.float64).reshape(-1, 3)
    fx, fy, cx, cy = (float(v) for v in intrinsics)

    uv = np.full((pts.shape[0], 2), np.nan, dtype=np.float64)
    if pts.shape[0] == 0:
        return uv, np.zeros((0,), dtype=bool)

    z = pts[:, 2]
    valid = z > _MIN_PROJECTION_Z
    if np.any(valid):
        z_valid = z[valid]
        uv[valid, 0] = fx * pts[valid, 0] / z_valid + cx
        uv[valid, 1] = fy * pts[valid, 1] / z_valid + cy
    return uv, valid


def project_detection(position, quat_xyzw, size, rotation, translation,
                      intrinsics):
    """World-frame box -> (centre_uv, corner_uv, corner_valid, centre_valid).

    ``centre_uv`` is None when the centre is behind the camera.
    """
    corners_world = obb_corners_world(position, quat_xyzw, size)
    centre_world = np.asarray(position, dtype=np.float64).reshape(1, 3)

    corners_cam = transform_points(corners_world, rotation, translation)
    centre_cam = transform_points(centre_world, rotation, translation)

    corner_uv, corner_valid = project_points(corners_cam, intrinsics)
    centre_uv, centre_valid = project_points(centre_cam, intrinsics)

    centre = centre_uv[0] if bool(centre_valid[0]) else None
    return centre, corner_uv, corner_valid, bool(centre_valid[0])


def source_color_bgr(source):
    """Overlay colour for a detection source label."""
    if str(source) == "gt_fallback":
        return COLOR_GT_FALLBACK_BGR
    return COLOR_PERCEPTION_BGR


def format_detection_label(record):
    """One-line summary drawn next to the box."""
    size = record.size if record.size is not None else (0.0, 0.0, 0.0)
    return "%s %.2fx%.2fx%.2f conf=%.2f" % (
        record.source, size[0], size[1], size[2], record.confidence)


def _finite_point(uv):
    return uv is not None and np.all(np.isfinite(uv))


def draw_detection_overlay(bgr, centre_uv, corner_uv, corner_valid, label,
                           color, label_origin=(8, 8)):
    """Draw the OBB wireframe, the centre cross and a label onto a BGR image.

    Edges with an endpoint behind the camera are skipped rather than clipped:
    a partially visible box should lose the off-screen edges, not grow a
    fabricated one across the frame.
    """
    import cv2  # noqa: WPS433  lazy: keep the geometry importable without cv2

    out = np.array(bgr, copy=True)
    corner_uv = np.asarray(corner_uv, dtype=np.float64).reshape(-1, 2)
    corner_valid = np.asarray(corner_valid, dtype=bool).reshape(-1)

    for a, b in OBB_EDGES:
        if a >= corner_uv.shape[0] or b >= corner_uv.shape[0]:
            continue
        if not (corner_valid[a] and corner_valid[b]):
            continue
        pa = corner_uv[a]
        pb = corner_uv[b]
        if not (_finite_point(pa) and _finite_point(pb)):
            continue
        cv2.line(out, (int(round(pa[0])), int(round(pa[1]))),
                 (int(round(pb[0])), int(round(pb[1]))), color, 2, cv2.LINE_AA)

    if _finite_point(centre_uv):
        cu, cv_ = int(round(centre_uv[0])), int(round(centre_uv[1]))
        cv2.circle(out, (cu, cv_), 5, color, 2, cv2.LINE_AA)
        cv2.line(out, (cu - 12, cv_), (cu + 12, cv_), color, 1, cv2.LINE_AA)
        cv2.line(out, (cu, cv_ - 12), (cu, cv_ + 12), color, 1, cv2.LINE_AA)

    if label:
        _draw_label(out, label, color, (0, 0, 0), origin=label_origin)
    return out


def draw_failure_overlay(bgr, reason):
    """Return the frame with a failure banner and no box.

    A failure still publishes an image: going silent would leave the last
    successful frame latched in RViz, which reads as "still detected".
    """
    out = np.array(bgr, copy=True)
    text = ("no detection: %s" % reason) if reason else "no detection"
    _draw_label(out, text, COLOR_FAILURE_BGR, (255, 255, 255))
    return out


def _draw_label(bgr, text, color, text_color, origin=(8, 8)):
    import cv2  # noqa: WPS433  lazy: see draw_detection_overlay

    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    x, y = origin
    cv2.rectangle(bgr, (x, y), (x + tw + 8, y + th + 8), color, -1)
    cv2.putText(bgr, text, (x + 4, y + th + 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1, cv2.LINE_AA)


def draw_timestamp_banner(image, lines, aligned=True, is_bgr=False):
    """Burn stamp/latency lines onto the bottom of an HxWx3 image.

    *image* is RGB unless *is_bgr* is True. Returns a copy. cv2 is lazy.
    """
    import cv2  # noqa: WPS433  lazy: see draw_detection_overlay

    out = np.array(image, copy=True)
    if out.ndim != 3 or out.shape[2] != 3:
        return out
    bgr = out if is_bgr else out[:, :, ::-1].copy()
    height = int(bgr.shape[0])
    rows = [str(line) for line in (lines or []) if str(line)]
    if not rows:
        return out
    bar = (0, 160, 0) if aligned else (0, 0, 220)
    y = height - 8
    for text in reversed(rows):
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        y -= th + 10
        if y < 0:
            break
        cv2.rectangle(bgr, (6, y - 2), (14 + tw, y + th + 6), bar, -1)
        cv2.putText(bgr, text, (10, y + th + 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)
    if is_bgr:
        return bgr
    return bgr[:, :, ::-1]


def format_stamp_sec(stamp):
    if stamp is None:
        return "none"
    try:
        return "%.3f" % float(stamp)
    except (TypeError, ValueError):
        return "none"


def stamp_alignment(left_stamp, right_stamp, tol=1e-6):
    """Return (aligned, right_minus_left_sec)."""
    if left_stamp is None or right_stamp is None:
        return False, None
    try:
        delta = float(right_stamp) - float(left_stamp)
    except (TypeError, ValueError):
        return False, None
    return abs(delta) <= float(tol), delta


def timestamp_banner_lines(color_stamp, overlay_stamp, dump_stamp=None,
                           infer_ms=None, cargo_stamp=None, pub_ms=None,
                           detect_stamp=None):
    """Lines burned onto RGB/overlay to confirm they are the same camera frame."""
    aligned, delta = stamp_alignment(color_stamp, overlay_stamp)
    lines = [
        "raw %s" % format_stamp_sec(color_stamp),
        "overlay %s" % format_stamp_sec(overlay_stamp),
    ]
    if aligned:
        lines.append("MATCH lat=%s" % (
            "0.000s" if delta is None else "%.3fs" % abs(delta)))
    else:
        lines.append("MISMATCH lat=%s" % (
            "none" if delta is None else "%+.3fs" % delta))
    if infer_ms is not None:
        try:
            lines.append("infer %.1fms" % float(infer_ms))
        except (TypeError, ValueError):
            pass
    if pub_ms is not None:
        try:
            lines.append("pub %.1fms" % float(pub_ms))
        except (TypeError, ValueError):
            pass
    if dump_stamp is not None:
        lag = None
        if color_stamp is not None:
            try:
                lag = float(dump_stamp) - float(color_stamp)
            except (TypeError, ValueError):
                lag = None
        if lag is None:
            lines.append("dump %s" % format_stamp_sec(dump_stamp))
        else:
            lines.append("dump %s lag=%.3fs" % (
                format_stamp_sec(dump_stamp), lag))
    if cargo_stamp is not None:
        cargo_ok, cargo_dt = stamp_alignment(color_stamp, cargo_stamp)
        lines.append("cargo %s %s" % (
            format_stamp_sec(cargo_stamp),
            "MATCH" if cargo_ok else (
                "MISMATCH" if cargo_dt is None else "MISMATCH %+.3fs" % cargo_dt)))
    if detect_stamp is not None:
        detect_ok, detect_dt = stamp_alignment(color_stamp, detect_stamp)
        lines.append("detect %s %s" % (
            format_stamp_sec(detect_stamp),
            "MATCH" if detect_ok else (
                "MISMATCH" if detect_dt is None else "MISMATCH %+.3fs" % detect_dt)))
    meta = {
        "color_stamp": color_stamp,
        "overlay_stamp": overlay_stamp,
        "dump_stamp": dump_stamp,
        "cargo_stamp": cargo_stamp,
        "detect_stamp": detect_stamp,
        "aligned": bool(aligned),
        "overlay_minus_raw_sec": delta,
        "infer_ms": infer_ms,
        "pub_ms": pub_ms,
    }
    if dump_stamp is not None and color_stamp is not None:
        try:
            meta["dump_lag_sec"] = float(dump_stamp) - float(color_stamp)
        except (TypeError, ValueError):
            meta["dump_lag_sec"] = None
    if cargo_stamp is not None:
        cargo_ok, cargo_dt = stamp_alignment(color_stamp, cargo_stamp)
        meta["cargo_matched"] = bool(cargo_ok)
        meta["cargo_minus_raw_sec"] = cargo_dt
    if detect_stamp is not None:
        detect_ok, detect_dt = stamp_alignment(color_stamp, detect_stamp)
        meta["detect_matched"] = bool(detect_ok)
        meta["detect_minus_raw_sec"] = detect_dt
    return lines, meta
