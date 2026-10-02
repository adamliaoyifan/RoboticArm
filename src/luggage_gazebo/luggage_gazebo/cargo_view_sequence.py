#!/usr/bin/env python3
"""Post-place cargo view order, aim candidates, and stop rule. No ROS."""

from __future__ import division

import hashlib
import json
import math

from luggage_perception.cargo_instance_tracker import rotation_from_xyzw
from luggage_perception.cargo_view_integration import (
    box_point_mask,
    cell_xy_container,
)
from luggage_planning.container_aim_utils import look_at_quaternion

VIEW_ORDER = ("aim", "lookback")
EYE_RAISES_M = (0.0, 0.15)
BLENDS = (1.0, 0.7, 0.4)
BOX_MARGIN_M = 0.05
MIN_POINTS = 30
MIN_VALID_DEPTH = 30
TF_FRESHNESS_SEC = 0.20
LOOKBACK_RAISE_M = 0.20
LOOKBACK_BACKOFF_M = 0.15
RIM_MARGIN_M = 0.15


def _normalize_quat(quat):
    x, y, z, w = (float(v) for v in quat)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quat_slerp(q0, q1, blend):
    """Slerp xyzw quaternions. ``blend`` 0 keeps ``q0``; 1 reaches ``q1``."""
    a = _normalize_quat(q0)
    b = list(_normalize_quat(q1))
    t = min(1.0, max(0.0, float(blend)))
    dot = sum(a[i] * b[i] for i in range(4))
    if dot < 0.0:
        b = [-v for v in b]
        dot = -dot
    if dot > 0.9995:
        out = [a[i] + t * (b[i] - a[i]) for i in range(4)]
        norm = math.sqrt(sum(v * v for v in out)) or 1.0
        return tuple(v / norm for v in out)
    theta = math.acos(min(1.0, dot))
    sin_theta = math.sin(theta)
    w0 = math.sin((1.0 - t) * theta) / sin_theta
    w1 = math.sin(t * theta) / sin_theta
    return tuple(w0 * a[i] + w1 * b[i] for i in range(4))


def _quat_from_matrix(rot):
    trace = float(rot[0, 0] + rot[1, 1] + rot[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return _normalize_quat((x, y, z, w))


def _optical_to_suction(cam_xyz, cam_q, t_cam_suction, r_cam_suction):
    """World suction pose for a camera pose and the camera-to-suction transform."""
    r_wc = rotation_from_xyzw(*cam_q)
    r_cs = rotation_from_xyzw(*r_cam_suction)
    t_cs = [float(v) for v in t_cam_suction]
    r_ws = r_wc.dot(r_cs)
    t_ws = r_wc.dot(t_cs) + [float(v) for v in cam_xyz]
    return [float(v) for v in t_ws], _quat_from_matrix(r_ws)


def box_corners(center, size, yaw):
    """Eight corners of an oriented box. Width is local +X, yaw about +Z."""
    cos_y = math.cos(float(yaw))
    sin_y = math.sin(float(yaw))
    hx, hy, hz = (0.5 * float(v) for v in size)
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                lx, ly = sx * hx, sy * hy
                corners.append((
                    float(center[0]) + cos_y * lx - sin_y * ly,
                    float(center[1]) + sin_y * lx + cos_y * ly,
                    float(center[2]) + sz * hz,
                ))
    return corners


def aim_candidates(cam_xyz, cam_q, target, t_cam_suction, r_cam_suction,
                   eye_raises=EYE_RAISES_M, blends=BLENDS):
    """Fixed-order camera aims. Each row is one suction command.

    ``eye_raises`` lifts the eye in world +Z. ``blends`` slerps from the
    current camera orientation toward the full look-at. The camera-to-suction
    transform is rigid, so the suction pose is the camera pose composed with it.
    """
    eye0 = [float(v) for v in cam_xyz]
    current_q = _normalize_quat(cam_q)
    rows = []
    index = 0
    for raise_m in eye_raises:
        eye = [eye0[0], eye0[1], eye0[2] + float(raise_m)]
        full_q = tuple(look_at_quaternion(eye, target))
        for blend in blends:
            cam_aim = quat_slerp(current_q, full_q, blend)
            suction_xyz, suction_q = _optical_to_suction(
                eye, cam_aim, t_cam_suction, r_cam_suction)
            rows.append({
                "index": index,
                "eye_raise_m": float(raise_m),
                "blend": float(blend),
                "camera_xyz": eye,
                "camera_q_xyzw": list(cam_aim),
                "target_xyz": [float(v) for v in target],
                "suction_xyz": suction_xyz,
                "suction_q_xyzw": list(suction_q),
                "t_cam_suction": [float(v) for v in t_cam_suction],
                "r_cam_suction_xyzw": [float(v) for v in r_cam_suction],
            })
            index += 1
    return rows


def lookback_eye(portal_cam_xyz, opening_normal, raise_m=LOOKBACK_RAISE_M,
                 backoff_m=LOOKBACK_BACKOFF_M):
    """Portal eye, raised, then moved out along the opening normal."""
    eye = [float(v) for v in portal_cam_xyz]
    normal = [float(v) for v in opening_normal]
    return [
        eye[0] + float(backoff_m) * normal[0],
        eye[1] + float(backoff_m) * normal[1],
        eye[2] + float(backoff_m) * normal[2] + float(raise_m),
    ]


def next_expected_revision(prev_response, stats_revision):
    """Chain revisions. A view that was not integrated keeps the stats revision."""
    if prev_response and prev_response.get("integrated"):
        return int(prev_response["resulting_map_revision"])
    return int(stats_revision)


def footprint_sensor_hits(surface, center, size, yaw):
    """Sensor-occupied cells whose centers lie in the planned box footprint.

    Z is the box center, so the test is the oriented XY footprint. Cells of
    an earlier box outside this footprint are not counted.
    """
    if not surface:
        return 0
    nx = int(surface.get("nx") or 0)
    ny = int(surface.get("ny") or 0)
    state = surface.get("state") or []
    confidence = surface.get("confidence") or []
    points = []
    cz = float(center[2])
    for ix in range(nx):
        if ix >= len(state):
            continue
        for iy in range(ny):
            if iy >= len(state[ix]):
                continue
            if (state[ix][iy] != "occupied"
                    or iy >= len(confidence[ix])
                    or confidence[ix][iy] != "sensor"):
                continue
            x, y = cell_xy_container(surface, ix, iy)
            points.append((x, y, cz))
    if not points:
        return 0
    mask = box_point_mask(points, center, size, yaw=yaw, margin=0.0)
    return int(sum(1 for flag in mask if flag))


def view_stop(trace):
    """True when this view put at least one sensor cell in the footprint."""
    info = trace or {}
    try:
        hits = int(info.get("footprint_hits") or 0)
        before = int(info.get("expected_map_revision"))
        after = int(info.get("resulting_map_revision"))
    except (TypeError, ValueError):
        return False
    return bool(info.get("success")) and hits >= 1 and after > before


def view_policy_record():
    return {
        "view_order": list(VIEW_ORDER),
        "stop": "footprint_hits>=1 and integrate success and revision increased",
        "eye_raises_m": list(EYE_RAISES_M),
        "blends": list(BLENDS),
        "box_margin_m": BOX_MARGIN_M,
        "min_points": MIN_POINTS,
        "min_valid_depth": MIN_VALID_DEPTH,
        "tf_freshness_sec": TF_FRESHNESS_SEC,
        "lookback_raise_m": LOOKBACK_RAISE_M,
        "lookback_backoff_m": LOOKBACK_BACKOFF_M,
        "rim_margin_m": RIM_MARGIN_M,
        "integrate_source": "placed_box_depth",
        "frozen": False,
    }


def policy_hash(record=None):
    payload = record if record is not None else view_policy_record()
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()
