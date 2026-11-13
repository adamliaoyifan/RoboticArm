#!/usr/bin/env python3
"""Post-place cargo view order, aim candidates, and stop rule. No ROS."""

from __future__ import division

import hashlib
import json
import math

from luggage_perception.cargo_instance_tracker import rotation_from_xyzw
from luggage_perception.cargo_view_diagnosis import project_box_corners
from luggage_perception.cargo_view_integration import (  # noqa: F401
    box_point_mask,
    cell_xy_container,
    footprint_sensor_hits,
)
from luggage_planning.geometry.container_aim_utils import look_at_quaternion

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
#: [ST09-I S5.7-4] ObservationPolicy cap: at most TWO camera targets
#: per observation request; whether a second VIEW is requested at all
#: is decided AFTER the first integration from the real
#: integration/revision/occupancy result (``view_stop``).
MAX_OBSERVATION_VIEWS = 2


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


def observation_target(slot_pose_position, box_height_m):
    """[ST09-I S5.7-4] The POLICY owns the observation target.

    The driver used to assemble the box-top aim point from slot meta
    itself; that decision moves here: the placed box's world center
    (slot support point lifted by half the box height).
    """
    target = [float(v) for v in slot_pose_position[:3]]
    target[2] += 0.5 * float(box_height_m)
    return target


def corners_in_camera(corners_world, eye, cam_q):
    """World box corners in the camera frame of an aim pose."""
    rot = rotation_from_xyzw(*[float(v) for v in cam_q])
    out = []
    for point in corners_world:
        delta = [float(point[i]) - float(eye[i]) for i in range(3)]
        out.append(tuple(float(v) for v in rot.T.dot(delta)))
    return out


def predict_in_frame(corners_world, eye, cam_q, camera_k=None,
                     width=0, height=0):
    """[ST09-I S5.7-4] In-frame VISIBILITY PREDICTION (pure gate).

    Projects the placed box corners through the aim's camera pose; a
    target with no projection on the image is not a candidate. Without
    camera intrinsics the gate admits (recorded ``no_camera_info``) —
    the same fail-open the driver applied, now owned by the policy.
    """
    if not camera_k or len(camera_k) < 6 or not width or not height:
        return True, {"reason": "no_camera_info"}
    corners = corners_in_camera(corners_world, eye, cam_q)
    projected = project_box_corners(
        corners, float(camera_k[0]), float(camera_k[4]),
        float(camera_k[2]), float(camera_k[5]),
        int(width), int(height))
    in_frame = (projected["in_frame"] > 0
                and not projected["behind_camera"])
    return bool(in_frame), projected


def rank_observation_aims(cam_xyz, cam_q, target, t_cam_suction,
                          r_cam_suction, corners_world,
                          camera_k=None, image_wh=(0, 0),
                          eye_raises=EYE_RAISES_M, blends=BLENDS,
                          max_views=MAX_OBSERVATION_VIEWS):
    """[ST09-I S5.7-4] ObservationPolicy target selection.

    Orders the frozen aim grid by PREDICTED visibility (in-frame first,
    original frozen order within each class) and caps the request at
    ``max_views`` camera targets. TCP conversion, MoveIt edges, and
    certification are NOT policy work — the caller routes each target
    through PlanObservationRouteV2.
    """
    rows = aim_candidates(
        cam_xyz, cam_q, target, t_cam_suction, r_cam_suction,
        eye_raises=eye_raises, blends=blends)
    ranked = []
    for row in rows:
        visible, projected = predict_in_frame(
            corners_world, row["camera_xyz"], row["camera_q_xyzw"],
            camera_k, image_wh[0] if image_wh else 0,
            image_wh[1] if image_wh else 0)
        ranked.append(dict(row, in_frame=bool(visible),
                           projection=projected))
    in_frame = [row for row in ranked if row["in_frame"]]
    rest = [row for row in ranked if not row["in_frame"]]
    ordered = in_frame + rest
    if max_views and max_views > 0:
        ordered = ordered[:int(max_views)]
    return ordered


def next_expected_revision(prev_response, stats_revision):
    """Chain revisions. A view that was not integrated keeps the stats revision."""
    if prev_response and prev_response.get("integrated"):
        return int(prev_response["resulting_map_revision"])
    return int(stats_revision)


def view_stop(trace):
    """True when this view observed the released box.

    Two modes, told apart by the trace's ``commit_mode`` field:

    * default (box measured by the integration itself, no ledger commit): the
      view put at least one sensor cell in the footprint. When the mapper
      publishes a pending state (``occupancy_confirmed`` in the trace) the view
      must also have cleared it.
    * ``commit_mode`` (the box was committed to the mapper ledger at release):
      the footprint cells are geometry, so a sensor cell there is not expected.
      The mapper's own confirmation is the only evidence: the view cleared at
      least one pending record (``last_settle.cleared``) and nothing is left
      pending. ``footprint_hits`` is not consulted.

    Absent ``occupancy_confirmed`` (gate off) the default rule holds.
    """
    info = trace or {}
    try:
        before = int(info.get("expected_map_revision"))
        after = int(info.get("resulting_map_revision"))
    except (TypeError, ValueError):
        return False
    advanced = bool(info.get("success")) and after > before
    if info.get("commit_mode"):
        cleared = (info.get("last_settle") or {}).get("cleared") or []
        return (advanced and bool(info.get("occupancy_confirmed", False))
                and len(cleared) > 0)
    try:
        hits = int(info.get("footprint_hits") or 0)
    except (TypeError, ValueError):
        return False
    if not bool(info.get("occupancy_confirmed", True)):
        return False
    return advanced and hits >= 1


#: One name for "the released box was never observed", shared with the
#: planner's entry gate (``luggage_planning.planning_contract.reasons.Reason``).
MAP_CONFIRMATION_REQUIRED = "MAP_CONFIRMATION_REQUIRED"


def map_confirmation_record(final_trace):
    """Did the bounded post-place views observe the released box?

    This is the driver's stop rule. The mapper's ``PendingConfirmation``
    owns the pending bit; the trace carries its published state.

    Confirmed only when the final view stopped on a footprint hit with an
    integrated, revision-advancing result. No detection is not an
    observation that the region is empty, so the map stays pending and
    the driver stops before the next box. ``view_reason`` keeps the
    integrator's own reason for the evidence bundle.
    """
    trace = final_trace or {}
    confirmed = view_stop(trace)
    view_reason = ""
    if not confirmed:
        view_reason = str(
            trace.get("empty_reason") or trace.get("reason_code")
            or trace.get("drop_reason") or "VIEW_EMPTY")
    return {
        "confirmed": bool(confirmed),
        "reason_code": "" if confirmed else MAP_CONFIRMATION_REQUIRED,
        "view_reason": view_reason,
    }


def view_policy_record():
    from luggage_gazebo.fresh_view_burst import MAX_FRAMES, WALL_BUDGET_S
    return {
        "max_fresh_frames_per_pose": MAX_FRAMES,
        "fresh_frame_start_budget_s": WALL_BUDGET_S,
        "view_order": list(VIEW_ORDER),
        "stop": "footprint_hits>=1 and integrate success and revision increased",
        "target": "slot_world + 0.5*box_height (policy-owned)",
        "in_frame_prediction": True,
        "max_observation_views": int(MAX_OBSERVATION_VIEWS),
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
