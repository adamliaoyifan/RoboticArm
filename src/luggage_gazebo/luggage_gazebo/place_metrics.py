#!/usr/bin/env python3
"""Aggregate place-smoke trials. Stdlib only, no ROS."""

from __future__ import division

from collections import Counter
from dataclasses import asdict, dataclass, field, fields
import math
import re
from typing import Optional

# After detach the suitcase falls onto the support. Declare settled only
# once a trailing window stays inside a few millimetres, then measure
# residual drift on a later sample. The fall itself is not drift.
SETTLE_BUDGET_SEC = 3.0
SETTLE_WINDOW_SEC = 0.4
SETTLE_MOVE_M = 0.004


# The arm failed on the way back to observe, after the box was already down.
# EXIT_FAILED_* keeps "the portal hop aborted" distinguishable from "the named
# observe pose was unreachable"; folding them into GOTO_FAILED would hide
# which stage to dump, the way PLACE_CANDIDATE_EXHAUSTED must stay out of
# BIN_FULL. EXIT_FAILED_STUCK additionally means the arm never recovered, so
# the next trial starts from wherever the exit aborted.
RETURN_FAIL_CODES = (
    "GOTO_FAILED", "EXIT_FAILED_RECOVERED", "EXIT_FAILED_STUCK")

PLACE_PASS_CODES = ("",) + RETURN_FAIL_CODES

PLACE_CORE_SEGMENTS = (
    "transit", "traverse", "insert", "descend", "retreat")


@dataclass
class PlaceTrial:
    index: int
    catalog_id: str = ""
    place_state: str = ""
    fail_code: str = ""
    segments_planned: int = 0
    segments_succeeded: int = 0
    descend_fraction: Optional[float] = None
    used_ompl_fallback_descend: Optional[bool] = None
    err_xy: Optional[float] = None
    err_z: Optional[float] = None
    err_yaw: Optional[float] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    drift: Optional[float] = None
    inside_inner_box: Optional[bool] = None
    vac_attach: Optional[bool] = None
    lost_payload: bool = False
    staging_degenerate: bool = False
    wall_time_sec: float = 0.0
    extras: dict = field(default_factory=dict)


def place_ok(record):
    """True when the place itself succeeded.

    A return failure after ``HOME`` does not undo the placement, so the codes
    in ``RETURN_FAIL_CODES`` pass at that state and only at that state.
    """
    code = str(getattr(record, "fail_code", "") or "")
    if not code:
        return True
    if code in RETURN_FAIL_CODES:
        return str(getattr(record, "place_state", "")) == "HOME"
    return False


def trial_to_dict(record):
    return asdict(record)


def trial_from_dict(data):
    allowed = {item.name for item in fields(PlaceTrial)}
    payload = {key: value for key, value in dict(data).items() if key in allowed}
    extras = dict(payload.get("extras") or {})
    for key, value in dict(data).items():
        if key not in allowed:
            extras[key] = value
    payload["extras"] = extras
    return PlaceTrial(**payload)


def xyz_separation(a, b):
    """Euclidean distance between two XYZ sequences."""
    return math.sqrt(sum(
        (float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def settle_verdict(samples, move_m=SETTLE_MOVE_M, window_sec=SETTLE_WINDOW_SEC):
    """Whether the trailing window of ``(t_sec, xyz)`` samples has stopped.

    ``settled`` is true only when that window spans ``window_sec`` and every
    sample in it stays within ``move_m`` of the window start. ``pose`` is the
    last sample. ``window_move_m`` is that in-window travel; the distance
    from the first sample (the fall) is intentionally not returned as drift.
    """
    rows = list(samples or [])
    if not rows:
        return {"settled": False, "pose": None, "window_move_m": None}
    pose = [float(v) for v in rows[-1][1][:3]]
    t_end = float(rows[-1][0])
    # Pose queries are sparse (an ``ign model`` call can take seconds), so
    # the stable interval is "from the newest sample at least window_sec
    # older than the end" through the end, not "only samples inside the
    # last window_sec" (that slice often holds a single point).
    older = [row for row in rows if float(row[0]) <= t_end - float(window_sec)]
    if not older:
        return {"settled": False, "pose": pose, "window_move_m": None}
    anchor = [float(v) for v in older[-1][1][:3]]
    segment = [row for row in rows if float(row[0]) + 1e-9 >= float(older[-1][0])]
    window_move = max(xyz_separation(anchor, row[1]) for row in segment)
    return {
        "settled": window_move <= float(move_m),
        "pose": pose,
        "window_move_m": float(window_move),
    }


def parse_ign_model_pose(text):
    """Parse `ign model --pose` XYZ + RPY. Ignores the entity-id bracket."""
    vecs = []
    for raw in re.findall(r"\[([^\[\]]+)\]", text or ""):
        parts = raw.replace(",", " ").split()
        try:
            nums = [float(p) for p in parts]
        except ValueError:
            continue
        if len(nums) == 3:
            vecs.append(nums)
    if len(vecs) >= 2:
        return list(vecs[-2]) + list(vecs[-1])
    if len(vecs) == 1:
        return list(vecs[0])
    return None


def _mean(values):
    if not values:
        return None
    return sum(values) / float(len(values))


def summarize(records):
    records = list(records or [])
    fail_codes = Counter(
        r.fail_code for r in records
        if r.fail_code and r.fail_code not in RETURN_FAIL_CODES)
    n = len(records)
    n_ok = sum(1 for r in records if place_ok(r))
    n_descend = sum(
        1 for r in records
        if r.descend_fraction is not None
        and r.descend_fraction >= 0.95
        and not r.used_ompl_fallback_descend)
    n_lost = sum(1 for r in records if r.lost_payload)
    n_inside = sum(1 for r in records if r.inside_inner_box)
    xy = [r.err_xy for r in records if r.err_xy is not None]
    z = [r.err_z for r in records if r.err_z is not None]
    drift = [r.drift for r in records if r.drift is not None]
    return {
        "n": n,
        "n_place_ok": n_ok,
        "place_pass_rate": (n_ok / float(n)) if n else 0.0,
        "n_descend_ok": n_descend,
        "n_lost_payload": n_lost,
        "n_inside_inner_box": n_inside,
        "fail_codes": dict(fail_codes),
        "err_xy_mean": _mean(xy),
        "err_z_mean": _mean(z),
        "drift_mean": _mean(drift),
    }
