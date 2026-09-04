#!/usr/bin/env python3
"""Aggregate place-smoke trials. Stdlib only, no ROS."""

from __future__ import division

from collections import Counter
from dataclasses import asdict, dataclass, field, fields
import re
from typing import Optional


PLACE_PASS_CODES = ("", "GOTO_FAILED")

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
    """True when the place itself succeeded. GOTO_FAILED after HOME is ok."""
    code = str(getattr(record, "fail_code", "") or "")
    if not code:
        return True
    if code == "GOTO_FAILED":
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
        r.fail_code for r in records if r.fail_code and r.fail_code != "GOTO_FAILED")
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
