#!/usr/bin/env python3
"""Identity-keyed dump freeze for eval drivers (no ROS).

A failure dump must serialize the stats record the gate evaluated, joined
to that record's image stamp. It must not wait for a later overlay or copy
the latest latch.
"""

from __future__ import division


def dump_tf_stamp_from_color(color):
    """Return (sec, nanosec) for a stamped TF lookup, or None.

    Callers must not fall back to Time() / latest when this is None.
    """
    if color is None:
        return None
    header = getattr(color, "header", None)
    stamp = getattr(header, "stamp", None) if header is not None else None
    if stamp is None:
        return None
    try:
        return int(stamp.sec), int(stamp.nanosec)
    except (TypeError, AttributeError, ValueError):
        return None


def stamp_sec_from_key(key):
    if key is None:
        return None
    if isinstance(key, tuple) and len(key) >= 2:
        return float(key[0]) + 1e-9 * float(key[1])
    try:
        return float(key)
    except (TypeError, ValueError):
        return None


def find_exact_stamp_key(items, stamp_sec, tol=1e-6):
    """Return the buffer key whose stamp matches *stamp_sec*, or None."""
    if stamp_sec is None or not items:
        return None
    target = float(stamp_sec)
    for key in items:
        value = stamp_sec_from_key(key)
        if value is None:
            continue
        if abs(value - target) <= float(tol):
            return key
    return None


def dump_seg_stats(freeze, live_latch):
    """Stats JSON for the dump. Frozen decision_record wins over the latch."""
    if freeze and freeze.get("decision_record") is not None:
        return freeze["decision_record"]
    return live_latch


def freeze_dump_bundle(decision_record, snaps,
                       names=("color", "depth", "overlay", "mask", "cargo"),
                       tol=1e-6):
    """Pin stamp-joined frames to the gated stats record.

    Does not wait for later frames. Missing joins are listed, not filled.
    """
    missing = []
    snapshot = {}
    join_key = None
    stamp = None
    join_dt = None
    record = decision_record if isinstance(decision_record, dict) else None
    if record is None:
        missing.append("decision_record")
    else:
        try:
            stamp = float(record.get("stamp"))
        except (TypeError, ValueError):
            stamp = None
        if stamp is None:
            missing.append("decision_stamp")
    snaps = snaps or {}
    if stamp is not None:
        join_key = find_exact_stamp_key(snaps.get("color") or {}, stamp, tol)
        if join_key is None:
            missing.append("stamp_joined_frames")
        else:
            joined = stamp_sec_from_key(join_key)
            if joined is not None:
                join_dt = abs(float(joined) - float(stamp))
            for name in names:
                snapshot[name] = (snaps.get(name) or {}).get(join_key)
            if snapshot.get("overlay") is None:
                missing.append("overlay")
    capture_complete = (
        "decision_record" not in missing
        and "decision_stamp" not in missing
        and "stamp_joined_frames" not in missing
        and "overlay" not in missing)
    return {
        "decision_record": record,
        "snapshot": snapshot,
        "join_key": join_key,
        "stamp": stamp,
        "join_dt": join_dt,
        "missing": missing,
        "capture_complete": capture_complete,
        "replay_possible": capture_complete,
    }
