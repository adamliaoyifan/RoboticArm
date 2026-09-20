"""Eval-only per-bag replay index sidecar. Not imported by online nodes.

Pass A of a replay indexes bag-derived facts (stamps, aux payloads,
camera_info, lidar scan stamps) that never change for an immutable bag —
yet every iteration of the tweak-algorithm→replay loop re-pays that pass.
This module persists those facts under the user cache keyed on bag
identity, so a warm replay skips Pass A entirely.

The cache is advisory: any doubt (parse error, schema mismatch, identity
mismatch, message-count mismatch against a fresh ``scan_bag``) is a miss
and the full pass runs — a bad sidecar can only cost time, never
correctness. Only bag-derived facts are stored; the join plan is always
recomputed from the cached stamps, so tolerance/stride changes can never
serve a stale plan.
"""
from __future__ import division

import hashlib
import json
import os
import tempfile

SCHEMA_VERSION = 1

# Each replay tool records its name: the two CLIs cache different fact
# sets for the same bag (site_pick never indexes lidar or K variants), so
# a sidecar written by the other tool must miss, not serve short fields.
_REQUIRED_KEYS = (
    "schema_version", "key", "bag", "producer", "color_topic",
    "depth_topic",
    "color_entries", "color_duplicates", "depth_entries",
    "depth_duplicates", "joint_payloads", "tcp_payloads", "camera_info",
    "camera_k_variants", "tf_static_message_count", "lidar_stamps",
    "tf_edges_file",
)


def default_cache_dir(cache_dir=""):
    """Cache root: explicit override, else XDG (~/.cache) — never out_root,
    so evidence trees stay clean of foreign cache files and iteration runs
    that switch output directories keep the cache."""
    if cache_dir:
        return os.path.abspath(os.path.expanduser(str(cache_dir)))
    base = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")
    return os.path.join(base, "luggage_perception", "replay_index")


def bag_identity(mcap_path):
    """(key, identity dict) binding the sidecar to immutable bag content.

    realpath+size+mtime: a copy, truncate or rewrite of the bag changes
    the identity and invalidates every sidecar for it.
    """
    real = os.path.realpath(str(mcap_path))
    stat = os.stat(real)
    identity = {
        "mcap_path": real,
        "size_bytes": int(stat.st_size),
        "mtime_ns": int(stat.st_mtime_ns),
    }
    payload = "\n".join([real, str(identity["size_bytes"]),
                         str(identity["mtime_ns"])])
    key = "v%d:%s" % (SCHEMA_VERSION, hashlib.sha256(
        payload.encode("utf-8")).hexdigest())
    return key, identity


def _sidecar_dir(cache_dir, key):
    hexid = key.split(":", 1)[1]
    return os.path.join(cache_dir, hexid[:2], hexid)


def sidecar_path(mcap_path, filename, cache_dir=""):
    """Path of a sibling file (e.g. tf_edges.npz) in the sidecar dir."""
    key, _identity = bag_identity(mcap_path)
    return os.path.join(
        _sidecar_dir(default_cache_dir(cache_dir), key), filename)


def load_index(mcap_path, scan, color_topic, depth_topic, cache_dir="",
               producer=""):
    """Parsed index dict for the bag, or None on any doubt.

    Cross-checks the stored entry counts against a fresh ``scan_bag``
    summary — that is what makes the cache safe to trust even for scored
    runs: a hand-edited sidecar that does not add up is rejected. The
    stored ``producer`` must equal the caller's: sidecars written by the
    other replay tool index fewer facts (site_pick records no lidar
    stamps and no K variants) and must miss rather than serve them.
    """
    try:
        key, _identity = bag_identity(mcap_path)
        path = os.path.join(_sidecar_dir(
            default_cache_dir(cache_dir), key), "index.json")
        if not os.path.isfile(path):
            return None
        with open(path, encoding="utf-8") as handle:
            index = json.load(handle)
        for name in _REQUIRED_KEYS:
            if name not in index:
                return None
        if int(index["schema_version"]) != SCHEMA_VERSION:
            return None
        if index["key"] != key:
            return None
        if index["producer"] != producer:
            return None
        if (index["color_topic"] != color_topic
                or index["depth_topic"] != depth_topic):
            return None
        for side in ("color", "depth"):
            entries = index["%s_entries" % side]
            stamps = [int(row[0]) for row in entries]
            if any(b <= a for a, b in zip(stamps, stamps[1:])):
                return None
            counted = len(stamps) + sum(
                int(n) for _stamp, n in index["%s_duplicates" % side])
            topic = color_topic if side == "color" else depth_topic
            if topic in scan.topics and counted != int(
                    scan.topics[topic]["message_count"]):
                return None
        return index
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        return None


def write_index(mcap_path, index, cache_dir="", producer=""):
    """Atomically persist the index dict; returns the path or None.

    The write is tmp-file + ``os.replace``: an interrupted run either
    leaves the previous complete sidecar or none at all — never a partial
    file that a later load would have to reject. ``producer`` names the
    writing tool and gates who may load it back (see load_index).
    """
    _key, identity = bag_identity(mcap_path)
    index = dict(index)
    index["schema_version"] = SCHEMA_VERSION
    index["key"] = _key
    index["producer"] = str(producer)
    index["bag"] = identity
    try:
        sidecar = _sidecar_dir(default_cache_dir(cache_dir), _key)
        os.makedirs(sidecar, exist_ok=True)
        fd, tmp_path = tempfile.mkstemp(dir=sidecar, suffix=".tmp")
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(index, handle, sort_keys=True)
            handle.write("\n")
        os.replace(tmp_path, os.path.join(sidecar, "index.json"))
        return os.path.join(sidecar, "index.json")
    except OSError:
        return None


def camera_info_payload(frame):
    """JSON-safe full CameraInfoFrame payload (round-trips every field
    downstream code touches: intrinsics, distortion, projection)."""
    return {
        "stamp": float(frame.stamp),
        "frame_id": str(frame.frame_id),
        "width": int(frame.width),
        "height": int(frame.height),
        "fx": float(frame.fx),
        "fy": float(frame.fy),
        "cx": float(frame.cx),
        "cy": float(frame.cy),
        "distortion_model": str(frame.distortion_model),
        "distortion_coeffs": [float(v) for v in frame.distortion_coeffs],
        "rectification": [float(v) for v in frame.rectification],
        "projection": [float(v) for v in frame.projection],
        "binning_x": int(frame.binning_x),
        "binning_y": int(frame.binning_y),
    }


def camera_info_frame_from_payload(payload):
    """Rebuild a CameraInfoFrame from camera_info_payload output."""
    from luggage_perception.sensor_types import CameraInfoFrame

    return CameraInfoFrame(
        stamp=float(payload["stamp"]),
        frame_id=str(payload["frame_id"]),
        width=int(payload["width"]),
        height=int(payload["height"]),
        fx=float(payload["fx"]),
        fy=float(payload["fy"]),
        cx=float(payload["cx"]),
        cy=float(payload["cy"]),
        distortion_model=str(payload["distortion_model"]),
        distortion_coeffs=tuple(
            float(v) for v in payload["distortion_coeffs"]),
        rectification=tuple(float(v) for v in payload["rectification"]),
        projection=tuple(float(v) for v in payload["projection"]),
        binning_x=int(payload["binning_x"]),
        binning_y=int(payload["binning_y"]),
    )


def count_jsonl_rows(path):
    """Row count of a jsonl file, or None when missing/unreadable."""
    try:
        with open(path, encoding="utf-8") as handle:
            return sum(1 for line in handle if line.strip())
    except OSError:
        return None
