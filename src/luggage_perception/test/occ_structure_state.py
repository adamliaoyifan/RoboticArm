"""Lossless step records for the OCC structure oracle.

Arrays are little-endian. JSON uses sorted keys and float.hex. NaN and
Inf stay in NPZ or in an explicit typed marker, never as bare JSON NaN.
"""

import hashlib
import json
import math

import numpy as np

from luggage_perception.cargo_voxel_delta import EVIDENCE_DTYPE, VoxelDeltaBuilder


def _f8(buffer_or_list, count=None):
    if isinstance(buffer_or_list, list):
        return np.asarray(buffer_or_list, dtype="<f8")
    array = np.frombuffer(buffer_or_list, dtype="<f8")
    return np.array(array if count is None else array[:count], dtype="<f8", copy=True)


def _i8(buffer):
    return np.array(np.frombuffer(buffer, dtype="<i8"), dtype="<i8", copy=True)


def _u8(buffer):
    return np.array(np.frombuffer(bytes(buffer), dtype=np.uint8), copy=True)


def attach_delta(mapper):
    mapper._voxel_delta = VoxelDeltaBuilder(mapper)


def take_delta(mapper):
    builder = getattr(mapper, "_voxel_delta", None)
    if builder is None:
        return None
    delta = builder.finalize()
    attach_delta(mapper)
    return delta


def mapper_payload(mapper, delta=None):
    occ = mapper._occupancy
    payload = {
        "grid": _u8(mapper._grid),
        "active": _u8(mapper._active),
        "source": _u8(mapper._source),
        "sensor_seen": _u8(mapper._sensor_seen),
        "labels": _i8(mapper._labels),
        "instance_ids": _i8(mapper._instance_ids),
        "sensor_labels": _i8(mapper._sensor_labels),
        "sensor_instance_ids": _i8(mapper._sensor_instance_ids),
        "sensor_values": _f8(mapper._sensor_values),
        "logodds": _f8(occ.values),
        "locked": np.asarray(occ.geometry_locked, dtype=np.uint8),
    }
    if delta is not None:
        payload["delta_indices"] = np.asarray(delta.indices, dtype="<i8")
        payload["delta_before"] = np.asarray(delta.before, dtype=EVIDENCE_DTYPE)
        payload["delta_after"] = np.asarray(delta.after, dtype=EVIDENCE_DTYPE)
        payload["delta_rebuild"] = np.asarray([1 if delta.rebuild else 0], dtype=np.uint8)
    return payload


def canon(value):
    """JSON-safe canonical form. Floats are hex. Non-finite values are marked."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            return {"nonfinite": "nan" if math.isnan(value) else ("+inf" if value > 0 else "-inf")}
        return {"hex": float.hex(value)}
    if isinstance(value, (np.floating,)):
        return canon(float(value))
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, dict):
        return {str(key): canon(value[key]) for key in value}
    if isinstance(value, (list, tuple)):
        return [canon(item) for item in value]
    return str(value)


def dumps(value):
    return json.dumps(canon(value), sort_keys=True, separators=(",", ":"))


def sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def surface_record(mapper):
    legacy = mapper.surface_map_2d()
    contract = mapper.surface_contract_v2("oracle", "oracle")
    return {
        "surface_v1": canon(legacy),
        "surface_v2": canon(contract),
        "revision": int(mapper.map_revision),
        "depth_ray_phase": int(mapper._depth_ray_phase),
    }
