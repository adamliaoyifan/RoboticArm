"""Gazebo 32FC1-metre depth to canonical 16UC1 millimetres.

ROS-free conversion used by the launch-owned sim adapter. Invalid, non-finite,
out-of-range, and truncated payloads fail closed: the caller must drop the
frame rather than publish guessed bytes.
"""
from __future__ import division

import math

import numpy as np

VALID_ENCODINGS = ("32FC1", "TYPE_32FC1")
FLOAT32_BYTES = 4


def convert_depth_metres_to_mm(
        encoding, width, height, step, data, near_m=0.26, max_m=3.0):
    """Convert one depth image payload.

    Returns ``(mm_bytes, None)`` on success or ``(None, reason)`` on failure.
    ``mm_bytes`` is little-endian uint16 row-major, ``step = width * 2``.
    """
    if encoding not in VALID_ENCODINGS:
        return None, "unsupported_encoding"
    try:
        width = int(width)
        height = int(height)
        step = int(step)
    except (TypeError, ValueError):
        return None, "malformed_header"
    if width <= 0 or height <= 0:
        return None, "malformed_header"
    min_step = width * FLOAT32_BYTES
    if step < min_step:
        return None, "malformed_step"
    payload = memoryview(data)
    expected = height * step
    if len(payload) < expected:
        return None, "truncated_payload"
    near_m = float(near_m)
    max_m = float(max_m)
    if not math.isfinite(near_m) or not math.isfinite(max_m) or max_m <= 0:
        return None, "invalid_range"
    if step == min_step:
        metres = np.frombuffer(payload[:expected], dtype=np.float32).reshape(
            height, width)
    else:
        metres = np.empty((height, width), dtype=np.float32)
        for row in range(height):
            start = row * step
            end = start + min_step
            metres[row] = np.frombuffer(
                payload[start:end], dtype=np.float32, count=width)
    finite = np.isfinite(metres)
    in_range = finite & (metres >= near_m) & (metres <= max_m)
    mm = np.zeros((height, width), dtype=np.uint16)
    scaled = np.rint(metres * np.float32(1000.0))
    scaled = np.clip(scaled, 0, 65535)
    mm[in_range] = scaled[in_range].astype(np.uint16)
    return mm.tobytes(), None
