#!/usr/bin/env python3
"""Parse /luggage/current_box JSON (no ROS). Schema 2.

The latched topic carries instance identity plus the perception MEASURED
geometry only. GT spawn fields (size/pose/mass/model_name) are behind the
``/pickup_box_spawner/get_current_box`` pull service for sim physics
backends, eval fixtures, scoring, and viz — chain modules must not read
them (docs/architecture/privilege_boundary.md).

Shapes:
  measured instance: {"schema": 2, "id": "box_...", "generation": N,
                      "measured": {"width", "depth", "height",
                                   "height_source", "yaw_valid",
                                   "stamp_sec"}}
  cleared platform:  {"schema": 2, "id": "", "generation": N}
"""

from __future__ import division

import json


def identity_from_current_box_payload(data):
    """Return ``(box_id, generation)``; empty id means no current box."""
    if not isinstance(data, dict):
        return "", 0
    box_id = str(data.get("id") or "")
    try:
        generation = int(data.get("generation") or 0)
    except (TypeError, ValueError):
        generation = 0
    return box_id, generation


def measured_from_current_box_payload(data):
    """Return the measured-geometry dict, or None when unmeasured.

    Keys: ``width``/``depth``/``height`` (m, PCA order — width is the
    larger footprint extent), ``height_source`` (DetectedLuggage constant),
    ``yaw_valid``, ``generation``. None covers a cleared platform, a
    spawned-but-unsynced instance, and malformed records.
    """
    if not isinstance(data, dict):
        return None
    measured = data.get("measured")
    if not isinstance(measured, dict):
        return None
    try:
        width = float(measured["width"])
        depth = float(measured["depth"])
        height = float(measured["height"])
    except (KeyError, TypeError, ValueError):
        return None
    try:
        height_source = int(measured.get("height_source") or 0)
    except (TypeError, ValueError):
        height_source = 0
    _box_id, generation = identity_from_current_box_payload(data)
    return {
        "width": width,
        "depth": depth,
        "height": height,
        "height_source": height_source,
        "yaw_valid": bool(measured.get("yaw_valid")),
        "generation": generation,
    }


def identity_from_current_box_json(text):
    """``identity_from_current_box_payload`` for a JSON string (or dict)."""
    return identity_from_current_box_payload(_loads(text))


def measured_from_current_box_json(text):
    """``measured_from_current_box_payload`` for a JSON string (or dict)."""
    return measured_from_current_box_payload(_loads(text))


def _loads(text):
    if isinstance(text, dict):
        return text
    if not text:
        return None
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return None
