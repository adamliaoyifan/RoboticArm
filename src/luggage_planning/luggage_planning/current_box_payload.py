#!/usr/bin/env python3
"""Parse /luggage/current_box JSON (no ROS).

The spawner publishes top-level width/depth/height plus pose, not a nested
``size`` dict. ClearCurrentBox publishes ``{"id": "", "generation": N}``.
"""

from __future__ import division

import json


def box_from_current_box_payload(data):
    """Return a gate-ready box dict, or None when the platform is empty.

    ``None`` covers a missing payload, a clear message (empty id / no
    model_name), and a record that lacks width/depth/height.
    """
    if not isinstance(data, dict):
        return None
    model_name = str(data.get("model_name") or "")
    box_id = str(data.get("id") or "")
    if not model_name and not box_id:
        return None
    try:
        width = float(data["width"])
        depth = float(data["depth"])
        height = float(data["height"])
    except (KeyError, TypeError, ValueError):
        return None
    pose = data.get("pose") or {}
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    try:
        generation = int(data.get("generation") or 0)
    except (TypeError, ValueError):
        generation = 0
    try:
        mass_kg = float(data.get("mass_kg") or 0.0)
    except (TypeError, ValueError):
        mass_kg = 0.0
    return {
        "id": box_id,
        "model_name": model_name,
        "size": [width, depth, height],
        "mass_kg": mass_kg,
        "xyz": [
            float(position.get("x", 0.0)),
            float(position.get("y", 0.0)),
            float(position.get("z", 0.0)),
        ],
        "quat": [
            float(orientation.get("x", 0.0)),
            float(orientation.get("y", 0.0)),
            float(orientation.get("z", 0.0)),
            float(orientation.get("w", 1.0)),
        ],
        "generation": generation,
    }


def parse_current_box_json(text):
    """Parse a JSON string (or dict) into ``box_from_current_box_payload``."""
    if isinstance(text, dict):
        return box_from_current_box_payload(text)
    if not text:
        return None
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    return box_from_current_box_payload(data)
