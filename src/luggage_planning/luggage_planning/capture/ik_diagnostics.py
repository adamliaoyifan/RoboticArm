"""Stable, line-oriented diagnostics for exploration IK experiments."""

from __future__ import division

import json


EVENT_PREFIX = "EXPLORE_IK_EVENT "


def format_event(payload):
    """Return a single machine-readable ROS log message for one IK event."""
    return EVENT_PREFIX + json.dumps(payload, sort_keys=True, separators=(",", ":"))
