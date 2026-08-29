#!/usr/bin/env python3
"""Runtime log-level selection for rospy nodes.

``ROSCONSOLE_CONFIG_FILE`` only configures log4cxx, i.e. the C++ nodes. rospy
nodes take their level from ``rospy.init_node(log_level=...)``, which has to be
decided *before* the node exists -- so it cannot come from a ``~private``
parameter. An environment variable is the one channel available at that point,
and roslaunch can set it per node or for a whole launch file.

Usage::

    from log_level_utils import resolve_log_level
    rospy.init_node("my_node", log_level=resolve_log_level())
"""

from __future__ import division

import os

ENV_VAR = "LUGGAGE_LOG_LEVEL"
DEFAULT_LEVEL = "info"

_NAME_TO_ROSPY = {
    "debug": 1,   # rospy.DEBUG
    "info": 2,    # rospy.INFO
    "warn": 4,    # rospy.WARN
    "warning": 4,
    "error": 8,   # rospy.ERROR
    "fatal": 16,  # rospy.FATAL
}


def normalize_level_name(name):
    """Canonical lowercase level name; unknown values fall back to the default."""
    candidate = str(name or "").strip().lower()
    if candidate in ("warning",):
        return "warn"
    if candidate in _NAME_TO_ROSPY:
        return candidate
    return DEFAULT_LEVEL


def resolve_log_level(default=DEFAULT_LEVEL, env=None):
    """Return the rospy log level constant selected by the environment.

    Numeric constants are used rather than ``rospy.DEBUG`` etc. so this module
    stays importable without a ROS install (unit tests, offline tooling).
    """
    source = os.environ if env is None else env
    raw = source.get(ENV_VAR, default)
    return _NAME_TO_ROSPY[normalize_level_name(raw)]


def resolve_level_name(default=DEFAULT_LEVEL, env=None):
    """Same selection as :func:`resolve_log_level`, as a name (for logging it)."""
    source = os.environ if env is None else env
    return normalize_level_name(source.get(ENV_VAR, default))
