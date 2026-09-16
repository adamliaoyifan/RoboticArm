"""Eval-only ROS domain isolation for bag replay.

The live Gazebo stack is an exclusive machine resource and conventionally
uses ``ROS_DOMAIN_ID=7``. Bag replay must never join that graph: playing
sensors onto the live sim (or the real arm) mixes clocks, TF, and
``/joint_states``. This module does not launch anything; it only refuses
the reserved domain and builds an env for an optional isolated MoveIt
helper.
"""
from __future__ import division

import os

# Convention from .cursor/rules/sim-lifecycle.mdc and the site-record notes.
LIVE_SIM_DOMAIN_ID = 7
DEFAULT_REPLAY_DOMAIN_ID = 42


class IsolatedDomainError(ValueError):
    """Raised when replay would share the live sim / site-record domain."""


def parse_domain_id(value, default=DEFAULT_REPLAY_DOMAIN_ID):
    """Parse a domain id, defaulting only when *value* is None or empty."""
    if value is None or value == "":
        return int(default)
    return int(value)


def assert_isolated_domain(domain_id, reserved=LIVE_SIM_DOMAIN_ID):
    """Refuse the live-sim domain. Offline mcap replay does not need ROS."""
    domain = parse_domain_id(domain_id)
    reserved = int(reserved)
    if domain == reserved:
        raise IsolatedDomainError(
            "refusing ROS_DOMAIN_ID=%s: that domain is reserved for the "
            "live sim / site recording. Replay perception is offline "
            "(no ROS graph). If you need MoveIt, pass --ros-domain-id "
            "%s (or any id other than %s) and --plan-moveit."
            % (reserved, DEFAULT_REPLAY_DOMAIN_ID, reserved))
    if domain < 0 or domain > 232:
        raise IsolatedDomainError(
            "ROS_DOMAIN_ID=%s is out of range [0, 232]" % domain)
    return domain


def isolated_replay_env(domain_id=None, extra=None, environ=None):
    """Copy *environ* with an isolated domain and localhost-only DDS.

    Never inherits a live ``ROS_DOMAIN_ID=7``: the returned mapping always
    overwrites it after ``assert_isolated_domain``.
    """
    domain = assert_isolated_domain(parse_domain_id(domain_id))
    env = dict(environ if environ is not None else os.environ)
    env["ROS_DOMAIN_ID"] = str(domain)
    env["ROS_LOCALHOST_ONLY"] = "1"
    if extra:
        env.update(extra)
    return env
