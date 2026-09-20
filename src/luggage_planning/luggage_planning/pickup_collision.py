#!/usr/bin/env python3
"""Pickup MoveIt AABB from a DetectedLuggage-like observation.

Catalog / spawn WDH must never enter this AABB. Pick waypoints target the
measured top; a collision object whose top face sits above that contact
blocks attach (pack_to_full PLAN_attach cartesian 0.500).

Height must be a measured support plane. TOP_ONLY is not a collision box.
"""

from __future__ import division

from luggage_planning.pick_authorization import (
    DETECT_FULL_GEOMETRY_REQUIRED,
    pick_authorized,
)


def _quat_xyzw(box):
    orientation = box.pose.orientation
    return [
        float(orientation.x), float(orientation.y),
        float(orientation.z), float(orientation.w),
    ]


def pickup_collision_aabb(box):
    """Return ``(center_xyz, quat_xyzw, size_wdh)`` for ``add_pickup_box``.

    Requires ``pick_authorized``: pose is the measured centre, size is
    measured WDH. Unmeasured height raises rather than inventing a slab.
    """
    if not pick_authorized(box):
        raise ValueError(
            "%s: pickup AABB needs measured support height"
            % DETECT_FULL_GEOMETRY_REQUIRED)
    quat = _quat_xyzw(box)
    width = float(box.width)
    depth = float(box.depth)
    height = float(box.height)
    x = float(box.pose.position.x)
    y = float(box.pose.position.y)
    return (
        [x, y, float(box.pose.position.z)],
        quat,
        [width, depth, height],
    )
