#!/usr/bin/env python3
"""Apply a trajectory-selector result onto carry segments.

ROS-free. Joint knots are stored on ``_joint_knots`` and, when the message
has the field, on ``joint_waypoints``.
"""

from __future__ import division

from luggage_planning.place_path_planner import apply_variant_to_segment
from luggage_planning.trajectory_selector import TrajectorySelector


CARRY_NAMES = ("transit", "traverse")


def write_joint_knots(segment, knots):
    """Store joint knots on a segment message or a stand-in object.

    Real rosidl messages have fixed ``__slots__`` and reject dynamic
    attributes, so ``_joint_knots`` only lands on stand-in objects; typed
    messages carry the knots in ``joint_waypoints`` alone (the field gates
    in apply_carry_segment decide whether joint_path may be sent at all).
    """
    values = [[float(v) for v in knot] for knot in (knots or [])]
    try:
        segment._joint_knots = values
    except AttributeError:
        pass  # typed message: joint_waypoints below is the carrier
    field = getattr(segment, "joint_waypoints", None)
    if field is None or not hasattr(field, "append"):
        return segment
    try:
        from trajectory_msgs.msg import JointTrajectoryPoint
    except ImportError:
        return segment
    del segment.joint_waypoints[:]
    for knot in values:
        point = JointTrajectoryPoint()
        point.positions = list(knot)
        segment.joint_waypoints.append(point)
    return segment


def read_joint_knots(segment):
    points = list(getattr(segment, "joint_waypoints", None) or [])
    if points and hasattr(points[0], "positions"):
        return [[float(v) for v in point.positions] for point in points]
    return [list(knot) for knot in (getattr(segment, "_joint_knots", None) or [])]


def _has_joint_field(segment):
    # Humble generated messages expose get_fields_and_field_types() -> dict;
    # there is no get_fields(). Guard against both spellings so a stub can
    # still fake either shape, but the real gate is the dict key.
    for name in ("get_fields_and_field_types", "get_fields"):
        getter = getattr(segment, name, None)
        if not callable(getter):
            continue
        try:
            fields = getter()
        except Exception:  # noqa: BLE001 - older generated messages
            return False
        if isinstance(fields, dict):
            return "joint_waypoints" in fields
        return any(getattr(field, "name", "") == "joint_waypoints"
                   for field in fields)
    return False


def apply_carry_segment(segment, selection, frame_convert=None):
    """Write the selected transit or traverse spec onto ``segment``.

    ``joint_path`` is sent only when ``MotionSegment`` has ``joint_waypoints``.
    Otherwise the polyline stays cartesian: a Python-only knot list would be
    dropped on the PlanMotion action boundary and the executor would see an
    empty joint path.
    """
    spec = ((selection or {}).get("segments") or {}).get(str(segment.name))
    if spec is None:
        waypoints = (selection or {}).get("waypoints")
        if waypoints and str(segment.name) in CARRY_NAMES:
            apply_variant_to_segment(segment, waypoints, frame_convert)
        return segment
    waypoints = spec.get("waypoints") or []
    if waypoints:
        apply_variant_to_segment(segment, waypoints, frame_convert)
    spec_type = str(spec.get("type") or segment.type)
    allow_ompl = bool(spec.get("allow_ompl_fallback", True))
    if spec_type == "joint_path" and not _has_joint_field(segment):
        spec_type = "cartesian"
        allow_ompl = True
    segment.type = spec_type
    if hasattr(segment, "allow_ompl_fallback"):
        segment.allow_ompl_fallback = allow_ompl
    if spec_type == "joint_path":
        write_joint_knots(segment, spec.get("joint_knots") or [])
    return segment


def select_in_process(surface, slots, portal, payload_wdh, atlas=None,
                      config=None, **kwargs):
    """Run ``TrajectorySelector`` in this process. Returns its tuple."""
    selector = TrajectorySelector(config) if config else TrajectorySelector.from_yaml()
    return selector.select(
        surface, slots, portal, payload_wdh, atlas=atlas, **kwargs)
