#!/usr/bin/env python3
"""Candidate-driven pick waypoints (ROS-free; math only).

Every pick waypoint is derived from the selected suction candidate's
``contact_pose`` and outward normal — never from the box-centre XY or a
global top Z (plan section D). The builder deliberately takes no box
argument, so altering ``DetectedLuggage.pose`` while holding candidates
fixed cannot change a pick waypoint (gate C0, structural).
"""

import math

from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion
from luggage_planning.waypoint_generator import (
    DEFAULT_PICK_CLEARANCES,
    pick_tool_yaw,
)

#: Failing to resolve the requested candidate on the observation is a
#: planning refusal, not a fallback to box-centre waypoints.
SUCTION_CANDIDATE_NOT_FOUND = "SUCTION_CANDIDATE_NOT_FOUND"
#: Valid top but no sealable candidate: no pick sequence may be built.
DETECT_NO_SEALABLE_PATCH = "DETECT_NO_SEALABLE_PATCH"
#: Retry recovery must retreat at least this far along the candidate
#: normal before any lateral motion may be planned (plan section D).
REVERSE_MIN_M = 0.08

#: Clearances measured along the candidate outward normal from the
#: contact point. Same magnitudes as the legacy box-centre clearances so
#: the motion envelope is unchanged for a flat top.
DEFAULT_CANDIDATE_CLEARANCES = dict(DEFAULT_PICK_CLEARANCES)


def quat_mul(a, b):
    """Hamilton product a * b."""
    aw, ax, ay, az = a.w, a.x, a.y, a.z
    bw, bx, by, bz = b.w, b.x, b.y, b.z
    return Quaternion(
        x=aw * bx + ax * bw + ay * bz - az * by,
        y=aw * by - ax * bz + ay * bw + az * bx,
        z=aw * bz + ax * by - ay * bx + az * bw,
        w=aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def quat_rotate(q, v):
    """Rotate vector v = (x, y, z) by quaternion q."""
    p = Quaternion(x=v[0], y=v[1], z=v[2], w=0.0)
    r = quat_mul(quat_mul(q, p), quat_conjugate(q))
    return (r.x, r.y, r.z)


def quat_angle_rad(a, b):
    """Rotation angle (radians) between quaternions a and b."""
    d = abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z)
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def quaternion_aligning_z_to(normal):
    """Minimal rotation taking world +Z onto ``normal`` (unit length).

    Mirrors the perception evaluator's helper so candidate quaternions
    and planning waypoints share one convention: identity for +Z,
    (1, 0, 0, 0) for -Z.
    """
    nx, ny, nz = (float(normal[0]), float(normal[1]), float(normal[2]))
    axis_x, axis_y = -ny, nx          # cross((0,0,1), n)
    s = math.hypot(axis_x, axis_y)
    c = nz
    if s < 1e-12:
        if c > 0.0:
            return Quaternion()
        return Quaternion(x=1.0, w=0.0)
    angle = math.atan2(s, c)
    half = math.sin(angle / 2.0)
    return Quaternion(
        x=axis_x / s * half,
        y=axis_y / s * half,
        z=0.0,
        w=math.cos(angle / 2.0),
    )


def candidate_normal(contact_orientation):
    """Outward unit normal carried by a candidate contact orientation."""
    n = quat_rotate(contact_orientation, (0.0, 0.0, 1.0))
    norm = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])
    if norm < 1e-12:
        raise ValueError("candidate orientation has degenerate normal")
    return (n[0] / norm, n[1] / norm, n[2] / norm)


def _quat_z(yaw):
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


#: 180 deg about X (+Z -> -Z). Explicit w=0.0: the dataclass default is 1.0.
_FLIP_X = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


def tool_orientation_for_candidate(normal, yaw_rad=0.0):
    """Suction-tool orientation opposing the candidate outward normal.

    Tool +Z points INTO the surface (opposite the outward normal); the
    yaw rotates the tool about the normal to the requested azimuth. For
    a world +Z normal this reduces exactly to the legacy tool-down
    quaternion of the same yaw.
    """
    align = quaternion_aligning_z_to(normal)
    return quat_mul(quat_mul(align, _quat_z(float(yaw_rad))), _FLIP_X)


def pose_along_normal(contact_pose, normal, distance_m, yaw_rad=0.0):
    """Pose at ``distance_m`` along the outward normal from the contact."""
    d = float(distance_m)
    return Pose(
        position=Point(
            x=contact_pose.position.x + normal[0] * d,
            y=contact_pose.position.y + normal[1] * d,
            z=contact_pose.position.z + normal[2] * d,
        ),
        orientation=tool_orientation_for_candidate(normal, yaw_rad),
    )


def _clearance(clearances, name):
    if clearances is None:
        clearances = DEFAULT_CANDIDATE_CLEARANCES
    return float(clearances.get(name, DEFAULT_CANDIDATE_CLEARANCES[name]))


def build_candidate_pick_segments(candidate, pick_clearances=None,
                                  detection_yaw=None, yaw_valid=False,
                                  fallback_yaw=0.0):
    """Pick segments for one suction candidate.

    ``candidate`` exposes ``contact`` (a ``Pose`` whose orientation's +Z
    is the outward surface normal). Segments: pre_grasp (free-space
    pose_target), approach and attach (straight-line Cartesian along the
    normal, no OMPL fallback, required fraction 1.0), pick_retreat
    (Cartesian lift along the normal, legacy OMPL fallback kept). The
    attach target position IS the candidate contact position.
    """
    contact = candidate.contact
    normal = candidate_normal(contact.orientation)
    yaw = pick_tool_yaw(detection_yaw, yaw_valid, fallback_yaw)

    pre_grasp = pose_along_normal(
        contact, normal, _clearance(pick_clearances, "pre_grasp"), yaw)
    approach = pose_along_normal(
        contact, normal, _clearance(pick_clearances, "approach"), yaw)
    attach = Pose(
        position=Point(
            x=contact.position.x,
            y=contact.position.y,
            z=contact.position.z,
        ),
        orientation=tool_orientation_for_candidate(normal, yaw),
    )
    pick_retreat = pose_along_normal(
        contact, normal, _clearance(pick_clearances, "pick_retreat"), yaw)

    return [
        MotionSegment(
            name="pre_grasp", type="pose_target", target_pose=pre_grasp),
        MotionSegment(
            name="approach", type="cartesian", target_pose=approach,
            allow_ompl_fallback=False, required_cartesian_fraction=1.0),
        MotionSegment(
            name="attach", type="cartesian", target_pose=attach,
            allow_ompl_fallback=False, required_cartesian_fraction=1.0),
        MotionSegment(
            name="pick_retreat", type="cartesian", target_pose=pick_retreat,
            keep_tool_down=True, allow_ompl_fallback=True),
    ]


def retry_reverse_segment(candidate, approach_pose):
    """Recovery segment retreating to the recorded approach pose.

    Cartesian straight line back along the attempted candidate normal;
    no OMPL fallback, required fraction 1.0. ``approach_pose`` is the
    approach segment's target recorded when it executed, not a fresh TF
    lookup, so replay and hardware agree.
    """
    contact = candidate.contact
    distance = reverse_distance_m(candidate, contact, approach_pose)
    if distance < REVERSE_MIN_M:
        raise ValueError(
            "retry reverse distance %.3f m below %.3f m"
            % (distance, REVERSE_MIN_M))
    return MotionSegment(
        name="retry_reverse", type="cartesian", target_pose=approach_pose,
        allow_ompl_fallback=False, required_cartesian_fraction=1.0,
    )


def reverse_distance_m(candidate, attach_pose, approach_pose):
    """Signed projection of (approach - attach) onto the candidate normal."""
    normal = candidate_normal(candidate.contact.orientation)
    delta = (
        approach_pose.position.x - attach_pose.position.x,
        approach_pose.position.y - attach_pose.position.y,
        approach_pose.position.z - attach_pose.position.z,
    )
    return (
        delta[0] * normal[0] + delta[1] * normal[1] + delta[2] * normal[2])


def attach_pose_error(built_pose, candidate, reference_orientation=None):
    """(position_mm, normal_deg, residual_deg) of a built attach pose.

    C0's 1 mm / 0.5 deg budget applies to the position and to the
    alignment of the tool's -Z axis with the candidate outward normal.
    Rotation about the normal is the yaw freedom; ``residual_deg``
    reports the full orientation deviation from *reference_orientation*
    (zero-yaw tool orientation when omitted) for diagnostics only.
    """
    contact = candidate.contact
    dx = built_pose.position.x - contact.position.x
    dy = built_pose.position.y - contact.position.y
    dz = built_pose.position.z - contact.position.z
    position_mm = 1000.0 * math.sqrt(dx * dx + dy * dy + dz * dz)

    normal = candidate_normal(contact.orientation)
    tool_minus_z = quat_rotate(built_pose.orientation, (0.0, 0.0, -1.0))
    dot = (tool_minus_z[0] * normal[0] + tool_minus_z[1] * normal[1]
           + tool_minus_z[2] * normal[2])
    normal_deg = math.degrees(
        math.acos(max(-1.0, min(1.0, dot))))

    if reference_orientation is None:
        reference_orientation = tool_orientation_for_candidate(normal, 0.0)
    residual_deg = math.degrees(
        quat_angle_rad(reference_orientation, built_pose.orientation))
    return (position_mm, normal_deg, residual_deg)
