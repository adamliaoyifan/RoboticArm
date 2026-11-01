"""Chain feasibility from atlas branch seeds: predict, don't try.

For one (slot contact point, directed heading) the atlas stores up to three
IK branch seeds per endpoint (contact set at the slot, transit set at the
carry pose). A placement whose refined slot-contact seed and refined
door-portal carry seed land in the SAME branch family -- wrap-aware joint
distance within the builder's distinctness threshold -- has a continuous
wrist path door -> slot: the in-between segments are ordinary refinements,
not branch flips. This module computes that verdict offline so placement
can rank by it at selection time (2026-10-13 seed-driven chain policy,
docs/architecture/placement.md).

Everything here is offline: ``offline_ik`` FK/refine, the loaded atlas, and
scene geometry. No ROS runtime, no MoveIt.
"""

from __future__ import division

import math

from luggage_planning.atlas_runtime import atlas_seeds_for_query
from luggage_planning.reachability_wavefront import joint_branch_distance

# Same distinctness threshold as select_distinct_branches at build time:
# two solutions within this wrap-aware distance are one branch family.
DEFAULT_BRANCH_GAP_RAD = 0.25



def _rpy_matrix(rpy):
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _to_world(container_in_world, point):
    origin, rpy = container_in_world
    rot = _rpy_matrix(rpy)
    return tuple(
        float(origin[i]) + sum(rot[i][j] * float(point[j]) for j in range(3))
        for i in range(3))


def _world_yaw(container_in_world, yaw_container):
    return float(yaw_container) + float(container_in_world[1][2])


def _refine_seeds(urdf, tcp_path, world_xyz, quat_xyzw, seeds):
    """refine_to_pose each seed onto the exact endpoint pose."""
    from luggage_planning.offline_ik import refine_to_pose
    refined, dropped = [], 0
    for seed in seeds:
        joints, _info = refine_to_pose(
            urdf, tcp_path, world_xyz, quat_xyzw, [float(v) for v in seed])
        if joints is None:
            dropped += 1
            continue
        refined.append([float(v) for v in joints])
    return refined, dropped


def _tool_down_quat_xyzw(world_yaw):
    from luggage_planning.waypoint_generator import _tool_down_quaternion
    quat = _tool_down_quaternion(world_yaw)
    return (quat.x, quat.y, quat.z, quat.w)


def chain_report(atlas, urdf, tcp_path, slot_contact_container,
                 portal_container, heading_container, container_in_world,
                 branch_gap_threshold=DEFAULT_BRANCH_GAP_RAD, max_seeds=3):
    """Same-branch verdict for one (slot, heading) against the portal.

    ``slot_contact_container``/``portal_container`` are (x, y, z) in
    ``container_link``; ``heading_container`` is the tool-down heading in
    the container frame (directed, not pi-folded); ``container_in_world``
    is ``(xyz, rpy)`` of container_link in the arm's world frame.

    Returns a dict; ``same_branch`` is True/False when both endpoints have
    refined seeds and ``None`` (unknown, never a reject) when either side
    has no usable seeds. ``branch_gap_rad`` is the wrap-aware
    ``joint_branch_distance`` of the best cross pair.
    """
    slot_xyz_world = _to_world(container_in_world, slot_contact_container)
    portal_xyz_world = _to_world(container_in_world, portal_container)
    quat = _tool_down_quat_xyzw(
        _world_yaw(container_in_world, heading_container))

    slot_q = atlas_seeds_for_query(
        atlas, float(slot_contact_container[0]),
        float(slot_contact_container[1]), float(slot_contact_container[2]),
        float(heading_container), seed_set="contact", max_seeds=max_seeds)
    portal_q = atlas_seeds_for_query(
        atlas, float(portal_container[0]), float(portal_container[1]),
        float(portal_container[2]), float(heading_container),
        seed_set="transit", max_seeds=max_seeds)

    slot_seeds, slot_dropped = _refine_seeds(
        urdf, tcp_path, slot_xyz_world, quat, slot_q.get("seeds") or [])
    portal_seeds, portal_dropped = _refine_seeds(
        urdf, tcp_path, portal_xyz_world, quat, portal_q.get("seeds") or [])

    report = {
        "slot_status": slot_q.get("status_name"),
        "portal_status": portal_q.get("status_name"),
        "slot_cell": slot_q.get("seed_cell"),
        "portal_cell": portal_q.get("seed_cell"),
        "n_slot_seeds": len(slot_seeds),
        "n_portal_seeds": len(portal_seeds),
        "refine_dropped": slot_dropped + portal_dropped,
        "branch_gap_rad": None,
        "same_branch": None,
        "reason": "",
    }
    if not slot_seeds or not portal_seeds:
        report["reason"] = (
            "no_slot_seeds" if not (slot_q.get("seeds") or [])
            else "no_portal_seeds" if not (portal_q.get("seeds") or [])
            else "refine_dropped_all")
        return report

    gap = min(
        joint_branch_distance(slot, portal)
        for slot in slot_seeds for portal in portal_seeds)
    report["branch_gap_rad"] = float(gap)
    report["same_branch"] = bool(gap <= float(branch_gap_threshold))
    return report
