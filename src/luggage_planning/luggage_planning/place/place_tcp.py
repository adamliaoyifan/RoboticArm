"""Container-slot and grasp transforms for the v2 place chain.

[ST09-I] the legacy selection/leg builders were deleted with the
geometric bypass: certified route legs come from ``route_producer``;
these pure transforms stay (grasp admission, world composition,
release-gap lift).
"""

from __future__ import division

import math

from luggage_description.frame_contracts import (
    ContractError,
    finite_number,
    require_horizontal,
)
from luggage_description.frame_transforms import (
    box_to_tool_target,
    rotate_vector,
    transform_pose,
)


FIXTURE_SELECTOR = {
    "inflate_m": 0.0,
    "arm_radius_m": 0.0,
    "mode": "unified",
    "max_candidates_per_slot": 5,
    "probe_budget": 0,
    "w_arm": 0.0,
}


def _pose_dict(position, quaternion):
    return {
        "position": [float(v) for v in position],
        "quaternion_xyzw": [float(v) for v in quaternion],
    }


def container_transform_dict(source_frame, target_frame, translation,
                             quaternion_xyzw, stamp_ns, scene_revision):
    """Admit a container-to-world transform before any place selection.

    Source is ``container_link`` and target is ``world``. A null stamp is
    the static scene transform. Translation must be three finite metres.
    """
    if not isinstance(source_frame, str) or not source_frame.strip():
        raise ContractError(
            "MISSING_IDENTITY", "container_source_frame", "nonempty", source_frame)
    if not isinstance(target_frame, str) or not target_frame.strip():
        raise ContractError(
            "MISSING_IDENTITY", "container_target_frame", "nonempty", target_frame)
    if source_frame != "container_link" or target_frame != "world":
        raise ContractError(
            "FRAME_CONTRACT_MISMATCH", "container_transform",
            ["container_link", "world"], [source_frame, target_frame])
    if stamp_ns is not None and (type(stamp_ns) is not int or stamp_ns < 0):
        raise ContractError(
            "INVALID_STAMP", "container_stamp_ns",
            "nonnegative integer or explicit null", stamp_ns)
    values = list(translation)
    if len(values) != 3:
        raise ContractError(
            "INVALID_NUMERIC_PAYLOAD", "translation", "3 finite numbers", values)
    admitted = [
        finite_number(float(value), "translation[%d]" % index)
        for index, value in enumerate(values)]
    quaternion_xyzw = [float(v) for v in quaternion_xyzw]
    require_horizontal(quaternion_xyzw)
    return {
        "source_frame": source_frame,
        "target_frame": target_frame,
        "translation": admitted,
        "quaternion_xyzw": quaternion_xyzw,
        "stamp_ns": stamp_ns,
        "scene_revision": scene_revision,
    }


def grasp_dict_from_contract(grasp, scene_revision):
    """PoseContract-shaped grasp: reference is the source frame, semantics the target."""
    if not isinstance(grasp, dict):
        raise ContractError("MISSING_IDENTITY", "grasp", "object", None)
    source = grasp.get("reference")
    target = grasp.get("orientation_semantics")
    if not isinstance(source, str) or not source.strip() or not isinstance(target, str) or not target.strip():
        raise ContractError("MISSING_IDENTITY", "grasp", "source and target frames", grasp)
    pose = grasp.get("position"), grasp.get("quaternion_xyzw")
    return {
        "source_frame": source,
        "target_frame": target,
        "translation": pose[0],
        "quaternion_xyzw": pose[1],
        "scene_revision": grasp.get("scene_revision", scene_revision),
    }


def rotated_grasp_offset(slot, grasp):
    return rotate_vector(slot["quaternion_xyzw"], grasp["translation"])


def world_slot_record(slot, transform):
    return transform_pose(slot, transform)


def world_tcp_record(slot, grasp, transform):
    tcp = box_to_tool_target(slot, grasp)
    return transform_pose(tcp, transform)


def lift_world_point(position, container_quaternion, lift):
    delta = rotate_vector(container_quaternion, (0.0, 0.0, float(lift)))
    return [position[i] + delta[i] for i in range(3)]


def _planning_pose(position, quaternion):
    return Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(
            x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]),
    )

