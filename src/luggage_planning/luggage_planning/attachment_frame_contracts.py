"""Measured attachment relation. Physics ground truth never enters this record."""

from __future__ import division

import math

from luggage_description.frame_contracts import (
    ContractError,
    finite_number,
    require_equal,
    validate_pose,
    vector,
    unit_quaternion,
)
from luggage_description.frame_diagnostics import at_boundary, validate_context
from luggage_description.frame_transforms import box_to_tool_target
from luggage_planning.pick_authorization import (
    DETECT_FULL_GEOMETRY_REQUIRED,
    HEIGHT_SOURCE_MEASURED_SUPPORT,
)
from luggage_planning.pick_frame_contracts import grasp_transform
from luggage_planning.vacuum_attach_utils import compose_transform, invert_transform


GRASP_TOLERANCE_M = 0.01
GRASP_TOLERANCE_RAD = 0.10


def _angle(left, right):
    from luggage_planning.motion_frame_contracts import rotation_angle
    return rotation_angle(left, right)


def _close(left, right, limit):
    return sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)) ** 0.5 <= limit


def _size(box):
    width = finite_number(box.get("width"), "width")
    depth = finite_number(box.get("depth"), "depth")
    height = finite_number(box.get("height"), "height")
    if width <= 0 or depth <= 0 or height <= 0:
        raise ContractError(
            "INVALID_NUMERIC_PAYLOAD", "size_wdh", "positive", [width, depth, height])
    if (not box.get("height_valid")
            or int(box.get("height_source", 0)) != HEIGHT_SOURCE_MEASURED_SUPPORT):
        raise ContractError(
            DETECT_FULL_GEOMETRY_REQUIRED, "height_source",
            HEIGHT_SOURCE_MEASURED_SUPPORT, box.get("height_source"))
    return [width, depth, height]


def require_measured_pose_agreement(box_pose, measured):
    require_equal(box_pose, "frame_id", measured.get("header_frame", "world"),
                  "FRAME_CONTRACT_MISMATCH")
    if int(box_pose["stamp_ns"]) != int(measured["stamp_ns"]):
        raise ContractError(
            "INVALID_STAMP", "stamp_ns", measured["stamp_ns"], box_pose["stamp_ns"])
    if str(box_pose.get("box_id", measured["id"])) != str(measured["id"]):
        raise ContractError(
            "REQUEST_IDENTITY_MISMATCH", "box_id", measured["id"], box_pose.get("box_id"))
    if not _close(box_pose["position"], measured["position"], 1e-6):
        raise ContractError(
            "POSE_REFERENCE_MISMATCH", "position", measured["position"], box_pose["position"])
    if _angle(box_pose["quaternion_xyzw"], measured["quaternion_xyzw"]) > 1e-6:
        raise ContractError(
            "ORIENTATION_SEMANTICS_MISMATCH", "quaternion_xyzw",
            measured["quaternion_xyzw"], box_pose["quaternion_xyzw"])


def _freeze(tool_pose, box_pose, measured, grasp, generation, expected_generation, box_id):
    validate_pose(tool_pose, "motion", tool_pose["scene_revision"])
    validate_pose(box_pose, "detection", box_pose["scene_revision"])
    require_equal(box_pose, "scene_revision", tool_pose["scene_revision"],
                  "CALIBRATION_REVISION_MISMATCH")
    if str(measured.get("id")) != str(box_id):
        raise ContractError(
            "REQUEST_IDENTITY_MISMATCH", "box_id", box_id, measured.get("id"))
    box_pose = dict(box_pose, box_id=str(box_id))
    if int(generation) != int(expected_generation):
        raise ContractError(
            "GRASP_BINDING_MISMATCH", "generation", expected_generation, generation)
    require_measured_pose_agreement(box_pose, measured)
    size = _size(measured)
    def _unit(quaternion):
        norm = math.sqrt(sum(float(v) * float(v) for v in quaternion))
        return [float(v) / norm for v in quaternion]

    actual = grasp_transform(
        box_pose["position"], box_pose["quaternion_xyzw"],
        tool_pose["position"], tool_pose["quaternion_xyzw"],
        box_pose["scene_revision"])
    actual["quaternion_xyzw"] = _unit(actual["quaternion_xyzw"])
    if grasp is not None:
        if not _close(grasp["translation"], actual["translation"], GRASP_TOLERANCE_M):
            raise ContractError(
                "GRASP_BINDING_MISMATCH", "grasp.translation",
                actual["translation"], grasp["translation"])
        if _angle(grasp["quaternion_xyzw"], actual["quaternion_xyzw"]) > GRASP_TOLERANCE_RAD:
            raise ContractError(
                "GRASP_BINDING_MISMATCH", "grasp.quaternion_xyzw",
                actual["quaternion_xyzw"], grasp["quaternion_xyzw"])
    tool_box = invert_transform(actual["translation"], actual["quaternion_xyzw"])
    # box_to_tool_target must reproduce the measured tool. That is the oracle.
    reproduced = box_to_tool_target(box_pose, {
        "source_frame": "suction_contact_frame",
        "target_frame": "box",
        "translation": actual["translation"],
        "quaternion_xyzw": actual["quaternion_xyzw"],
        "scene_revision": box_pose["scene_revision"],
    })
    if not _close(reproduced["position"], tool_pose["position"], 1e-9):
        raise ContractError(
            "GRASP_BINDING_MISMATCH", "reproduced_tool",
            tool_pose["position"], reproduced["position"])
    return {
        "t_box_tool": actual,
        "t_tool_box_translation": list(tool_box[0]),
        "t_tool_box_quaternion": list(tool_box[1]),
        "size_wdh": size,
        "box_id": str(measured["id"]),
        "generation": int(generation),
        "scene_revision": box_pose["scene_revision"],
        "box_pose": box_pose,
        "tool_stamp_ns": int(tool_pose["stamp_ns"]),
    }


def admit_attachment(tool_pose, box_pose, measured, grasp, generation,
                     expected_generation, box_id, context):
    def operation():
        from dataclasses import replace
        validate_context(box_pose, context)
        validate_context(tool_pose, replace(context, reference="suction_contact_frame"))
        return _freeze(
            tool_pose, box_pose, measured, grasp, generation,
            expected_generation, box_id)
    return at_boundary(box_pose if isinstance(box_pose, dict) else {}, context, operation)


def compose_box_world(attachment, tool_position, tool_quaternion):
    """Planning box pose from the frozen measured relation and a new tool pose."""
    position, quat = compose_transform(
        tool_position, tool_quaternion,
        attachment["t_tool_box_translation"], attachment["t_tool_box_quaternion"])
    return [list(position), list(quat)]


def planning_pose_unchanged(before, after):
    return before == after


def admit_release(attachment, final_tool, *, request_id, scene_revision,
                  box_id, generation, expected_generation):
    """Validate all measured release evidence before invoking a detach backend."""
    if not isinstance(request_id, str) or not request_id.strip():
        raise ContractError("MISSING_IDENTITY", "request_id", "nonempty", request_id)
    if not scene_revision or not box_id:
        raise ContractError("MISSING_IDENTITY", "release", "scene and box identity", box_id)
    if not attachment or not final_tool:
        raise ContractError("MISSING_IDENTITY", "release", "attachment and measured tool pose", None)
    for field, expected in (("scene_revision", scene_revision), ("box_id", box_id),
                            ("generation", int(generation))):
        require_equal(attachment, field, expected, "GRASP_BINDING_MISMATCH")
    if int(generation) != int(expected_generation):
        raise ContractError("GRASP_BINDING_MISMATCH", "generation", expected_generation, generation)
    validate_pose(final_tool, "motion", scene_revision)
    require_equal(final_tool, "request_id", request_id, "REQUEST_IDENTITY_MISMATCH")
    require_equal(final_tool, "time_basis", "acquisition", "INVALID_STAMP")
    vector(attachment.get("t_tool_box_translation"), 3, "t_tool_box_translation")
    unit_quaternion(attachment.get("t_tool_box_quaternion"), "t_tool_box_quaternion")
    if type(attachment.get("tool_stamp_ns")) is not int or attachment["tool_stamp_ns"] < 0:
        raise ContractError("INVALID_STAMP", "tool_stamp_ns", "attachment acquisition stamp", attachment.get("tool_stamp_ns"))
    if final_tool["stamp_ns"] < attachment["tool_stamp_ns"]:
        raise ContractError("INVALID_STAMP", "stamp_ns", "at or after attachment", final_tool["stamp_ns"])
    position, quaternion = compose_box_world(
        attachment, final_tool["position"], final_tool["quaternion_xyzw"])
    return dict(accepted=True, code="OK", request_id=request_id,
                scene_revision=scene_revision, box_id=box_id, generation=int(generation),
                stamp_ns=final_tool["stamp_ns"], box_position=position,
                box_quaternion_xyzw=quaternion)
