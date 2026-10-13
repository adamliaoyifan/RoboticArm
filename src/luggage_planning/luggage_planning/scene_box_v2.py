"""Typed scene-box admission. v1 add_placed_box stays a verbatim world insert."""

from luggage_description.frame_contracts import (
    ContractError,
    finite_number,
    require_horizontal,
)
from luggage_description.frame_diagnostics import BoundaryContext, BoundaryContractError
from luggage_description.frame_transforms import transform_pose
from luggage_description.ros_frame_bindings.diagnostics import diagnostic_to_message
from luggage_description.ros_frame_bindings.poses import pose_from_message
from luggage_description.scene_tf_publisher import rpy_to_quaternion
from luggage_planning.motion_frame_contracts import scene_transform
from luggage_planning.planning_scene_client import build_collision_object_v2


def _context(request_id, scene_revision):
    return BoundaryContext(
        interface_id="F17",
        producer="scene_manager",
        consumer="planning_scene",
        request_id=str(request_id or "scene-box"),
        scene_revision=str(scene_revision),
        frame_id="container_link",
        reference="box_center",
        time_basis="scene",
    )


def admit_placed_box(slot_message, size_wdh, box_id, scene_revision, origin_xyz, rpy):
    context = _context(slot_message.identity.request_id, scene_revision)
    pose = pose_from_message(slot_message, "slot", context)
    if not isinstance(box_id, str) or not box_id.strip():
        raise ContractError("MISSING_IDENTITY", "box_id", "nonempty", box_id)
    size = []
    for index, value in enumerate(size_wdh):
        number = finite_number(float(value), "size_wdh[%d]" % index)
        if number <= 0.0:
            raise ContractError(
                "INVALID_NUMERIC_PAYLOAD", "size_wdh", "positive", number)
        size.append(number)
    if len(size) != 3:
        raise ContractError("INVALID_NUMERIC_PAYLOAD", "size_wdh", "3 numbers", size_wdh)
    quaternion = rpy_to_quaternion(rpy)
    require_horizontal(quaternion)
    transform = scene_transform(origin_xyz, rpy, scene_revision)
    world = transform_pose(pose, transform)
    return context, pose, world, size, box_id.strip()


def handle_add_scene_box_v2(request, response, scene_client, scene_revision,
                            origin_xyz, rpy):
    context = _context(
        getattr(request.slot.identity, "request_id", ""), scene_revision)
    try:
        _context_out, _pose, world, size, box_id = admit_placed_box(
            request.slot, request.size_wdh, request.box_id,
            scene_revision, origin_xyz, rpy)
        obj = build_collision_object_v2(
            box_id, world["position"], world["quaternion_xyzw"], size)
        ok, message = scene_client.add_collision_object(obj)
        response.success = bool(ok)
        response.message = message
        response.diagnostic = diagnostic_to_message(world, _context_out, None)
        try:
            response.admitted_world = world
        except AttributeError:
            pass
        return response
    except BoundaryContractError as exc:
        response.success = False
        response.message = exc.code
        response.diagnostic = diagnostic_to_message({}, context, exc)
        return response
    except ContractError as exc:
        wrapped = BoundaryContractError(exc, {}, context)
        response.success = False
        response.message = exc.code
        response.diagnostic = diagnostic_to_message({}, context, wrapped)
        return response


def handle_remove_scene_box_v2(request, response, scene_client):
    if not str(request.box_id).strip():
        response.success = False
        response.message = "MISSING_IDENTITY"
        return response
    ok, message = scene_client.remove_object(str(request.box_id))
    response.success = bool(ok)
    response.message = message
    return response
