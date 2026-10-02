"""VacuumCommandV2 wire conversion. Planning poses stay on the measured record."""

from luggage_msgs.srv import VacuumCommandV2
from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import BoundaryContext, BoundaryContractError
from luggage_description.ros_frame_bindings.diagnostics import diagnostic_to_message
from luggage_description.ros_frame_bindings.identity import require_message
from luggage_description.frame_contracts import unit_quaternion
from luggage_description.ros_frame_bindings.poses import pose_from_message
from luggage_planning.attachment_frame_contracts import (
    admit_attachment,
    admit_release,
    compose_box_world,
)
from luggage_planning.pick_authorization import HEIGHT_SOURCE_MEASURED_SUPPORT


def measured_dict(box, box_id):
    return {
        "id": str(box_id or box.id),
        "header_frame": str(box.header.frame_id or "world"),
        "stamp_ns": int(box.header.stamp.sec) * 10**9 + int(box.header.stamp.nanosec),
        "position": [box.pose.position.x, box.pose.position.y, box.pose.position.z],
        "quaternion_xyzw": [
            box.pose.orientation.x, box.pose.orientation.y,
            box.pose.orientation.z, box.pose.orientation.w],
        "width": float(box.width),
        "depth": float(box.depth),
        "height": float(box.height),
        "height_valid": bool(box.height_valid),
        "height_source": int(box.height_source),
    }


def _context(request, scene_revision):
    return BoundaryContext(
        interface_id="F18",
        producer="vacuum_command_v2",
        consumer="vacuum_backend",
        request_id=str(request.request_id),
        scene_revision=str(scene_revision),
        frame_id="world",
        reference="box_center",
        time_basis="acquisition",
    )


def _fail(response, context, record, error):
    response.success = False
    response.message = getattr(error, "code", str(error))
    response.diagnostic = diagnostic_to_message(
        record if isinstance(record, dict) else {}, context, error)
    return response


def handle_vacuum_command_v2(request, response, *, scene_revision, generation,
                             resolve_tool, backend, physical_relation=None):
    """Admit a measured snapshot, then attach or detach. GT is not read here."""
    require_message(request, VacuumCommandV2.Request)
    try:
        context = _context(request, scene_revision)
    except ContractError as error:
        from luggage_msgs.msg import BoundaryDiagnosticV2
        response.success = False
        response.message = error.code
        response.diagnostic = BoundaryDiagnosticV2(
            contract_version=2, interface_id="F18", request_id=str(request.request_id),
            producer="vacuum_command_v2", consumer="vacuum_backend", accepted=False,
            code=error.code, field="request_id", expected="nonempty request identity",
            received=str(request.request_id))
        return response
    record = {}
    try:
        if not request.enable:
            final = resolve_tool(None) if resolve_tool else None
            detach = admit_release(
                getattr(backend, "last_attach_record", None), final,
                request_id=str(request.request_id), scene_revision=scene_revision,
                box_id=str(request.box_id), generation=int(request.generation),
                expected_generation=int(generation))
            if str(request.scene_revision) != scene_revision:
                raise ContractError("CALIBRATION_REVISION_MISMATCH", "scene_revision",
                                    scene_revision, str(request.scene_revision))
            ok, message = backend.detach({"detach_record": detach})
            backend.last_detach_record = detach
            response.success = bool(ok)
            response.message = message
            error = None if ok else ContractError(
                "RELEASE_BACKEND_FAILED", "detach", "successful release", message)
            response.diagnostic = diagnostic_to_message({}, context, error)
            return response
        box_pose = pose_from_message(request.box_pose, "detection", context)
        record = box_pose
        # F18: freeze T_box_tool from the live TCP at the attach command,
        # not the detection-stamp TCP (pickup_observe is camera-centered).
        tool = resolve_tool(None) if resolve_tool else None
        if not tool:
            raise ContractError(
                "MISSING_IDENTITY", "tool_pose", "tool at attach", None)
        grasp = None
        if request.has_grasp:
            grasp = {
                "translation": [
                    float(request.grasp.pose.position.x),
                    float(request.grasp.pose.position.y),
                    float(request.grasp.pose.position.z)],
                "quaternion_xyzw": unit_quaternion([
                    float(request.grasp.pose.orientation.x),
                    float(request.grasp.pose.orientation.y),
                    float(request.grasp.pose.orientation.z),
                    float(request.grasp.pose.orientation.w)], "grasp.quaternion_xyzw"),
            }
        measured = measured_dict(request.measured_box, request.box_id)
        if int(measured["height_source"]) != HEIGHT_SOURCE_MEASURED_SUPPORT:
            raise ContractError(
                "DETECT_FULL_GEOMETRY_REQUIRED", "height_source",
                HEIGHT_SOURCE_MEASURED_SUPPORT, measured["height_source"])
        attachment = admit_attachment(
            tool, box_pose, measured, grasp, int(request.generation),
            int(generation), str(request.box_id), context)
        ok, message = backend.attach({
            "attach_record": attachment,
            "physical_relation": physical_relation,
            "model_name": str(request.box_id),
            "scene_box_size": attachment["size_wdh"],
        })
        response.success = bool(ok)
        response.message = message
        response.diagnostic = diagnostic_to_message(box_pose, context, None if ok else None)
        return response
    except BoundaryContractError as exc:
        return _fail(response, context, record, exc)
    except ContractError as exc:
        wrapped = BoundaryContractError(exc, record, context)
        return _fail(response, context, record, wrapped)

