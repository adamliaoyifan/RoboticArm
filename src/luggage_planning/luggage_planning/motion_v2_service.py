"""PlanMotionV2 / ProbeMotionSegmentV2 callbacks. Admission precedes effects."""

from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import BoundaryContext, BoundaryContractError
from luggage_planning.motion_frame_adapters import (
    apply_diagnostic,
    record_from_message,
    segment_namespace,
)
from luggage_planning.motion_frame_contracts import (
    MOTION_FRAME,
    TARGET_LINK,
    ExecutionContext,
    admit_motion_segment,
)


_BASES = ("acquisition", "robot_state", "scene")


def execution_context(message, scene_revision, link=TARGET_LINK):
    basis = str(message.time_basis) if str(message.time_basis) in _BASES else "scene"
    request_id = str(message.request_id or "").strip() or " "
    boundary = BoundaryContext(
        interface_id="F15",
        producer="motion_producer",
        consumer="motion_executor",
        request_id=request_id,
        scene_revision=str(scene_revision),
        frame_id=MOTION_FRAME,
        reference="suction_contact_frame",
        time_basis=basis,
    )
    return ExecutionContext(boundary, str(link or TARGET_LINK), MOTION_FRAME)


def _rejected(response, record, context, error):
    response.success = False
    response.ik_ok = False
    response.fraction = 0.0
    response.message = getattr(error, "code", "INVALID_RECORD")
    apply_diagnostic(response, record, context, error)
    return response


def handle_probe_v2(executor, message, scene_revision, response, link=TARGET_LINK,
                    start_joints=None):
    record = {}
    context = execution_context(message, scene_revision, link)
    try:
        record = record_from_message(message)
        result = executor.probe_segment_v2(
            record, context, start_joints=start_joints)
    except BoundaryContractError as exc:
        return _rejected(response, record, context, exc)
    except ContractError as exc:
        return _rejected(response, record, context, BoundaryContractError(exc, record, context.boundary))
    response.success = bool(result.get("success", result.get("ik_ok")))
    response.ik_ok = bool(result.get("ik_ok"))
    fraction = result.get("fraction")
    response.fraction = -1.0 if fraction is None else float(fraction)
    response.message = str(result.get("message") or "")
    response.moveit_error_code = int(result.get("moveit_error_code") or 0)
    apply_diagnostic(response, record, context, None)
    if str(response.message).startswith("FK_VERIFY"):
        response.success = False
        response.ik_ok = False
    return response


def handle_plan_v2(executor, message, scene_revision, response, prepare_occupancy,
                   link=TARGET_LINK, current_joints=None, execute_timeout=45.0):
    """Admit first. Occupancy and execution run only after the record is valid."""
    record = {}
    context = execution_context(message, scene_revision, link)
    try:
        record = record_from_message(message)
        admitted = admit_motion_segment(record, context)
    except BoundaryContractError as exc:
        response.success = False
        response.fraction = 0.0
        response.message = exc.code
        apply_diagnostic(response, record, context, exc)
        return response
    except ContractError as exc:
        wrapped = BoundaryContractError(exc, record, context.boundary)
        response.success = False
        response.message = exc.code
        apply_diagnostic(response, record, context, wrapped)
        return response
    if prepare_occupancy is not None and admitted.segment_type != "joint_path":
        occ = prepare_occupancy(segment_namespace(admitted))
        if occ:
            response.success = False
            response.message = str(occ)
            apply_diagnostic(response, record, context, None)
            return response
    result = executor.execute_segment_v2(
        record, context, current_joints=current_joints, execute_timeout=execute_timeout)
    response.success = bool(result.success)
    response.message = str(result.message)
    response.fraction = float(result.fraction)
    response.used_ompl_fallback = bool(result.used_ompl_fallback)
    response.moveit_error_code = int(result.moveit_error_code)
    apply_diagnostic(response, record, context, None)
    return response
