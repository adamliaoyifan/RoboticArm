"""VacuumCommandV2 wire conversion. Planning poses stay on the measured record."""

import json

from luggage_msgs.srv import VacuumCommandV2
from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import BoundaryContext, BoundaryContractError
from luggage_description.ros_frame_bindings.diagnostics import diagnostic_to_message
from luggage_description.ros_frame_bindings.identity import require_message
from luggage_description.frame_contracts import unit_quaternion
from luggage_description.ros_frame_bindings.poses import pose_from_message
from luggage_planning.attachment.attachment_frame_contracts import (
    admit_attachment,
    admit_release,
    compose_box_world,
)
from luggage_planning.pick.pick_authorization import HEIGHT_SOURCE_MEASURED_SUPPORT


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


def pose_scene_revision(pose):
    """Scene revision carried by a pose record or a PoseContractV2 message."""
    if isinstance(pose, dict):
        return str(pose.get("scene_revision") or "")
    identity = getattr(pose, "identity", None)
    if identity is not None:
        return str(getattr(identity, "scene_revision", "") or "")
    return ""


def attachment_revision_evidence(request, box_pose, tool_pose, bound_revision):
    """Name every scene revision an attach refusal can be blamed on.

    The equality check itself stays in the frame contract. This snapshot
    only records the values that were compared so a VACUUM_ATTACH dump
    shows which side is stale.
    """
    measured = getattr(request, "measured_box", None)
    header = getattr(measured, "header", None)
    stamp = getattr(header, "stamp", None)
    stamp_ns = None
    if stamp is not None:
        stamp_ns = int(getattr(stamp, "sec", 0)) * 10 ** 9 + int(
            getattr(stamp, "nanosec", 0))
    generation = getattr(request, "generation", None)
    return {
        "request_scene_revision": str(getattr(request, "scene_revision", "") or ""),
        "box_pose_scene_revision": pose_scene_revision(box_pose),
        "tool_pose_scene_revision": pose_scene_revision(tool_pose),
        "bound_scene_revision": str(bound_revision or ""),
        "measured_stamp_ns": stamp_ns,
        "box_id": str(getattr(request, "box_id", "") or ""),
        "generation": None if generation is None else int(generation),
    }


def revision_evidence_from_message(message):
    """Parse the revision snapshot appended to a mismatch response."""
    text = str(message or "")
    _code, sep, payload = text.partition(" ")
    if not sep or not payload.startswith("{"):
        return None
    try:
        data = json.loads(payload)
    except ValueError:
        return None
    if not isinstance(data, dict):
        return None
    return data


def vacuum_attach_revision_dump(service_message, *, live_scene_revision, fallback):
    """Driver-side VACUUM_ATTACH record: service snapshot plus live identity."""
    evidence = revision_evidence_from_message(service_message)
    if evidence is None:
        evidence = dict(fallback or {})
    else:
        evidence = dict(evidence)
    evidence["live_scene_revision"] = str(live_scene_revision or "")
    # The handler's bound revision is the adapter authority for this call.
    # The driver's live identity is recorded separately so a stale bind
    # (example scene or atlas scene_revision) is visible beside it.
    evidence["adapter_scene_revision"] = str(
        evidence.get("bound_scene_revision") or "")
    return evidence


def bound_attachment_scene_revision(runtime_identity, fallback_hash):
    """Revision the attach freeze is bound to.

    A frame-runtime identity is the live scene. Its revision is used even
    when a fallback file hash is also available, and an empty identity
    revision does not fall through to that hash. The fallback is only the
    legacy path with no measured identity at all. Atlas metadata is not a
    source.
    """
    if runtime_identity is not None:
        return str(runtime_identity.get("scene_revision") or "").strip()
    return str(fallback_hash or "")


def _mismatch_message(error, evidence):
    code = getattr(error, "code", str(error))
    if evidence is None or code != "CALIBRATION_REVISION_MISMATCH":
        return code
    return code + " " + json.dumps(
        evidence, sort_keys=True, separators=(",", ":"))


def _fail(response, context, record, error, evidence=None):
    response.success = False
    response.message = _mismatch_message(error, evidence)
    response.diagnostic = diagnostic_to_message(
        record if isinstance(record, dict) else {}, context, error)
    return response


def _encode_measured_attachment(attachment, request, scene_revision, geometry_policy):
    from luggage_planning.planning_contract.attachment_geometry import freeze_geometry
    from luggage_planning.planning_contract._base import ContractViolation
    from luggage_planning.attachment.attachment_geometry_policy import load_attachment_geometry_policy
    from luggage_planning.attachment.attachment_geometry_transport import geometry_to_message
    try:
        policy = geometry_policy if geometry_policy is not None else load_attachment_geometry_policy()
        geometry = freeze_geometry(attachment['size_wdh'],
            attachment['t_tool_box_translation'], attachment['t_tool_box_quaternion'], policy)
        geometry_message = geometry_to_message(geometry)
    except ContractViolation as error:
        raise ContractError('ATTACHMENT_GEOMETRY_INVALID', 'conservative_geometry',
                            'qualified bounded geometry', str(error)) from error
    # One fresh record at the attachment boundary. Nominal fields retain
    # their measured meaning for physical retention and release evidence.
    attachment = dict(attachment, conservative_geometry=geometry)
    from luggage_planning.place.place_service import pose_message
    actual = attachment['t_box_tool']
    measured_grasp = pose_message({
        'contract_version': 2, 'frame_id': 'suction_contact_frame',
        'length_unit': 'm', 'time_basis': 'scene',
        'scene_revision': scene_revision, 'request_id': str(request.request_id),
        'stamp_ns': attachment['tool_stamp_ns'],
        'reference': 'suction_contact_frame', 'orientation_semantics': 'box',
        'position': actual['translation'],
        'quaternion_xyzw': actual['quaternion_xyzw'],
    })
    return attachment, measured_grasp, geometry_message


def handle_vacuum_command_v2(request, response, *, scene_revision, generation,
                             resolve_tool, backend, physical_relation=None,
                             geometry_policy=None):
    """Admit a measured snapshot, then attach or detach. GT is not read here."""
    require_message(request, VacuumCommandV2.Request)
    response.has_measured_attachment = False
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
    tool = None
    try:
        if not request.enable:
            tool = resolve_tool(None) if resolve_tool else None
            detach = admit_release(
                getattr(backend, "last_attach_record", None), tool,
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
        # Resolve the contact pose before admission so a revision refusal
        # can name the tool revision as well as the detection revision.
        tool = resolve_tool(None) if resolve_tool else None
        box_pose = pose_from_message(request.box_pose, "detection", context)
        record = box_pose
        # F18: freeze T_box_tool from the live TCP at the attach command,
        # not the detection-stamp TCP (pickup_observe is camera-centered).
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
        # Reject malformed geometry before any actuator effect.
        attachment, measured_grasp, geometry_message = _encode_measured_attachment(
            attachment, request, scene_revision, geometry_policy)
        from luggage_planning.attachment.vacuum_instance_identity import actuator_instance_id
        ok, message = backend.attach({
            "attach_record": attachment,
            "tool_pose": tool,
            "physical_relation": physical_relation,
            "model_name": actuator_instance_id(request),
            "scene_box_size": attachment["size_wdh"],
        })
        response.success = bool(ok)
        response.message = message
        if ok:
            # The actuator can block for seconds while waiting for seal. Read
            # the real TCP again after confirmation, preserving its own stamp.
            # A failure here must still report the successful physical seal;
            # no measured relation and no automatic detach can be fabricated.
            try:
                confirmed_tool = resolve_tool(None) if resolve_tool else None
                confirmed = admit_attachment(confirmed_tool, box_pose, measured, grasp,
                    int(request.generation), int(generation), str(request.box_id), context)
                if confirmed['tool_stamp_ns'] < attachment['tool_stamp_ns']:
                    raise ContractError('ATTACHMENT_TCP_STAMP_REGRESSED', 'tool_pose.stamp_ns',
                                        attachment['tool_stamp_ns'], confirmed['tool_stamp_ns'])
                attachment, measured_grasp, geometry_message = _encode_measured_attachment(
                    confirmed, request, scene_revision, geometry_policy)
                backend.last_attach_record = attachment
            except Exception as error:
                response.message = 'SEALED_ATTACHMENT_MEASUREMENT_UNQUALIFIED:' + str(error)
                response.diagnostic = diagnostic_to_message(box_pose, context, ContractError(
                    'SEALED_ATTACHMENT_MEASUREMENT_UNQUALIFIED', 'confirmed_tool_pose',
                    'qualified measured TCP after confirmed seal', str(error)))
                return response
            response.measured_grasp = measured_grasp
            response.attached_box_wdh = list(attachment['size_wdh'])
            response.conservative_geometry = geometry_message
            response.has_measured_attachment = True
        response.diagnostic = diagnostic_to_message(box_pose, context, None if ok else ContractError(
            "ATTACHMENT_BACKEND_REFUSED", "backend", "successful physical attachment", message))
        return response
    except BoundaryContractError as exc:
        return _fail(response, context, record, exc, _revision_evidence(
            request, record, tool, scene_revision))
    except ContractError as exc:
        wrapped = BoundaryContractError(exc, record, context)
        return _fail(response, context, record, wrapped, _revision_evidence(
            request, record, tool, scene_revision))


def _revision_evidence(request, record, tool, scene_revision):
    box_pose = record if record else getattr(request, "box_pose", None)
    return attachment_revision_evidence(request, box_pose, tool, scene_revision)

