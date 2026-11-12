"""Barrier effects: verified plan first, measured attachment, release then retreat."""
from dataclasses import replace

import pytest

from luggage_description.frame_contracts import ContractError
from luggage_planning.joint_operation.contracts import seal_operation_plan
from luggage_planning.joint_operation.execution import (
    JointOperationExecution, ValidationReceipt,
)
from st09_contracts.attachment import (
    ATTACHMENT_SCHEMA_VERSION, AttachmentRecord, AttachmentSourceKind,
    AttachmentValidity,
)
from st09_contracts.attachment_geometry import freeze_geometry
from test_joint_operation_contracts import example_plan


def setup():
    plan = seal_operation_plan(example_plan())
    engine = JointOperationExecution(start_tolerance_rad=.01)
    command = engine.start(plan, plan.identity, plan.plan_digest)
    return plan, engine, command


def receipt(plan, command, geometry=None, **changes):
    result = ValidationReceipt(plan.identity, plan.plan_digest, plan.certificate_id,
                               command.leg_index, True, "OK",
                               plan.legs[command.leg_index].knots[0].positions,
                               geometry.digest() if geometry else "")
    return replace(result, **changes)


def confirmed_record(plan, geometry):
    return AttachmentRecord(
        schema_version=ATTACHMENT_SCHEMA_VERSION, cargo_id="cargo", box_id="box",
        request_id="req", cycle_id="cycle", attachment_incarnation=1,
        tcp_frame="suction_contact_frame", box_frame="world",
        dimensions_wdh=geometry.measured_dimensions_wdh, symmetry="none",
        t_tcp_box_translation=geometry.t_tool_box_translation,
        t_tcp_box_quaternion_xyzw=geometry.t_tool_box_quaternion_xyzw,
        acquisition_stamp_ns=100, scene_revision="scene", calibration_id="calibration",
        source=AttachmentSourceKind.MEASURED_ATTACH_CONFIRMED,
        source_method="same_stamp_measured_pose_and_vacuum", evidence_digest="a"*64,
        validity=AttachmentValidity.VALID, collision_envelope=geometry.conservative)


def at_attachment():
    plan, engine, cmd = setup()
    motion = engine.validation_finished(cmd.token, receipt(plan, cmd))
    assert motion.kind == "execute_leg"
    cmd = engine.motion_finished(motion.token, success=True)
    assert cmd.kind == "confirm_attachment"
    return plan, engine, cmd


def at_carry(geometry=None):
    plan, engine, cmd = at_attachment()
    geometry = geometry or plan.predicted_attachment
    record = confirmed_record(plan, geometry)
    cmd = engine.attachment_confirmed(cmd.token, record=record,
                                      geometry=geometry, generation=plan.identity.generation)
    assert cmd.kind == "validate_attached_suffix"
    return plan, engine, cmd, geometry


def test_complete_reuse_executes_exact_original_knots_and_both_barriers():
    plan, engine, cmd = setup()
    executed = []
    assert cmd.kind == "validate_start"
    cmd = engine.validation_finished(cmd.token, receipt(plan, cmd))
    executed.append(engine.pending_leg(cmd.token))
    cmd = engine.motion_finished(cmd.token, success=True)
    assert cmd.kind == "confirm_attachment"
    geometry = plan.predicted_attachment
    cmd = engine.attachment_confirmed(cmd.token, record=confirmed_record(plan, geometry),
                                      geometry=geometry, generation=plan.identity.generation)
    cmd = engine.validation_finished(cmd.token, receipt(plan, cmd, geometry))
    executed.append(engine.pending_leg(cmd.token))
    cmd = engine.motion_finished(cmd.token, success=True)
    assert cmd.kind == "confirm_release" and engine.may_have_payload
    cmd = engine.release_confirmed(cmd.token, success=True)
    assert cmd.kind == "apply_release_and_validate_retreat"
    assert not engine.may_have_payload
    cmd = engine.validation_finished(cmd.token, receipt(plan, cmd))
    executed.append(engine.pending_leg(cmd.token))
    assert engine.motion_finished(cmd.token, success=True) is None
    assert engine.state == "COMPLETE"
    assert tuple(executed) == plan.legs
    assert all(a is b for a, b in zip(executed, plan.legs))
    # Consumer has no planner dependency: every issued motion is an original leg.


def test_no_execution_before_preflight_and_bad_plan_never_starts():
    plan, engine, cmd = setup()
    with pytest.raises(ContractError, match="STALE_OPERATION_CALLBACK"):
        engine.pending_leg(cmd.token)
    assert engine.motion_finished(cmd.token, success=True) is None
    other = JointOperationExecution(start_tolerance_rad=.01)
    with pytest.raises(ContractError, match="OPERATION_DIGEST_MISMATCH"):
        other.start(plan, plan.identity, "wrong")
    assert other.state == "IDLE"


@pytest.mark.parametrize("changes,reason", [
    ({"accepted": False, "reason": "COLLISION"}, "COLLISION"),
    ({"actual_joints": (1.,)*6}, "ACTUAL_START_JOINTS_MISMATCH"),
    ({"plan_digest": "old"}, "VALIDATION_IDENTITY_MISMATCH"),
    ({"next_leg_index": 1}, "VALIDATION_IDENTITY_MISMATCH"),
    ({"certificate_id": "old"}, "VALIDATION_IDENTITY_MISMATCH"),
    ({"identity": replace(example_plan().identity, map_revision=8)},
     "VALIDATION_IDENTITY_MISMATCH"),
])
def test_preflight_refusals_emit_zero_motion(changes, reason):
    plan, engine, cmd = setup()
    assert engine.validation_finished(cmd.token, receipt(plan, cmd, **changes)) is None
    assert engine.state == "STOPPED" and engine.reason == reason


@pytest.mark.parametrize("change", ["pre_attach", "invalidated", "other_box", "wrong_geometry"])
def test_confirmed_measured_attachment_required_before_carry(change):
    plan, engine, cmd = at_attachment()
    geometry = plan.predicted_attachment
    record = confirmed_record(plan, geometry)
    if change == "pre_attach":
        record = replace(record, source=AttachmentSourceKind.MEASURED_PRE_ATTACH_ESTIMATE)
    elif change == "invalidated":
        record = replace(record, validity=AttachmentValidity.INVALIDATED,
                          invalidation_reason="slip")
    elif change == "other_box":
        record = replace(record, box_id="other")
    else:
        geometry = freeze_geometry((.50, .40, .25), geometry.t_tool_box_translation,
                                   geometry.t_tool_box_quaternion_xyzw,
                                   geometry.conservative.limits)
    assert engine.attachment_confirmed(cmd.token, record=record,
                                       geometry=geometry, generation=2) is None
    assert engine.state == "CARRY_FAULT" and engine.may_have_payload


def test_changed_actual_geometry_is_revalidated_without_rewriting_prediction():
    predicted = example_plan().predicted_attachment
    actual = freeze_geometry((.50, .40, .25), predicted.t_tool_box_translation,
                             predicted.t_tool_box_quaternion_xyzw,
                             predicted.conservative.limits)
    plan, engine, cmd, geometry = at_carry(actual)
    assert geometry != plan.predicted_attachment
    # A receipt for predicted geometry cannot release a different actual payload.
    assert engine.validation_finished(cmd.token, receipt(plan, cmd, predicted)) is None
    assert engine.state == "CARRY_FAULT"
    plan, engine, cmd, geometry = at_carry(actual)
    accepted = engine.validation_finished(cmd.token, receipt(plan, cmd, actual))
    assert accepted.kind == "execute_leg"
    assert engine.plan.predicted_attachment == predicted


def test_suffix_refusal_preserves_payload_and_emits_no_old_carry_motion():
    plan, engine, cmd, geometry = at_carry()
    assert engine.validation_finished(cmd.token, receipt(
        plan, cmd, geometry, accepted=False, reason="ACTUAL_TARGET_INVALID")) is None
    assert engine.state == "CARRY_FAULT" and engine.may_have_payload


def test_release_requires_confirmation_and_world_scene_validation():
    plan, engine, cmd, geometry = at_carry()
    motion = engine.validation_finished(cmd.token, receipt(plan, cmd, geometry))
    release = engine.motion_finished(motion.token, success=True)
    assert engine.release_confirmed(release.token, success=False) is None
    assert engine.state == "CARRY_FAULT" and engine.may_have_payload
    plan, engine, cmd, geometry = at_carry()
    motion = engine.validation_finished(cmd.token, receipt(plan, cmd, geometry))
    release = engine.motion_finished(motion.token, success=True)
    validate = engine.release_confirmed(release.token, success=True)
    assert engine.motion_finished(validate.token, success=True) is None
    assert engine.validation_finished(validate.token, receipt(
        plan, validate, accepted=False, reason="RELEASED_BOX_BLOCKS_RETREAT")) is None
    assert engine.state == "STOPPED"


def test_duplicate_callbacks_cannot_double_execute_or_double_attach():
    plan, engine, cmd = setup()
    motion = engine.validation_finished(cmd.token, receipt(plan, cmd))
    assert engine.validation_finished(cmd.token, receipt(plan, cmd)) is None
    attach = engine.motion_finished(motion.token, success=True)
    assert engine.motion_finished(motion.token, success=True) is None
    geometry = plan.predicted_attachment
    record = confirmed_record(plan, geometry)
    validate = engine.attachment_confirmed(attach.token, record=record,
                                          geometry=geometry, generation=2)
    assert validate.kind == "validate_attached_suffix"
    assert engine.attachment_confirmed(attach.token, record=record,
                                       geometry=geometry, generation=2) is None


def test_carry_motion_failure_never_issues_release():
    plan, engine, cmd, geometry = at_carry()
    motion = engine.validation_finished(cmd.token, receipt(plan, cmd, geometry))
    assert engine.motion_finished(motion.token, success=False) is None
    assert engine.state == "CARRY_FAULT" and engine.may_have_payload


def test_motion_transport_error_cancels_before_allowing_recovery():
    plan, engine, cmd, geometry = at_carry()
    motion = engine.validation_finished(cmd.token, receipt(plan, cmd, geometry))
    cancel = engine.event_failed(motion.token, "ACTION_TRANSPORT_FAILED")
    assert cancel.kind == "cancel_motion"
    assert engine.state == "CANCELLING" and engine.may_have_payload
    assert engine.motion_finished(motion.token, success=False) is None
    assert engine.state == "CARRY_FAULT"


def test_carry_cancellation_waits_for_terminal_motion_and_never_issues_release():
    plan, engine, cmd, geometry = at_carry()
    motion = engine.validation_finished(cmd.token, receipt(plan, cmd, geometry))
    cancel = engine.cancel()
    assert cancel.kind == "cancel_motion" and cancel.token == motion.token
    assert engine.state == "CANCELLING" and engine.may_have_payload
    assert engine.cancel() is None
    assert engine.motion_finished(motion.token, success=True) is None
    assert engine.state == "CARRY_FAULT" and engine.may_have_payload
