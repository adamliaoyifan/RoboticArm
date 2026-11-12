"""Behavior of complete, identity-bound plans independent of ROS or IK mocks."""
from dataclasses import replace

import pytest

from luggage_description.frame_contracts import ContractError
from luggage_planning.joint_operation.contracts import (
    JointKnot, JointOperationPlan, OperationBarrier, OperationIdentity,
    OperationLeg, admit_operation_plan, seal_operation_plan,
    OPERATION_JOINTS,
)
from luggage_planning.placement_prior.contracts import Transform
from st09_contracts.attachment_geometry import freeze_geometry
from st09_contracts.payload_envelope import EnvelopeLimits


def example_plan():
    identity = OperationIdentity("req", "scene", "snapshot", "hull", "model",
                                 7, "observation", 12, 15, "box", 2, "contact")
    # Geometry uses measured dimensions; no catalog is involved.
    attachment = freeze_geometry((.55, .40, .25), (0., 0., .125),
                                 (1., 0., 0., 0.),
                                 EnvelopeLimits(0., (0., 0., 0.), (0., 0.), 0.))
    grasp = Transform(attachment.t_tool_box_translation,
                      attachment.t_tool_box_quaternion_xyzw)
    pick_tool = Transform((1., 0., 1.), (1., 0., 0., 0.))
    place_tool = Transform((0., 1., 1.), (0., 1., 0., 0.))
    q = [tuple([i*.1]*6) for i in range(4)]
    legs = []
    for i, phase in enumerate(("pick", "carry", "retreat")):
        knots = (JointKnot(q[i], (0.,)*6, (0.,)*6, (), 0),
                 JointKnot(q[i+1], (0.,)*6, (0.,)*6, (), 10**9))
        legs.append(OperationLeg(phase, phase, "req", "scene", 15,
                                 OPERATION_JOINTS, knots))
    return JointOperationPlan(identity, "candidate", "certificate",
                              pick_tool.compose(grasp), pick_tool,
                              place_tool.compose(grasp), place_tool,
                              Transform(tuple(v-d for v, d in zip(
                                  place_tool.compose(grasp).translation, (0., 0., .05))),
                                  place_tool.compose(grasp).quaternion_xyzw),
                              attachment, q[0], tuple(legs),
                              (OperationBarrier(1, "attachment_confirmed"),
                               OperationBarrier(2, "release_confirmed")), "")


def test_complete_chain_admitted_and_keeps_directed_grasp():
    plan = seal_operation_plan(example_plan())
    assert admit_operation_plan(plan, plan.identity, plan.plan_digest) == plan
    assert len(plan.plan_digest) == 64
    assert plan.world_from_pick_tool != plan.world_from_place_tool
    assert plan.identity.observation_stamp_ns != plan.identity.robot_state_stamp_ns
    assert plan.world_from_settled_box != plan.world_from_place_box


def test_settled_target_is_independent_and_digest_bound():
    plan = seal_operation_plan(example_plan())
    p = plan.world_from_settled_box
    changed = replace(plan, world_from_settled_box=Transform(
        (p.translation[0]+.02, *p.translation[1:]), p.quaternion_xyzw))
    with pytest.raises(ContractError, match="OPERATION_DIGEST_MISMATCH"):
        admit_operation_plan(changed, plan.identity, plan.plan_digest)


def test_missing_or_rotated_settled_pose_refused():
    plan = example_plan()
    with pytest.raises(ContractError, match="INVALID_OPERATION_PLAN"):
        seal_operation_plan(replace(plan, world_from_settled_box=None))
    with pytest.raises(ContractError, match="SETTLED_ORIENTATION_MISMATCH"):
        seal_operation_plan(replace(plan, world_from_settled_box=Transform(
            plan.world_from_settled_box.translation, (0., 0., 0., 1.))))


@pytest.mark.parametrize("field,value", [
    ("request_id", "other"), ("scene_revision", "other"),
    ("snapshot_id", "other"), ("geometry_hash", "other"),
    ("model_sha256", "other"), ("map_revision", 8),
    ("observation_id", "other"), ("observation_stamp_ns", 13),
    ("robot_state_stamp_ns", 16), ("box_id", "other"),
    ("generation", 3), ("contact_model_hash", "other"),
])
def test_independently_pinned_identity_must_match(field, value):
    plan = seal_operation_plan(example_plan())
    expected = replace(plan.identity, **{field: value})
    with pytest.raises(ContractError, match="OPERATION_IDENTITY_MISMATCH"):
        admit_operation_plan(plan, expected, plan.plan_digest)


@pytest.mark.parametrize("phases", [
    ("pick", "retreat"), ("pick", "carry"), ("carry", "retreat"),
    ("pick", "retreat", "carry"), ("pick", "carry", "pick"),
])
def test_partial_or_regressing_chain_cannot_be_sealed(phases):
    plan = example_plan()
    legs = tuple(replace(leg, phase=phases[i], name=str(i))
                 for i, leg in enumerate(plan.legs[:len(phases)]))
    with pytest.raises(ContractError, match="INCOMPLETE_OPERATION_PLAN"):
        seal_operation_plan(replace(plan, legs=legs))


@pytest.mark.parametrize("barriers", [(), (OperationBarrier(2, "attachment_confirmed"),),
    (OperationBarrier(1, "release_confirmed"), OperationBarrier(2, "attachment_confirmed")),
    (OperationBarrier(2, "release_confirmed"), OperationBarrier(1, "attachment_confirmed")),
])
def test_missing_or_misordered_event_barriers_reject(barriers):
    with pytest.raises(ContractError, match="INVALID_OPERATION_BARRIERS"):
        seal_operation_plan(replace(example_plan(), barriers=barriers))


def test_postplan_joint6_rewrite_rejected_even_if_endpoints_unchanged():
    plan = seal_operation_plan(example_plan())
    carry = plan.legs[1]
    middle = replace(carry.knots[0], time_from_start_ns=500_000_000,
                     positions=carry.knots[0].positions[:5]+(2.,))
    changed = replace(carry, knots=(carry.knots[0], middle, carry.knots[1]))
    tampered = replace(plan, legs=(plan.legs[0], changed, plan.legs[2]))
    with pytest.raises(ContractError, match="OPERATION_DIGEST_MISMATCH"):
        admit_operation_plan(tampered, plan.identity, plan.plan_digest)


def test_winding_mismatch_not_hidden_by_modulo_2pi():
    plan = example_plan()
    leg = plan.legs[1]
    knot = replace(leg.knots[0], positions=leg.knots[0].positions[:5]+(6.383185307,))
    leg = replace(leg, knots=(knot, leg.knots[1]))
    with pytest.raises(ContractError, match="JOINT_CHAIN_DISCONTINUITY"):
        seal_operation_plan(replace(plan, legs=(plan.legs[0], leg, plan.legs[2])))


@pytest.mark.parametrize("mutation,code", [
    (lambda leg: replace(leg, trajectory_request_id="other"), "REQUEST_IDENTITY_MISMATCH"),
    (lambda leg: replace(leg, trajectory_scene_revision="other"), "CALIBRATION_REVISION_MISMATCH"),
    (lambda leg: replace(leg, knots=()), "INVALID_JOINT_TRAJECTORY"),
    (lambda leg: replace(leg, joint_names=tuple(reversed(leg.joint_names))), "INVALID_JOINT_NAMES"),
    (lambda leg: replace(leg, knots=(leg.knots[0], replace(leg.knots[1], time_from_start_ns=0))), "INVALID_TRAJECTORY_TIME"),
    (lambda leg: replace(leg, knots=(leg.knots[0], replace(leg.knots[1], velocities=()))), "NONSTATIONARY_OPERATION_BARRIER"),
])
def test_bad_trajectory_cannot_be_sealed(mutation, code):
    plan = example_plan()
    with pytest.raises(ContractError, match=code):
        seal_operation_plan(replace(plan, legs=(mutation(plan.legs[0]),)+plan.legs[1:]))


def test_changing_pick_tool_without_matching_box_is_rejected():
    plan = example_plan()
    changed = Transform(plan.world_from_pick_tool.translation, (0., 1., 0., 0.))
    with pytest.raises(ContractError, match="GRASP_BINDING_MISMATCH"):
        seal_operation_plan(replace(plan, world_from_pick_tool=changed))


def test_plan_input_lists_are_frozen_and_not_aliased():
    plan = example_plan()
    legs = list(plan.legs)
    frozen = replace(plan, legs=legs)
    legs.clear()
    assert len(frozen.legs) == 3
