"""Generated wire round trips and malformed consumer inputs (ROS required)."""
import pytest

pytest.importorskip("rclpy")
from rclpy.serialization import serialize_message, deserialize_message
from luggage_msgs.msg import JointOperationPlanV2
from luggage_msgs.srv import PrepareJointOperationV2
from luggage_description.frame_contracts import ContractError
from luggage_planning.joint_operation.adapters import (
    operation_plan_from_message, operation_plan_to_message,
)
from luggage_planning.joint_operation.contracts import (
    OPERATION_JOINTS, seal_operation_plan,
)
from luggage_planning.motion.motion_frame_contracts import CANONICAL_JOINTS
from test_joint_operation_contracts import example_plan


def encoded():
    plan = seal_operation_plan(example_plan())
    return plan, operation_plan_to_message(plan, plan.identity, plan.plan_digest)


def test_complete_plan_survives_real_cdr_roundtrip():
    assert OPERATION_JOINTS == CANONICAL_JOINTS
    plan, message = encoded()
    wire = serialize_message(message)
    received = deserialize_message(wire, JointOperationPlanV2)
    assert operation_plan_from_message(received, plan.identity, plan.plan_digest) == plan
    assert len(wire) > 0


@pytest.mark.parametrize("mutation,code", [
    (lambda m: setattr(m, "observation_stamp_valid", False), "INVALID_STAMP"),
    (lambda m: setattr(m, "map_revision", 8), "OPERATION_IDENTITY_MISMATCH"),
    (lambda m: setattr(m, "barriers", []), "INVALID_OPERATION_BARRIERS"),
    (lambda m: setattr(m.legs[1].trajectory.trajectory.header, "frame_id", "world"),
     "JOINT_SPACE_LABEL_MISMATCH"),
    (lambda m: setattr(m.legs[1].trajectory.trajectory.points[-1], "positions", [.2]*5+[2.]),
     "JOINT_CHAIN_DISCONTINUITY"),
    (lambda m: setattr(m, "contract_version", 0), "CONTRACT_VERSION_MISMATCH"),
])
def test_generated_malformed_inputs_refused(mutation, code):
    plan, message = encoded()
    mutation(message)
    received = deserialize_message(serialize_message(message), JointOperationPlanV2)
    with pytest.raises(ContractError, match=code):
        operation_plan_from_message(received, plan.identity, plan.plan_digest)


def test_service_success_carries_exact_plan_and_failure_is_inactive():
    plan, message = encoded()
    response = PrepareJointOperationV2.Response()
    response.success, response.reason, response.plan = True, "PLAN_READY", message
    received = deserialize_message(serialize_message(response), PrepareJointOperationV2.Response)
    assert operation_plan_from_message(received.plan, plan.identity, plan.plan_digest) == plan
    failure = PrepareJointOperationV2.Response()
    failure.success, failure.reason = False, "SEARCH_INCOMPLETE"
    failure = deserialize_message(serialize_message(failure), PrepareJointOperationV2.Response)
    assert not failure.success and not failure.plan.legs
    with pytest.raises(ContractError, match="CONTRACT_VERSION_MISMATCH"):
        operation_plan_from_message(failure.plan, plan.identity, plan.plan_digest)
