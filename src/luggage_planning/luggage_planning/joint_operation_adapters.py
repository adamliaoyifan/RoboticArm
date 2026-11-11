"""Node-layer generated ROS bindings for the complete operation contract."""
from dataclasses import asdict

from geometry_msgs.msg import Pose
from luggage_msgs.msg import (
    JointOperationBarrierV2, JointOperationLegV2, JointOperationPlanV2,
)
from luggage_description.frame_contracts import ContractError
from luggage_description.frame_diagnostics import BoundaryContext
from luggage_description.ros_frame_bindings.identity import require_message
from luggage_description.ros_frame_bindings.joints import (
    joints_from_message, joints_to_message,
)
from luggage_planning.attachment_geometry_transport import (
    geometry_from_message, geometry_to_message,
)
from luggage_planning.joint_operation_contracts import (
    JointKnot, JointOperationPlan, OperationBarrier, OperationIdentity,
    OperationLeg, admit_operation_plan,
)
from luggage_planning.placement_prior_contracts import Transform


def _joint_context(identity):
    return BoundaryContext("joint_operation", "joint_planner", "joint_executor",
                           identity.request_id, identity.scene_revision,
                           "", "", "robot_state")


def _pose_to_message(transform):
    message = Pose()
    for name, value in zip(("x", "y", "z"), transform.translation):
        setattr(message.position, name, value)
    for name, value in zip(("x", "y", "z", "w"), transform.quaternion_xyzw):
        setattr(message.orientation, name, value)
    return message


def _transform_from_message(message):
    return Transform(tuple(getattr(message.position, name) for name in ("x", "y", "z")),
                     tuple(getattr(message.orientation, name) for name in ("x", "y", "z", "w")))


def operation_plan_to_message(plan, expected_identity, expected_digest):
    plan = admit_operation_plan(plan, expected_identity, expected_digest)
    message = JointOperationPlanV2()
    message.contract_version = 2
    for name, value in asdict(plan.identity).items():
        setattr(message, name, value)
    message.observation_stamp_valid = True
    message.robot_state_stamp_valid = True
    for name in ("candidate_id", "certificate_id", "plan_digest"):
        setattr(message, name, getattr(plan, name))
    for name in ("world_from_pick_box", "world_from_pick_tool",
                 "world_from_place_box", "world_from_place_tool", "world_from_settled_box"):
        setattr(message, name, _pose_to_message(getattr(plan, name)))
    message.predicted_attachment = geometry_to_message(plan.predicted_attachment)
    message.start_joints = list(plan.start_joints)
    legs = []
    for leg in plan.legs:
        output = JointOperationLegV2()
        output.name, output.phase = leg.name, leg.phase
        output.trajectory = joints_to_message(leg.trajectory_record(),
                                              _joint_context(expected_identity))
        legs.append(output)
    message.legs = legs
    barriers = []
    for barrier in plan.barriers:
        output = JointOperationBarrierV2()
        output.before_leg_index = barrier.before_leg_index
        output.event = barrier.event
        barriers.append(output)
    message.barriers = barriers
    return message


def operation_plan_from_message(message, expected_identity, expected_digest):
    """Decode against consumer-owned identity/digest, never echo as expectations."""
    require_message(message, JointOperationPlanV2)
    if message.contract_version != 2:
        raise ContractError("CONTRACT_VERSION_MISMATCH", "contract_version", 2,
                            message.contract_version)
    if not message.observation_stamp_valid or not message.robot_state_stamp_valid:
        raise ContractError("INVALID_STAMP", "operation_stamps", "explicit valid stamps",
                            (message.observation_stamp_valid, message.robot_state_stamp_valid))
    identity = OperationIdentity(**{name: getattr(message, name)
                                    for name in OperationIdentity.__dataclass_fields__})
    legs = []
    for leg in message.legs:
        record = joints_from_message(leg.trajectory, _joint_context(expected_identity))
        legs.append(OperationLeg(leg.name, leg.phase, record["request_id"],
                                 record["scene_revision"], record["stamp_ns"],
                                 tuple(record["joint_names"]),
                                 tuple(JointKnot(**point) for point in record["points"])))
    plan = JointOperationPlan(
        identity, message.candidate_id, message.certificate_id,
        *(_transform_from_message(getattr(message, name)) for name in (
            "world_from_pick_box", "world_from_pick_tool",
            "world_from_place_box", "world_from_place_tool", "world_from_settled_box")),
        geometry_from_message(message.predicted_attachment),
        tuple(float(value) for value in message.start_joints),
        tuple(legs), tuple(OperationBarrier(b.before_leg_index, b.event)
                           for b in message.barriers), message.plan_digest)
    return admit_operation_plan(plan, expected_identity, expected_digest)
