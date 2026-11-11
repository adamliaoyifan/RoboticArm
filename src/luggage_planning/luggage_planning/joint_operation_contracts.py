"""ROS-free complete pick/carry/place/retreat plan binding.

This checks structure and identity, not collision freedom. Only the planner's
whole-chain certifier may seal a plan after checking all three load scenes.
The executor must additionally check certificate liveness, actual starting q,
and measured attachment at the barrier. Joint winding is preserved exactly.
"""
from dataclasses import asdict, dataclass, replace
import hashlib
import json

from luggage_description.frame_contracts import ContractError, finite_number, vector
from luggage_description.frame_sequence_contracts import (
    unsigned_integer, validate_joint_trajectory,
)
from luggage_planning.placement_prior_contracts import Transform
from st09_contracts.attachment_geometry import AttachmentGeometry

# Match the executor's six named joints without importing its ROS/model loaders.
OPERATION_JOINTS = ("elfin_joint1", "elfin_joint2", "elfin_joint3",
                    "elfin_joint4", "elfin_joint5", "elfin_joint6")


def require(condition, code, field, expected, actual):
    if not condition:
        raise ContractError(code, field, expected, actual)


@dataclass(frozen=True)
class OperationIdentity:
    request_id: str
    scene_revision: str
    snapshot_id: str
    geometry_hash: str
    model_sha256: str
    map_revision: int
    observation_id: str
    observation_stamp_ns: int
    robot_state_stamp_ns: int
    box_id: str
    generation: int
    contact_model_hash: str

    def __post_init__(self):
        for name in ("request_id", "scene_revision", "snapshot_id", "geometry_hash",
                     "model_sha256", "observation_id", "box_id", "contact_model_hash"):
            value = getattr(self, name)
            require(isinstance(value, str) and bool(value.strip()),
                    "MISSING_IDENTITY", name, "nonempty string", value)
        for name in ("map_revision", "observation_stamp_ns", "robot_state_stamp_ns",
                     "generation"):
            unsigned_integer(getattr(self, name), 64, name)


@dataclass(frozen=True)
class JointKnot:
    positions: tuple
    velocities: tuple
    accelerations: tuple
    effort: tuple
    time_from_start_ns: int

    def __post_init__(self):
        for name in ("positions", "velocities", "accelerations", "effort"):
            object.__setattr__(self, name, tuple(
                finite_number(value, name) for value in getattr(self, name)))


@dataclass(frozen=True)
class OperationLeg:
    name: str
    phase: str  # pick: box in world; carry: attached; retreat: placed in world
    trajectory_request_id: str
    trajectory_scene_revision: str
    robot_state_stamp_ns: int
    joint_names: tuple
    knots: tuple

    def __post_init__(self):
        object.__setattr__(self, "joint_names", tuple(self.joint_names))
        object.__setattr__(self, "knots", tuple(self.knots))

    def trajectory_record(self):
        return dict(contract_version=2, kind="joint_trajectory",
                    request_id=self.trajectory_request_id,
                    scene_revision=self.trajectory_scene_revision,
                    stamp_ns=self.robot_state_stamp_ns, time_basis="robot_state",
                    angle_unit="rad", joint_names=list(self.joint_names),
                    points=[asdict(knot) for knot in self.knots])


@dataclass(frozen=True)
class OperationBarrier:
    before_leg_index: int
    event: str  # attachment_confirmed | release_confirmed


@dataclass(frozen=True)
class JointOperationPlan:
    identity: OperationIdentity
    candidate_id: str
    certificate_id: str
    # World-frame directed poses, metres and xyzw. No modulo-pi reduction.
    world_from_pick_box: Transform
    world_from_pick_tool: Transform
    world_from_place_box: Transform
    world_from_place_tool: Transform
    world_from_settled_box: Transform
    predicted_attachment: AttachmentGeometry
    start_joints: tuple
    legs: tuple
    barriers: tuple
    plan_digest: str

    def __post_init__(self):
        object.__setattr__(self, "start_joints", vector(self.start_joints, 6, "start_joints"))
        for name in ("legs", "barriers"):
            object.__setattr__(self, name, tuple(getattr(self, name)))


def _same_pose(left, right):
    # Numerical equality only; task/calibration tolerances belong to certifiers.
    return (max(abs(a-b) for a, b in zip(left.translation, right.translation)) <= 1e-8
            and min(max(abs(a-s*b) for a, b in zip(
                left.quaternion_xyzw, right.quaternion_xyzw)) for s in (-1, 1)) <= 1e-8)


def _validate_structure(plan):
    require(type(plan) is JointOperationPlan, "INVALID_OPERATION_PLAN", "plan",
            "JointOperationPlan", type(plan).__name__)
    require(type(plan.identity) is OperationIdentity, "INVALID_OPERATION_PLAN",
            "identity", "OperationIdentity", type(plan.identity).__name__)
    identity = plan.identity
    # Recheck types at trust boundaries as well as at normal construction.
    identity.__post_init__()
    for name in ("candidate_id", "certificate_id"):
        value = getattr(plan, name)
        require(isinstance(value, str) and bool(value.strip()),
                "MISSING_IDENTITY", name, "nonempty string", value)
    require(type(plan.predicted_attachment) is AttachmentGeometry,
            "INVALID_OPERATION_PLAN", "predicted_attachment", "AttachmentGeometry",
            type(plan.predicted_attachment).__name__)
    attachment = plan.predicted_attachment
    attachment.__post_init__()
    tool_box = Transform(attachment.t_tool_box_translation,
                         attachment.t_tool_box_quaternion_xyzw)
    for end in ("pick", "place"):
        box = getattr(plan, "world_from_" + end + "_box")
        tool = getattr(plan, "world_from_" + end + "_tool")
        require(type(box) is Transform and type(tool) is Transform,
                "INVALID_OPERATION_PLAN", end + "_pose", "Transform", (box, tool))
        require(_same_pose(tool.compose(tool_box), box), "GRASP_BINDING_MISMATCH",
                end + "_pose", "T_world_tool * T_tool_box == T_world_box", (tool, box))
    settled = plan.world_from_settled_box
    require(type(settled) is Transform, "INVALID_OPERATION_PLAN", "settled_pose",
            "explicit world-frame nominal box pose", type(settled).__name__)
    require(_same_pose(settled, Transform(settled.translation,
                                        plan.world_from_place_box.quaternion_xyzw)),
            "SETTLED_ORIENTATION_MISMATCH", "settled_pose",
            "same directed box orientation as release", settled)
    previous_q = vector(plan.start_joints, 6, "start_joints")
    require(bool(plan.legs), "INCOMPLETE_OPERATION_PLAN", "legs", "complete chain", ())
    phases = []
    names = set()
    for index, leg in enumerate(plan.legs):
        require(type(leg) is OperationLeg, "INVALID_OPERATION_PLAN", "leg",
                "OperationLeg", type(leg).__name__)
        require(isinstance(leg.name, str) and bool(leg.name.strip()) and leg.name not in names,
                "INVALID_OPERATION_PLAN", "leg.name", "unique nonempty name", leg.name)
        names.add(leg.name)
        require(leg.phase in ("pick", "carry", "retreat"), "INVALID_OPERATION_PHASE",
                "leg.phase", "pick/carry/retreat", leg.phase)
        phases.append(leg.phase)
        require(leg.joint_names == OPERATION_JOINTS, "INVALID_JOINT_NAMES",
                "leg.joint_names", OPERATION_JOINTS, leg.joint_names)
        require(all(type(knot) is JointKnot for knot in leg.knots),
                "INVALID_OPERATION_PLAN", "knots", "JointKnot tuple", leg.knots)
        validate_joint_trajectory(leg.trajectory_record(), identity.scene_revision)
        require(leg.trajectory_request_id == identity.request_id,
                "REQUEST_IDENTITY_MISMATCH", "leg.request_id", identity.request_id,
                leg.trajectory_request_id)
        require(leg.robot_state_stamp_ns == identity.robot_state_stamp_ns,
                "INVALID_OPERATION_PLAN", "leg.robot_state_stamp_ns",
                identity.robot_state_stamp_ns, leg.robot_state_stamp_ns)
        require(len(leg.knots) >= 2 and leg.knots[0].time_from_start_ns == 0,
                "INVALID_OPERATION_PLAN", "leg.time", "at least two knots starting at zero",
                tuple(k.time_from_start_ns for k in leg.knots))
        require(max(abs(a-b) for a, b in zip(previous_q, leg.knots[0].positions)) <= 1e-8,
                "JOINT_CHAIN_DISCONTINUITY", "leg[%d]" % index, previous_q,
                leg.knots[0].positions)
        previous_q = leg.knots[-1].positions
    # Exact transitions; an absent phase or phase regression cannot be PLAN_READY.
    compressed = tuple(p for i, p in enumerate(phases) if i == 0 or p != phases[i-1])
    require(compressed == ("pick", "carry", "retreat"), "INCOMPLETE_OPERATION_PLAN",
            "phases", ("pick", "carry", "retreat"), compressed)
    expected = (OperationBarrier(phases.index("carry"), "attachment_confirmed"),
                OperationBarrier(phases.index("retreat"), "release_confirmed"))
    require(plan.barriers == expected, "INVALID_OPERATION_BARRIERS", "barriers",
            expected, plan.barriers)
    for barrier in plan.barriers:
        unsigned_integer(barrier.before_leg_index, 32, "before_leg_index")
        before = plan.legs[barrier.before_leg_index-1].knots[-1]
        after = plan.legs[barrier.before_leg_index].knots[0]
        for knot in (before, after):
            require(len(knot.velocities) == 6 and all(v == 0 for v in knot.velocities),
                    "NONSTATIONARY_OPERATION_BARRIER", "velocities",
                    "explicit zero velocity at event barrier", knot.velocities)


def operation_digest(plan):
    payload = asdict(plan)
    payload.pop("plan_digest")
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def seal_operation_plan(plan):
    """Planner-only after whole-chain certification; does not issue a certificate."""
    _validate_structure(plan)
    return replace(plan, plan_digest=operation_digest(plan))


def admit_operation_plan(plan, expected_identity, expected_digest):
    """Bind to independently retained request/scene and approved plan digest."""
    _validate_structure(plan)
    require(plan.identity == expected_identity, "OPERATION_IDENTITY_MISMATCH",
            "identity", expected_identity, plan.identity)
    actual = operation_digest(plan)
    require(bool(expected_digest) and plan.plan_digest == expected_digest == actual,
            "OPERATION_DIGEST_MISMATCH", "plan_digest", expected_digest, actual)
    return plan
