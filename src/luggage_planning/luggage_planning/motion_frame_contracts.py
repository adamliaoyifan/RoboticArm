"""ROS-free v2 motion admission, compensation, and FK budget.

Producers emit these records. The executor and planner consume them.
Nothing here repairs an input or talks to MoveIt.
"""

from __future__ import division

from dataclasses import dataclass
import math

from luggage_description.frame_contracts import (
    VERSION,
    ContractError,
    finite_number,
    require_equal,
    require_horizontal,
    unit_quaternion,
    validate_pose,
)
from luggage_description.frame_diagnostics import at_boundary, validate_context
from luggage_description.frame_sequence_contracts import validate_joint_trajectory
from luggage_description.frame_transforms import quaternion_product, rotate_vector
from luggage_description.scene_tf_publisher import rpy_to_quaternion


MOTION_FRAME = "world"
TARGET_LINK = "suction_contact_frame"
MODEL_ROOT_OFFLINE = "elfin_base_link"
SEGMENT_TYPES = ("pose_target", "cartesian", "joint_path")
CANONICAL_JOINTS = (
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
)
POLICY_FLAGS = (
    "keep_tool_down", "keep_camera_down", "lock_wrist", "allow_ompl_fallback",
)
FK_POSITION_M = 0.002
FK_ROTATION_RAD = 0.02
SEGMENT_FIELDS = (
    "contract_version", "request_id", "scene_revision", "stamp_ns", "time_basis",
    "name", "segment_type", "target_link", "policy_valid", *POLICY_FLAGS,
    "required_cartesian_fraction", "target_pose", "waypoints", "joint_trajectory",
)
QUERY_OFFSET_FRAMES = ("container_link", "world", "elfin_base_link")

def assert_canonical_joints(executor_joints):
    """The executor's joint tuple is the only admitted order."""
    if tuple(executor_joints) != CANONICAL_JOINTS:
        raise ContractError(
            "INVALID_JOINT_NAMES", "executor_joints",
            list(CANONICAL_JOINTS), list(executor_joints))


@dataclass(frozen=True)
class ExecutionContext:
    """Boundary identity plus the link and planning frame the executor owns."""

    boundary: object
    target_link: str
    planning_frame: str


@dataclass(frozen=True)
class AdmittedSegment:
    segment_type: str
    name: str
    target_link: str
    request_id: str
    scene_revision: str
    stamp_ns: int
    time_basis: str
    keep_tool_down: bool
    keep_camera_down: bool
    lock_wrist: bool
    allow_ompl_fallback: bool
    required_cartesian_fraction: float
    target_pose: object
    waypoints: tuple
    joint_names: object
    joint_points: object


def _conjugate(quaternion):
    x, y, z, w = unit_quaternion(quaternion)
    return (-x, -y, -z, w)


def rotation_angle(left, right):
    """Shortest rotation angle. ``q`` and ``-q`` are the same rotation."""
    a = unit_quaternion(left)
    b = unit_quaternion(right)
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = min(1.0, max(0.0, dot))
    return 2.0 * math.acos(dot)


def position_error(left, right):
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def container_rotation(rpy):
    return rpy_to_quaternion(rpy)


def scene_transform(origin_xyz, rpy, scene_revision):
    """Static container_link -> world transform from a scene origin and RPY."""
    quaternion = container_rotation(rpy)
    return {
        "source_frame": "container_link",
        "target_frame": "world",
        "translation": [float(v) for v in origin_xyz],
        "quaternion_xyzw": list(quaternion),
        "scene_revision": str(scene_revision),
        "stamp_ns": None,
    }


def map_container_point(xyz, origin_xyz, quaternion_xyzw):
    """Full container-to-world point map. Horizontal yaw-only scenes match the legacy formula."""
    require_horizontal(quaternion_xyzw)
    rotated = rotate_vector(quaternion_xyzw, xyz)
    origin = [float(v) for v in origin_xyz]
    return [origin[i] + rotated[i] for i in range(3)]


def compose_orientation(container_quat, box_quat):
    """World orientation of a container-frame box. Identity box quat is not copied through."""
    require_horizontal(container_quat)
    return list(quaternion_product(
        unit_quaternion(container_quat), unit_quaternion(box_quat)))


def lift_vector(lift_m, container_quat):
    """``(0, 0, lift)`` expressed in world. A horizontal container adds it to world Z."""
    require_horizontal(container_quat)
    return list(rotate_vector(container_quat, (0.0, 0.0, float(lift_m))))


def framed_query_offset(offset, frame, container_quat, base_quat=None):
    """Return a container-frame offset. The default frame is passed through unchanged."""
    values = [finite_number(v, "query_offset") for v in offset]
    if len(values) != 3:
        raise ContractError(
            "INVALID_NUMERIC_PAYLOAD", "query_offset", "3 numbers", offset)
    name = "container_link" if frame in (None, "") else str(frame)
    if name not in QUERY_OFFSET_FRAMES:
        raise ContractError(
            "FRAME_CONTRACT_MISMATCH", "query_offset_frame",
            QUERY_OFFSET_FRAMES, name)
    if name == "container_link":
        return values
    if name == "world":
        return list(rotate_vector(_conjugate(container_quat), values))
    if base_quat is None:
        raise ContractError(
            "MISSING_IDENTITY", "base_quaternion", "unit quaternion", None)
    world = rotate_vector(base_quat, values)
    return list(rotate_vector(_conjugate(container_quat), world))


def fk_budget(verifier, joints, target_position, target_quaternion):
    """Return ``(ok, message)``. Threshold equality passes. Missing FK does not pass."""
    if verifier is None or not callable(verifier):
        return False, "FK_VERIFY_UNAVAILABLE"
    try:
        reported = verifier(joints)
        xyz, quat = reported[0], reported[1]
        if len(xyz) != 3 or len(quat) != 4:
            raise ValueError("shape")
        for value in list(xyz) + list(quat):
            finite_number(value, "fk")
    except ContractError:
        return False, "FK_VERIFY_UNAVAILABLE"
    except Exception:
        return False, "FK_VERIFY_UNAVAILABLE"
    pos = position_error(xyz, target_position)
    ang = rotation_angle(quat, target_quaternion)
    # Binary floats can sit a few ulps over a decimal threshold. Equality passes.
    if pos > FK_POSITION_M + 1e-12 or ang > FK_ROTATION_RAD + 1e-12:
        return False, "FK_VERIFY_BUDGET pos=%.6f rad=%.6f" % (pos, ang)
    return True, "FK_VERIFY_OK"


def _policy(record):
    if record.get("policy_valid") is not True:
        raise ContractError(
            "INVALID_NUMERIC_PAYLOAD", "policy_valid", True, record.get("policy_valid"))
    flags = {}
    for name in POLICY_FLAGS:
        value = record.get(name)
        if type(value) is not bool:
            raise ContractError("INVALID_NUMERIC_PAYLOAD", name, "bool", value)
        flags[name] = value
    fraction = finite_number(
        record.get("required_cartesian_fraction"), "required_cartesian_fraction")
    if fraction < 0.0 or fraction > 1.0:
        raise ContractError(
            "INVALID_NUMERIC_PAYLOAD", "required_cartesian_fraction",
            "[0,1]", fraction)
    flags["required_cartesian_fraction"] = fraction
    return flags


def _reorder_joints(names, points):
    if set(names) != set(CANONICAL_JOINTS):
        raise ContractError(
            "INVALID_JOINT_NAMES", "joint_names", list(CANONICAL_JOINTS), names)
    index = {name: pos for pos, name in enumerate(names)}
    order = [index[name] for name in CANONICAL_JOINTS]

    def take(values):
        if not values:
            return []
        return [values[i] for i in order]

    ordered = []
    for point in points:
        ordered.append({
            "positions": take(list(point["positions"])),
            "velocities": take(list(point.get("velocities") or [])),
            "accelerations": take(list(point.get("accelerations") or [])),
            "effort": take(list(point.get("effort") or [])),
            "time_from_start_ns": int(point["time_from_start_ns"]),
        })
    return ordered


def _empty_joint(payload):
    return payload in (None, {}) or (
        not payload.get("joint_names") and not payload.get("points")
        and not payload.get("contract_version"))


def _admit(record, context):
    if not isinstance(record, dict):
        raise ContractError("INVALID_RECORD", "record", "object", record)
    from luggage_description.ros_frame_bindings.identity import reject_extra_fields
    reject_extra_fields(record, SEGMENT_FIELDS)
    require_equal(record, "contract_version", VERSION, "CONTRACT_VERSION_MISMATCH")
    for field in ("request_id", "scene_revision", "name", "target_link"):
        value = record.get(field)
        if not isinstance(value, str) or not value.strip():
            raise ContractError("MISSING_IDENTITY", field, "nonempty string", value)
    require_equal(
        record, "scene_revision", context.boundary.scene_revision,
        "CALIBRATION_REVISION_MISMATCH")
    require_equal(
        record, "request_id", context.boundary.request_id,
        "REQUEST_IDENTITY_MISMATCH")
    stamp = record.get("stamp_ns")
    if type(stamp) is not int or stamp < 0:
        raise ContractError("INVALID_STAMP", "stamp_ns", "nonnegative integer", stamp)
    if record.get("time_basis") not in ("acquisition", "robot_state", "scene"):
        raise ContractError(
            "TIME_BASIS_MISMATCH", "time_basis",
            ("acquisition", "robot_state", "scene"), record.get("time_basis"))
    if context.planning_frame != MOTION_FRAME or context.boundary.frame_id != MOTION_FRAME:
        raise ContractError(
            "FRAME_CONTRACT_MISMATCH", "planning_frame", MOTION_FRAME,
            context.planning_frame)
    segment_type = record.get("segment_type")
    if segment_type not in SEGMENT_TYPES:
        raise ContractError(
            "UNKNOWN_SEGMENT_TYPE", "segment_type", SEGMENT_TYPES, segment_type)
    if record["target_link"] != context.target_link or context.target_link != TARGET_LINK:
        raise ContractError(
            "TARGET_LINK_MISMATCH", "target_link", TARGET_LINK, record["target_link"])
    policy = _policy(record)
    waypoints = record.get("waypoints") or []
    if not isinstance(waypoints, (list, tuple)):
        raise ContractError("INVALID_RECORD", "waypoints", "list", waypoints)
    joint = record.get("joint_trajectory")
    target = record.get("target_pose")
    if segment_type == "joint_path":
        if target not in (None, {}) or waypoints:
            raise ContractError(
                "INVALID_RECORD", "inactive_cartesian", "empty", target or waypoints)
        if _empty_joint(joint):
            raise ContractError(
                "MISSING_IDENTITY", "joint_trajectory", "timed trajectory", joint)
        require_equal(record, "time_basis", "robot_state", "TIME_BASIS_MISMATCH")
        if policy["required_cartesian_fraction"] != 0.0:
            raise ContractError(
                "INVALID_NUMERIC_PAYLOAD", "required_cartesian_fraction",
                0.0, policy["required_cartesian_fraction"])
        validate_joint_trajectory(joint, context.boundary.scene_revision)
        validate_context(joint, context.boundary, spatial=False)
        require_equal(joint, "request_id", record["request_id"], "REQUEST_IDENTITY_MISMATCH")
        points = _reorder_joints(list(joint["joint_names"]), list(joint["points"]))
        return AdmittedSegment(
            segment_type=segment_type, name=record["name"], target_link=TARGET_LINK,
            request_id=record["request_id"], scene_revision=record["scene_revision"],
            stamp_ns=stamp, time_basis="robot_state",
            joint_names=CANONICAL_JOINTS, joint_points=tuple(points),
            target_pose=None, waypoints=(), **policy)
    if not _empty_joint(joint):
        raise ContractError(
            "INVALID_RECORD", "inactive_joint", "empty", joint)
    if segment_type == "pose_target" and waypoints:
        raise ContractError("INVALID_RECORD", "inactive_waypoints", "empty", waypoints)
    if not isinstance(target, dict):
        raise ContractError("MISSING_IDENTITY", "target_pose", "motion pose", target)
    validate_pose(target, "motion", context.boundary.scene_revision)
    validate_context(target, context.boundary)
    for pose in (target, *waypoints):
        require_equal(pose, "request_id", record["request_id"], "WAYPOINT_IDENTITY_MISMATCH")
        require_equal(pose, "scene_revision", record["scene_revision"], "WAYPOINT_IDENTITY_MISMATCH")
        require_equal(pose, "frame_id", MOTION_FRAME, "WAYPOINT_IDENTITY_MISMATCH")
        require_equal(pose, "time_basis", record["time_basis"], "TIME_BASIS_MISMATCH")
    for pose in waypoints:
        validate_pose(pose, "motion", context.boundary.scene_revision)
        validate_context(pose, context.boundary)
    return AdmittedSegment(
        segment_type=segment_type, name=record["name"], target_link=TARGET_LINK,
        request_id=record["request_id"], scene_revision=record["scene_revision"],
        stamp_ns=stamp, time_basis=record["time_basis"],
        target_pose=target, waypoints=tuple(waypoints),
        joint_names=None, joint_points=None, **policy)


def admit_motion_segment(record, context):
    """Admit one segment. Failures are ``BoundaryContractError`` with a diagnostic."""
    payload = record if isinstance(record, dict) else {}
    return at_boundary(payload, context.boundary, lambda: _admit(record, context))
