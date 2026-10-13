"""Complete pose transforms for explicit v2 boundary records, without ROS.

Transforms map source coordinates into target coordinates. Poses carry xyzw
quaternions; vectors rotate without translation. Inputs are never mutated.
"""

from copy import deepcopy

from luggage_description.frame_contracts import (
    ContractError, require_equal, unit_quaternion, validate_envelope, vector,
)


def quaternion_product(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def rotate_vector(q, xyz):
    q = unit_quaternion(q)
    xyz = vector(xyz, 3, "vector")
    rotated = quaternion_product(quaternion_product(q, (*xyz, 0.0)),
                                 (-q[0], -q[1], -q[2], q[3]))
    return rotated[:3]


def transform_pose(record, transform):
    """Apply T_target_source; dynamic transforms require the exact record stamp.

    stamp_ns=null explicitly means a static transform bound to scene_revision.
    This function changes frame only, never the represented physical point.
    """
    if not isinstance(transform, dict):
        raise ContractError("INVALID_RECORD", "transform", "object", transform)
    validate_envelope(record, transform.get("scene_revision"))
    require_equal(record, "kind", "pose", "RECORD_KIND_MISMATCH")
    source = transform.get("source_frame")
    target = transform.get("target_frame")
    if not isinstance(source, str) or not source.strip() or not isinstance(target, str) or not target.strip():
        raise ContractError("MISSING_IDENTITY", "transform_frames", "explicit source/target", [source, target])
    require_equal(record, "frame_id", source, "FRAME_CONTRACT_MISMATCH")
    if "stamp_ns" not in transform:
        raise ContractError("INVALID_STAMP", "transform.stamp_ns", "integer or explicit null for static", None)
    stamp = transform["stamp_ns"]
    if stamp is not None and (type(stamp) is not int or stamp != record["stamp_ns"]):
        raise ContractError("TRANSFORM_TIME_MISMATCH", "transform.stamp_ns", record["stamp_ns"], stamp)
    position = vector(record.get("position"), 3, "position")
    orientation = unit_quaternion(record.get("quaternion_xyzw"))
    translation = vector(transform.get("translation"), 3, "translation")
    rotation = unit_quaternion(transform.get("quaternion_xyzw"), "transform.quaternion_xyzw")
    rotated = rotate_vector(rotation, position)
    out = deepcopy(record)
    out["frame_id"] = target
    out["position"] = [translation[i] + rotated[i] for i in range(3)]
    out["quaternion_xyzw"] = list(quaternion_product(rotation, orientation))
    return out


def box_to_tool_target(box, box_from_tool):
    """T_frame_tool = T_frame_box * T_box_tool, including off-centre contact.

    The caller selects a directed box representative/grasp branch beforehand.
    This operation does not fold tool yaw modulo π or select an IK branch.
    """
    if not isinstance(box_from_tool, dict):
        raise ContractError("INVALID_RECORD", "grasp_transform", "object", box_from_tool)
    validate_envelope(box, box_from_tool.get("scene_revision"))
    require_equal(box, "kind", "pose", "RECORD_KIND_MISMATCH")
    require_equal(box, "reference", "box_center", "POSE_REFERENCE_MISMATCH")
    require_equal(box, "orientation_semantics", "box_axis_pi", "ORIENTATION_SEMANTICS_MISMATCH")
    require_equal(box_from_tool, "source_frame", "suction_contact_frame", "POSE_REFERENCE_MISMATCH")
    require_equal(box_from_tool, "target_frame", "box", "POSE_REFERENCE_MISMATCH")
    position = vector(box.get("position"), 3, "position")
    orientation = unit_quaternion(box.get("quaternion_xyzw"))
    offset = rotate_vector(orientation, box_from_tool.get("translation"))
    tool_orientation = unit_quaternion(box_from_tool.get("quaternion_xyzw"))
    out = deepcopy(box)
    out["reference"] = "suction_contact_frame"
    out["orientation_semantics"] = "tool_quaternion"
    out["position"] = [position[i] + offset[i] for i in range(3)]
    out["quaternion_xyzw"] = list(quaternion_product(orientation, tool_orientation))
    return out


def rotate_covariance3(quaternion_xyzw, covariance):
    """Rotate a finite symmetric 3x3 covariance (row-major) as R C R^T.

    Translation never applies to vectors/covariances. This transports a
    positional/vector covariance, not a 6D pose covariance or uncertainty model.
    The caller owns the statistical validity of the supplied covariance.
    """
    values = vector(covariance, 9, "covariance")
    if any(abs(values[3*i+j] - values[3*j+i]) > 1e-9 for i in range(3) for j in range(3)):
        raise ContractError("INVALID_COVARIANCE", "covariance", "symmetric 3x3", covariance)
    columns = [rotate_vector(quaternion_xyzw, axis)
               for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    rotation = [[columns[j][i] for j in range(3)] for i in range(3)]
    return [sum(rotation[i][a] * values[3*a+b] * rotation[j][b]
                for a in range(3) for b in range(3))
            for i in range(3) for j in range(3)]
