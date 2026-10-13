"""ROS-free path and joint-space invariants for the v2 binding adapters."""
from luggage_description.frame_contracts import (
    ContractError, finite_number, require_equal, validate_envelope, validate_pose,
)


def unsigned_integer(value, bits, field):
    if type(value) is not int or not 0 <= value < 2**bits:
        raise ContractError("INTEGER_OUT_OF_RANGE", field, f"uint{bits}", value)
    return value


def validate_waypoints(record, profile, scene_revision, geometry_hash, map_revision):
    validate_envelope(record, scene_revision)
    require_equal(record, "kind", "waypoints", "RECORD_KIND_MISMATCH")
    if not isinstance(geometry_hash, str) or not geometry_hash.strip():
        raise ContractError("MISSING_IDENTITY", "expected_geometry_hash", "nonempty string", geometry_hash)
    unsigned_integer(map_revision, 64, "expected_map_revision")
    require_equal(record, "geometry_hash", geometry_hash, "GEOMETRY_IDENTITY_MISMATCH")
    require_equal(record, "map_revision", map_revision, "MAP_REVISION_MISMATCH")
    poses = record.get("waypoints")
    if not isinstance(poses, list) or not poses:
        raise ContractError("INVALID_WAYPOINTS", "waypoints", "nonempty list", poses)
    for pose in poses:
        validate_pose(pose, profile, scene_revision)
        for field in ("frame_id", "request_id", "scene_revision"):
            require_equal(pose, field, record[field], "WAYPOINT_IDENTITY_MISMATCH")


def validate_joint_trajectory(record, scene_revision):
    if not isinstance(record, dict):
        raise ContractError("INVALID_RECORD", "record", "object", record)
    require_equal(record, "contract_version", 2, "CONTRACT_VERSION_MISMATCH")
    require_equal(record, "kind", "joint_trajectory", "RECORD_KIND_MISMATCH")
    for field in ("request_id", "scene_revision"):
        if not isinstance(record.get(field), str) or not record[field].strip():
            raise ContractError("MISSING_IDENTITY", field, "nonempty string", record.get(field))
    require_equal(record, "scene_revision", scene_revision, "CALIBRATION_REVISION_MISMATCH")
    require_equal(record, "angle_unit", "rad", "UNIT_MISMATCH")
    require_equal(record, "time_basis", "robot_state", "TIME_BASIS_MISMATCH")
    stamp = record.get("stamp_ns")
    if type(stamp) is not int or stamp < 0:
        raise ContractError("INVALID_STAMP", "stamp_ns", "nonnegative nanoseconds", stamp)
    unsigned_integer(stamp, 64, "stamp_ns")
    for field in ("frame_id", "reference", "length_unit"):
        if field in record:
            raise ContractError("JOINT_SPACE_LABEL_MISMATCH", field, "absent in joint space", record[field])
    names = record.get("joint_names")
    if (not isinstance(names, list) or not names
            or any(not isinstance(n, str) or not n.strip() for n in names)
            or len(set(names)) != len(names)):
        raise ContractError("INVALID_JOINT_NAMES", "joint_names", "nonempty unique names", names)
    points = record.get("points")
    if not isinstance(points, list) or not points:
        raise ContractError("INVALID_JOINT_TRAJECTORY", "points", "nonempty list", points)
    previous = -1
    for point in points:
        if not isinstance(point, dict):
            raise ContractError("INVALID_JOINT_TRAJECTORY", "point", "object", point)
        for field in ("positions", "velocities", "accelerations", "effort"):
            values = point.get(field)
            lengths = (len(names),) if field == "positions" else (0, len(names))
            if not isinstance(values, (list, tuple)) or len(values) not in lengths:
                raise ContractError("JOINT_VECTOR_SIZE_MISMATCH", field, lengths, values)
            for value in values:
                finite_number(value, field)
        time = point.get("time_from_start_ns")
        if type(time) is not int or not previous < time < 2**31 * 10**9:
            raise ContractError("INVALID_TRAJECTORY_TIME", "time_from_start_ns", "strictly increasing ROS duration", time)
        previous = time
