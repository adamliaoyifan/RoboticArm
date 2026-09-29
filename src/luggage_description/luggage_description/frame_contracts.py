"""Strict, ROS-free v2 boundary checks. Adoption by live nodes is separate.

These functions validate declared semantics; they cannot detect plausible
coordinates dishonestly labelled with the expected frame. Independent geometry
oracles and boundary traces remain necessary. No function repairs an input.
"""

import math


VERSION = 2
QUATERNION_NORM_TOLERANCE = 1e-6
POSE_PROFILES = {
    "detection": ("world", "box_center", "box_axis_pi", ("acquisition",)),
    "contact": ("world", "surface_contact", "surface_normal", ("acquisition",)),
    "slot": ("container_link", "box_center", "box_axis_pi", ("scene",)),
    "map_tcp": ("container_link", "suction_contact_frame", "tool_quaternion",
                ("scene", "robot_state", "acquisition")),
    "motion": ("world", "suction_contact_frame", "tool_quaternion",
               ("scene", "robot_state", "acquisition")),
    "base_tool": ("elfin_base_link", "suction_contact_frame", "tool_quaternion",
                  ("scene", "robot_state", "acquisition")),
}


class ContractError(ValueError):
    """Machine-readable failure; the caller adds producer/consumer identity."""

    def __init__(self, code, field, expected, received):
        self.code = code
        self.field = field
        self.expected = expected
        self.received = received
        super().__init__(f"{code}: {field}: expected {expected!r}, got {received!r}")

    def as_dict(self):
        return {"code": self.code, "field": self.field,
                "expected": self.expected, "received": self.received}


def require_equal(record, field, expected, code):
    actual = record.get(field)
    if type(actual) is not type(expected) or actual != expected:
        raise ContractError(code, field, expected, actual)


def finite_number(value, field):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ContractError("INVALID_NUMERIC_PAYLOAD", field, "finite number", value)
    return float(value)


def vector(value, size, field):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ContractError("INVALID_NUMERIC_PAYLOAD", field, f"{size} numbers", value)
    return tuple(finite_number(v, f"{field}[{i}]") for i, v in enumerate(value))


def unit_quaternion(value, field="quaternion_xyzw"):
    q = vector(value, 4, field)
    norm = math.sqrt(sum(v * v for v in q))
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise ContractError("INVALID_QUATERNION", field, "unit quaternion", value)
    return q


def validate_envelope(record, scene_revision):
    if not isinstance(record, dict):
        raise ContractError("INVALID_RECORD", "record", "object", record)
    require_equal(record, "contract_version", VERSION, "CONTRACT_VERSION_MISMATCH")
    for field in ("frame_id", "request_id", "scene_revision"):
        if not isinstance(record.get(field), str) or not record[field].strip():
            raise ContractError("MISSING_IDENTITY", field, "nonempty string", record.get(field))
    if not isinstance(scene_revision, str) or not scene_revision.strip():
        raise ContractError("MISSING_IDENTITY", "expected_scene_revision", "nonempty string", scene_revision)
    require_equal(record, "scene_revision", scene_revision, "CALIBRATION_REVISION_MISMATCH")
    require_equal(record, "length_unit", "m", "UNIT_MISMATCH")
    stamp = record.get("stamp_ns")
    if type(stamp) is not int or stamp < 0:
        raise ContractError("INVALID_STAMP", "stamp_ns", "nonnegative integer nanoseconds", stamp)
    if record.get("time_basis") not in ("acquisition", "robot_state", "scene"):
        raise ContractError("TIME_BASIS_MISMATCH", "time_basis", "acquisition/robot_state/scene", record.get("time_basis"))


def validate_pose(record, profile, scene_revision):
    """Validate one specified boundary profile, without guessing or conversion."""
    if profile not in POSE_PROFILES:
        raise ContractError("UNKNOWN_PROFILE", "profile", sorted(POSE_PROFILES), profile)
    validate_envelope(record, scene_revision)
    frame, reference, orientation, time_bases = POSE_PROFILES[profile]
    require_equal(record, "kind", "pose", "RECORD_KIND_MISMATCH")
    require_equal(record, "frame_id", frame, "FRAME_CONTRACT_MISMATCH")
    require_equal(record, "reference", reference, "POSE_REFERENCE_MISMATCH")
    require_equal(record, "orientation_semantics", orientation, "ORIENTATION_SEMANTICS_MISMATCH")
    if record["time_basis"] not in time_bases:
        raise ContractError("TIME_BASIS_MISMATCH", "time_basis", time_bases, record["time_basis"])
    vector(record.get("position"), 3, "position")
    unit_quaternion(record.get("quaternion_xyzw"))


def validate_surface(record, scene_revision, geometry_hash, map_revision):
    """V2 surfaces store container-origin z, with null for unobserved tops.

    Null is not a claim that the floor is absent. Floor prior is a separate
    policy; it must not turn unknown occupancy into a measured empty column.
    """
    validate_envelope(record, scene_revision)
    require_equal(record, "kind", "surface", "RECORD_KIND_MISMATCH")
    require_equal(record, "frame_id", "container_link", "FRAME_CONTRACT_MISMATCH")
    require_equal(record, "height_datum", "container_origin", "HEIGHT_DATUM_MISMATCH")
    if type(record.get("map_revision")) is not int or record["map_revision"] < 0:
        raise ContractError("MAP_REVISION_MISMATCH", "map_revision", "nonnegative integer", record.get("map_revision"))
    if not isinstance(record.get("geometry_hash"), str) or not record["geometry_hash"]:
        raise ContractError("MISSING_IDENTITY", "geometry_hash", "nonempty string", record.get("geometry_hash"))
    if not isinstance(geometry_hash, str) or not geometry_hash:
        raise ContractError("MISSING_IDENTITY", "expected_geometry_hash", "nonempty string", geometry_hash)
    if type(map_revision) is not int or map_revision < 0:
        raise ContractError("MAP_REVISION_MISMATCH", "expected_map_revision", "nonnegative integer", map_revision)
    require_equal(record, "geometry_hash", geometry_hash, "GEOMETRY_IDENTITY_MISMATCH")
    require_equal(record, "map_revision", map_revision, "MAP_REVISION_MISMATCH")
    vector(record.get("origin_xy_container"), 2, "origin_xy_container")
    resolution = finite_number(record.get("resolution_m"), "resolution_m")
    if resolution <= 0:
        raise ContractError("INVALID_GRID", "resolution_m", "positive metres", resolution)
    floor = finite_number(record.get("floor_z_container"), "floor_z_container")
    ceiling = finite_number(record.get("ceiling_z_container"), "ceiling_z_container")
    if ceiling <= floor:
        raise ContractError("INVALID_HEIGHT_BOUNDS", "ceiling_z_container", "> floor", ceiling)
    tops, states = record.get("surface_z_container"), record.get("state")
    confidence = record.get("confidence")
    if not isinstance(tops, list) or not tops or not isinstance(tops[0], list) or not tops[0]:
        raise ContractError("INVALID_GRID", "surface_z_container", "nonempty rectangular grid", tops)
    width = len(tops[0])
    for field, grid in (("surface_z_container", tops), ("state", states), ("confidence", confidence)):
        if (not isinstance(grid, list) or len(grid) != len(tops)
                or any(not isinstance(row, list) or len(row) != width for row in grid)):
            raise ContractError("INVALID_GRID", field, f"{len(tops)}x{width}", grid)
    relative = record.get("height_above_floor")
    if "height_above_floor" in record:
        if (not isinstance(relative, list) or len(relative) != len(tops)
                or any(not isinstance(row, list) or len(row) != width for row in relative)):
            raise ContractError("INVALID_GRID", "height_above_floor", f"{len(tops)}x{width}", relative)
    for i, row in enumerate(tops):
        for j, top in enumerate(row):
            state, source = states[i][j], confidence[i][j]
            field = f"surface_z_container[{i}][{j}]"
            if state not in ("unknown", "free", "occupied"):
                raise ContractError("INVALID_GRID_STATE", field, "unknown/free/occupied", state)
            if state == "unknown":
                if top is not None or source != "unknown":
                    raise ContractError("UNKNOWN_SURFACE_HAS_VALUE", field, "null/unknown", [top, source])
            else:
                z = finite_number(top, field)
                if source not in ("sensor", "verified_geometry"):
                    raise ContractError("UNVERIFIED_SURFACE", field, "sensor/verified_geometry", source)
                if z < floor or z > ceiling or (state == "free" and abs(z - floor) > 1e-9):
                    raise ContractError("SURFACE_HEIGHT_OUT_OF_RANGE", field, [floor, ceiling], z)
            if relative is not None:
                rel = relative[i][j]
                if top is None:
                    if rel is not None:
                        raise ContractError("HEIGHT_DATUM_MISMATCH", field, None, rel)
                elif abs(finite_number(rel, "height_above_floor") - (top - floor)) > 1e-9:
                    raise ContractError("HEIGHT_DATUM_MISMATCH", field, top - floor, rel)


def validate_atlas(record, expected_identity):
    """Concrete tool-pose atlas only; π-aggregated reachability is not implied."""
    if not isinstance(record, dict):
        raise ContractError("INVALID_RECORD", "atlas", "object", record)
    required = {
        "contract_version": VERSION, "position_frame": "container_link",
        "orientation_frame": "container_link", "reference": "suction_contact_frame",
        "orientation_semantics": "concrete_tool_pose", "angle_unit": "rad",
        "length_unit": "m", "height_datum": "container_origin",
        "tool_down_reference": "container_negative_z",
    }
    for key, value in required.items():
        require_equal(record, key, value, "ATLAS_SEMANTICS_MISMATCH")
    period = finite_number(record.get("yaw_period_rad"), "yaw_period_rad")
    if abs(period - 2.0 * math.pi) > 1e-9:
        raise ContractError("ATLAS_SEMANTICS_MISMATCH", "yaw_period_rad", 2.0 * math.pi, period)
    for key in ("scene_revision", "geometry_hash", "urdf_hash", "payload_hash"):
        expected = expected_identity.get(key)
        if not isinstance(expected, str) or not expected:
            raise ContractError("MISSING_IDENTITY", key, "nonempty expected identity", expected)
        require_equal(record, key, expected, "ATLAS_IDENTITY_MISMATCH")
    bins = record.get("yaw_bins_rad")
    if not isinstance(bins, list) or not bins:
        raise ContractError("ATLAS_SEMANTICS_MISMATCH", "yaw_bins_rad", "nonempty bins", bins)
    values = [finite_number(v, "yaw_bins_rad") for v in bins]
    if any(v < 0 or v >= period for v in values) or len(set(values)) != len(values):
        raise ContractError("ATLAS_SEMANTICS_MISMATCH", "yaw_bins_rad", "unique bins in [0,2pi)", bins)


def require_horizontal(quaternion_xyzw, tolerance=1e-9):
    """2.5D container scope gate; full SE(3) utilities have no such restriction."""
    x, y, z, w = unit_quaternion(quaternion_xyzw)
    up = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x*x + y*y))
    if max(abs(up[0]), abs(up[1]), abs(up[2] - 1.0)) > tolerance:
        raise ContractError("UNSUPPORTED_CONTAINER_TILT", "container_up", [0, 0, 1], up)
