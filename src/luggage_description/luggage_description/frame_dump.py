"""Explicit, lossless frame dump v2. Measured input and GT never share a record.

Legacy surface adapter ``surface_floor_relative_v1`` only accepts an explicitly
labelled container_link grid whose heights are relative to the supplied floor.
It does not infer a datum from numeric magnitudes or recognize arbitrary dumps.
"""
from copy import deepcopy
import json

from luggage_description.frame_contracts import (
    ContractError, finite_number, validate_pose, validate_surface, vector,
)
from luggage_description.frame_transforms import transform_pose

SCHEMA = "luggage.frame_dump/2"


def fail(code, field, expected, actual):
    raise ContractError(code, field, expected, actual)


def validate_dump(dump, *, scene_revision, geometry_hash, request_id, map_revision):
    if not isinstance(dump, dict) or dump.get("schema") != SCHEMA:
        fail("UNKNOWN_DUMP_SCHEMA", "schema", SCHEMA, dump.get("schema") if isinstance(dump, dict) else None)
    required = {"schema", "contract_version", "record_kind", "request_id", "scene_revision",
                "geometry_hash", "map_revision", "pose_profile", "pose", "body_dimensions_m",
                "surface", "transform_snapshot", "provenance"}
    if set(dump) != required:
        fail("DUMP_FIELD_MISMATCH", "fields", sorted(required), sorted(dump))
    expected = dict(contract_version=2, scene_revision=scene_revision,
                    geometry_hash=geometry_hash, request_id=request_id, map_revision=map_revision)
    for key, value in expected.items():
        if value is None or value == "":
            fail("MISSING_IDENTITY", key, "consumer identity", value)
        if type(dump[key]) is not type(value) or dump[key] != value:
            fail("DUMP_IDENTITY_MISMATCH", key, value, dump[key])
    if dump["record_kind"] not in ("measured", "gt_reference"):
        fail("DUMP_RECORD_KIND", "record_kind", "measured/gt_reference", dump["record_kind"])
    pose = dump["pose"]
    validate_pose(pose, dump["pose_profile"], scene_revision)
    if pose["request_id"] != request_id:
        fail("DUMP_IDENTITY_MISMATCH", "pose.request_id", request_id, pose["request_id"])
    size = vector(dump["body_dimensions_m"], 3, "body_dimensions_m")
    if any(d <= 0 for d in size):
        fail("INVALID_DIMENSIONS", "body_dimensions_m", "positive body X/Y/Z", size)
    if dump["surface"] is not None:
        validate_surface(dump["surface"], scene_revision, geometry_hash, map_revision)
        if dump["surface"]["request_id"] != request_id:
            fail("DUMP_IDENTITY_MISMATCH", "surface.request_id", request_id, dump["surface"]["request_id"])
    transform = dump["transform_snapshot"]
    transformed = transform_pose(pose, transform)
    if transformed["frame_id"] != "world":
        fail("FRAME_CONTRACT_MISMATCH", "transform.target_frame", "world", transformed["frame_id"])
    if not isinstance(dump["provenance"], dict) or not dump["provenance"]:
        fail("MISSING_PROVENANCE", "provenance", "nonempty mapping", dump["provenance"])
    return deepcopy(dump)


def read_dump(path, **expected):
    with open(path, encoding="utf-8") as stream:
        return validate_dump(json.load(stream), **expected)


def write_dump(path, dump, **expected):
    validated = validate_dump(dump, **expected)
    # Exclusive create: preserved failure input must never be overwritten.
    with open(path, "x", encoding="utf-8") as stream:
        json.dump(validated, stream, sort_keys=True, allow_nan=False)
        stream.write("\n")


def world_marker_payload(dump, **expected):
    dump = validate_dump(dump, **expected)
    world = transform_pose(dump["pose"], dump["transform_snapshot"])
    if world["reference"] != "box_center":
        fail("POSE_REFERENCE_MISMATCH", "reference", "box_center", world["reference"])
    return {"frame_id": "world", "stamp_ns": world["stamp_ns"],
            "request_id": world["request_id"], "position": world["position"],
            "quaternion_xyzw": world["quaternion_xyzw"],
            "scale_xyz": dump["body_dimensions_m"], "record_kind": dump["record_kind"]}


def adapt_legacy_surface(record, *, source_format, frame_id, height_datum,
                         floor_z_container, ceiling_z_container, identity):
    if source_format != "surface_floor_relative_v1":
        fail("UNKNOWN_LEGACY_FORMAT", "source_format", "surface_floor_relative_v1", source_format)
    if frame_id != "container_link" or height_datum != "floor_relative":
        fail("LEGACY_DATUM_CONFLICT", "frame/datum", "container_link/floor_relative", [frame_id, height_datum])
    for key, expected in (("frame_id", frame_id), ("height_datum", height_datum)):
        if key in record and record[key] != expected:
            fail("LEGACY_DATUM_CONFLICT", key, expected, record[key])
    floor = finite_number(floor_z_container, "floor_z_container")
    heights = record["height"]
    states = record["state"]
    confidence = record["confidence"]
    tops = []
    for row, state_row in zip(heights, states):
        if len(row) != len(state_row):
            fail("INVALID_GRID", "height/state", "same shape", [row, state_row])
        tops.append([None if state == "unknown" and height is None else
                     finite_number(height, "legacy.height") + floor
                     for height, state in zip(row, state_row)])
    if len(heights) != len(states):
        fail("INVALID_GRID", "height/state", "same shape", [len(heights), len(states)])
    surface = dict(identity, contract_version=2, kind="surface", frame_id="container_link",
                   length_unit="m", height_datum="container_origin",
                   floor_z_container=floor, ceiling_z_container=ceiling_z_container,
                   origin_xy_container=deepcopy(record["origin_xy_container"]),
                   resolution_m=record["resolution_m"], surface_z_container=tops,
                   state=deepcopy(states), confidence=deepcopy(confidence))
    validate_surface(surface, identity["scene_revision"], identity["geometry_hash"], identity["map_revision"])
    return surface
