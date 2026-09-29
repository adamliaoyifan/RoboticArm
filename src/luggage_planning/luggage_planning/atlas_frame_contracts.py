"""Frame-contract helpers for the reachability atlas (ROS-free).

ST06 (row F14): the v2 atlas stores DIRECTED concrete-tool yaw bins in
``container_link`` with full artifact identity. These helpers build the
advertised target pose, the artifact contract record and its identity hashes,
so the builder, the runtime query paths and the offline certification share
one implementation. The contract record is validated by
``luggage_description.frame_contracts.validate_atlas``; a pi-folded or
frame-swapped record fails there, never at query time.

The sampled target is a tool-down pose (suction +Z along base -Z) with a
directed yaw about container +Z. The builder solves IK in base_link, so the
container-frame bin yaw is mapped through the scene-config container
rotation (a horizontal container makes that a constant Z offset).
"""

from __future__ import division

import hashlib
import json
import math

TWO_PI = 2.0 * math.pi

# Directed concrete-tool yaw bins: 0 and pi stay distinct branches (a
# pi-periodic footprint is a DECISION-layer aggregation, never an artifact
# fold). Matches the contract fixture bins in frame_contract_cases.json.
DIRECTED_YAW_BINS = (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)

# Tool-down quaternion (suction +Z -> base -Z); identical to the builder's
# ``self._tool_down`` and placement_motion_filter's tool-down pose.
TOOL_DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)

_HORIZONTAL_TOL = 1e-9


def _rpy_to_matrix(rpy):
    """Rz(yaw) @ Ry(pitch) @ Rx(roll), matching the atlas builder exactly."""
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _quat_multiply(q1, q2):
    """Hamilton product (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def require_horizontal_container(container_rpy, tolerance=_HORIZONTAL_TOL):
    """Reject a tilted container before any target construction.

    The 2.5D admission gate lives at container geometry; this is the atlas
    copy of that gate. Returns the Z-yaw offset (rad) of the container in
    base_link on success.
    """
    roll, pitch, yaw = (float(v) for v in container_rpy)
    if abs(math.sin(roll)) > tolerance or abs(math.sin(pitch)) > tolerance:
        raise ValueError(
            "container is not horizontal (rpy=%r); the 2.5D atlas does not "
            "sample tilted containers" % (tuple(container_rpy),))
    # Normalise into (-pi, pi] so base_yaw stays a plain sum downstream.
    return math.atan2(math.sin(yaw), math.cos(yaw))


def container_yaw_offset(container_rpy):
    """Z-yaw of a horizontal container in base_link (rad, in (-pi, pi])."""
    return require_horizontal_container(container_rpy)


def tool_target_in_base(x, y, z, yaw_container, container_xyz,
                        container_rpy, tool_down_quat=TOOL_DOWN_QUAT):
    """Advertised suction_contact_frame target for one atlas sample.

    ``x, y, z`` is the container_link cell centre and ``yaw_container`` the
    DIRECTED container-frame tool yaw. Returns ``(position, quaternion_xyzw)``
    in base_link: the container rotation applied to the position, and the
    container-yaw-mapped base yaw composed with the tool-down rotation.
    """
    offset = require_horizontal_container(container_rpy)
    base_yaw = float(yaw_container) + offset
    R = _rpy_to_matrix(container_rpy)
    base_pos = (
        float(container_xyz[0])
        + R[0][0] * x + R[0][1] * y + R[0][2] * z,
        float(container_xyz[1])
        + R[1][0] * x + R[1][1] * y + R[1][2] * z,
        float(container_xyz[2])
        + R[2][0] * x + R[2][1] * y + R[2][2] * z,
    )
    yaw_quat = (
        0.0, 0.0, math.sin(base_yaw * 0.5), math.cos(base_yaw * 0.5))
    return base_pos, _quat_multiply(yaw_quat, tool_down_quat)


def wrapped_yaw_error(a, b):
    """Directed angular distance in [0, pi] (2pi wrap, no pi fold)."""
    d = abs((float(a) - float(b) + math.pi) % TWO_PI - math.pi)
    return d


def snap_yaw_directed(yaw, yaw_bins):
    """Nearest directed bin over 2pi. Returns ``(bin_yaw, error_rad)``.

    0 and pi are distinct bins: a pi-equivalent query yaw snaps to whichever
    directed branch it actually names, never onto its pi twin.
    """
    bins = [float(b) for b in (yaw_bins or (0.0,))]
    best_bin, best_err = bins[0], float("inf")
    for bin_yaw in bins:
        err = wrapped_yaw_error(yaw, bin_yaw)
        if err < best_err:
            best_err, best_bin = err, bin_yaw
    return best_bin, best_err


def _canonical_hash(payload):
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.md5(text.encode("utf-8")).hexdigest()


def geometry_hash(inner_dimensions, floor_z, ceiling_z, opening_side):
    """Identity of the container geometry block the grid was sampled in."""
    return _canonical_hash({
        "inner_dimensions": [float(v) for v in inner_dimensions],
        "floor_z": float(floor_z),
        "ceiling_z": float(ceiling_z),
        "opening_side": str(opening_side),
    })


def payload_hash(payload_section):
    """Identity of the attached-payload block (empty-load included).

    Deterministic over the canonical JSON of the exact section written into
    the artifact, so stamping a v2 contract onto a differently-payloaded
    artifact cannot reproduce the stored hash.
    """
    section = {
        "enabled": bool(payload_section.get("enabled")),
        "shape": payload_section.get("shape"),
        "size": ([float(v) for v in payload_section["size"]]
                 if payload_section.get("size") is not None else None),
        "attach_link": payload_section.get("attach_link"),
        "offset": ([float(v) for v in payload_section["offset"]]
                   if payload_section.get("offset") is not None else None),
        "touch_links": (list(payload_section["touch_links"])
                        if payload_section.get("touch_links") is not None
                        else None),
    }
    return _canonical_hash(section)


def atlas_contract_record(yaw_bins, scene_tf_hash, urdf_hash,
                          geometry_hash_hex, payload_hash_hex):
    """Assemble the v2 contract block for an atlas artifact.

    The returned record must pass
    ``frame_contracts.validate_atlas`` with the same identity; the function
    self-checks and raises ContractError on any drift before the caller
    writes metadata.
    """
    record = {
        "contract_version": 2,
        "position_frame": "container_link",
        "orientation_frame": "container_link",
        "reference": "suction_contact_frame",
        "orientation_semantics": "concrete_tool_pose",
        "angle_unit": "rad",
        "length_unit": "m",
        "height_datum": "container_origin",
        "tool_down_reference": "container_negative_z",
        "yaw_period_rad": TWO_PI,
        "yaw_bins_rad": [float(b) % TWO_PI for b in yaw_bins],
        "scene_revision": str(scene_tf_hash),
        "geometry_hash": str(geometry_hash_hex),
        "urdf_hash": str(urdf_hash),
        "payload_hash": str(payload_hash_hex),
    }
    from luggage_description.frame_contracts import validate_atlas
    validate_atlas(record, {
        "scene_revision": record["scene_revision"],
        "geometry_hash": record["geometry_hash"],
        "urdf_hash": record["urdf_hash"],
        "payload_hash": record["payload_hash"],
    })
    return record
