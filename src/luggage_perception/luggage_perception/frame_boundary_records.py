#!/usr/bin/env python3
"""ST03 v2 boundary record builders for perception (ROS-free).

These builders turn module-level algorithm results into strict v2 boundary
records understood by the ST02 substrate (``frame_contracts``,
``frame_message_adapters``). They never publish, look up TF, invent stamps
or repair inputs. Dimensions and 2D masks keep travelling on the unchanged
v1 messages; a v2 record carries pose + identity only.
"""

import math

from luggage_description.frame_contracts import (
    ContractError, validate_envelope, validate_pose,
)
from luggage_description.frame_diagnostics import at_boundary


def stamp_ns_from_seconds(stamp_sec):
    """Exact acquisition stamp in integer nanoseconds (float seconds in).

    A float stamp is the transport representation; the v2 record keeps the
    nanosecond integer. ``None``/negative values are rejected here rather
    than defaulted to ``now()``.
    """
    value = float(stamp_sec)
    if not math.isfinite(value) or value < 0.0:
        raise ContractError("INVALID_STAMP", "stamp", "finite nonnegative seconds",
                            stamp_sec)
    return int(round(value * 1e9))


def detection_boundary(center_world, box_yaw, stamp_sec, scene_revision,
                       request_id, frame_id="world", yaw_valid=True,
                       fallback_yaw=0.0):
    """Detection-profile record + yaw provenance for one observation.

    Returns ``(record, yaw_source)``. ``yaw_source`` is ``"measured"`` when
    the estimator's yaw is trusted and ``"yaw_fallback"`` when a near-square
    footprint forced the declared fallback; the fallback is recorded, never
    relabelled as measured. The record itself is exactly the pose-contract
    field set (extra provenance is not silently carried into the message).
    """
    record = {
        "contract_version": 2,
        "frame_id": str(frame_id),
        "length_unit": "m",
        "stamp_ns": stamp_ns_from_seconds(stamp_sec),
        "time_basis": "acquisition",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "pose",
        "reference": "box_center",
        "orientation_semantics": "box_axis_pi",
        "position": [float(center_world[0]), float(center_world[1]),
                     float(center_world[2])],
        "quaternion_xyzw": [0.0, 0.0,
                            math.sin(float(box_yaw) * 0.5),
                            math.cos(float(box_yaw) * 0.5)],
    }
    source = "measured" if yaw_valid else "yaw_fallback"
    if not yaw_valid:
        # The declared fallback replaces the unusable measured yaw; the
        # quaternion is rebuilt so record and provenance cannot disagree.
        record["quaternion_xyzw"] = [0.0, 0.0,
                                     math.sin(float(fallback_yaw) * 0.5),
                                     math.cos(float(fallback_yaw) * 0.5)]
    validate_pose(record, "detection", str(scene_revision))
    return record, source


def contact_boundary(candidate, stamp_sec, scene_revision, request_id,
                     frame_id=None):
    """Contact-profile record for one suction candidate.

    ``candidate`` is duck-typed on the v1 ``SuctionCandidate`` message or
    the perception ``SuctionCandidateRecord``: world contact position,
    quaternion with +Z along the outward surface normal, and the
    observation identity. The candidate's own frame must already be the
    acquisition frame (world); nothing here re-labels it.
    """
    frame = str(frame_id if frame_id is not None else candidate.frame)
    position = (
        float(candidate.contact_pose.position.x
              if hasattr(candidate, "contact_pose") else candidate.center_world[0]),
        float(candidate.contact_pose.position.y
              if hasattr(candidate, "contact_pose") else candidate.center_world[1]),
        float(candidate.contact_pose.position.z
              if hasattr(candidate, "contact_pose") else candidate.center_world[2]),
    )
    if hasattr(candidate, "contact_pose"):
        orientation = candidate.contact_pose.orientation
        quaternion = (float(orientation.x), float(orientation.y),
                      float(orientation.z), float(orientation.w))
    else:
        quaternion = tuple(float(q) for q in candidate.quaternion_xyzw)
    record = {
        "contract_version": 2,
        "frame_id": frame,
        "length_unit": "m",
        "stamp_ns": stamp_ns_from_seconds(stamp_sec),
        "time_basis": "acquisition",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "pose",
        "reference": "surface_contact",
        "orientation_semantics": "surface_normal",
        "position": list(position),
        "quaternion_xyzw": list(quaternion),
    }
    validate_pose(record, "contact", str(scene_revision))
    return record


def admit_sensor_observation(record, context):
    """F02 admission for one sensor-observation boundary record.

    The observation keeps its truthful sensor frame and its own acquisition
    stamp; admission rejects (before any publish) a missing frame identity,
    an absent/invalid stamp, a non-acquisition time basis, a stale scene
    revision or nonfinite geometry. Points are optional; when present every
    component must be finite. Raises ``BoundaryContractError``.
    """
    def checks():
        if not isinstance(record, dict):
            raise ContractError("INVALID_RECORD", "observation", "object", record)
        frame = record.get("frame_id")
        if not isinstance(frame, str) or not frame.strip():
            raise ContractError("MISSING_IDENTITY", "frame_id",
                                "truthful sensor frame", frame)
        stamp = record.get("stamp_ns")
        if type(stamp) is not int or stamp < 0:
            raise ContractError("INVALID_STAMP", "stamp_ns",
                                "explicit nonnegative nanoseconds", stamp)
        points = record.get("points")
        if points is not None:
            for index, point in enumerate(points):
                if not isinstance(point, (list, tuple)) or len(point) != 3:
                    raise ContractError("INVALID_NUMERIC_PAYLOAD",
                                        "points[%d]" % index, "xyz numbers", point)
                for value in point:
                    if type(value) not in (int, float) or not math.isfinite(value):
                        raise ContractError(
                            "INVALID_NUMERIC_PAYLOAD", "points[%d]" % index,
                            "finite numbers", value)
        validate_envelope(record, context.scene_revision)
        if record.get("time_basis") != "acquisition":
            raise ContractError("TIME_BASIS_MISMATCH", "time_basis",
                                "acquisition", record.get("time_basis"))
        if context.frame_id and record.get("frame_id") != context.frame_id:
            # An expected frame is declared only when the boundary has one
            # configured truth; multi-product observations keep their own
            # truthful sensor frames and are checked for presence above.
            raise ContractError("FRAME_CONTRACT_MISMATCH", "frame_id",
                                context.frame_id, record.get("frame_id"))
    return at_boundary(record, context, checks)


def opening_boundary(estimate, frame_id, scene_revision, request_id):
    """F05 explicit-frame record for one container-opening estimate.

    Not a v2 ROS message: the opening's ROS consumer is the ST07 sweep, so
    this record keeps the nested frame relationship explicit in-process.
    The opening pose lives in ``frame_id``; aperture vertices stay in the
    opening frame (origin at the aperture centre, +Z the outward normal,
    +X the width axis). ``pose_covariance`` (3x3 row-major) travels with the
    record so consumers rotate it as a matrix, never as points.
    """
    stamp_ns = stamp_ns_from_seconds(estimate.stamp)
    normal = tuple(float(v) for v in estimate.normal)
    # Orientation aligning world +Z onto the opening normal: same minimal
    # rotation the suction evaluator uses, so both share one convention.
    axis_x, axis_y = -normal[1], normal[0]
    s = math.hypot(axis_x, axis_y)
    c = normal[2]
    if s < 1e-12:
        quaternion = (1.0, 0.0, 0.0, 0.0) if c < 0.0 else (0.0, 0.0, 0.0, 1.0)
    else:
        angle = math.atan2(s, c)
        half = math.sin(angle / 2.0)
        quaternion = (axis_x / s * half, axis_y / s * half, 0.0,
                      math.cos(angle / 2.0))
    half_w, half_h = float(estimate.width) / 2.0, float(estimate.height) / 2.0
    covariance = estimate.pose_covariance
    if hasattr(covariance, "reshape"):
        covariance = covariance.reshape(-1)
    return {
        "contract_version": 2,
        "frame_id": str(frame_id),
        "stamp_ns": stamp_ns,
        "time_basis": "acquisition",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "opening",
        "center": [float(v) for v in estimate.center],
        "normal": list(normal),
        "width_axis": [float(v) for v in estimate.width_axis],
        "quaternion_xyzw": list(quaternion),
        "aperture_vertices_opening": [
            [-half_w, -half_h, 0.0], [half_w, -half_h, 0.0],
            [half_w, half_h, 0.0], [-half_w, half_h, 0.0],
        ],
        "width": float(estimate.width),
        "height": float(estimate.height),
        "pose_covariance": [float(v) for v in covariance],
        "confidence": float(estimate.confidence),
        "source_status": str(estimate.source_status),
    }
