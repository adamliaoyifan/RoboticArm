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


def stamp_ns_from_sec_nanosec(sec, nanosec):
    """Exact conversion of a ROS sec/nanosec pair to integer nanoseconds.

    This is the identity-preserving path: no float is constructed, so an
    epoch-scale stamp keeps every nanosecond.
    """
    if (isinstance(sec, bool) or isinstance(nanosec, bool)
            or not isinstance(sec, int) or not isinstance(nanosec, int)):
        raise ContractError("INVALID_STAMP", "stamp",
                            "integer sec and nanosec", (sec, nanosec))
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ContractError("INVALID_STAMP", "stamp",
                            "sec >= 0 and 0 <= nanosec < 1e9", (sec, nanosec))
    return sec * 1_000_000_000 + nanosec


def stamp_ns_from_seconds(stamp_sec):
    """Float-seconds transport form of the acquisition stamp.

    A float stamp is a transport representation, not the identity: float64
    at epoch scale (~1.7e9 s) resolves only ~0.2 us, so nanoseconds are
    already gone before this function runs. Callers holding the original
    header stamp must pass its integer nanoseconds (or the stamp object)
    through :func:`stamp_ns` instead of a float round-trip.
    """
    value = float(stamp_sec)
    if not math.isfinite(value) or value < 0.0:
        raise ContractError("INVALID_STAMP", "stamp", "finite nonnegative seconds",
                            stamp_sec)
    return int(round(value * 1e9))


def stamp_ns(stamp):
    """Exact acquisition stamp in integer nanoseconds from any stamp form.

    Accepts integer nanoseconds (returned as-is), a ROS time / header-stamp
    object carrying integer ``sec``/``nanosec`` (exact integer conversion),
    or float seconds (transport form via :func:`stamp_ns_from_seconds`).
    ``None``/negative values are rejected here rather than defaulted to
    ``now()``.
    """
    if stamp is None or isinstance(stamp, bool):
        raise ContractError("INVALID_STAMP", "stamp",
                            "int ns, sec/nanosec object or float seconds",
                            stamp)
    if isinstance(stamp, int):
        if stamp < 0:
            raise ContractError("INVALID_STAMP", "stamp",
                                "nonnegative nanoseconds", stamp)
        return stamp
    sec = getattr(stamp, "sec", None)
    nanosec = getattr(stamp, "nanosec", None)
    if isinstance(sec, int) and not isinstance(sec, bool) \
            and isinstance(nanosec, int) and not isinstance(nanosec, bool):
        return stamp_ns_from_sec_nanosec(sec, nanosec)
    if isinstance(stamp, float):
        return stamp_ns_from_seconds(stamp)
    raise ContractError("INVALID_STAMP", "stamp",
                        "int ns, sec/nanosec object or float seconds", stamp)


def detection_boundary(center_world, box_yaw, stamp, scene_revision,
                       request_id, frame_id="world", yaw_valid=True,
                       fallback_yaw=0.0):
    """Detection-profile record + yaw provenance for one observation.

    Returns ``(record, yaw_source)``. ``yaw_source`` is ``"measured"`` when
    the estimator's yaw is trusted and ``"yaw_fallback"`` when a near-square
    footprint forced the declared fallback; the fallback is recorded, never
    relabelled as measured. The record itself is exactly the pose-contract
    field set (extra provenance is not silently carried into the message).
    ``stamp`` keeps the measured acquisition stamp exactly: integer
    nanoseconds or the ROS sec/nanosec header stamp, never a float
    round-trip at epoch scale.
    """
    record = {
        "contract_version": 2,
        "frame_id": str(frame_id),
        "length_unit": "m",
        "stamp_ns": stamp_ns(stamp),
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


def contact_boundary(candidate, stamp, scene_revision, request_id,
                     frame_id=None):
    """Contact-profile record for one suction candidate.

    ``candidate`` is duck-typed on the v1 ``SuctionCandidate`` message or
    the perception ``SuctionCandidateRecord``: world contact position,
    quaternion with +Z along the outward surface normal, and the
    observation identity. The candidate's own frame must already be the
    acquisition frame (world); nothing here re-labels it. ``stamp`` keeps
    the measured acquisition stamp exactly (integer nanoseconds or the ROS
    sec/nanosec stamp object).
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
        "stamp_ns": stamp_ns(stamp),
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


def _quaternion_from_axes(axis_x, axis_y, axis_z):
    """Unit quaternion of the rotation with columns ``[x y z]`` (xyzw).

    Shepperd's method on the rotation matrix; the axes must already be
    orthonormal.
    """
    m00, m10, m20 = axis_x
    m01, m11, m21 = axis_y
    m02, m12, m22 = axis_z
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        quaternion = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s,
                      0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        quaternion = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s,
                      (m21 - m12) / s)
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        quaternion = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s,
                      (m02 - m20) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        quaternion = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s,
                      (m10 - m01) / s)
    norm = math.sqrt(sum(v * v for v in quaternion))
    return tuple(v / norm for v in quaternion)


def _opening_axes(normal, width_axis):
    """Right-handed aperture basis: +X the width axis, +Z the normal.

    The width axis is orthogonalized against the normal and re-normalized;
    a width axis parallel to the normal (degenerate aperture plane) is a
    contract rejection, not a silently rotated aperture.
    """
    z = tuple(float(v) for v in normal)
    w = tuple(float(v) for v in width_axis)
    for vector in (z, w):
        if not all(math.isfinite(v) for v in vector):
            raise ContractError("INVALID_NUMERIC_PAYLOAD", "opening_axes",
                                "finite normal and width axis", vector)
    z_norm = math.sqrt(sum(v * v for v in z))
    w_norm = math.sqrt(sum(v * v for v in w))
    if z_norm < 1e-12 or w_norm < 1e-12:
        raise ContractError("INVALID_NUMERIC_PAYLOAD", "opening_axes",
                            "nonzero normal and width axis",
                            [list(normal), list(width_axis)])
    z = tuple(v / z_norm for v in z)
    w = tuple(v / w_norm for v in w)
    dot = sum(w[i] * z[i] for i in range(3))
    x = tuple(w[i] - dot * z[i] for i in range(3))
    x_norm = math.sqrt(sum(v * v for v in x))
    if x_norm < 1e-9:
        raise ContractError("INVALID_NUMERIC_PAYLOAD", "opening_axes",
                            "width axis not parallel to the normal",
                            [list(normal), list(width_axis)])
    x = tuple(v / x_norm for v in x)
    y = (z[1] * x[2] - z[2] * x[1],
         z[2] * x[0] - z[0] * x[2],
         z[0] * x[1] - z[1] * x[0])
    return x, y, z


def opening_boundary(estimate, frame_id, scene_revision, request_id):
    """F05 explicit-frame record for one container-opening estimate.

    Not a v2 ROS message: the opening's ROS consumer is the ST07 sweep, so
    this record keeps the nested frame relationship explicit in-process.
    The opening pose lives in ``frame_id``; aperture vertices stay in the
    opening frame (origin at the aperture centre, +Z the outward normal,
    +X the width axis). ``pose_covariance`` (3x3 row-major) travels with the
    record so consumers rotate it as a matrix, never as points.

    The quaternion is the full aperture basis — +X onto the declared width
    axis, +Z onto the normal — so a non-square aperture keeps its width
    direction; a normal-only shortest arc would square-ize it.
    """
    stamp_value = getattr(estimate, "stamp_ns", None)
    if stamp_value is None:
        stamp_value = estimate.stamp
    stamp_ns_exact = stamp_ns(stamp_value)
    axis_x, axis_y, axis_z = _opening_axes(estimate.normal,
                                           estimate.width_axis)
    quaternion = _quaternion_from_axes(axis_x, axis_y, axis_z)
    half_w, half_h = float(estimate.width) / 2.0, float(estimate.height) / 2.0
    covariance = estimate.pose_covariance
    if hasattr(covariance, "reshape"):
        covariance = covariance.reshape(-1)
    return {
        "contract_version": 2,
        "frame_id": str(frame_id),
        "stamp_ns": stamp_ns_exact,
        "time_basis": "acquisition",
        "scene_revision": str(scene_revision),
        "request_id": str(request_id),
        "kind": "opening",
        "center": [float(v) for v in estimate.center],
        "normal": [float(v) for v in estimate.normal],
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
