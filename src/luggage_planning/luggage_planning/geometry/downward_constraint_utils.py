#!/usr/bin/env python3
"""Downward-orientation constraints for camera-down / suction-down planning.

The smart-explore pipeline must keep BOTH the camera optical axis and the
suction normal near base -Z during motion. They are at a fixed angle
(``inter_axis_deg`` ~= 12.25 deg for the current URDF mount) in the
suction_panel frame, so they cannot both point exactly at -Z. The
suction-down camera orientation tilts the camera by exactly that inter-axis
angle so the suction lands on -Z (0 deg) while the camera sits at
~12.25 deg - inside the 15 deg camera budget and the 5 deg suction budget.

This module is split so the geometry is unit-testable without ROS:

  * pure math: ``tilt_from_down``, ``quaternion_align_vectors``,
    ``downward_orientations_from_matrix``, ``feasibility_check``,
    ``link_z_tilt_deg``, ``validate_downward_tilts``, ``optical_axis_from_quat``,
    ``yaw_about_down``, ``align_tilt_azimuth``.
  * one TF helper (``compute_downward_orientations``) called lazily by the
    nodes; it imports rospy/tf2_ros locally so importing this module never
    requires a running ROS master.
"""

from __future__ import division

import math

# base_link -Z (gravity). The arm base is mounted upright, so base -Z is the
# "straight down" reference for both the camera and the suction normal.
DOWN_AXIS = (0.0, 0.0, -1.0)


def _normalize(vec):
    norm = math.sqrt(sum(v * v for v in vec))
    if norm < 1e-12:
        return (0.0, 0.0, 0.0)
    return tuple(v / norm for v in vec)


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def angle_deg_between(vec_a, vec_b):
    """Angle in degrees between two vectors (each normalized internally)."""
    a = _normalize(vec_a)
    b = _normalize(vec_b)
    cos_t = max(-1.0, min(1.0, _dot(a, b)))
    return math.degrees(math.acos(cos_t))


def tilt_from_down(vec):
    """Degrees between ``vec`` and base -Z (0 == pointing straight down)."""
    return angle_deg_between(vec, DOWN_AXIS)


def _quaternion_to_matrix(q):
    """Return a 3x3 rotation matrix (row-major) for quaternion (x, y, z, w)."""
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _matrix_column(mat, col):
    return (mat[0][col], mat[1][col], mat[2][col])


def _rotate_vector(mat, vec):
    return (
        mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2],
        mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2],
        mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2],
    )


def _quaternion_multiply(q1, q2):
    """Hamilton product ``q1 * q2`` (each (x, y, z, w)): apply ``q2`` then ``q1``."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def optical_axis_from_quat(quat):
    """Local +Z of ``quat`` expressed in the frame ``quat`` is defined in.

    For a ``camera_down_quat`` (a camera orientation in base_frame) this is
    the camera optical axis in base_frame.
    """
    matrix = _quaternion_to_matrix(quat)
    return _matrix_column(matrix, 2)


def yaw_about_down(quat, yaw_rad):
    """Left-multiply ``quat`` by a world-frame rotation of ``yaw_rad`` about
    the vertical axis, using base +Z (not ``DOWN_AXIS``) so a positive
    ``yaw_rad`` increases ``atan2(y, x)`` by that amount -- the standard
    right-handed convention ``align_tilt_azimuth`` relies on.

    Base +Z and ``DOWN_AXIS`` (base -Z) are the same line, so rotating about
    either changes only the azimuthal (horizontal) direction of any vector
    expressed in base_frame -- the tilt-from-down angle of both the suction
    normal and the camera optical axis is unaffected. This is why yaw can be
    freely re-picked after ``quaternion_align_vectors`` (whose
    minimal-rotation construction leaves an arbitrary azimuth) without
    changing either downward tilt budget.
    """
    vertical_axis = (0.0, 0.0, 1.0)
    half = float(yaw_rad) * 0.5
    s = math.sin(half)
    yaw_quat = (
        vertical_axis[0] * s, vertical_axis[1] * s, vertical_axis[2] * s,
        math.cos(half))
    return _quaternion_multiply(yaw_quat, quat)


def align_tilt_azimuth(camera_down_quat, toward_xy):
    """Re-yaw ``camera_down_quat`` so the optical +Z's horizontal projection
    points toward ``toward_xy`` (e.g. ``-opening_normal``, into the
    container), leaving the tilt-from-down of both the camera axis and the
    suction normal unchanged (see ``yaw_about_down``).

    ``toward_xy`` is a direction in the base_frame XY plane (only its x/y
    components are used; need not be normalized). Returns
    ``camera_down_quat`` unchanged if either the camera's current horizontal
    projection or ``toward_xy`` is degenerate (camera pointing straight down,
    or ``toward_xy`` has ~zero horizontal component) -- there is then no
    azimuth to align.
    """
    cam_z = optical_axis_from_quat(camera_down_quat)
    cam_xy_norm = math.sqrt(cam_z[0] * cam_z[0] + cam_z[1] * cam_z[1])
    target_xy_norm = math.sqrt(
        toward_xy[0] * toward_xy[0] + toward_xy[1] * toward_xy[1])
    if cam_xy_norm < 1e-9 or target_xy_norm < 1e-9:
        return camera_down_quat
    cam_azimuth = math.atan2(cam_z[1], cam_z[0])
    target_azimuth = math.atan2(toward_xy[1], toward_xy[0])
    return yaw_about_down(camera_down_quat, target_azimuth - cam_azimuth)


def quaternion_align_vectors(v_from, v_to):
    """Minimal rotation (x, y, z, w) mapping unit vector ``v_from`` -> ``v_to``.

    Returns a unit quaternion. When ``v_from`` == -``v_to`` the rotation is
    180 deg about an arbitrary perpendicular axis (the camera/suction case:
    aligning a local +Z with base -Z).
    """
    a = _normalize(v_from)
    b = _normalize(v_to)
    d = _dot(a, b)
    if d >= 1.0 - 1e-12:
        return (0.0, 0.0, 0.0, 1.0)  # identity
    if d <= -1.0 + 1e-12:
        axis = _cross(a, (0.0, 0.0, 1.0))
        if _dot(axis, axis) < 1e-12:
            axis = _cross(a, (0.0, 1.0, 0.0))
        axis = _normalize(axis)
        return (axis[0], axis[1], axis[2], 0.0)
    axis = _normalize(_cross(a, b))
    half = math.acos(max(-1.0, min(1.0, d))) * 0.5
    s = math.sin(half)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))


def downward_orientations_from_matrix(cam_to_suc_matrix):
    """Derive downward target quaternions from the camera->suction rotation.

    ``cam_to_suc_matrix`` is the rotation of ``suction_contact_frame`` relative
    to ``camera_depth_optical_frame`` (i.e. it maps suction-local vectors into
    the camera frame), as produced by ``tf_buffer.lookup_transform(camera,
    suction)``.

    Returns a dict with:

      * ``inter_axis_deg``: angle between the camera optical +Z and the suction
        +Z (in the suction_panel frame; ~12.25 deg for the current mount).
      * ``camera_down_quat`` (x, y, z, w): camera orientation in ``base_frame``
        that places the suction normal on base -Z. The camera optical +Z then
        sits ``inter_axis_deg`` off -Z - inside the camera tilt budget.
      * ``suction_down_quat`` (x, y, z, w): suction_contact_frame orientation
        in ``base_frame`` with its +Z on base -Z (the keep_tool_down target).
      * ``suction_normal_in_cam``: suction +Z expressed in the camera frame.
    """
    suction_normal_in_cam = _rotate_vector(cam_to_suc_matrix, (0.0, 0.0, 1.0))
    inter_axis_deg = angle_deg_between((0.0, 0.0, 1.0), suction_normal_in_cam)
    camera_down_quat = quaternion_align_vectors(suction_normal_in_cam, DOWN_AXIS)
    suction_down_quat = quaternion_align_vectors((0.0, 0.0, 1.0), DOWN_AXIS)
    return {
        "inter_axis_deg": inter_axis_deg,
        "camera_down_quat": camera_down_quat,
        "suction_down_quat": suction_down_quat,
        "suction_normal_in_cam": suction_normal_in_cam,
    }


def compute_downward_orientations(
        cam_to_suc_xyzw, camera_frame="", suction_frame="", base_frame=""):
    """Derive downward target quaternions from a camera->suction rotation.

    ``cam_to_suc_xyzw`` is (x, y, z, w). Looking up TF belongs in the
    Phase 6 node, not this library.
    """
    matrix = _quaternion_to_matrix(cam_to_suc_xyzw)
    result = downward_orientations_from_matrix(matrix)
    result["camera_frame"] = camera_frame
    result["suction_frame"] = suction_frame
    result["base_frame"] = base_frame
    return result


def feasibility_check(inter_axis_deg, camera_max_tilt_deg, suction_max_tilt_deg):
    """Return (ok, message) for the dual downward constraint.

    With the suction-down camera orientation the achieved tilts are
    camera_tilt = ``inter_axis_deg`` and suction_tilt = 0, so the binding
    feasibility condition is that the camera budget admits the inter-axis
    tilt: ``camera_max_tilt_deg >= inter_axis_deg``. The plan's looser sum
    check (``camera_max + suction_max >= inter_axis``) is reported too.
    """
    if camera_max_tilt_deg + 1e-6 < inter_axis_deg:
        return False, (
            "infeasible mount: camera_max_tilt=%.2f deg < inter-axis=%.2f deg; "
            "the suction-down camera orientation needs the camera tilted by "
            "the inter-axis angle" % (camera_max_tilt_deg, inter_axis_deg)
        )
    if camera_max_tilt_deg + suction_max_tilt_deg + 1e-6 < inter_axis_deg:
        return False, (
            "infeasible tolerances: camera_max+suction_max=%.2f deg < "
            "inter-axis=%.2f deg" % (
                camera_max_tilt_deg + suction_max_tilt_deg, inter_axis_deg)
        )
    return True, (
        "feasible: inter-axis=%.2f deg, camera<= %.2f deg (achieved %.2f), "
        "suction<= %.2f deg (achieved 0.0)" % (
            inter_axis_deg, camera_max_tilt_deg, inter_axis_deg,
            suction_max_tilt_deg)
    )


def link_z_tilt_deg(orientation_xyzw):
    """Tilt (deg) of a link's local +Z axis from base -Z.

    ``orientation_xyzw`` is the link's orientation in ``base_frame`` (e.g. from
    /compute_fk). The link's local +Z in base is the third column of the
    rotation matrix.
    """
    matrix = _quaternion_to_matrix(orientation_xyzw)
    z_in_base = _matrix_column(matrix, 2)
    return tilt_from_down(z_in_base)


def validate_downward_tilts(
        camera_tilts_deg, suction_tilts_deg, camera_max_deg, suction_max_deg):
    """Validate per-point camera/suction tilt series against the budgets.

    Returns (ok, max_camera_tilt, max_suction_tilt, worst_index). ``ok`` is
    False when any sampled point exceeds either budget; ``worst_index`` is the
    point with the largest excess (-1 if all within budget or series empty).
    """
    max_camera = max(camera_tilts_deg) if camera_tilts_deg else 0.0
    max_suction = max(suction_tilts_deg) if suction_tilts_deg else 0.0
    worst_excess = 0.0
    worst_index = -1
    for i, (cam, suc) in enumerate(zip(camera_tilts_deg, suction_tilts_deg)):
        excess = max(cam - camera_max_deg, suc - suction_max_deg)
        if excess > worst_excess:
            worst_excess = excess
            worst_index = i
    ok = worst_excess <= 1e-6
    return ok, max_camera, max_suction, worst_index
