"""Rigid transforms, circle/cylinder fits, and URDF RPY."""

from __future__ import division

import math

import numpy as np


def as_unit(vector):
    vector = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = np.linalg.norm(vector)
    if norm < 1e-15:
        raise ValueError("zero vector")
    return vector / norm


def make_T(rotation, translation):
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(rotation, dtype=np.float64)
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return matrix


def invert_T(matrix):
    matrix = np.asarray(matrix, dtype=np.float64)
    rotation = matrix[:3, :3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T.dot(matrix[:3, 3])
    return inverse


def rpy_to_R(rpy):
    """URDF fixed-axis RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    roll, pitch, yaw = [float(v) for v in rpy]
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def R_to_rpy(rotation):
    """Inverse of rpy_to_R (URDF xyz / fixed-axis)."""
    rotation = np.asarray(rotation, dtype=np.float64)
    sy = -rotation[2, 0]
    cy = math.sqrt(max(0.0, 1.0 - sy * sy))
    if cy > 1e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(sy, cy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(sy, cy)
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=np.float64)


def R_to_quat_xyzw(rotation):
    rotation = np.asarray(rotation, dtype=np.float64)
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([x, y, z, w], dtype=np.float64)
    return quat / np.linalg.norm(quat)


def rotation_angle_deg(rotation):
    value = (np.trace(rotation) - 1.0) * 0.5
    value = min(1.0, max(-1.0, float(value)))
    return math.degrees(math.acos(value))


def rotation_axis(rotation):
    skew = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ], dtype=np.float64)
    norm = np.linalg.norm(skew)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0])
    return skew / norm


def orthonormal_frame(x_axis, z_axis):
    x_axis = as_unit(x_axis)
    z_axis = as_unit(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_norm = np.linalg.norm(y_axis)
    if y_norm < 1e-9:
        raise ValueError("x_axis parallel to z_axis")
    y_axis = y_axis / y_norm
    x_axis = np.cross(y_axis, z_axis)
    x_axis = as_unit(x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    if np.linalg.det(rotation) < 0.0:
        y_axis = -y_axis
        rotation = np.column_stack((x_axis, y_axis, z_axis))
    return rotation


def fit_plane(points):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    centroid = points.mean(axis=0)
    centred = points - centroid
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    normal = as_unit(vh[-1])
    residuals = centred.dot(normal)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    return centroid, normal, rms, int(points.shape[0])


def fit_circle_2d(xy):
    """Algebraic circle fit. Returns centre, radius, rms."""
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    x = xy[:, 0]
    y = xy[:, 1]
    A = np.column_stack((2.0 * x, 2.0 * y, np.ones(xy.shape[0])))
    b = x * x + y * y
    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    centre = sol[:2]
    radius = math.sqrt(max(sol[2] + centre.dot(centre), 0.0))
    rms = float(np.sqrt(np.mean((np.linalg.norm(xy - centre, axis=1) - radius) ** 2)))
    return centre, radius, rms


def fit_cylinder_from_points(points, axis_hint):
    """Project points onto a plane normal to axis_hint and fit a circle."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    axis = as_unit(axis_hint)
    origin = points.mean(axis=0)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(axis.dot(ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    x_axis = as_unit(np.cross(axis, ref))
    y_axis = as_unit(np.cross(axis, x_axis))
    uv = np.column_stack(((points - origin).dot(x_axis), (points - origin).dot(y_axis)))
    centre_uv, radius, rms = fit_circle_2d(uv)
    centre = origin + centre_uv[0] * x_axis + centre_uv[1] * y_axis
    axial = (points - centre).dot(axis)
    return {
        "centre": centre,
        "axis": axis,
        "radius": float(radius),
        "rms": float(rms),
        "axial_min": float(axial.min()),
        "axial_max": float(axial.max()),
        "support": int(points.shape[0]),
    }


def kabsch(source, target):
    """Return R, t mapping source rows onto target rows."""
    source = np.asarray(source, dtype=np.float64).reshape(-1, 3)
    target = np.asarray(target, dtype=np.float64).reshape(-1, 3)
    src_c = source.mean(axis=0)
    dst_c = target.mean(axis=0)
    h = (source - src_c).T.dot(target - dst_c)
    u, _, vt = np.linalg.svd(h)
    rotation = vt.T.dot(u.T)
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T.dot(u.T)
    translation = dst_c - rotation.dot(src_c)
    aligned = source.dot(rotation.T) + translation
    residuals = np.linalg.norm(aligned - target, axis=1)
    return rotation, translation, residuals


def two_hole_plane_frame(centre_a, centre_b, normal, origin_on_plane=True):
    """Right-handed frame: x along A->B, z = oriented seating normal."""
    x_axis = as_unit(np.asarray(centre_b) - np.asarray(centre_a))
    z_axis = as_unit(normal)
    rotation = orthonormal_frame(x_axis, z_axis)
    midpoint = 0.5 * (np.asarray(centre_a, dtype=np.float64) + np.asarray(centre_b, dtype=np.float64))
    if origin_on_plane:
        origin = midpoint - z_axis * z_axis.dot(midpoint - np.asarray(centre_a, dtype=np.float64))
        origin = midpoint
    else:
        origin = midpoint
    return make_T(rotation, origin)


def transform_report(matrix, parent_frame, child_frame, units="m"):
    rotation = np.asarray(matrix, dtype=np.float64)[:3, :3]
    translation = np.asarray(matrix, dtype=np.float64)[:3, 3]
    return {
        "parent_frame": parent_frame,
        "child_frame": child_frame,
        "notation": "^%s T_%s" % (parent_frame, child_frame),
        "multiplication": "p_parent = T * p_child (homogeneous column)",
        "units": units,
        "matrix_4x4": matrix.tolist(),
        "translation": translation.tolist(),
        "quaternion_xyzw": R_to_quat_xyzw(rotation).tolist(),
        "rpy_urdf_xyz": R_to_rpy(rotation).tolist(),
        "rpy_convention": "URDF fixed-axis RPY, R = Rz(yaw)*Ry(pitch)*Rx(roll)",
        "inverse_4x4": invert_T(matrix).tolist(),
    }
