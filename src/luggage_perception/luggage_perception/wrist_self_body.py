#!/usr/bin/env python3
"""Wrist-camera suction-panel silhouette (no ROS).

The D435 is bolted to the suction panel, so the plate occupies a fixed
region in every frame. Project the panel collision meshes into the camera
optical frame and rasterize a boolean mask. The mount chain is rigid: the
silhouette does not depend on arm pose.
"""

from __future__ import division

import math
import os
import struct

import numpy as np

# URDF camera_depth_optical_joint: rpy="-pi/2 0 -pi/2" on camera_link.
_OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)


def rpy_matrix(roll, pitch, yaw):
    """URDF RPY: R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def transform_matrix(xyz, rpy):
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rpy_matrix(rpy[0], rpy[1], rpy[2])
    mat[:3, 3] = np.asarray(xyz, dtype=np.float64).reshape(3)
    return mat


def matrix_from_translation_quaternion(xyz, quat_xyzw):
    """tf2 transform: translation + quaternion (x, y, z, w)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        rot = np.eye(3, dtype=np.float64)
    else:
        x, y, z, w = x / n, y / n, z / n, w / n
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = np.asarray(xyz, dtype=np.float64).reshape(3)
    return mat


def invert_transform(mat):
    out = np.eye(4, dtype=np.float64)
    rot = np.asarray(mat, dtype=np.float64)[:3, :3]
    out[:3, :3] = rot.T
    out[:3, 3] = -rot.T.dot(mat[:3, 3])
    return out


def optical_from_panel_matrix(adapter_xyz, adapter_rpy, camera_xyz, camera_rpy):
    """T_optical_from_panel from the fixed mount chain in the URDF.

    panel --adapter_joint--> eef_mount_adapter --camera_joint--> camera_link
    --optical_joint--> camera_depth_optical_frame.
    """
    t_panel_from_adapter = transform_matrix(adapter_xyz, adapter_rpy)
    t_adapter_from_cam = transform_matrix(camera_xyz, camera_rpy)
    t_cam_from_optical = transform_matrix((0.0, 0.0, 0.0), _OPTICAL_RPY)
    return (invert_transform(t_cam_from_optical)
            .dot(invert_transform(t_adapter_from_cam))
            .dot(invert_transform(t_panel_from_adapter)))


def load_binary_stl(path):
    """(N, 3, 3) float64 triangles from a binary STL. ASCII STLs raise."""
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) < 84:
        raise ValueError("STL too small: %s" % path)
    if data[:5].lower() == b"solid" and b"facet" in data[:256].lower():
        raise ValueError("ASCII STL is not supported: %s" % path)
    count = struct.unpack_from("<I", data, 80)[0]
    need = 84 + count * 50
    if len(data) < need:
        raise ValueError("truncated STL %s: %d tris, %d bytes"
                         % (path, count, len(data)))
    tris = np.empty((count, 3, 3), dtype=np.float64)
    offset = 84
    for i in range(count):
        xyz = struct.unpack_from("<9f", data, offset + 12)
        tris[i] = np.asarray(xyz, dtype=np.float64).reshape(3, 3)
        offset += 50
    return tris


def transform_triangles(tris, mat):
    pts = np.asarray(tris, dtype=np.float64).reshape(-1, 3)
    hom = np.ones((pts.shape[0], 4), dtype=np.float64)
    hom[:, :3] = pts
    out = hom.dot(np.asarray(mat, dtype=np.float64).T)[:, :3]
    return out.reshape(tris.shape)


def rasterize_triangles(tris_cam, fx, fy, cx, cy, width, height):
    """Fill projected triangles with z>0 into an HxW bool mask."""
    width, height = int(width), int(height)
    mask = np.zeros((height, width), dtype=bool)
    tris = np.asarray(tris_cam, dtype=np.float64).reshape(-1, 3, 3)
    zmin = 1e-4
    for tri in tris:
        if np.any(tri[:, 2] <= zmin):
            continue
        u = fx * tri[:, 0] / tri[:, 2] + cx
        v = fy * tri[:, 1] / tri[:, 2] + cy
        poly = np.stack((u, v), axis=1)
        umin = int(np.floor(poly[:, 0].min()))
        umax = int(np.ceil(poly[:, 0].max()))
        vmin = int(np.floor(poly[:, 1].min()))
        vmax = int(np.ceil(poly[:, 1].max()))
        umin, umax = max(0, umin), min(width - 1, umax)
        vmin, vmax = max(0, vmin), min(height - 1, vmax)
        if umax < umin or vmax < vmin:
            continue
        ys, xs = np.mgrid[vmin:vmax + 1, umin:umax + 1]
        pix = np.stack((xs, ys), axis=-1).astype(np.float64)

        def _edge(a, b, p):
            return ((p[..., 0] - a[0]) * (b[1] - a[1])
                    - (p[..., 1] - a[1]) * (b[0] - a[0]))

        w0 = _edge(poly[1], poly[2], pix)
        w1 = _edge(poly[2], poly[0], pix)
        w2 = _edge(poly[0], poly[1], pix)
        inside = (((w0 >= 0) & (w1 >= 0) & (w2 >= 0))
                  | ((w0 <= 0) & (w1 <= 0) & (w2 <= 0)))
        mask[vmin:vmax + 1, umin:umax + 1] |= inside
    return mask


def dilate_mask(mask, radius_px):
    """Disk dilation; radius 0 returns a bool copy."""
    src = np.asarray(mask, dtype=bool)
    radius = int(radius_px)
    if radius <= 0:
        return src.copy()
    height, width = src.shape
    out = np.zeros_like(src)
    ys, xs = np.nonzero(src)
    if ys.size == 0:
        return out
    for dy in range(-radius, radius + 1):
        yy = ys + dy
        ok_y = (yy >= 0) & (yy < height)
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            xx = xs + dx
            ok = ok_y & (xx >= 0) & (xx < width)
            out[yy[ok], xx[ok]] = True
    return out


def load_panel_triangles(mesh_paths, mesh_origin):
    """Load STLs and shift them into the suction_panel link frame."""
    origin = transform_matrix(mesh_origin, (0.0, 0.0, 0.0))
    chunks = []
    for path in mesh_paths:
        if not os.path.isfile(path):
            raise IOError("panel mesh missing: %s" % path)
        chunks.append(transform_triangles(load_binary_stl(path), origin))
    if not chunks:
        raise ValueError("no panel meshes")
    return np.concatenate(chunks, axis=0)


def panel_mask_from_meshes(mesh_paths, mesh_origin, t_optical_from_panel,
                           fx, fy, cx, cy, width, height, dilate_px=0):
    """Boolean HxW silhouette of the suction panel in the camera image."""
    tris_panel = load_panel_triangles(mesh_paths, mesh_origin)
    tris_cam = transform_triangles(tris_panel, t_optical_from_panel)
    mask = rasterize_triangles(tris_cam, fx, fy, cx, cy, width, height)
    return dilate_mask(mask, dilate_px)
