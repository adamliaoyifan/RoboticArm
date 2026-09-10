#!/usr/bin/env python3
"""Shared depth deprojection maths (no ROS).

PF-R9 g2: consumers reconstruct geometry locally from the colour-aligned
depth image instead of transporting a camera point cloud. The aligned
depth pixel grid IS the colour pixel grid, so deprojection uses the
colour intrinsics directly and needs no extrinsics:

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    z = depth_mm * 0.001

``z == 0`` (or non-finite for float inputs) means "no data" and is
excluded — a missing measurement is never deprojected to the origin.
"""

from __future__ import division

import numpy as np


def deproject_selected(depth_image, uu, vu, intrinsics, out=None):
    """Deproject selected pixels of an aligned depth image.

    Args:
        depth_image: (H, W) uint16-family view (millimetres; big-endian
            views are valid — numpy converts per element on read).
        uu, vu: integer pixel coordinate arrays (column, row).
        intrinsics: object with fx, fy, cx, cy of the colour grid.
        out: optional caller-owned (capacity, 3) float32 buffer; the
            return is then a view into it, valid until the buffer's next
            use (PF-R10 C2: the escaping output buffer is the one whose
            size varies frame to frame and ratchets RSS; see
            deproject_stride).

    Returns:
        (N, 3) float32 points in the colour optical frame, pixels whose
        depth is 0/non-finite excluded, plus the kept-count.
    """
    z_mm = np.asarray(depth_image)[vu, uu].astype(np.float32)
    z = z_mm * np.float32(0.001)
    valid = np.isfinite(z) & (z > 0.0)
    uu = np.asarray(uu)[valid]
    vu = np.asarray(vu)[valid]
    z = z[valid]
    x = (uu.astype(np.float32) - np.float32(intrinsics.cx)) * z \
        / np.float32(intrinsics.fx)
    y = (vu.astype(np.float32) - np.float32(intrinsics.cy)) * z \
        / np.float32(intrinsics.fy)
    if out is not None:
        n = int(z.shape[0])
        if (out.ndim != 2 or out.shape[1] != 3
                or out.dtype != np.float32 or out.shape[0] < n):
            raise ValueError(
                "out must be a (capacity>=n, 3) float32 buffer; got %r"
                % (out,))
        view = out[:n]
        view[:, 0] = x
        view[:, 1] = y
        view[:, 2] = z
        return view, n
    points = np.stack((x, y, z), axis=1).astype(np.float32, copy=False)
    return points, int(points.shape[0])


def deproject_stride(depth_image, intrinsics, stride=1, out=None):
    """Deproject a strided pixel grid of an aligned depth image.

    Deterministic stride subsampling over the full frame: rows and columns
    ``[::stride]`` of the (H, W) grid, pixel coordinates preserved (not
    re-indexed). Used for the detector's support annulus; the retained
    density scales as 1/stride^2.

    ``out`` optionally supplies a caller-owned ``(capacity, 3)`` float32
    buffer. The return value is then a VIEW into it holding the valid
    rows; the caller must consume the view before the next call that
    shares the buffer. When ``out`` is given, the function performs no
    size-varying heap allocation of its own: the per-frame heap churn of
    a varying-size ``np.stack`` is exactly what ratchets RSS in the
    multithreaded executor (PF-R10 C2 measurement), because freed
    chunks of one size cannot serve the next frame's slightly different
    size. A fixed-capacity buffer is freed-and-reused at one address.
    """
    depth = np.asarray(depth_image)
    stride = max(1, int(stride))
    rows = np.arange(0, depth.shape[0], stride)
    cols = np.arange(0, depth.shape[1], stride)
    vu_grid, uu_grid = np.meshgrid(rows, cols, indexing="ij")
    z = depth[::stride, ::stride].reshape(-1).astype(
        np.float32) * np.float32(0.001)
    valid = np.isfinite(z) & (z > 0.0)
    n = int(np.count_nonzero(valid))
    uu = uu_grid.reshape(-1)[valid].astype(np.float32)
    vu = vu_grid.reshape(-1)[valid].astype(np.float32)
    zv = z[valid]
    if out is None:
        x = (uu - np.float32(intrinsics.cx)) * zv / np.float32(intrinsics.fx)
        y = (vu - np.float32(intrinsics.cy)) * zv / np.float32(intrinsics.fy)
        points = np.empty((n, 3), np.float32)
        points[:, 0] = x
        points[:, 1] = y
        points[:, 2] = zv
        return points, n
    if out.ndim != 2 or out.shape[1] != 3 or out.dtype != np.float32 \
            or out.shape[0] < n:
        raise ValueError(
            "out must be a (capacity>=n, 3) float32 buffer; got %r" % (
                out,))
    view = out[:n]
    view[:, 0] = (uu - np.float32(intrinsics.cx)) * zv \
        / np.float32(intrinsics.fx)
    view[:, 1] = (vu - np.float32(intrinsics.cy)) * zv \
        / np.float32(intrinsics.fy)
    view[:, 2] = zv
    return view, n
