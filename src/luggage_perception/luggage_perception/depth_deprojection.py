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


def deproject_selected(depth_image, uu, vu, intrinsics):
    """Deproject selected pixels of an aligned depth image.

    Args:
        depth_image: (H, W) uint16-family view (millimetres; big-endian
            views are valid — numpy converts per element on read).
        uu, vu: integer pixel coordinate arrays (column, row).
        intrinsics: object with fx, fy, cx, cy of the colour grid.

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
    points = np.stack((x, y, z), axis=1).astype(np.float32, copy=False)
    return points, int(points.shape[0])


def deproject_stride(depth_image, intrinsics, stride=1):
    """Deproject a strided pixel grid of an aligned depth image.

    Deterministic stride subsampling over the full frame: rows and columns
    ``[::stride]`` of the (H, W) grid, pixel coordinates preserved (not
    re-indexed). Used for the detector's support annulus; the retained
    density scales as 1/stride^2.
    """
    depth = np.asarray(depth_image)
    stride = max(1, int(stride))
    grid = depth[::stride, ::stride]
    rows = np.arange(0, depth.shape[0], stride)
    cols = np.arange(0, depth.shape[1], stride)
    vu_grid, uu_grid = np.meshgrid(rows, cols, indexing="ij")
    z = grid.reshape(-1).astype(np.float32) * np.float32(0.001)
    valid = np.isfinite(z) & (z > 0.0)
    uu = uu_grid.reshape(-1)[valid].astype(np.float32)
    vu = vu_grid.reshape(-1)[valid].astype(np.float32)
    z = z[valid]
    x = (uu - np.float32(intrinsics.cx)) * z / np.float32(intrinsics.fx)
    y = (vu - np.float32(intrinsics.cy)) * z / np.float32(intrinsics.fy)
    points = np.stack((x, y, z), axis=1).astype(np.float32, copy=False)
    return points, int(points.shape[0])
