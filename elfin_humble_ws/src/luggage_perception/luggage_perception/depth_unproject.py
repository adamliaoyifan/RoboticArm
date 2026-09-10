"""Unproject colour-aligned Z16 depth to XYZ (no ROS).

Used by the preprocessor so D555 compressed PNG never has to become a
PointCloud2 topic before the algorithm. Stride matches the old organized
cloud decimation (sample every Nth row/column, keep original K).
"""

from __future__ import division

import numpy as np


def unproject_z16(depth, fx, fy, cx, cy, scale=0.001, stride=1):
    """Return (N,3) float32 xyz in the image optical frame.

    ``depth`` is HxW uint16 millimetres (or any 2-D array in ``scale``
    units). Invalid / zero depth is dropped. ``stride`` >= 1 subsamples
    the pixel grid the same way an organized cloud ``[::stride, ::stride]``
    would.
    """
    depth = np.asarray(depth)
    if depth.ndim != 2 or depth.size == 0:
        return np.zeros((0, 3), dtype=np.float32)
    fx = float(fx)
    fy = float(fy)
    if fx <= 1e-6 or fy <= 1e-6:
        return np.zeros((0, 3), dtype=np.float32)
    stride = max(1, int(stride))
    height, width = int(depth.shape[0]), int(depth.shape[1])
    sampled = np.ascontiguousarray(depth[::stride, ::stride])
    u = np.arange(0, width, stride, dtype=np.float32)
    v = np.arange(0, height, stride, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    z = sampled.astype(np.float32) * float(scale)
    valid = np.isfinite(z) & (z > 0.0)
    x = (uu - float(cx)) / fx * z
    y = (vv - float(cy)) / fy * z
    return np.stack((x[valid], y[valid], z[valid]), axis=1)
