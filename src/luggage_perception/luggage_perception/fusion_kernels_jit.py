"""JIT-compiled fusion kernels (B round, 2026-10-27).

Same pure-function signatures as the numpy kernels in
cargo_volume_mapper; @njit(cache=True) compiles to machine code on
first call and caches to disk. The mapper imports these INSTEAD of
the numpy versions when numba is available (graceful fallback to the
numpy path when not).

Quality contract: these produce IDENTICAL results to the numpy path
(oracle-tested). The only difference is execution speed.
"""

import math

import numpy as np

try:
    from numba import njit
    HAS_NUMBA = True
except ImportError:
    HAS_NUMBA = False
    def njit(*args, **kwargs):
        """No-op decorator when numba is unavailable."""
        if len(args) == 1 and callable(args[0]):
            return args[0]
        def wrapper(func):
            return func
        return wrapper


@njit(cache=True)
def _jit_world_to_local(points, center, cos_neg_yaw, sin_neg_yaw):
    """Batch world->local. Same formula as _world_to_local."""
    n = points.shape[0]
    local = np.empty((n, 3), dtype=np.float64)
    for i in range(n):
        dx = points[i, 0] - center[0]
        dy = points[i, 1] - center[1]
        local[i, 0] = cos_neg_yaw * dx - sin_neg_yaw * dy
        local[i, 1] = sin_neg_yaw * dx + cos_neg_yaw * dy
        local[i, 2] = points[i, 2] - center[2]
    return local


@njit(cache=True)
def _jit_local_to_voxel(local, resolution, half_l, half_w, half_h,
                         nx, ny, nz, active):
    """Batch local->voxel. Returns (valid_mask, flat_index).

    Mirrors _local_to_voxel exactly: boundary check -> int() -> clamp
    -> active check. active must be a bool array (not bytearray).
    """
    n = local.shape[0]
    flat = np.empty(n, dtype=np.int64)
    valid = np.empty(n, dtype=np.bool_)
    for i in range(n):
        lx = local[i, 0]
        ly = local[i, 1]
        lz = local[i, 2]
        # Boundary: abs(x) > half -> reject (same as original)
        if abs(lx) > half_l or abs(ly) > half_w or abs(lz) > half_h:
            valid[i] = False
            flat[i] = 0
            continue
        ix = int((lx + half_l) / resolution)
        iy = int((ly + half_w) / resolution)
        iz = int((lz + half_h) / resolution)
        # Clamp (same as original min/max)
        if ix < 0:
            ix = 0
        elif ix > nx - 1:
            ix = nx - 1
        if iy < 0:
            iy = 0
        elif iy > ny - 1:
            iy = ny - 1
        if iz < 0:
            iz = 0
        elif iz > nz - 1:
            iz = nz - 1
        idx = ix + nx * (iy + ny * iz)
        # Active mask check
        if active is not None and not active[idx]:
            valid[i] = False
            flat[i] = 0
        else:
            valid[i] = True
            flat[i] = idx
    return valid, flat
