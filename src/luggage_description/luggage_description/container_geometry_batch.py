"""Read-only batched queries over the authoritative container descriptor.

Point inclusion reproduces contains_point (including its wall_z EPS branch).
Line clipping is geometry only, never permission to integrate a depth return.
"""
import numpy as np
from luggage_description.container_geometry import EPS


def contains_points(hull, points):
    points = np.asarray(points, dtype=float).reshape((-1, 3))
    x, y, z = points.T
    aabb = (np.isfinite(points).all(axis=1)
            & (x >= -hull.half_x - EPS) & (x <= hull.half_x + EPS)
            & (y >= -hull.half_y - EPS) & (y <= hull.half_y + EPS)
            & (z >= hull.floor_z - EPS) & (z <= hull.ceiling_z + EPS))
    inside = aabb.copy()
    c = hull.chamfer
    if c is not None:
        slope = (c.wall_y - c.floor_y) / (c.wall_z - hull.floor_z)
        limit = np.where(z >= c.wall_z - EPS, hull.half_y,
                         np.minimum(hull.half_y, slope*z + c.floor_y - slope*hull.floor_z))
        inside &= y <= limit + EPS
    return inside


def segment_hull_intervals(hull, origins, ends):
    """Closed mathematical hull intersection; (near, far, valid), t in [0,1].

    A zero-length point inside has [0,1]; nonfinite lines are invalid.
    This exact plane query is deliberately not used to shrink legacy voxel
    ray evidence, whose boundary semantics are voxel-center based.
    """
    ends = np.asarray(ends, dtype=float).reshape((-1, 3))
    origins = np.broadcast_to(np.asarray(origins, dtype=float), ends.shape)
    direction = ends-origins
    near, far = np.zeros(len(ends)), np.ones(len(ends))
    valid = np.isfinite(ends).all(axis=1) & np.isfinite(origins).all(axis=1)
    planes = [((1,0,0),hull.half_x),((-1,0,0),hull.half_x),
              ((0,1,0),hull.half_y),((0,-1,0),hull.half_y),
              ((0,0,1),hull.ceiling_z),((0,0,-1),-hull.floor_z)]
    c = hull.chamfer
    if c is not None:
        slope = (c.wall_y-c.floor_y)/(c.wall_z-hull.floor_z)
        planes.append(((0,1,-slope),c.floor_y-slope*hull.floor_z))
    for normal, bound in planes:
        # Elementwise sums avoid a BLAS thread pool for an Nx3 query.
        denominator = sum(direction[:, i]*normal[i] for i in range(3))
        numerator = bound-sum(origins[:, i]*normal[i] for i in range(3))
        parallel = denominator == 0
        valid &= ~(parallel & (numerator < 0))
        crossing = np.divide(numerator, denominator, out=np.zeros_like(near), where=~parallel)
        near = np.where(denominator < 0, np.maximum(near,crossing),near)
        far = np.where(denominator > 0, np.minimum(far,crossing),far)
    valid &= near <= far
    return near, far, valid
