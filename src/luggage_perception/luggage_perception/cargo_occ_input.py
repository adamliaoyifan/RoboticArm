"""OCC-only preparation. Never mutate a shared perception frame or map.

Hits are cropped before first-point downsampling. Rays retain the original
uncropped downsample order, so the mapper's ray budget sees the same evidence.
"""
from dataclasses import dataclass
from types import MappingProxyType
import math
import time
import numpy as np
from luggage_description.container_geometry_batch import contains_points
from luggage_description.container_geometry import EPS
from luggage_perception.luggage_box_estimator import voxel_downsample


@dataclass(frozen=True)
class OccInputPolicy:
    minimum_z: float | None = None
    center: tuple = (0., 0., 0.)
    yaw: float = 0.


@dataclass(frozen=True)
class PreparedOccObservation:
    hit_points: np.ndarray
    ray_points: np.ndarray
    counts: object
    timings: object


def prepare_occ_observation(points, origin, hull, resolution, policy):
    start = time.perf_counter()
    raw = np.asarray(points, dtype=float).reshape((-1,3))
    finite = raw[np.isfinite(raw).all(axis=1)]
    if policy.minimum_z is not None and not math.isfinite(policy.minimum_z):
        raise ValueError('minimum_z must be finite')
    if origin is not None and (len(origin) != 3 or not np.isfinite(origin).all()):
        raise ValueError('origin must be finite XYZ')
    height = finite if policy.minimum_z is None else finite[finite[:,2] >= policy.minimum_z]
    # Legacy algorithm fixtures may use a translated/yawed mapper. Product
    # nodes are already container_link, center=(0,0,hull mid-Z), yaw=0.
    local = height-np.asarray(policy.center)
    c, s = math.cos(policy.yaw), math.sin(policy.yaw)
    container = np.column_stack((c*local[:,0]+s*local[:,1],
                                -s*local[:,0]+c*local[:,1],
                                local[:,2]+(hull.floor_z+hull.ceiling_z)*.5))
    inside = contains_points(hull,container)
    aabb = (np.abs(container[:,0]) <= hull.half_x+EPS) & (np.abs(container[:,1]) <= hull.half_y+EPS)
    aabb &= (container[:,2] >= hull.floor_z-EPS) & (container[:,2] <= hull.ceiling_z+EPS)
    cropped = time.perf_counter()
    # Keep this stream for legacy n_kept and ray sampling; crop does not spend
    # discarded ray budget on new rays. Reuse the result when every point is in.
    rays = voxel_downsample(height,resolution)
    hits = rays if inside.all() else voxel_downsample(height[inside],resolution)
    rays.setflags(write=False)
    hits.setflags(write=False)
    end = time.perf_counter()
    return PreparedOccObservation(hits,rays,MappingProxyType(dict(
        raw=len(raw),finite=len(finite),height_kept=len(height),inside_aabb=int(aabb.sum()),
        inside_hull=int(inside.sum()),hit_downsampled=len(hits),ray_candidates=len(rays))),
        MappingProxyType(dict(crop_s=cropped-start,downsample_s=end-cropped,prepare_s=end-start)))
