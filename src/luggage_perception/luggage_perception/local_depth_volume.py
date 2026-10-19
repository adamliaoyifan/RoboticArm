"""Bounded local measured volume kernel; not wired to the online map owner.

Uses the existing log-odds policy. Every frame is transactional: invalid input or
budget exhaustion leaves state and stamp unchanged. Geometry supplies no misses.
"""
import itertools
import math
import time
import numpy as np
from luggage_description.container_geometry import contains_point
from luggage_perception.depth_physical_boundary import admit_physical_depth_ray
from luggage_perception.voxel_log_odds import LogOddsGrid, UNKNOWN


class LocalDepthVolume:
    MAX_CELLS = 1000000
    MAX_RAYS = 2000

    def __init__(self, hull, boundary_mesh, lower, upper, resolution, *, occupancy_params=None):
        self.hull = hull
        self.mesh = boundary_mesh
        self.lower = np.array(lower,dtype=float)
        self.upper = np.array(upper,dtype=float)
        self.resolution = float(resolution)
        if (self.lower.shape!=(3,) or self.upper.shape!=(3,)
                or not np.isfinite([self.lower,self.upper]).all()
                or not math.isfinite(self.resolution) or self.resolution<=0
                or np.any(self.upper<=self.lower)):
            raise ValueError('finite positive local volume bounds required')
        counts = (self.upper-self.lower)/self.resolution
        if np.any(np.abs(counts-np.rint(counts))>1e-8):
            raise ValueError('local bounds must span whole voxels')
        self.dimensions = tuple(int(v) for v in np.rint(counts))
        total = math.prod(self.dimensions)
        if not 1<=total<=self.MAX_CELLS or min(self.dimensions)<1:
            raise ValueError('local volume cell budget exceeded')
        # A convex-hull-contained ROI has no inactive/cropped voxels to confuse
        # the consumer. Reject unsupported ROIs rather than imply free outside.
        if not all(contains_point(hull,corner) for corner in itertools.product(
                *zip(self.lower,self.upper))):
            raise ValueError('local ROI must be fully inside usable hull')
        self.lower.setflags(write=False)
        self.upper.setflags(write=False)
        self._occupancy = LogOddsGrid(total,**(occupancy_params or {}))
        self._stamp_ns = None
        self._revision = 0

    def _index(self, point):
        if np.any(point<self.lower) or np.any(point>=self.upper):
            return None
        x,y,z = np.floor((point-self.lower)/self.resolution).astype(int)
        nx,ny,_ = self.dimensions
        return int(x+nx*(y+ny*z))

    def _misses(self, origin, end, deadline, clock):
        direction = end-origin
        near,far = 0.,1.
        for axis in range(3):
            if abs(direction[axis])<1e-15:
                if not self.lower[axis]<=origin[axis]<self.upper[axis]:return set()
                continue
            a=(self.lower[axis]-origin[axis])/direction[axis]
            b=(self.upper[axis]-origin[axis])/direction[axis]
            near,far=max(near,min(a,b)),min(far,max(a,b))
            if near>=far:return set()
        length=np.linalg.norm(direction)*(far-near)
        steps=max(1,int(math.ceil(length/(self.resolution*.5))))
        indices=set()
        for step in range(steps):
            if clock()>=deadline:raise TimeoutError('local volume frame deadline')
            point=origin+direction*(near+(far-near)*step/steps)
            index=self._index(point)
            if index is not None:indices.add(index)
        return indices

    def integrate(self, points, origin, *, stamp_ns, scene_revision, mesh_sha256,
                  deadline, clock=time.monotonic):
        if (scene_revision!=self.mesh.scene_revision or mesh_sha256!=self.mesh.mesh_sha256):
            raise ValueError('local volume identity mismatch')
        if type(stamp_ns) is not int or stamp_ns<0 or (self._stamp_ns is not None and stamp_ns<=self._stamp_ns):
            raise ValueError('new exact acquisition stamp required')
        points=np.asarray(points,dtype=float)
        origin=np.asarray(origin,dtype=float)
        if (points.ndim!=2 or points.shape[1:]!=(3,) or not 1<=len(points)<=self.MAX_RAYS
                or origin.shape!=(3,) or not np.isfinite(points).all() or not np.isfinite(origin).all()
                or not math.isfinite(deadline)):
            raise ValueError('bounded finite measured frame required')
        misses,hits=set(),set()
        admitted=0
        for point in points:
            if clock()>=deadline:raise TimeoutError('local volume frame deadline')
            hit=self.mesh.first_hit(origin,point,deadline=deadline,clock=clock)
            ray=admit_physical_depth_ray(self.hull,origin,point,math.sqrt(3)*self.resolution,
                hit,scene_revision=scene_revision,mesh_sha256=mesh_sha256)
            if ray is None:continue
            admitted+=1
            misses.update(self._misses(origin,np.array(ray.free_end),deadline,clock))
            if ray.obstacle is not None:
                index=self._index(np.array(ray.obstacle))
                if index is not None:hits.add(index)
        if clock()>=deadline:raise TimeoutError('local volume frame deadline')
        misses.difference_update(hits)
        for index in misses:self._occupancy.apply_miss(index)
        for index in hits:self._occupancy.apply_hit(index)
        self._stamp_ns=stamp_ns
        self._revision+=1
        return dict(admitted_rays=admitted,miss_cells=len(misses),hit_cells=len(hits),
                    revision=self._revision,stamp_ns=stamp_ns)

    def snapshot(self):
        return dict(scene_revision=self.mesh.scene_revision,mesh_sha256=self.mesh.mesh_sha256,
                    geometry_hash=self.hull.geometry_hash,lower=self.lower.tolist(),
                    dimensions=self.dimensions,resolution_m=self.resolution,
                    acquisition_stamp_ns=self._stamp_ns,revision=self._revision,
                    states=tuple(self._occupancy.states()),confirmed=False)

    def state_at(self, point):
        index=self._index(np.asarray(point,dtype=float))
        return UNKNOWN if index is None else self._occupancy.state_at(index)
