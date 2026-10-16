"""Immutable container-frame static mesh queries for measured depth admission.

The owner must supply an authoritative scene-bound mesh; this module does not
fetch models, transform frames, publish a map, or infer free space from geometry.
A bounded AABB tree accelerates first-positive-hit queries without dropping faces.
"""
from dataclasses import dataclass
import hashlib
import time
import numpy as np
from luggage_perception.depth_physical_boundary import BoundaryHit


@dataclass(frozen=True)
class _Node:
    low: np.ndarray
    high: np.ndarray
    indices: object = None
    left: object = None
    right: object = None


class StaticBoundaryMesh:
    MAX_TRIANGLES = 20000
    LEAF_TRIANGLES = 32

    def __init__(self, triangles, *, scene_revision, expected_sha256):
        if not scene_revision or not expected_sha256:
            raise ValueError('explicit scene and mesh identity required')
        values = np.array(triangles, dtype='<f8', order='C', copy=True)
        if (values.ndim != 3 or values.shape[1:] != (3,3)
                or not 1 <= len(values) <= self.MAX_TRIANGLES or not np.isfinite(values).all()):
            raise ValueError('bounded finite triangle array required')
        digest = hashlib.sha256(values.tobytes()).hexdigest()
        if digest != expected_sha256:
            raise ValueError('static mesh identity mismatch')
        values.setflags(write=False)
        self._triangles = values
        self._scene_revision = scene_revision
        self._mesh_sha256 = digest
        self._low = values.min(axis=1)
        self._high = values.max(axis=1)
        self._centres = (self._low+self._high)*.5
        self._root = self._build(np.arange(len(values)))

    @property
    def scene_revision(self):
        return self._scene_revision

    @property
    def mesh_sha256(self):
        return self._mesh_sha256

    def _build(self, indices):
        low = self._low[indices].min(axis=0)
        high = self._high[indices].max(axis=0)
        if len(indices) <= self.LEAF_TRIANGLES:
            return _Node(low,high,indices=indices)
        axis = int(np.argmax(np.ptp(self._centres[indices],axis=0)))
        ordered = indices[np.argsort(self._centres[indices,axis],kind='stable')]
        middle = len(ordered)//2
        return _Node(low,high,left=self._build(ordered[:middle]),right=self._build(ordered[middle:]))

    @staticmethod
    def _entry(node, origin, direction, nearest):
        low,high = 0.,nearest
        for axis in range(3):
            if abs(direction[axis]) < 1e-15:
                if origin[axis] < node.low[axis]-1e-10 or origin[axis] > node.high[axis]+1e-10:
                    return None
                continue
            a = (node.low[axis]-origin[axis])/direction[axis]
            b = (node.high[axis]-origin[axis])/direction[axis]
            low,high = max(low,min(a,b)),min(high,max(a,b))
            if low > high+1e-10:
                return None
        return low

    def _leaf_hit(self, indices, origin, direction):
        triangles = self._triangles[indices]
        a,b = triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]
        cross = np.cross(direction,b)
        determinant = np.einsum('ij,ij->i',a,cross)
        mask = np.abs(determinant)>1e-10
        inverse = np.zeros_like(determinant)
        inverse[mask] = 1./determinant[mask]
        delta = origin-triangles[:,0]
        u = np.einsum('ij,ij->i',delta,cross)*inverse
        q = np.cross(delta,a)
        v = np.dot(q,direction)*inverse
        distance = np.einsum('ij,ij->i',b,q)*inverse
        mask &= (u>=-1e-9)&(v>=-1e-9)&(u+v<=1.+1e-9)&(distance>1e-9)
        return float(distance[mask].min()) if mask.any() else float('inf')

    def first_hit(self, origin, measured_end, *, deadline, clock=time.monotonic):
        """Return identity-bound hit or None; timeout raises, never means clear."""
        origin,end = np.asarray(origin,dtype=float),np.asarray(measured_end,dtype=float)
        if origin.shape!=(3,) or end.shape!=(3,) or not np.isfinite([origin,end]).all():
            raise ValueError('finite three-coordinate ray required')
        if not np.isfinite(deadline):
            raise ValueError('finite deadline required')
        direction = end-origin
        length = np.linalg.norm(direction)
        if length<=1e-12:
            raise ValueError('nonzero ray required')
        direction = direction/length
        nearest = float('inf')
        pending = [self._root]
        while pending:
            if clock()>=deadline:
                raise TimeoutError('static boundary query deadline')
            node = pending.pop()
            if self._entry(node,origin,direction,nearest) is None:
                continue
            if node.indices is not None:
                nearest = min(nearest,self._leaf_hit(node.indices,origin,direction))
            else:
                children = [(self._entry(child,origin,direction,nearest),child)
                            for child in (node.left,node.right)]
                pending.extend(child for entry,child in sorted(
                    [(entry,child) for entry,child in children if entry is not None],
                    key=lambda pair:pair[0],reverse=True))
        if not np.isfinite(nearest):
            return None
        return BoundaryHit(tuple(origin),tuple(direction),nearest,self.scene_revision,self.mesh_sha256)
