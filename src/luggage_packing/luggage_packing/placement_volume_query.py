"""Same-source, container-absolute box-volume admission with constant query work.

This module consumes admitted grids; it does not establish snapshot identity or
resample cells. All positive-volume intersections must be observed free. Face
contact is allowed, occupied-cell erosion is deliberately absent.
"""
import math

import numpy as np

from luggage_packing.placement_solver import (
    REASON_VOLUME_OCCUPIED, REASON_VOLUME_UNKNOWN, REASON_VOLUME_UNVERIFIED,
)


class KnownFreeVolume:
    """Two integral volumes, built once per immutable XYZ evidence snapshot."""

    def __init__(self, evidence):
        self.error = REASON_VOLUME_UNVERIFIED
        if not evidence or evidence.get('error'):
            return
        try:
            self.resolution = float(evidence['resolution'])
            self.dimensions = tuple(int(evidence[key]) for key in ('nx', 'ny', 'nz'))
            self.origin = (*map(float, evidence['origin_xy']), float(evidence['floor_z']))
            if (self.resolution <= 0 or not math.isfinite(self.resolution)
                    or len(self.origin) != 3 or not all(map(math.isfinite, self.origin))
                    or min(self.dimensions) <= 0):
                return
            raw = evidence['states']
            states = (np.frombuffer(raw, dtype=np.uint8) if isinstance(raw, bytes)
                      else np.asarray(raw)).reshape(tuple(reversed(self.dimensions)))
            # Inactive cells are never authorization, even if their state byte
            # happened to retain an earlier free value.
            active = evidence.get('active')
            if active is None:
                active = np.ones(states.shape, dtype=bool)
            else:
                active = np.asarray(active, dtype=bool).reshape(states.shape)
            self.unknown = self._prefix(~active | ((states != 1) & (states != 2)))
            self.occupied = self._prefix(active & (states == 2))
            self.error = None
        except (KeyError, TypeError, ValueError, OverflowError):
            return

    @staticmethod
    def _prefix(mask):
        prefix = np.pad(mask.astype(np.int32), ((1, 0),) * 3)
        for axis in range(3):
            np.cumsum(prefix, axis=axis, dtype=np.int32, out=prefix)
        return prefix

    @staticmethod
    def _count(prefix, lower, upper):
        x0, y0, z0 = lower
        x1, y1, z1 = upper
        return int(prefix[z1, y1, x1] - prefix[z0, y1, x1]
                   - prefix[z1, y0, x1] - prefix[z1, y1, x0]
                   + prefix[z0, y0, x1] + prefix[z0, y1, x0]
                   + prefix[z1, y0, x0] - prefix[z0, y0, x0])

    def __call__(self, candidate):
        if self.error:
            return self.error
        center = tuple(map(float, candidate['center_container']))
        size = (*map(float, candidate['footprint']), float(candidate['size'][2]))
        if (len(center) != 3 or len(size) != 3 or min(size) <= 0
                or not all(map(math.isfinite, (*center, *size)))):
            return REASON_VOLUME_UNVERIFIED
        lower, upper = [], []
        for value, extent, origin, count in zip(center, size, self.origin, self.dimensions):
            lo = (value - extent / 2 - origin) / self.resolution
            hi = (value + extent / 2 - origin) / self.resolution
            # Dimensionless roundoff allowance only; no physical padding or
            # permitted penetration. A face exactly on a voxel boundary does
            # not intersect the next cell with positive volume.
            first, stop = math.floor(lo + 1e-9), math.ceil(hi - 1e-9)
            if first < 0 or stop > count or first >= stop:
                return REASON_VOLUME_UNVERIFIED
            lower.append(first)
            upper.append(stop)
        if self._count(self.unknown, lower, upper):
            return REASON_VOLUME_UNKNOWN
        if self._count(self.occupied, lower, upper):
            return REASON_VOLUME_OCCUPIED
        return None


def evidence_from_volume(volume):
    """Adapt an already admitted O/G volume without changing resolution/datum."""
    if volume is None:
        return None
    nx, ny, nz = volume.dimensions_xyz
    return dict(resolution=volume.resolution_m, nx=nx, ny=ny, nz=nz,
                origin_xy=volume.origin_container[:2], floor_z=volume.origin_container[2],
                states=volume.states, active=volume.active)
