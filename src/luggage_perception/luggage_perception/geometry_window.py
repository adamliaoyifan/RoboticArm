"""Bounded stationary, same-instance geometry history; no shape/catalog prior.

The median reports repeatability. Lateral extent and observed bottom are retained
for the stationary instance. Nominal top stays at the current observed plane. It cannot establish
absolute accuracy or turn an unobserved lid edge into a measured body edge.
Observed suction support is owned by the acquisition and is never fused here.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading
import numpy as np


@dataclass(frozen=True)
class WindowGeometry:
    center: tuple
    dimensions: tuple
    stamps_ns: tuple
    median_dimensions: tuple
    spread_dimensions: tuple
    observed_top_range_world_m: tuple

    def diagnostic(self):
        return dict(acquisition_stamps_ns=self.stamps_ns, median_dimensions_m=self.median_dimensions,
                    observed_spread_m=self.spread_dimensions, body_bound_m=self.dimensions,
                    observed_top_range_world_m=self.observed_top_range_world_m,
                    nominal_top='current_observation',accuracy_scope='repeatability_only', support_fusion=False)


class StationaryGeometryWindow:
    def __init__(self, *, capacity=5, max_span_ns=1_000_000_000):
        if type(capacity) is not int or not 1 <= capacity <= 32 or max_span_ns <= 0:
            raise ValueError('bounded geometry history required')
        self._lock=threading.RLock()
        self.rows = deque(maxlen=capacity)
        self.max_span_ns, self.identity = int(max_span_ns), None
        self.retained_bounds = None
        self.anchor_rotation = None

    def reset(self):
        with self._lock: self._reset()

    def _reset(self):
        self.rows.clear()
        self.identity = None
        self.retained_bounds = None
        self.anchor_rotation = None

    def observe(self, identity, stamp_ns, center, yaw, dimensions, *, stationary=True):
        with self._lock:
            return self._observe(identity,stamp_ns,center,yaw,dimensions,stationary=stationary)

    def _observe(self, identity, stamp_ns, center, yaw, dimensions, *, stationary):
        center, dimensions = np.asarray(center,dtype=float), np.asarray(dimensions,dtype=float)
        if (center.shape != (3,) or dimensions.shape != (3,) or min(dimensions) <= 0
                or not np.isfinite(np.r_[center,dimensions,yaw]).all() or stamp_ns < 0):
            raise ValueError('valid measured geometry required')
        if not stationary:
            self.reset()
            raise ValueError('STATIONARY_GEOMETRY_REQUIRED')
        if identity != self.identity:
            self.reset()
            self.identity = identity
        elif self.rows and stamp_ns < self.rows[-1][0]:
            raise ValueError('OUT_OF_ORDER_GEOMETRY_ACQUISITION')
        elif self.rows and stamp_ns == self.rows[-1][0]:
            _, old_center, old_yaw, old_dimensions = self.rows[-1]
            if (not np.array_equal(center,old_center) or yaw != old_yaw
                    or not np.array_equal(dimensions,old_dimensions)):
                raise ValueError('CONFLICTING_GEOMETRY_ACQUISITION')
        while self.rows and stamp_ns-self.rows[0][0] > self.max_span_ns:
            self.rows.popleft()
        if not self.rows or stamp_ns != self.rows[-1][0]:
            self.rows.append((int(stamp_ns),center.copy(),float(yaw),dimensions.copy()))
        def rotation(angle):
            c,s = math.cos(angle),math.sin(angle)
            return np.array(((c,-s,0.),(s,c,0.),(0.,0.,1.)))
        current_rotation = rotation(yaw)
        corners = []
        for _, xyz, angle, size in self.rows:
            local = np.array([(x,y,z) for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size
            corners.extend((local@rotation(angle).T+xyz-center)@current_rotation)
        # A bounded history supplies repeatability statistics. A separate
        # constant-size box in the first measured yaw retains body extent for
        # this stationary instance, so a later occluded window cannot shrink it.
        world = np.asarray(corners)@current_rotation.T+center
        if self.anchor_rotation is None: self.anchor_rotation=current_rotation.copy()
        anchored=world@self.anchor_rotation
        lower_world, upper_world = anchored.min(axis=0),anchored.max(axis=0)
        if self.retained_bounds is not None:
            lower_world=np.minimum(lower_world,self.retained_bounds[0])
            upper_world=np.maximum(upper_world,self.retained_bounds[1])
        self.retained_bounds=(lower_world,upper_world)
        world_corners=np.array([(x,y,z) for x in (lower_world[0],upper_world[0])
            for y in (lower_world[1],upper_world[1]) for z in (lower_world[2],upper_world[2])])
        world_corners=world_corners@self.anchor_rotation.T
        corners=(world_corners-center)@current_rotation
        lower, upper = corners.min(axis=0), corners.max(axis=0)
        # The nominal box top must stay on the current observed top plane:
        # contact is not moved to a temporal maximum. Retain the observed
        # bottom and report top variation as measurement evidence, not support.
        top_range=(min(row[1][2]+row[3][2]/2 for row in self.rows),
                   max(row[1][2]+row[3][2]/2 for row in self.rows))
        upper[2]=dimensions[2]/2
        size = upper-lower
        xyz = center+current_rotation@((lower+upper)/2)
        observed = np.array([row[3] for row in self.rows])
        return WindowGeometry(tuple(xyz),tuple(size),tuple(row[0] for row in self.rows),
                              tuple(np.median(observed,axis=0)),tuple(np.ptp(observed,axis=0)),top_range)
