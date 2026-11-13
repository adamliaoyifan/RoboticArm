"""Geometric admission of a selected continuous, container-absolute box pose.

Search grids propose centers. This predicate checks the supplied center without
snapping it, and is also used by the production search query. Volume evidence
must have been bound to the same surface by the caller.
"""
import math
from collections import OrderedDict
import numpy as np

from luggage_packing.placement_solver import (
    _default_params, _resolved_hull_margin, _hull_legal,
    placement_constraint_verdict,
)
from luggage_description.container_geometry import normalize_descriptor


class SelectedPoseAdmission:
    def __init__(self, surface, box_size, *, volume_verifier, params=None, placed_aabbs=()):
        self.surface, self.size = surface, tuple(map(float, box_size))
        if len(self.size) != 3 or min(self.size) <= 0 or not all(map(math.isfinite, self.size)):
            raise ValueError('positive finite box dimensions required')
        self.hull = normalize_descriptor(surface['geometry_descriptor'])
        self.cfg = _default_params()
        self.cfg.update(params or {})
        self.cfg['hull_margin'] = _resolved_hull_margin(self.cfg, surface['resolution'])
        self.volume, self.placed = volume_verifier, tuple(placed_aabbs)
        self.heights=np.asarray(surface['height'],dtype=float)
        self.unknown=np.asarray(surface['state']) == 'unknown'
        self.unknown_prefix=self._prefix(self.unknown)
        self.layers=OrderedDict()

    @staticmethod
    def _prefix(mask):
        return np.pad(np.asarray(mask,dtype=float),((1,0),(1,0))).cumsum(0).cumsum(1)

    @staticmethod
    def _count(prefix, first, stop):
        x0,y0=first; x1,y1=stop
        return prefix[x1,y1]-prefix[x0,y1]-prefix[x1,y0]+prefix[x0,y0]

    @staticmethod
    def _area(prefix, lower, upper):
        # Integral of a piecewise-constant native cell field at continuous
        # coordinates. Bilinear interpolation includes exact partial-cell area.
        def primitive(x,y):
            nx,ny=prefix.shape[0]-1,prefix.shape[1]-1
            x,y=min(max(x,0.),nx),min(max(y,0.),ny)
            ix,iy=min(math.floor(x),nx-1),min(math.floor(y),ny-1)
            fx,fy=x-ix,y-iy
            return ((1-fx)*(1-fy)*prefix[ix,iy]+fx*(1-fy)*prefix[ix+1,iy]
                +(1-fx)*fy*prefix[ix,iy+1]+fx*fy*prefix[ix+1,iy+1])
        x0,y0=lower;x1,y1=upper
        return primitive(x1,y1)-primitive(x0,y1)-primitive(x1,y0)+primitive(x0,y0)

    def _layer(self,bottom):
        key=round(bottom,12)  # floating arithmetic cache key, never a pose snap
        if key not in self.layers:
            contact=(self.heights <= bottom+1e-9) & (self.heights >= bottom-self.cfg['support_tol']-1e-9)
            self.layers[key]=(contact,self._prefix(contact),self._prefix(self.heights > bottom+1e-9))
            if len(self.layers)>32:self.layers.popitem(last=False)
        self.layers.move_to_end(key)
        return self.layers[key]

    def candidate(self, center, yaw):
        center = tuple(map(float, center))
        yaw = float(yaw)
        if (len(center) != 3 or not all(map(math.isfinite, (*center, yaw)))
                or abs(math.remainder(yaw, math.pi/2)) > 1e-6):
            raise ValueError('finite center and directed quarter turn required')
        rotated = round(yaw/(math.pi/2)) % 2
        footprint = self.size[:2][::-1] if rotated else self.size[:2]
        return dict(center_container=center, size=self.size, footprint=footprint,
                    heading_theta=yaw, box_yaw=yaw, detail={})

    def __call__(self, candidate):
        cfg, surface = self.cfg, self.surface
        center = candidate['center_container']
        footprint, height = candidate['footprint'], candidate['size'][2]
        bottom = center[2] - height/2 - self.hull.floor_z
        if not _hull_legal(center, footprint, height, self.hull, cfg['hull_margin'])[0]:
            return 'outside_hull'
        if self.hull.ceiling_z - (center[2]+height/2) < cfg['clearance_margin'] - 1e-9:
            return 'insufficient_clearance'
        resolution = float(surface['resolution'])
        nx, ny = int(surface['nx']), int(surface['ny'])
        origin = (-nx*resolution/2, -ny*resolution/2)
        lo = [center[a]-footprint[a]/2 for a in range(2)]
        hi = [center[a]+footprint[a]/2 for a in range(2)]
        first = [math.floor((lo[a]-origin[a])/resolution+1e-9) for a in range(2)]
        stop = [math.ceil((hi[a]-origin[a])/resolution-1e-9) for a in range(2)]
        if any(first[a] < 0 or stop[a] > (nx,ny)[a] for a in range(2)):
            return 'outside_hull'
        contact, support_prefix, penetration_prefix=self._layer(bottom)
        if self._count(penetration_prefix,first,stop)>0:
            return 'edge_occupied_overlap'
        if abs(bottom)>1e-9 and self._count(self.unknown_prefix,first,stop)>0:
            return 'unknown_above_floor'
        lower=[(lo[a]-origin[a])/resolution for a in range(2)]
        upper=[(hi[a]-origin[a])/resolution for a in range(2)]
        total=math.prod(upper[a]-lower[a] for a in range(2))
        support=self._area(support_prefix,lower,upper)
        if total<=0 or support/total < cfg['min_support_ratio']-1e-9:
            return 'insufficient_support'
        region=(slice(first[0],stop[0]),slice(first[1],stop[1]))
        contact_heights=self.heights[region][contact[region]]
        if (contact_heights.size and np.ptp(contact_heights)/max(footprint)
                > cfg['max_support_tilt_ratio']+1e-9):
            return 'support_tilt_unsupported'
        reason, _ = placement_constraint_verdict(candidate, surface['inner_size'][2],
            placed_aabbs=self.placed, inner_size=surface['inner_size'],
            smallest_size=self.size, hull=self.hull, floor_z=self.hull.floor_z,
            same_layer_clearance_m=cfg.get('same_layer_clearance_m') or 0.,
            support_tol=cfg['support_tol'], corridor_as_ranking=True)
        if reason: return reason
        if self.volume is None: return 'volume_evidence_unavailable'
        try:
            return self.volume(candidate)
        except Exception:
            return 'volume_evidence_unavailable'
