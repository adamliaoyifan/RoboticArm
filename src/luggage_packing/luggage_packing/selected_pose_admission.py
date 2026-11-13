"""Geometric admission of a selected continuous, container-absolute box pose.

Search grids propose centers. This predicate checks the supplied center without
snapping it, and is also used by the production search query. Volume evidence
must have been bound to the same surface by the caller.
"""
import math

from luggage_packing.placement_solver import (
    _default_params, _resolved_hull_margin, _hull_legal,
    placement_constraint_verdict,
)
from luggage_description.container_geometry import normalize_descriptor
from luggage_packing.horizontal_contact_support import HorizontalContactSupport


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
        self.support = HorizontalContactSupport(surface)
        self.origin_xy_container = self.support.origin_xy_container

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
        return self._check(candidate)

    def explain(self, candidate):
        """Bounded diagnostics from the same checks; missing fields were not reached.

        Heights and coordinates explicitly state their datum. No grid arrays or
        observations are copied; the caller retains the authoritative snapshot.
        """
        detail = dict(schema='selected_pose_admission_v2',
            center_container=list(candidate['center_container']),
            nominal_dimensions_wdh=list(candidate['size']),
            heading_container_rad=candidate['heading_theta'],
            resolution_m=float(self.surface['resolution']),
            grid_origin_xy_container=list(self.origin_xy_container),
            grid_origin_source='declared' if 'origin_local' in self.surface else 'legacy_grid_centre',
            min_support_ratio=self.cfg['min_support_ratio'])
        reason = self._check(candidate, detail)
        detail.update(accepted=reason is None, reason=reason)
        return detail

    def _check(self, candidate, detail=None):
        cfg, surface = self.cfg, self.surface
        center = candidate['center_container']
        footprint, height = candidate['footprint'], candidate['size'][2]
        bottom = center[2] - height/2 - self.hull.floor_z
        if detail is not None:
            detail['bottom_z_floor_relative_container_m'] = bottom
        if not _hull_legal(center, footprint, height, self.hull, cfg['hull_margin'])[0]:
            return 'outside_hull'
        if self.hull.ceiling_z - (center[2]+height/2) < cfg['clearance_margin'] - 1e-9:
            return 'insufficient_clearance'
        support = self.support.evaluate(center[:2], footprint, bottom, cfg['min_support_ratio'])
        if detail is not None:
            detail.update({key: value for key, value in support.items() if key != 'reason'})
        if support['reason']:
            return support['reason']
        reason, _ = placement_constraint_verdict(candidate, surface['inner_size'][2],
            placed_aabbs=self.placed, inner_size=surface['inner_size'],
            smallest_size=self.size, hull=self.hull, floor_z=self.hull.floor_z,
            same_layer_clearance_m=cfg.get('same_layer_clearance_m') or 0.,
            support_tol=cfg['support_tol'], corridor_as_ranking=True)
        if reason: return reason
        if self.volume is None: return 'volume_evidence_unavailable'
        try:
            if detail is not None:
                detail['volume_checked'] = True
            return self.volume(candidate)
        except Exception:
            return 'volume_evidence_unavailable'
