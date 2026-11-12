"""Reuse an admitted placement only with unchanged local support evidence.

Coordinates and source cell origins are container-absolute. This is a proof
reuse gate, not candidate search: changed support requires a new request.
"""
import math

from luggage_packing.placement_volume_query import KnownFreeVolume, evidence_from_volume


def recheck_placement(before, after, volume, *, center_container, box_size, yaw_rad):
    """Return None if the original placement proof survives, otherwise a reason.

    The caller has admitted both complete surface/volume snapshots. Exact local
    state/height/confidence equality preserves the original support decision;
    remote observations may change. No measurement tolerance is guessed here.
    """
    fixed = ('geometry_hash', 'scene_revision', 'frame_id', 'resolution_m',
             'origin_xy_container', 'floor_z_container', 'ceiling_z_container')
    if any(before[key] != after[key] for key in fixed):
        return 'REFRESH_PLACEMENT_GRID_CHANGED'
    shapes = [tuple(len(row) for row in record['state']) for record in (before, after)]
    if shapes[0] != shapes[1] or not shapes[0] or len(set(shapes[0])) != 1:
        return 'REFRESH_PLACEMENT_GRID_CHANGED'
    if (not math.isfinite(yaw_rad) or abs(math.remainder(yaw_rad, math.pi / 2)) > 1e-6
            or len(center_container) != 3 or len(box_size) != 3
            or not all(map(math.isfinite, (*center_container, *box_size))) or min(box_size) <= 0):
        return 'REFRESH_PLACEMENT_GEOMETRY_INVALID'
    footprint = tuple(box_size[:2])
    if round(yaw_rad / (math.pi / 2)) % 2:
        footprint = footprint[::-1]
    r = after['resolution_m']
    nx, ny = len(shapes[1]), shapes[1][0]
    if (volume is None or volume.resolution_m != r
            or tuple(volume.origin_container) != (*after['origin_xy_container'], after['floor_z_container'])
            or tuple(volume.dimensions_xyz[:2]) != (nx, ny)):
        return 'REFRESH_PLACEMENT_VOLUME_GRID_MISMATCH'
    bounds = []
    for center, extent, origin, count in zip(center_container, footprint,
                                             after['origin_xy_container'], (nx, ny)):
        # Same dimensionless roundoff rule as KnownFreeVolume. Every source
        # cell with positive overlap, including boundary slivers, participates.
        lo = math.floor((center - extent / 2 - origin) / r + 1e-9)
        hi = math.ceil((center + extent / 2 - origin) / r - 1e-9)
        if lo < 0 or hi > count or lo >= hi:
            return 'REFRESH_PLACEMENT_SUPPORT_OUTSIDE'
        bounds.append((lo, hi))
    for x in range(*bounds[0]):
        for y in range(*bounds[1]):
            if any(before[key][x][y] != after[key][x][y]
                   for key in ('state', 'surface_z_container', 'confidence')):
                return 'REFRESH_PLACEMENT_SUPPORT_CHANGED'
    verdict = KnownFreeVolume(evidence_from_volume(volume))(
        dict(center_container=center_container, size=box_size, footprint=footprint))
    return 'REFRESH_PLACEMENT_' + verdict.upper() if verdict else None
