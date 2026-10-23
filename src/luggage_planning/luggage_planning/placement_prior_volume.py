"""Admit exact owner volume and preserve every unknown/occupied active voxel."""
import json
import math
from luggage_description.cargo_grid_contract import MAX_VOLUME_CELLS


def blocked_volume_runs(response, confirmed_surface):
    """Return container-frame (centre, size) boxes; merge only vertical runs.

    Free gaps are never bridged. Inactive hull cells are omitted, not certified
    free; static container geometry remains mandatory in the private scene.
    """
    surface = confirmed_surface.record()
    confirmation = json.loads(confirmed_surface.confirmation_json)
    if (not response.success or response.frame_id != 'container_link' or
            response.scene_revision != surface['scene_revision'] or
            response.geometry_hash != surface['geometry_hash'] or
            response.map_revision != surface['map_revision'] or
            response.acquisition_stamp_ns != confirmation['acquisition_stamp_ns'] or
            response.map_source != 'measured' or not response.occupancy_confirmed or
            response.has_pending_changes):
        raise ValueError('volume/surface identity or confirmation mismatch')
    nx, ny, nz = response.dimensions_xyz
    if min(nx,ny,nz) <= 0 or nx*ny*nz > MAX_VOLUME_CELLS:
        raise ValueError('volume dimensions exceed bounded contract')
    states, active = tuple(response.states), tuple(response.active)
    if (len(states) != nx*ny*nz or len(active) != len(states) or
            any(v not in (0,1,2) for v in states) or
            any(type(v) is not bool for v in active)):
        raise ValueError('invalid volume cell arrays')
    resolution = response.resolution_m
    origin = tuple(response.origin_container)
    expected_origin = (*surface['origin_xy_container'],surface['floor_z_container'])
    if (not math.isfinite(resolution) or resolution <= 0 or
            abs(resolution-surface['resolution_m']) > 1e-9 or
            len(origin) != 3 or any(not math.isfinite(v) for v in origin) or
            any(abs(a-b)>1e-9 for a,b in zip(origin,expected_origin)) or
            nx != len(surface['state']) or ny != len(surface['state'][0]) or
            nz != math.ceil((surface['ceiling_z_container']-origin[2])/resolution)):
        raise ValueError('volume/surface grid geometry mismatch')
    boxes = []
    for y in range(ny):
        for x in range(nx):
            start = None
            for z in range(nz+1):
                index = x + nx*(y+ny*z)
                blocked = z < nz and active[index] and states[index] != 1
                if blocked and start is None:
                    start = z
                if not blocked and start is not None:
                    bottom = origin[2]+start*resolution
                    top = min(origin[2]+z*resolution,surface['ceiling_z_container'])
                    boxes.append(((origin[0]+(x+.5)*resolution,
                                   origin[1]+(y+.5)*resolution,(bottom+top)/2),
                                  (resolution,resolution,top-bottom)))
                    if len(boxes) > 20_000:
                        raise ValueError('private obstacle budget exceeded')
                    start = None
    return tuple(boxes)
