"""Independent scalar integrate loop.

Copied from CargoVolumeMapper.integrate_points(_vectorized=False) at
baseline a34472d0682f35172d1fc80a0cfb654a0b8ddc16. This module is the
test-side reference. It must not call the vectorized product path.
"""

import math

from luggage_perception.cargo_volume_mapper import FREE, SOURCE_GEOMETRY, SOURCE_NONE


def scalar_integrate(mapper, points, origin=None, labels=None,
                     instance_ids=None, ray_points=None):
    """Same control flow as the retired product scalar branch."""
    hit_records = {}
    diagnostics = dict(rays_selected=0, ray_intersecting=0, samples_visited=0)
    miss_indices = set()
    for i, pt in enumerate(points):
        x, y, z = pt[0], pt[1], pt[2]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        lbl = int(labels[i]) if labels is not None else 0
        iid = int(instance_ids[i]) if instance_ids is not None else 0
        local = mapper._world_to_local(x, y, z)
        vox = mapper._local_to_voxel(*local)
        if vox is not None:
            hit_records[mapper._index(*vox)] = (x, y, z, lbl, iid)

    if origin is not None:
        ox, oy, oz = origin
        ray_points = points if ray_points is None else ray_points
        if mapper.max_raycast_points and len(ray_points) > mapper.max_raycast_points:
            stride = max(1, len(ray_points) // mapper.max_raycast_points)
            ray_points = ray_points[::stride]
        for pt in ray_points:
            x, y, z = pt[0], pt[1], pt[2]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            diagnostics["rays_selected"] += 1
            miss_indices.update(
                mapper._raycast_indices((ox, oy, oz), (x, y, z), diagnostics)
            )
    miss_indices.difference_update(hit_records)
    mapper._observe_voxels(miss_indices)
    mapper._observe_voxels(hit_records)
    for idx in miss_indices:
        mapper._grid[idx] = mapper._occupancy.apply_miss(idx)
        mapper._record_sensor_miss(idx)
        if mapper._grid[idx] == FREE and mapper._source[idx] != SOURCE_GEOMETRY:
            mapper._source[idx] = SOURCE_NONE
            mapper._labels[idx] = 0
            mapper._instance_ids[idx] = 0
    for x, y, z, lbl, iid in hit_records.values():
        mapper.mark_occupied_world(x, y, z, label=lbl, instance_id=iid)
    if hit_records or miss_indices:
        mapper._revision += 1
    return diagnostics
