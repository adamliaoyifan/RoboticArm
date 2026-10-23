"""Copy owner voxel data under the caller's map serialization boundary."""
from luggage_description.cargo_grid_contract import MAX_VOLUME_CELLS


def copy_volume(mapper):
    size = mapper.nx * mapper.ny * mapper.nz
    if not 0 < size <= MAX_VOLUME_CELLS:
        raise ValueError('volume cell budget exceeded')
    if mapper.yaw != 0. or mapper.center[:2] != [0., 0.]:
        raise ValueError('snapshot requires container-aligned owner map')
    return dict(resolution_m=mapper.resolution,
                origin_container=(-mapper.inner_l/2, -mapper.inner_w/2,
                                  mapper.center[2]-mapper.inner_h/2),
                dimensions_xyz=(mapper.nx,mapper.ny,mapper.nz),
                states=bytes(mapper._grid), active=tuple(bool(v) for v in mapper._active))
