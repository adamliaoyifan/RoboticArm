"""Read-only, vectorized projections of the owner's x-fastest voxel buffers.

No cached mutable aliases: every public projection is a detached Python value.
The hull kernel remains the geometry authority; it is separable in X.
"""
import numpy as np
from luggage_description.container_geometry import contains_point, EPS


def active_mask(mapper):
    nx, ny, nz = mapper.nx, mapper.ny, mapper.nz
    r = mapper.resolution
    mid_z = (mapper._hull.floor_z + mapper._hull.ceiling_z) * .5
    x = -mapper.inner_l * .5 + (np.arange(nx) + .5) * r
    y = -mapper.inner_w * .5 + (np.arange(ny) + .5) * r
    z = -mapper.inner_h * .5 + (np.arange(nz) + .5) * r
    yz = np.array([[contains_point(mapper._hull, (0., yy, zz + mid_z))
                    for yy in y] for zz in z], dtype=bool)
    allowed_x = (x >= -mapper._hull.half_x - EPS) & (x <= mapper._hull.half_x + EPS)
    mask = yz[:, :, None] & allowed_x[None, None, :]
    # A user-supplied additional predicate may depend on X; never assume
    # it is the hull kernel. Product nodes use the authoritative descriptor.
    if mapper._hull_local_inside is not None:
        for iz, iy, ix in np.argwhere(mask):
            mask[iz, iy, ix] = bool(mapper._hull_local_inside(x[ix], y[iy], z[iz]))
    return bytearray(mask.astype(np.uint8).tobytes())


def _arrays(mapper):
    shape = (mapper.nz, mapper.ny, mapper.nx)
    grid = np.frombuffer(mapper._grid, dtype=np.uint8).reshape(shape)
    active = np.frombuffer(mapper._active, dtype=np.uint8).reshape(shape).astype(bool)
    return grid, active


def surface_columns(mapper):
    grid, active = _arrays(mapper)
    occupied = (grid == 2) & active
    observed = (grid != 0) & active
    unknown = (grid == 0) & active
    has_top = occupied.any(axis=0)
    top = mapper.nz - 1 - occupied[::-1].argmax(axis=0)
    top_height = np.where(has_top, np.minimum((top + 1) * mapper.resolution, mapper.inner_h), 0.)
    state = np.where(has_top, 'occupied', np.where(unknown.any(axis=0), 'unknown', 'free'))
    source = np.frombuffer(mapper._source, dtype=np.uint8).reshape(grid.shape)
    top_source = np.take_along_axis(source, top[None, :, :], axis=0)[0]
    confidence = np.where(has_top, np.where(top_source == 2, 'geometry', 'sensor'),
                          np.where(state == 'free', 'sensor', 'none'))
    count = active.sum(axis=0)
    ratio = np.divide(observed.sum(axis=0), count, out=np.zeros(count.shape), where=count != 0)
    labels = np.frombuffer(mapper._labels, dtype=np.int64).reshape(grid.shape)
    valid_label = occupied & (labels > 0)
    semantic = np.zeros(count.shape, dtype=np.int64)
    # Preserve legacy tie-breaking: the first label encountered bottom-up.
    for iy, ix in np.argwhere(valid_label.any(axis=0)):
        values = labels[:, iy, ix][valid_label[:, iy, ix]]
        unique, first, frequencies = np.unique(values, return_index=True, return_counts=True)
        winners = np.flatnonzero(frequencies == frequencies.max())
        semantic[iy, ix] = unique[winners[np.argmin(first[winners])]]
    return {name: value.T.tolist() for name, value in dict(
        height=top_height, clearance=mapper.inner_h-top_height, state=state,
        known_ratio=ratio, confidence=confidence, semantic_label=semantic).items()}


def occupancy_counts(mapper):
    grid, active = _arrays(mapper)
    selected = grid[active]
    counts = np.bincount(selected, minlength=3)
    labels = np.frombuffer(mapper._labels, dtype=np.int64).reshape(grid.shape)
    valid = labels[(grid == 2) & active & (labels > 0)]
    values, frequency = np.unique(valid, return_counts=True)
    return (int(active.sum()), int(active.size-active.sum()),
            *(int(v) for v in counts[:3]),
            {int(k): int(v) for k, v in zip(values, frequency)})
