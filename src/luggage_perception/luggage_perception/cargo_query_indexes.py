"""Store-owned derived counts and frontier membership (schema 1).

Only initialize/apply_delta/reset mutate the owner. Queries use detached,
deeply immutable summaries. Native array operations never change primary OCC.
"""
from collections import Counter
from dataclasses import dataclass
from types import MappingProxyType

import numpy as np

from luggage_perception.cargo_grid_projection import occupancy_counts


@dataclass(frozen=True)
class QuerySummary:
    identity: object
    values: object
    visited_voxels: int = 0

    def as_dict(self):
        out = dict(self.values)
        out['label_distribution'] = dict(out['label_distribution'])
        return out


class QueryIndexOwner:
    def __init__(self):
        self._counts = None
        self._labels = Counter()
        self._frontiers = None
        self._frontier_count = 0
        self._last_work = MappingProxyType({})

    @property
    def work(self):
        return self._last_work

    def initialize(self, mapper):
        active, inactive, unknown, free, occupied, labels = occupancy_counts(mapper)
        self._active = active
        self._inactive = inactive
        self._counts = np.asarray([unknown, free, occupied], dtype=np.int64)
        self._labels = Counter(labels)
        shape = (mapper.nz, mapper.ny, mapper.nx)
        grid = np.frombuffer(mapper._grid, 'u1').reshape(shape)
        observed = (grid == 1) | (grid == 2)
        neighbor = np.zeros(shape, dtype=bool)
        # Predicate intentionally does not require neighbor active, as legacy.
        for axis in range(3):
            a, b = [slice(None)] * 3, [slice(None)] * 3
            a[axis], b[axis] = slice(1, None), slice(None, -1)
            neighbor[tuple(a)] |= observed[tuple(b)]
            neighbor[tuple(b)] |= observed[tuple(a)]
        self._frontiers = ((grid == 0) & neighbor &
                           np.frombuffer(mapper._active, 'u1').reshape(shape).astype(bool)).ravel()
        self._frontier_count = int(self._frontiers.sum())
        self._last_work = MappingProxyType({'rebuild': True,
            'frontier_evaluated': grid.size, 'changed_cells': grid.size})

    def reset(self, mapper):
        self.initialize(mapper)

    def apply_delta(self, mapper, delta):
        if delta.rebuild:
            self.reset(mapper)
            return
        indices, old, new = delta.indices, delta.before, delta.after
        active = np.frombuffer(mapper._active, 'u1')[indices].astype(bool)
        for sign, records in ((-1, old), (1, new)):
            self._counts += sign * np.bincount(records['state'][active], minlength=3)
            valid = active & (records['state'] == 2) & (records['label'] > 0)
            labels, counts = np.unique(records['label'][valid], return_counts=True)
            for label, count in zip(labels, counts):
                self._labels[int(label)] += sign * int(count)
                if self._labels[int(label)] == 0:
                    del self._labels[int(label)]
        changed = indices[old['state'] != new['state']]
        nx, ny, nz = mapper.nx, mapper.ny, mapper.nz
        x, y, z = changed % nx, changed // nx % ny, changed // (nx * ny)
        candidates = [changed]
        for coords, extent, step in ((x, nx, 1), (y, ny, nx), (z, nz, nx * ny)):
            candidates.extend((changed[coords > 0] - step, changed[coords + 1 < extent] + step))
        candidates = np.unique(np.concatenate(candidates))
        previous = int(self._frontiers[candidates].sum())
        grid = np.frombuffer(mapper._grid, 'u1')
        x, y, z = candidates % nx, candidates // nx % ny, candidates // (nx * ny)
        observed_neighbor = np.zeros(len(candidates), dtype=bool)
        for coords, extent, step in ((x, nx, 1), (y, ny, nx), (z, nz, nx * ny)):
            for mask, offset in ((coords > 0, -step), (coords + 1 < extent, step)):
                states = grid[candidates[mask] + offset]
                observed_neighbor[mask] |= (states == 1) | (states == 2)
        membership = ((grid[candidates] == 0) & observed_neighbor &
                      np.frombuffer(mapper._active, 'u1')[candidates].astype(bool))
        self._frontiers[candidates] = membership
        self._frontier_count += int(membership.sum()) - previous
        self._last_work = MappingProxyType({'rebuild': False,
            'frontier_evaluated': len(candidates), 'changed_cells': len(indices)})

    def freeze(self, mapper, identity):
        unknown, free, occupied = (int(v) for v in self._counts)
        voxel_volume = mapper.resolution ** 3
        values = dict(total_voxels=self._active, inactive_count=self._inactive,
            unknown_count=unknown, free_count=free, occupied_count=occupied,
            unknown_ratio=float(unknown) / self._active if self._active else 0.,
            occupancy_ratio=float(occupied) / self._active if self._active else 0.,
            occupied_volume=float(occupied) * voxel_volume, free_volume=float(free) * voxel_volume,
            label_distribution=MappingProxyType({str(k): self._labels[k] for k in sorted(self._labels)}),
            map_revision=mapper.map_revision, committed_box_count=len(mapper._placed_boxes),
            geometry_hash=mapper.geometry_hash, frontier_count=self._frontier_count)
        return QuerySummary(identity, MappingProxyType(values))

    def legacy_frontier_indices(self, mapper):
        """Legacy ix/iy/iz order, distinct from new flat-index pagination."""
        flat = np.flatnonzero(self._frontiers)
        ix, iy, iz = flat % mapper.nx, flat // mapper.nx % mapper.ny, flat // (mapper.nx * mapper.ny)
        order = np.lexsort((iz, iy, ix))
        return [(int(ix[i]), int(iy[i]), int(iz[i])) for i in order]
