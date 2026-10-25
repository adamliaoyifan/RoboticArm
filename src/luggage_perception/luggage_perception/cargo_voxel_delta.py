"""Compact, read-only evidence deltas; never changes occupancy decisions.

The mapper calls record_before at its existing mutation boundaries. Store
finalizes once per publication. First-old/final-new semantics preserve overlap,
repeated hits, and changes confined to sensor shadow or ownership.
"""
from dataclasses import dataclass

import numpy as np


EVIDENCE_DTYPE = np.dtype([
    ('state', 'u1'), ('source', 'u1'), ('label', '<i8'), ('instance', '<i8'),
    ('logodds', '<f8'), ('locked', '?'), ('seen', '?'), ('shadow', '<f8'),
    ('shadow_label', '<i8'), ('shadow_instance', '<i8'),
])


def immutable_array(values):
    """Bytes-backed array: consumers cannot turn writeability back on."""
    values = np.ascontiguousarray(values)
    return np.frombuffer(values.tobytes(), dtype=values.dtype).reshape(values.shape)


def capture_evidence(mapper, indices):
    records = np.empty(len(indices), dtype=EVIDENCE_DTYPE)
    for field, data, dtype in (
        ('state', mapper._grid, 'u1'), ('source', mapper._source, 'u1'),
        ('label', mapper._labels, '<i8'), ('instance', mapper._instance_ids, '<i8'),
        ('seen', mapper._sensor_seen, 'u1'), ('shadow', mapper._sensor_values, '<f8'),
        ('shadow_label', mapper._sensor_labels, '<i8'),
        ('shadow_instance', mapper._sensor_instance_ids, '<i8'),
    ):
        records[field] = np.frombuffer(data, dtype=dtype)[indices]
    # Primary evidence deliberately remains in its legacy Python lists.
    # Gather only the touched entries. Built-in list getters avoid a Python
    # generator frame per entry without copying/scanning the primary lists.
    records['logodds'] = np.fromiter(
        map(mapper._occupancy.values.__getitem__,indices),float,len(indices))
    records['locked'] = np.fromiter(
        map(mapper._occupancy.geometry_locked.__getitem__,indices),bool,len(indices))
    return records


@dataclass(frozen=True)
class GeometryRaster:
    key: tuple
    indices: np.ndarray


@dataclass(frozen=True)
class DeltaBatch:
    indices: np.ndarray
    before: np.ndarray
    after: np.ndarray
    rebuild: bool = False
    geometry_rasters: tuple = ()


class VoxelDeltaBuilder:
    """Transaction-local owner; compact arrays, no per-map voxel objects."""

    def __init__(self, mapper, before_lookup=None):
        self._mapper = mapper
        self._before_lookup = before_lookup
        self._seen = set()
        self._chunks = []
        self._rebuild = False
        self._geometry_rasters = {}

    def record_before(self, indices):
        if self._rebuild:
            return
        fresh = []
        for index in indices:
            index = int(index)
            if index not in self._seen:
                self._seen.add(index)
                fresh.append(index)
        if fresh:
            keys = np.asarray(fresh, dtype=np.int64)
            # Store may bind the derived owner's unchanged prior facts.
            # Standalone mapper users retain the original immediate capture.
            before = (capture_evidence(self._mapper, keys)
                      if self._before_lookup is None else None)
            self._chunks.append((keys, before))

    def record_reset(self):
        self._rebuild = True
        self._chunks.clear()
        self._seen.clear()
        self._geometry_rasters.clear()

    def record_geometry(self, center, size, yaw, indices):
        key = (tuple(center), tuple(size), float(yaw))
        self._geometry_rasters[key] = GeometryRaster(
            key, immutable_array(np.asarray(indices, dtype=np.int32)))

    def finalize(self):
        if self._chunks:
            indices = np.concatenate([chunk[0] for chunk in self._chunks])
            before = (np.concatenate([chunk[1] for chunk in self._chunks])
                      if self._before_lookup is None else self._before_lookup(indices))
            after = capture_evidence(self._mapper, indices)
            changed = np.zeros(len(indices), dtype=bool)
            for name in EVIDENCE_DTYPE.names:
                equal = before[name] == after[name]
                if before[name].dtype.kind == 'f':
                    equal |= np.isnan(before[name]) & np.isnan(after[name])
                changed |= ~equal
            selected = np.flatnonzero(changed)
            order = selected[np.argsort(indices[selected])]
            # Gather each wide record once. Filtering and then sorting the
            # records copied both full structured buffers twice per finalize.
            record_bytes = np.dtype(('V', EVIDENCE_DTYPE.itemsize))
            indices = indices[order]
            before = before.view(record_bytes)[order].view(EVIDENCE_DTYPE)
            after = after.view(record_bytes)[order].view(EVIDENCE_DTYPE)
        else:
            indices = np.empty(0, dtype=np.int64)
            before = after = np.empty(0, dtype=EVIDENCE_DTYPE)
        return DeltaBatch(*(immutable_array(a) for a in (indices, before, after)),
                          rebuild=self._rebuild,
                          geometry_rasters=tuple(self._geometry_rasters.values()))
