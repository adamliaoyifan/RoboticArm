"""Exact legacy column projection, updated only at dirty XY columns.

Only initialize/update_columns mutate the owner. Frozen 16x16 tiles share
unchanged data between publications; exports always create detached values.
The existing full cargo_grid_projection.surface_columns remains the oracle.
"""
from dataclasses import dataclass
from types import MappingProxyType
import hashlib
import json
import math
import time

import numpy as np

from luggage_perception.cargo_column_math import column_summary
from luggage_perception.cargo_grid_projection import surface_columns
from luggage_perception.cargo_voxel_delta import immutable_array


TILE_EDGE = 16
FIELD_DTYPES = MappingProxyType(dict(height='<f8', clearance='<f8', state='<U8',
                                    known_ratio='<f8', confidence='<U8', semantic_label='<i8'))


@dataclass(frozen=True)
class SurfaceTile:
    start: tuple
    shape: tuple
    fields: object


@dataclass(frozen=True)
class SurfaceProjection:
    shape: tuple
    tiles: tuple

    def extract(self, start=(0, 0), stop=None):
        stop = self.shape if stop is None else stop
        shape = tuple(max(0, b-a) for a, b in zip(start, stop))
        fields = {name: np.empty(shape, dtype=dtype) for name, dtype in FIELD_DTYPES.items()}
        for tile in self.tiles:
            low = tuple(max(a, b) for a, b in zip(start, tile.start))
            high = tuple(min(a, b+n) for a, b, n in zip(stop, tile.start, tile.shape))
            if any(a >= b for a, b in zip(low, high)):
                continue
            target = tuple(slice(a-o, b-o) for a, b, o in zip(low, high, start))
            source = tuple(slice(a-o, b-o) for a, b, o in zip(low, high, tile.start))
            for name in FIELD_DTYPES:
                fields[name][target] = tile.fields[name][source]
        return MappingProxyType({name: immutable_array(values) for name, values in fields.items()})

    def as_dict(self):
        return {name: values.tolist() for name, values in self.extract().items()}


def project_column_subset(mapper, columns):
    """Dirty columns only. Integer indexing copies those Z columns."""
    shape = (mapper.nz, mapper.ny * mapper.nx)
    grid = np.frombuffer(mapper._grid, "u1").reshape(shape)[:, columns]
    active = np.frombuffer(mapper._active, "u1").reshape(shape)[:, columns].astype(bool)
    source = np.frombuffer(mapper._source, "u1").reshape(shape)[:, columns]
    labels = np.frombuffer(mapper._labels, "<i8").reshape(shape)[:, columns]
    return column_summary(
        grid, active, source, labels, mapper.resolution, mapper.inner_h)


class IncrementalSurfaceProjection:
    def __init__(self):
        self._projection = None
        self._work = MappingProxyType({})
        self._total_columns = 0

    @property
    def work(self):
        return self._work

    def initialize(self, mapper):
        full = surface_columns(mapper)
        arrays = {name: np.asarray(full[name], dtype=dtype) for name, dtype in FIELD_DTYPES.items()}
        tiles = []
        for y in range(0, mapper.ny, TILE_EDGE):
            for x in range(0, mapper.nx, TILE_EDGE):
                shape = (min(TILE_EDGE, mapper.nx-x), min(TILE_EDGE, mapper.ny-y))
                fields = MappingProxyType({name: immutable_array(values[x:x+shape[0], y:y+shape[1]])
                                           for name, values in arrays.items()})
                tiles.append(SurfaceTile((x, y), shape, fields))
        self._projection = SurfaceProjection((mapper.nx, mapper.ny), tuple(tiles))
        self._total_columns = mapper.nx*mapper.ny
        self._work = MappingProxyType(dict(rebuild=True, columns_recomputed=self._total_columns,
                                          tiles_copied=len(tiles), total_columns=self._total_columns))

    def update_columns(self, mapper, columns):
        columns = np.unique(np.asarray(columns, dtype=np.int64))
        if np.any(columns < 0) or np.any(columns >= mapper.nx*mapper.ny):
            raise ValueError('dirty column outside grid')
        if not len(columns):
            self._work = MappingProxyType(dict(rebuild=False, columns_recomputed=0,
                                              tiles_copied=0, total_columns=self._total_columns))
            return
        values = project_column_subset(mapper, columns)
        x, y = columns % mapper.nx, columns//mapper.nx
        bx = (mapper.nx+TILE_EDGE-1)//TILE_EDGE
        ids = x//TILE_EDGE+bx*(y//TILE_EDGE)
        tiles = list(self._projection.tiles)
        for tile_id in np.unique(ids):
            old = tiles[tile_id]
            selected = np.flatnonzero(ids == tile_id)
            fields = {}
            for name in FIELD_DTYPES:
                updated = old.fields[name].copy()
                updated[x[selected]-old.start[0], y[selected]-old.start[1]] = values[name][selected]
                fields[name] = immutable_array(updated)
            tiles[tile_id] = SurfaceTile(old.start, old.shape, MappingProxyType(fields))
        self._projection = SurfaceProjection(self._projection.shape, tuple(tiles))
        self._total_columns += len(columns)
        self._work = MappingProxyType(dict(rebuild=False, columns_recomputed=len(columns),
                                          tiles_copied=len(np.unique(ids)), total_columns=self._total_columns))

    def freeze(self):
        return self._projection

    def as_dict(self):
        """Memoized full-projection dict keyed by frozen identity.

        update_columns/initialize REPLACE the frozen projection object,
        so object identity is a valid change token. The tolist() over
        the full projection is ~26ms at 1cm (the largest single
        snapshot-build item after the digest); an unchanged projection
        (e.g. the metadata-only I1 transaction) reuses the identical
        nested rows. The dict is consumed read-only by the snapshot
        contract (JSON + freeze-to-tuples), never mutated."""
        projection = self._projection
        if projection is None:
            return None
        cache = getattr(self, '_dict_cache', None)
        if cache is not None and cache[0] is projection:
            # NEVER hand out a mutable alias into the cache (user audit
            # 2026-10-27): fresh per-row copies per call. The expensive
            # numpy->str conversion is cached; these are cheap C-level
            # list copies.
            return {name: [row[:] for row in rows]
                    for name, rows in cache[1].items()}
        result = projection.as_dict()
        self._dict_cache = (projection, result)
        return {name: [row[:] for row in rows]
                for name, rows in result.items()}


def _surface_window(grid, region):
    """Projection ROI selects whole columns by XY footprint, independent of Z.

    Accept XY AABBs or finite XYZ AABBs (Z is validated but not applied to a
    whole-column projection). Integer windows use the same strict positive
    intersection rule as the 3D cell API, including exact cell boundaries.
    """
    from luggage_perception.cargo_map_query import QueryError
    nx, ny = grid.shape[:2]
    if region is None:
        return None, (0, 0), (nx, ny)
    try:
        low, high = (tuple(float(v) for v in point) for point in region)
        if (len(low) != len(high) or len(low) not in (2, 3)
                or not all(math.isfinite(v) for v in low+high)
                or any(a >= b for a, b in zip(low, high))):
            raise ValueError()
    except (TypeError, ValueError, OverflowError):
        raise QueryError('INVALID_REGION') from None
    matches = []
    for axis, size in enumerate((nx, ny)):
        indices = np.arange(size)
        a = grid.origin[axis]+indices*grid.resolution
        b = grid.origin[axis]+(indices+1)*grid.resolution
        matches.append(np.flatnonzero((b > low[axis]) & (a < high[axis])))
    normalized = (low[:2], high[:2])
    if not all(len(m) for m in matches):
        return normalized, (0, 0), (0, 0)
    return normalized, tuple(int(m[0]) for m in matches), tuple(int(m[-1])+1 for m in matches)


def _frozen_rows(array):
    return tuple(tuple(row) for row in array.tolist())


def read_surface(handle, data, region):
    from luggage_perception.cargo_map_query import QueryError, SurfaceResult
    started = time.perf_counter_ns()
    if data.surface_projection is None:
        raise QueryError('QUERY_INDEX_UNAVAILABLE')
    _, start, stop = _surface_window(data.grid, region)
    fields = {name: _frozen_rows(array)
              for name, array in data.surface_projection.extract(start, stop).items()}
    snapshot = data.map_snapshot
    for name in ('surface_z_container', 'height_above_floor', 'contract_confidence'):
        source = getattr(snapshot, name)
        fields[name] = tuple(tuple(row[start[1]:stop[1]]) for row in source[start[0]:stop[0]])
    shape = tuple(b-a for a, b in zip(start, stop))
    return SurfaceResult('OK', '', handle.identity, data.grid, returned_count=shape[0]*shape[1],
        examined_cells=0, elapsed_ns=time.perf_counter_ns()-started,
        origin_index_xy=start, shape_xy=shape, columns=MappingProxyType(fields),
        floor_z_container=snapshot.floor_z_container, ceiling_z_container=snapshot.ceiling_z_container)


def read_surface_changes(handle, data, previous, region, page):
    from luggage_perception.cargo_map_query import (
        MAX_PAGE, QueryError, QueryPage, SurfaceColumn, SurfacePatchResult,
    )
    started = time.perf_counter_ns()
    if data.surface_projection is None or previous.surface_projection is None:
        raise QueryError('QUERY_INDEX_UNAVAILABLE')
    normalized, start, stop = _surface_window(data.grid, region)
    page = page if page is not None else QueryPage()
    if (not isinstance(page, QueryPage) or type(page.limit) is not int
            or not 1 <= page.limit <= MAX_PAGE or not isinstance(page.cursor, str)):
        raise QueryError('INVALID_FILTER')
    query_hash = hashlib.sha256(json.dumps(('surface_changes', tuple(previous.identity), normalized),
                                          separators=(',', ':')).encode()).hexdigest()
    last = handle._decode_cursor(data, page.cursor, query_hash) if page.cursor else -1
    nx = data.grid.shape[0]
    selected = []
    examined = 0
    for tile, old in zip(data.surface_projection.tiles, previous.surface_projection.tiles):
        if tile is old:
            continue
        low = tuple(max(a, b) for a, b in zip(start, tile.start))
        high = tuple(min(a, b+n) for a, b, n in zip(stop, tile.start, tile.shape))
        if any(a >= b for a, b in zip(low, high)):
            continue
        mask = np.zeros(tile.shape, dtype=bool)
        for name in FIELD_DTYPES:
            mask |= tile.fields[name] != old.fields[name]
        examined += tile.shape[0]*tile.shape[1]
        x, y = np.nonzero(mask)
        x, y = x+tile.start[0], y+tile.start[1]
        flat = x+nx*y
        keep = (flat > last) & (x >= low[0]) & (x < high[0]) & (y >= low[1]) & (y < high[1])
        for xx, yy, index in zip(x[keep], y[keep], flat[keep]):
            selected.append((int(index), int(xx), int(yy), tile))
        selected.sort(key=lambda value: value[0])
        del selected[page.limit+1:]
    complete = len(selected) <= page.limit
    selected = selected[:page.limit]
    patches = []
    snapshot = data.map_snapshot
    for _, x, y, tile in selected:
        a, b = x-tile.start[0], y-tile.start[1]
        values = {name: array[a, b].item() for name, array in tile.fields.items()}
        patches.append(SurfaceColumn(x, y, **values,
            surface_z_container=snapshot.surface_z_container[x][y],
            height_above_floor=snapshot.height_above_floor[x][y],
            contract_confidence=snapshot.contract_confidence[x][y]))
    cursor = '' if complete else handle._encode_cursor(data, query_hash, selected[-1][0])
    return SurfacePatchResult('OK', '', handle.identity, data.grid, complete=complete,
        next_cursor=cursor, returned_count=len(patches), examined_cells=examined,
        elapsed_ns=time.perf_counter_ns()-started, since_identity=previous.identity,
        patches=tuple(patches))
