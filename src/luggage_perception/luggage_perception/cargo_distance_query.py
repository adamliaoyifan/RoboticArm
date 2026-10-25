"""Bounded exact point-to-voxel-AABB branch-and-bound over frozen blocks.

No distance field or nearest-parent state survives a query: removing an
obstacle is immediately visible in the next immutable generation. Unknown
is a separate target class; occupied_only does not certify unknown as free.
"""
from dataclasses import dataclass
import heapq
import itertools
import math
import time

import numpy as np

from luggage_description.container_geometry import contains_point
from luggage_perception.cargo_map_query import (
    BLOCK_EDGE, DistanceBatch, DistanceMatch, DistancePoint, QueryError,
)
from luggage_perception.cargo_voxel_delta import immutable_array


MAX_EXAMINED_CELLS = 2_000_000
MAX_EXAMINED_BLOCKS = 50_000
MAX_TIMEOUT_MS = 200.


@dataclass(frozen=True)
class DistanceBudget:
    max_examined_cells: int = MAX_EXAMINED_CELLS
    max_examined_blocks: int = MAX_EXAMINED_BLOCKS
    timeout_ms: float = MAX_TIMEOUT_MS


@dataclass(frozen=True)
class DistanceIndex:
    lower: np.ndarray
    upper: np.ndarray

    @classmethod
    def initialize(cls, blocks, origin, resolution):
        start = np.asarray([b.start for b in blocks], dtype=np.float64)
        shape = np.asarray([b.shape for b in blocks], dtype=np.float64)
        origin = np.asarray(origin)
        return cls(immutable_array(origin+start*resolution),
                   immutable_array(origin+(start+shape)*resolution))

    def byte_size(self):
        return 1024+self.lower.nbytes+self.upper.nbytes


class _Work:
    def __init__(self, budget, started):
        self.budget = budget
        self.deadline = started+int(budget.timeout_ms*1e6)
        self.cells = self.blocks = 0

    def admits(self, size):
        return (self.cells+size <= self.budget.max_examined_cells
                and self.blocks+1 <= self.budget.max_examined_blocks
                and time.perf_counter_ns() < self.deadline)


def _zero_match(handle, data, point, state, work):
    """Prove distance zero without scanning a whole containing block.

    Test closed cell volumes, including neighboring cells on exact grid
    planes. The +/-1 bracket handles division rounding; the final predicate
    uses the same AABB arithmetic as branch-and-bound. Ascending flat IDs
    preserve ties even where a point belongs to up to eight closed volumes.
    """
    grid = data.grid
    choices = []
    for coordinate, origin, extent in zip(point, grid.origin, grid.shape):
        base = math.floor((coordinate-origin)/grid.resolution)
        choices.append(tuple(i for i in (base-1, base, base+1)
            if 0 <= i < extent and origin+i*grid.resolution <= coordinate
            <= origin+(i+1)*grid.resolution))
    nx, ny, _ = grid.shape
    bx, by = ((n+BLOCK_EDGE-1)//BLOCK_EDGE for n in grid.shape[:2])
    candidates = sorted((x+nx*(y+ny*z), x, y, z)
                        for x, y, z in itertools.product(*choices))
    for flat, x, y, z in candidates:
        block = data.blocks[x//BLOCK_EDGE+bx*(y//BLOCK_EDGE+by*(z//BLOCK_EDGE))]
        if block.state_counts[state] == 0:
            continue
        if not work.admits(1):
            return DistanceMatch('QUERY_BUDGET_EXHAUSTED', False)
        work.cells += 1
        work.blocks += 1
        offset = int(np.searchsorted(block.flat_indices, flat))
        if block.active[offset] and block.records()['state'][offset] == state:
            return DistanceMatch('MATCH', True, 0., 0., handle._cell_record(block, offset))
    return None


def _nearest(handle, data, point, state, radius, work, counts, lower_squared, volumes):
    candidates = np.flatnonzero(counts[:, state] > 0)
    if not len(candidates):
        return DistanceMatch('NO_MATCH', True)
    zero = _zero_match(handle, data, point, state, work)
    if zero is not None:
        return zero
    # Heap tuples include block ID to keep traversal deterministic. Equal
    # lower bounds must be visited: another block may contain a smaller flat ID.
    candidates = candidates[lower_squared[candidates] <= radius*radius]
    queue = list(zip(lower_squared[candidates].tolist(), candidates.tolist()))
    heapq.heapify(queue)
    best_squared = radius*radius
    best = None
    nx, ny, _ = data.grid.shape
    r = data.grid.resolution
    origin = np.asarray(data.grid.origin)
    while queue:
        lower, block_id = heapq.heappop(queue)
        if lower > best_squared:
            break
        block = data.blocks[block_id]
        key = (block_id, state)
        cached = volumes.get(key)
        # A batch visits the same immutable blocks for many points. Decode
        # their state membership and cell volumes once, without retaining a
        # distance field or transferring any work to the map writer. Charge
        # both the initial membership scan and every point/volume test.
        examined = int(counts[block_id, state])
        if cached is None:
            examined += len(block.flat_indices)
        if not work.admits(examined):
            return DistanceMatch('QUERY_BUDGET_EXHAUSTED', False,
                                 lower_bound_m=math.sqrt(min(lower, best_squared)))
        work.cells += examined
        work.blocks += 1
        if cached is None:
            records = block.records()
            offsets = np.flatnonzero((records['state'] == state)
                                    & np.frombuffer(block.active, 'u1').astype(bool))
            flat = block.flat_indices[offsets]
            coordinates = np.column_stack((flat % nx, flat//nx % ny, flat//(nx*ny)))
            low = origin+coordinates*r
            high = origin+(coordinates+1)*r
            cached = volumes[key] = (offsets, low, high)
        offsets, low, high = cached
        displacement = np.maximum(np.maximum(low-point, point-high), 0.)
        squared = np.einsum('ij,ij->i', displacement, displacement)
        minimum = float(squared.min())
        # flat_indices and therefore offsets are already ascending.
        offset = int(offsets[np.flatnonzero(squared == minimum)[0]])
        flat_id = int(block.flat_indices[offset])
        if (minimum < best_squared or (minimum == best_squared
                and (best is None or flat_id < best[0]))):
            best_squared, best = minimum, (flat_id, block, offset)
    if best is None:
        return DistanceMatch('RADIUS_CLEAR', False, lower_bound_m=radius)
    return DistanceMatch('MATCH', True, math.sqrt(best_squared), math.sqrt(best_squared),
                         handle._cell_record(best[1], best[2]))


def read_nearest(handle, data, points, radius, unknown_policy, budget):
    started = time.perf_counter_ns()
    try:
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= 256:
            raise ValueError()
        if not np.isfinite(points).all():
            raise ValueError()
        radius = float(radius)
        diagonal = float(np.linalg.norm(np.asarray(data.grid.shape)*data.grid.resolution))
        if not math.isfinite(radius) or not 0 <= radius <= diagonal:
            raise ValueError()
        if unknown_policy not in ('separate', 'occupied_only'):
            raise ValueError()
        budget = budget if budget is not None else DistanceBudget()
        if (not isinstance(budget, DistanceBudget)
                or type(budget.max_examined_cells) is not int
                or not 0 <= budget.max_examined_cells <= MAX_EXAMINED_CELLS
                or type(budget.max_examined_blocks) is not int
                or not 0 <= budget.max_examined_blocks <= MAX_EXAMINED_BLOCKS
                or not math.isfinite(budget.timeout_ms)
                or not 0 <= budget.timeout_ms <= MAX_TIMEOUT_MS):
            raise ValueError()
    except (TypeError, ValueError, OverflowError):
        raise QueryError('INVALID_FILTER') from None
    if data.distance_index is None:
        raise QueryError('QUERY_INDEX_UNAVAILABLE')
    work = _Work(budget, started)
    counts = np.asarray([block.state_counts for block in data.blocks])
    origin = np.asarray(data.grid.origin)
    upper = origin+np.asarray(data.grid.shape)*data.grid.resolution
    results = []
    volumes = {}
    for point in points:
        status = ''
        if np.any(point < origin) or np.any(point >= upper):
            status = 'OUTSIDE_MAP'
        elif not contains_point(data.grid.hull, point):
            status = 'OUTSIDE_HULL'
        if status:
            match = DistanceMatch(status, False)
            results.append(DistancePoint(tuple(float(v) for v in point), status, match,
                                         match if unknown_policy == 'separate' else None))
            continue
        delta = np.maximum(np.maximum(data.distance_index.lower-point,
                                      point-data.distance_index.upper), 0.)
        lower_squared = np.einsum('ij,ij->i', delta, delta)
        occupied = _nearest(handle, data, point, 2, radius, work, counts, lower_squared, volumes)
        unknown = (_nearest(handle, data, point, 0, radius, work, counts, lower_squared, volumes)
                   if unknown_policy == 'separate' else None)
        status = ('QUERY_BUDGET_EXHAUSTED'
                  if any(m is not None and m.status == 'QUERY_BUDGET_EXHAUSTED'
                         for m in (occupied, unknown)) else 'OK')
        results.append(DistancePoint(tuple(float(v) for v in point), status, occupied, unknown))
    exhausted = any(p.status == 'QUERY_BUDGET_EXHAUSTED' for p in results)
    return DistanceBatch('QUERY_BUDGET_EXHAUSTED' if exhausted else 'OK',
        'QUERY_BUDGET_EXHAUSTED' if exhausted else '', handle.identity, data.grid,
        complete=not exhausted, returned_count=len(results), examined_cells=work.cells,
        examined_blocks=work.blocks, elapsed_ns=time.perf_counter_ns()-started,
        unknown_policy=unknown_policy, results=tuple(results))
