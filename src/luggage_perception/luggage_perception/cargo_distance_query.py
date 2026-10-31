"""Bounded exact point-to-voxel-AABB branch-and-bound over frozen blocks.

No distance field or nearest-parent state survives a query: removing an
obstacle is immediately visible in the next immutable generation. Unknown
is a separate target class; occupied_only does not certify unknown as free.
"""
from dataclasses import dataclass
import heapq
import itertools
import math
import os
import time
from types import MappingProxyType

import numpy as np

from luggage_description.container_geometry import contains_point
from luggage_perception.cargo_map_query import (
    BLOCK_EDGE, DistanceBatch, DistanceMatch, DistancePoint, QueryError,
)
from luggage_perception.cargo_voxel_delta import immutable_array

try:
    from luggage_perception.distance_kernels_jit import (
        HAS_NUMBA, kernel_nearest,
        STATUS_BUDGET, STATUS_MATCH, STATUS_RADIUS_CLEAR)
except ImportError:  # pragma: no cover - kernel module is optional
    HAS_NUMBA = False
_USE_JIT = HAS_NUMBA and os.environ.get('DISTANCE_JIT', '1') != '0'
import weakref
#: id-keyed generation cache. The pinned batch object defines __eq__
#: without __hash__ (unhashable), so WeakKeyDictionary cannot key it;
#: id keys with a weakref death-callback are safe instead: the entry
#: keeps the object alive, and ids cannot be reused while it lives.
_JIT_TABLES_CACHE = {}


def _jit_tables_for_data(data):
    entry = _JIT_TABLES_CACHE.get(id(data))
    if entry is None:
        cleanup = lambda _ref, key=id(data): _JIT_TABLES_CACHE.pop(key, None)
        entry = _JIT_TABLES_CACHE[id(data)] = (
            {}, weakref.ref(data, cleanup))
    return entry[0]


#: Per-block decoded segments, keyed by the block OBJECT identity: the
#: replica reuses unchanged block objects across generations, so under a
#: write stream only changed blocks pay the decode (a generation-scoped
#: table cache alone re-decoded every block per generation -- at 2 Hz
#: writes with ~1 distance query per generation that tax landed on
#: nearly every query, max 219 ms in the C2 rehearsal).
_BLOCK_SEGMENT_CACHE = {}


def _block_segment(block, state, origin, r):
    per_block = _BLOCK_SEGMENT_CACHE.get(id(block))
    if per_block is None:
        cleanup = lambda _ref, key=id(block): _BLOCK_SEGMENT_CACHE.pop(key, None)
        per_block = _BLOCK_SEGMENT_CACHE[id(block)] = (
            {}, weakref.ref(block, cleanup))
    segment = per_block[0].get(state)
    if segment is None:
        members = block.distance_members[state]
        offsets = np.frombuffer(members.offsets, dtype='<u2')
        starts = np.frombuffer(members.starts, dtype='<u2').astype(np.int64)
        ends = np.frombuffer(members.ends, dtype='<u2').astype(np.int64)
        leaf_ids = np.frombuffer(members.leaf_ids, dtype='u1')
        low = origin+(np.asarray(block.start)+block.distance_layout.lower[leaf_ids])*r
        high = origin+(np.asarray(block.start)+block.distance_layout.upper[leaf_ids])*r
        xyz = block.distance_layout.cell_coordinates[offsets]+np.asarray(block.start)
        segment = per_block[0][state] = dict(
            leaf_low=low, leaf_high=high, starts=starts, ends=ends,
            mem_low=origin+xyz*r, mem_high=origin+(xyz+1)*r,
            mem_flat=np.asarray(block.flat_indices, dtype=np.int64)[offsets],
            mem_offset=offsets.astype(np.int64))
    return segment


MAX_EXAMINED_CELLS = 2_000_000
MAX_EXAMINED_BLOCKS = 50_000
MAX_TIMEOUT_MS = 200.
# Request-local leaves limit distance arithmetic within a 16-cube block.
# They are query acceleration, never a second occupancy representation.
DISTANCE_LEAF_EDGE = 4


@dataclass(frozen=True)
class DistanceLeafLayout:
    order: np.ndarray
    leaf_ids: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    cell_coordinates: np.ndarray

    @classmethod
    def initialize(cls, shape):
        offsets = np.arange(math.prod(shape))
        xyz = np.column_stack((offsets % shape[0], offsets//shape[0] % shape[1],
                               offsets//(shape[0]*shape[1])))
        dimensions = (np.asarray(shape)+DISTANCE_LEAF_EDGE-1)//DISTANCE_LEAF_EDGE
        small = xyz//DISTANCE_LEAF_EDGE
        ids = small[:,0]+dimensions[0]*(small[:,1]+dimensions[1]*small[:,2])
        leaf = np.arange(math.prod(dimensions))
        low = np.column_stack((leaf % dimensions[0], leaf//dimensions[0] % dimensions[1],
            leaf//(dimensions[0]*dimensions[1])))*DISTANCE_LEAF_EDGE
        high = np.minimum(low+DISTANCE_LEAF_EDGE, shape)
        return cls(immutable_array(np.argsort(ids,kind='stable').astype('<u2')),
            immutable_array(ids.astype('u1')),immutable_array(low.astype('u1')),
            immutable_array(high.astype('u1')), immutable_array(xyz.astype('u1')))

    def byte_size(self):
        return 1024+sum(a.nbytes for a in (
            self.order,self.leaf_ids,self.lower,self.upper,self.cell_coordinates))


@dataclass(frozen=True)
class DistanceLeafMembers:
    offsets: bytes
    starts: bytes
    ends: bytes
    leaf_ids: bytes

    def byte_size(self):
        return 512+len(self.offsets)+len(self.starts)+len(self.ends)+len(self.leaf_ids)


def freeze_distance_members(layout, records, allowed):
    """Derived state membership only; no distances or primary facts stored."""
    result = {}
    for state in (0,2):
        selected = allowed & (records['state']==state)
        offsets = layout.order[selected[layout.order]]
        counts = np.bincount(layout.leaf_ids[offsets],minlength=len(layout.lower))
        ids = np.flatnonzero(counts)
        ends = np.cumsum(counts[ids])
        starts = np.r_[0,ends[:-1]] if len(ends) else np.empty(0,dtype=np.int64)
        result[state] = DistanceLeafMembers(offsets.astype('<u2').tobytes(),
            starts.astype('<u2').tobytes(),ends.astype('<u2').tobytes(),ids.astype('u1').tobytes())
    return MappingProxyType(result)


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
        # Blocks retain native x-fast order, including clipped edge blocks.
        # The already validated coordinate selects its row directly.
        sx, sy, _ = block.shape
        offset = (x-block.start[0])+sx*((y-block.start[1])+sy*(z-block.start[2]))
        if block.active[offset] and block.records()['state'][offset] == state:
            return DistanceMatch('MATCH', True, 0., 0., handle._cell_record(block, offset))
    return None


def _jit_tables(data, state, cand_ids, volumes, origin, r):
    """Flattened traversal tables for one state, cached PER GENERATION
    (snapshot data is immutable), so the eager decode of every
    candidate block's frozen membership amortizes across all queries on
    that snapshot; a request-scoped volumes lookup alone would re-decode
    per request and cost more than the lazy per-block path it replaces
    (measured: +8 ms/request before this cache). scratch rows are
    per-block cursors, re-initialized lazily on each traversal's first
    pop, so sharing them across points and requests is safe."""
    key = ('__jit_tables', state)
    tables = volumes.get(key)
    if tables is not None:
        return tables
    try:
        per_state = _jit_tables_for_data(data)
    except TypeError:      # unweakrefable pinned-batch type: request scope
        per_state = {}
    tables = per_state.get(state)
    if tables is not None:
        volumes[key] = tables
        return tables
    leaf_lows, leaf_highs, leaf_gstarts, leaf_gends = [], [], [], []
    mem_lows, mem_highs, mem_flats, mem_offsets = [], [], [], []
    leaf_base = [0]
    member_base = 0
    for block_id in cand_ids:
        block = data.blocks[block_id]
        segment = _block_segment(block, state, origin, r)
        leaf_lows.append(segment['leaf_low'])
        leaf_highs.append(segment['leaf_high'])
        leaf_gstarts.append(member_base+segment['starts'])
        leaf_gends.append(member_base+segment['ends'])
        mem_lows.append(segment['mem_low'])
        mem_highs.append(segment['mem_high'])
        mem_flats.append(segment['mem_flat'])
        mem_offsets.append(segment['mem_offset'])
        member_base += len(segment['mem_offset'])
        leaf_base.append(leaf_base[-1]+len(segment['leaf_low']))
    max_leaves = max((len(x) for x in leaf_lows), default=1) or 1
    n_blocks = len(cand_ids)
    tables = dict(
        block_ids=np.asarray(cand_ids, dtype=np.int64),
        blk_leaf_base=np.asarray(leaf_base, dtype=np.int64),
        leaf_low=np.concatenate(leaf_lows) if leaf_lows else np.empty((0, 3)),
        leaf_high=np.concatenate(leaf_highs) if leaf_highs else np.empty((0, 3)),
        leaf_gstart=np.concatenate(leaf_gstarts) if leaf_gstarts else np.empty(0, np.int64),
        leaf_gend=np.concatenate(leaf_gends) if leaf_gends else np.empty(0, np.int64),
        mem_low=np.concatenate(mem_lows) if mem_lows else np.empty((0, 3)),
        mem_high=np.concatenate(mem_highs) if mem_highs else np.empty((0, 3)),
        mem_flat=np.concatenate(mem_flats) if mem_flats else np.empty(0, np.int64),
        mem_offset=np.concatenate(mem_offsets) if mem_offsets else np.empty(0, np.int64),
        scratch_bounds=np.zeros((n_blocks, max_leaves)),
        scratch_order=np.zeros((n_blocks, max_leaves+2), dtype=np.int64))
    per_state[state] = tables
    volumes[key] = tables
    return tables


def prewarm_tables(data):
    """Fill the generation-level JIT table cache for both distance
    states on a snapshot the replica is about to publish, so the first
    query on that generation hits the cache instead of assembling on
    the query thread (C2 rehearsal: with 2 Hz full-volume write clouds
    nearly every generation changes ~120 of 175 blocks, and the
    per-generation assembly -- 7-9 ms per state -- landed on the first
    query of each generation as the dominant steady-spike component).
    Candidates are computed exactly as read_nearest computes them, so
    the warmed entry is the one the query will find. Pure cache fill;
    no-ops without JIT or a distance index, and never changes query
    semantics."""
    if not _USE_JIT or data.distance_index is None:
        return
    counts = np.asarray([block.state_counts for block in data.blocks])
    origin = np.asarray(data.grid.origin)
    volumes = {}
    for state in (0, 2):
        cand_ids = np.flatnonzero(counts[:, state] > 0)
        if len(cand_ids):
            _jit_tables(data, state, cand_ids, volumes, origin,
                        data.grid.resolution)


def _nearest_jit(handle, data, point, state, radius, work, candidate_blocks,
                 lower_squared, volumes):
    """JIT twin of _nearest: identical traversal semantics (see
    distance_kernels_jit). Both budget halves are enforced at the same
    admission sites as the reference: counts per pop, the clock at a
    bounded stride inside the kernel against the caller's own ABSOLUTE
    deadline, passed through UNCHANGED (both clocks are perf_counter_ns
    in production) -- no re-anchoring read exists, so no two-read gap
    can re-base the budget; see the kernel module header for the
    stride and overshoot bound."""
    cand_ids = candidate_blocks[state]
    tables = _jit_tables(data, state, cand_ids, volumes,
                         np.asarray(data.grid.origin), data.grid.resolution)
    row = lower_squared[tables['block_ids']]
    admitted = np.flatnonzero(row <= radius*radius).astype(np.int64)
    # An empty radius-admitted set is RADIUS_CLEAR (empty heap, best
    # stays None) exactly as the reference traversal produces; the
    # NO_MATCH status belongs to the caller's empty-state-candidates
    # check and is already handled before dispatch.
    cells = np.array([work.cells], dtype=np.int64)
    blocks = np.array([work.blocks], dtype=np.int64)
    status, best_sq, bound_sq, best_flat, best_member, best_bpos = kernel_nearest(
        point, float(radius*radius), admitted, row[admitted],
        tables['blk_leaf_base'], tables['leaf_low'], tables['leaf_high'],
        tables['leaf_gstart'], tables['leaf_gend'],
        tables['mem_low'], tables['mem_high'], tables['mem_flat'],
        work.budget.max_examined_cells, work.budget.max_examined_blocks,
        work.deadline,
        cells, blocks, tables['scratch_bounds'], tables['scratch_order'])
    work.cells, work.blocks = int(cells[0]), int(blocks[0])
    if status == STATUS_BUDGET:
        return DistanceMatch('QUERY_BUDGET_EXHAUSTED', False,
                             lower_bound_m=math.sqrt(bound_sq))
    if status == STATUS_RADIUS_CLEAR:
        return DistanceMatch('RADIUS_CLEAR', False, lower_bound_m=radius)
    block_id = int(tables['block_ids'][best_bpos])
    offset = int(tables['mem_offset'][best_member])
    return DistanceMatch('MATCH', True, math.sqrt(best_sq), math.sqrt(best_sq),
                         handle._cell_record(data.blocks[block_id], offset))


def _nearest(handle, data, point, state, radius, work, candidate_blocks, lower_squared, volumes):
    candidates = candidate_blocks[state]
    if not len(candidates):
        return DistanceMatch('NO_MATCH', True)
    if _USE_JIT:
        zero = _zero_match(handle, data, point, state, work)
        if zero is not None:
            return zero
        return _nearest_jit(handle, data, point, state, radius, work,
                            candidate_blocks, lower_squared, volumes)
    zero = _zero_match(handle, data, point, state, work)
    if zero is not None:
        return zero
    # Heap tuples include block ID to keep traversal deterministic. Equal
    # lower bounds must be visited: another block may contain a smaller flat ID.
    candidates = candidates[lower_squared[candidates] <= radius*radius]
    queue = [(float(lower_squared[b]), int(b), -1) for b in candidates]
    heapq.heapify(queue)
    best_squared = radius*radius
    best = None
    r = data.grid.resolution
    origin = np.asarray(data.grid.origin)
    # Each block exposes only its next leaf to the global heap. Sorted
    # bounds and stable leaf IDs preserve the original global visit order
    # without allocating heap entries for leaves that are never visited.
    leaf_cursors = {}

    def push_next_leaf(block_id):
        order, bounds, position = leaf_cursors[block_id]
        if position < len(order):
            index = int(order[position])
            if bounds[index] <= best_squared:
                heapq.heappush(queue, (float(bounds[index]), block_id, index))
                leaf_cursors[block_id][2] += 1

    while queue:
        lower, block_id, leaf = heapq.heappop(queue)
        if lower > best_squared:
            break
        block = data.blocks[block_id]
        key = (block_id, state)
        cached = volumes.get(key)
        # Decode each visited block's immutable derived membership once.
        # Heap leaves then test only volumes
        # whose exact bounding box can improve/tie the current nearest cell.
        # Every hierarchy visit and leaf volume test is charged. Reading
        # precomputed membership does not scan primary/block voxel records.
        examined = 0 if leaf == -1 else (
            int(cached[4][leaf]-cached[3][leaf]))
        if not work.admits(examined):
            return DistanceMatch('QUERY_BUDGET_EXHAUSTED', False,
                                 lower_bound_m=math.sqrt(min(lower, best_squared)))
        work.cells += examined
        work.blocks += 1
        if cached is None:
            members = block.distance_members[state]
            offsets = np.frombuffer(members.offsets,dtype='<u2')
            starts = np.frombuffer(members.starts,dtype='<u2')
            ends = np.frombuffer(members.ends,dtype='<u2')
            ids = np.frombuffer(members.leaf_ids,dtype='u1')
            low = origin+(np.asarray(block.start)+block.distance_layout.lower[ids])*r
            high = origin+(np.asarray(block.start)+block.distance_layout.upper[ids])*r
            # The final dict memoizes only leaf volumes actually visited in
            # this request. The retained index holds membership, not distance.
            cached = volumes[key] = (offsets, low, high, starts, ends, {})
        offsets, leaf_low, leaf_high, starts, ends, cells = cached
        if leaf == -1:
            delta = np.maximum(np.maximum(leaf_low-point, point-leaf_high), 0.)
            bounds = np.einsum('ij,ij->i', delta, delta)
            order = np.flatnonzero(bounds <= best_squared)
            order = order[np.argsort(bounds[order], kind='stable')]
            leaf_cursors[block_id] = [order, bounds, 0]
            push_next_leaf(block_id)
            continue
        start, end = starts[leaf], ends[leaf]
        offsets = offsets[start:end]
        if leaf not in cells:
            # Layouts are immutable and shared by equal block shapes. Integer
            # addition preserves the previous world-AABB arithmetic exactly.
            xyz = block.distance_layout.cell_coordinates[offsets]+np.asarray(block.start)
            cells[leaf] = (origin+xyz*r,origin+(xyz+1)*r)
        low, high = cells[leaf]
        displacement = np.maximum(np.maximum(low-point, point-high), 0.)
        squared = np.einsum('ij,ij->i', displacement, displacement)
        minimum = float(squared.min())
        # Cells within a leaf remain ascending after the stable grouping.
        offset = int(offsets[np.flatnonzero(squared == minimum)[0]])
        flat_id = int(block.flat_indices[offset])
        if (minimum < best_squared or (minimum == best_squared
                and (best is None or flat_id < best[0]))):
            best_squared, best = minimum, (flat_id, block, offset)
        push_next_leaf(block_id)
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
    # Membership is fixed for this pinned batch; filter once for each target
    # state rather than rescanning the same block summaries for every point.
    candidate_blocks = {state: np.flatnonzero(counts[:, state] > 0) for state in (0, 2)}
    origin = np.asarray(data.grid.origin)
    upper = origin+np.asarray(data.grid.shape)*data.grid.resolution
    # Block lower bounds for every point in one broadcast: identical
    # element arithmetic to the former per-point (n_blocks, 3) pass, so
    # admission masks and heap keys are bit-for-bit unchanged.
    delta = np.maximum(np.maximum(data.distance_index.lower[None, :, :]
                                  - points[:, None, :],
                                  points[:, None, :]
                                  - data.distance_index.upper[None, :, :]), 0.)
    lower_squared_all = np.einsum('ijk,ijk->ij', delta, delta)
    results = []
    volumes = {}
    for index, point in enumerate(points):
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
        if _USE_JIT and time.perf_counter_ns() >= work.deadline:
            # Fast fail before dispatching into the kernel (the kernel
            # checks its own clock against the absolute deadline at the
            # same admission sites, so an expiry mid-traversal -- or in
            # the dispatch-to-entry gap -- also returns
            # QUERY_BUDGET_EXHAUSTED exactly like the reference's failed
            # admits).
            match = DistanceMatch('QUERY_BUDGET_EXHAUSTED', False)
            results.append(DistancePoint(tuple(float(v) for v in point),
                                         'QUERY_BUDGET_EXHAUSTED', match,
                                         match if unknown_policy == 'separate' else None))
            continue
        lower_squared = lower_squared_all[index]
        occupied = _nearest(handle, data, point, 2, radius, work, candidate_blocks, lower_squared, volumes)
        unknown = (_nearest(handle, data, point, 0, radius, work, candidate_blocks, lower_squared, volumes)
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
