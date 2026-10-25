"""Schema-2 read-only voxel queries over immutable, versioned block data.

Coordinates are container_link AABBs, with positive-volume cell intersection.
Legacy mapper enumeration remains separate; these pages use flat-index order.
Snapshot handles contain no mapper or live-array reference. Their provider
retains at most two generations and expires handles after five wall seconds.
"""
from dataclasses import dataclass, field
import base64
import hashlib
import hmac
import json
import math
import time
from typing import NamedTuple

import numpy as np

INDEX_SCHEMA_VERSION = 2
BLOCK_EDGE = 16
MAX_PAGE = 4096
CELL_DTYPE = np.dtype([('state', 'u1'), ('source', 'u1'), ('label', '<i8'), ('instance', '<i8')])


class QueryError(ValueError):
    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class QueryIdentity(NamedTuple):
    epoch: str
    revision: int
    content_digest: str
    voxel_digest: str
    index_schema_version: int = INDEX_SCHEMA_VERSION


@dataclass(frozen=True)
class GridInfo:
    shape: tuple
    origin: tuple
    resolution: float
    frame_id: str
    geometry_hash: str
    hull: object

    def __post_init__(self):
        if (type(self.shape) is not tuple or type(self.origin) is not tuple
                or len(self.shape) != 3 or any(type(v) is not int or v <= 0 for v in self.shape)
                or len(self.origin) != 3 or not all(math.isfinite(v) for v in self.origin)
                or not math.isfinite(self.resolution) or self.resolution <= 0
                or self.frame_id != 'container_link'):
            raise ValueError('invalid query grid')


@dataclass(frozen=True)
class CellKey:
    map_epoch: str
    ix: int
    iy: int
    iz: int


@dataclass(frozen=True)
class InstanceKey:
    kind: str
    identifier: object
    namespace: str = ''


@dataclass(frozen=True)
class QueryPage:
    limit: int = 1024
    cursor: str = ''


@dataclass(frozen=True)
class CellRecord:
    key: CellKey
    flat_index: int
    state: int
    source: int
    label: int
    instance_id: int
    geometry_membership: tuple = ()


@dataclass(frozen=True)
class QueryResult:
    status: str
    reason: str
    snapshot_identity: QueryIdentity
    grid: GridInfo
    complete: bool = True
    next_cursor: str = ''
    returned_count: int = 0
    examined_cells: int = 0
    examined_blocks: int = 0
    elapsed_ns: int = 0

    @property
    def voxel_identity(self):
        return self.snapshot_identity.voxel_digest

    @property
    def index_version(self):
        return self.snapshot_identity.index_schema_version

    @property
    def frame_id(self):
        return self.grid.frame_id


@dataclass(frozen=True)
class CellPage(QueryResult):
    cells: tuple = ()


@dataclass(frozen=True)
class CountResult(QueryResult):
    count: int = 0


@dataclass(frozen=True)
class MapSummary(QueryResult):
    values: object = None
    occupancy_confirmed: bool = False
    index_status: str = 'READY'
    acquisition_stamp_ns: int = 0
    map_source: str = ''
    cargo_observed: bool = False
    pending_changes: tuple = ()
    change_event: str = ''

    def as_dict(self):
        result = dict(self.values)
        result['label_distribution'] = dict(result['label_distribution'])
        return result


@dataclass(frozen=True)
class SurfaceResult(QueryResult):
    origin_index_xy: tuple = (0, 0)
    shape_xy: tuple = (0, 0)
    columns: object = None
    floor_z_container: float = 0.
    ceiling_z_container: float = 0.

    def as_dict(self):
        return {name: [list(row) for row in values] for name, values in self.columns.items()}


@dataclass(frozen=True)
class SurfaceColumn:
    ix: int
    iy: int
    height: float
    clearance: float
    state: str
    known_ratio: float
    confidence: str
    semantic_label: int
    surface_z_container: object
    height_above_floor: object
    contract_confidence: str


@dataclass(frozen=True)
class SurfacePatchResult(QueryResult):
    since_identity: object = None
    patches: tuple = ()


@dataclass(frozen=True)
class DistanceMatch:
    status: str
    exact: bool
    distance_m: object = None
    lower_bound_m: float = 0.
    cell: object = None


@dataclass(frozen=True)
class DistancePoint:
    point: tuple
    status: str
    occupied: DistanceMatch
    unknown: object = None


@dataclass(frozen=True)
class DistanceBatch(QueryResult):
    unknown_policy: str = 'separate'
    results: tuple = ()


@dataclass(frozen=True)
class VoxelBlock:
    block_id: int
    start: tuple
    shape: tuple
    flat_indices: np.ndarray = field(repr=False, compare=False)
    evidence: bytes = field(repr=False)
    active: bytes = field(repr=False)
    frontier: bytes = field(repr=False)
    state_counts: tuple
    frontier_count: int
    sensor_members: object = field(repr=False, compare=False)
    box_members: object = field(repr=False, compare=False)
    fact_digest: str
    distance_layout: object = field(repr=False, compare=False)
    distance_members: object = field(repr=False, compare=False)
    _retained_bytes: int = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        # Cache this block's owned payload charge. Shape-shared immutable
        # distance layouts are charged once by the registry across generations.
        size = (len(self.evidence) + len(self.active) + len(self.frontier)
                + self.flat_indices.nbytes
                + sum(len(v) for v in self.sensor_members.values())
                + sum(len(v) for v in self.box_members.values())
                + sum(v.byte_size() for v in self.distance_members.values())
                + 1024 + 448*(len(self.sensor_members)+len(self.box_members)))
        object.__setattr__(self, '_retained_bytes', size)

    def records(self):
        return np.frombuffer(self.evidence, dtype=CELL_DTYPE)

    def byte_size(self):
        # Owned payloads plus conservative container allowance. Registry also
        # charges each distinct shared distance layout reachable from blocks.
        return self._retained_bytes


@dataclass(frozen=True)
class QuerySnapshotData:
    identity: QueryIdentity
    grid: GridInfo
    blocks: tuple
    sensor_blocks: object
    box_blocks: object
    summary_values: object
    map_snapshot: object
    cursor_secret: bytes = field(repr=False)
    owner_evidence_bytes: int = 0
    unbound_geometry_count: int = 0
    surface_projection: object = None
    distance_index: object = None


def normalize_region(region):
    if region is None:
        return None
    try:
        low, high = (tuple(float(v) for v in p) for p in region)
        if (len(low) != 3 or len(high) != 3
                or not all(math.isfinite(v) for v in low + high)
                or any(a >= b for a, b in zip(low, high))):
            raise ValueError()
        return low, high
    except (ValueError, TypeError, OverflowError):
        raise QueryError('INVALID_REGION') from None


def block_bounds(block, grid):
    low = np.asarray(grid.origin) + np.asarray(block.start) * grid.resolution
    return low, low + np.asarray(block.shape) * grid.resolution


def region_intersects(low, high, region):
    return region is None or bool(np.all(high > region[0]) and np.all(low < region[1]))


@dataclass(frozen=True)
class MapQuerySnapshot:
    """A bounded read handle; provider owns lifecycle, never primary facts."""
    identity: QueryIdentity
    grid: GridInfo
    _token: str = field(repr=False)
    _provider: object = field(repr=False, compare=False)

    def _data(self):
        return self._provider.resolve(self._token, self.identity)

    def close(self):
        """Release a query session early; closing never changes map data."""
        self._provider.close(self)

    def __enter__(self):
        self._data()
        return self

    def __exit__(self, *_):
        self.close()

    def summary(self):
        start = time.perf_counter_ns()
        data = self._data()
        return MapSummary('OK', '', self.identity, self.grid,
                          elapsed_ns=time.perf_counter_ns()-start,
                          values=data.summary_values,
                          occupancy_confirmed=data.map_snapshot.occupancy_confirmed,
                          acquisition_stamp_ns=data.map_snapshot.acquisition_stamp_ns,
                          map_source=data.map_snapshot.map_source,
                          cargo_observed=data.map_snapshot.cargo_observed,
                          pending_changes=data.map_snapshot.pending_changes,
                          change_event=data.map_snapshot.change_event)

    def get_cell(self, key):
        data = self._data()
        if not isinstance(key, CellKey) or key.map_epoch != self.identity.epoch:
            raise QueryError('SNAPSHOT_IDENTITY_MISMATCH')
        coordinates = (key.ix, key.iy, key.iz)
        if any(type(v) is not int or not 0 <= v < n for v, n in zip(coordinates, self.grid.shape)):
            raise QueryError('OUTSIDE_MAP')
        nx, ny, _ = self.grid.shape
        bx, by = ((n+BLOCK_EDGE-1)//BLOCK_EDGE for n in self.grid.shape[:2])
        block_id = key.ix//BLOCK_EDGE + bx*(key.iy//BLOCK_EDGE + by*(key.iz//BLOCK_EDGE))
        block = data.blocks[block_id]
        flat = key.ix + nx*(key.iy + ny*key.iz)
        offset = int(np.searchsorted(block.flat_indices, flat))
        if not block.active[offset]:
            raise QueryError('OUTSIDE_HULL')
        return self._cell_record(block, offset)

    def _cell_record(self, block, offset):
        flat = int(block.flat_indices[offset])
        nx, ny, _ = self.grid.shape
        row = block.records()[offset]
        boxes = tuple(sorted(identifier for identifier, members in block.box_members.items()
                             if offset in np.frombuffer(members, dtype='<u2')))
        return CellRecord(CellKey(self.identity.epoch, flat % nx, flat//nx % ny, flat//(nx*ny)),
                          flat, int(row['state']), int(row['source']), int(row['label']),
                          int(row['instance']), boxes)

    def _cell_records(self, block, offsets):
        """Expand a page's memberships once per block, preserving row order.

        Stored member offsets are sorted. Batched binary searches avoid
        scanning each box's full member list for each output cell.
        """
        offsets = np.asarray(offsets, dtype=np.int64)
        memberships = [[] for _ in offsets]
        for identifier in sorted(block.box_members):
            members = np.frombuffer(block.box_members[identifier], dtype='<u2')
            if not len(members):
                continue
            positions = np.searchsorted(members, offsets)
            matched = (positions < len(members)) & (members[np.minimum(positions, len(members)-1)] == offsets)
            for index in np.flatnonzero(matched):
                memberships[index].append(identifier)
        flat = block.flat_indices[offsets]
        rows = block.records()[offsets]
        nx, ny, _ = self.grid.shape
        # Convert each column once to built-in ints instead of constructing
        # structured NumPy scalars and converting their fields for every cell.
        return [CellRecord(CellKey(self.identity.epoch, i % nx, i//nx % ny, i//(nx*ny)),
            i, state, source, label, instance, tuple(boxes))
            for i, state, source, label, instance, boxes in zip(
                flat.tolist(), rows['state'].tolist(), rows['source'].tolist(),
                rows['label'].tolist(), rows['instance'].tolist(), memberships)]

    def query_cells(self, region=None, states=(0, 1, 2), instance_filter=None, page=None):
        return self._query(region, states, instance_filter, page, frontier=False)

    def query_frontiers(self, region=None, page=None):
        return self._query(region, (0,), None, page, frontier=True)

    def query_instance(self, instance_key, region=None, page=None):
        return self._query(region, (2,), instance_key, page, frontier=False)

    def surface(self, region=None):
        from luggage_perception.cargo_surface_projection import read_surface
        return read_surface(self, self._data(), region)

    def nearest_obstacles(self, points, max_radius_m=.30, unknown_policy='separate', *, budget=None):
        """Exact point-to-cell-volume distance; never a collision certificate.

        'separate' reports independent occupied and unknown distances;
        'occupied_only' explicitly leaves unknown unqueried, never free.
        Optional budget can only reduce the fixed read-work/time limits.
        """
        from luggage_perception.cargo_distance_query import read_nearest
        return read_nearest(self, self._data(), points, max_radius_m, unknown_policy, budget)

    def surface_changes(self, since_identity, region=None, page=None):
        from luggage_perception.cargo_surface_projection import read_surface_changes
        data = self._data()
        previous = self._provider.lookup_since(self.identity, since_identity)
        return read_surface_changes(self, data, previous, region, page)

    def _instance(self, data, instance):
        if instance is None:
            return None, None, None
        if not isinstance(instance, InstanceKey):
            raise QueryError('INVALID_FILTER')
        if instance.kind == 'sensor_instance':
            if type(instance.identifier) is not int or instance.identifier <= 0:
                raise QueryError('INVALID_FILTER')
            # Legacy producer has no reliable session binding. An explicit
            # epoch-local namespace prevents cross-run identity fabrication.
            if instance.namespace != 'legacy:' + self.identity.epoch:
                raise QueryError('INSTANCE_BINDING_MISSING')
            return 'sensor', instance.identifier, data.sensor_blocks.get(instance.identifier, ())
        if instance.kind == 'committed_box':
            if not isinstance(instance.identifier, str) or not instance.identifier:
                raise QueryError('INVALID_FILTER')
            if instance.namespace not in ('', self.identity.epoch):
                raise QueryError('INSTANCE_BINDING_MISSING')
            if instance.identifier not in data.box_blocks:
                raise QueryError('INSTANCE_BINDING_MISSING')
            return 'box', instance.identifier, data.box_blocks[instance.identifier]
        raise QueryError('INVALID_FILTER')

    def _query(self, region, states, instance, page, *, frontier):
        started = time.perf_counter_ns()
        data = self._data()
        region = normalize_region(region)
        try:
            states = tuple(sorted(set(states)))
        except TypeError:
            raise QueryError('INVALID_FILTER') from None
        if not states or any(type(s) is not int or s not in (0, 1, 2) for s in states):
            raise QueryError('INVALID_FILTER')
        page = page if page is not None else QueryPage()
        if (not isinstance(page, QueryPage) or type(page.limit) is not int
                or not 1 <= page.limit <= MAX_PAGE or not isinstance(page.cursor, str)):
            raise QueryError('INVALID_FILTER')
        kind, identifier, instance_blocks = self._instance(data, instance)
        query_hash = hashlib.sha256(json.dumps((region, states, kind, identifier, frontier),
                                              separators=(',', ':')).encode()).hexdigest()
        last = self._decode_cursor(data, page.cursor, query_hash) if page.cursor else -1
        block_ids = instance_blocks if instance_blocks is not None else range(len(data.blocks))
        # A block's minimum flat index is a safe ordering lower bound. Once
        # it exceeds the bounded selected prefix, no later block can win.
        blocks = sorted((data.blocks[i] for i in block_ids), key=lambda b: int(b.flat_indices[0]))
        selected = []
        examined_cells = examined_blocks = 0
        nx, ny, _ = self.grid.shape
        for block in blocks:
            if len(selected) > page.limit and int(block.flat_indices[0]) > selected[-1][0]:
                break
            if int(block.flat_indices[-1]) <= last:
                continue
            if frontier and block.frontier_count == 0:
                continue
            if not any(block.state_counts[s] for s in states):
                continue
            if not region_intersects(*block_bounds(block, self.grid), region):
                continue
            if kind:
                members = (block.sensor_members if kind == 'sensor' else block.box_members)[identifier]
                offsets = np.frombuffer(members, dtype='<u2').astype(np.int64)
            else:
                offsets = np.arange(len(block.flat_indices), dtype=np.int64)
            examined_blocks += 1
            examined_cells += len(offsets)
            rows = block.records()[offsets]
            flat = block.flat_indices[offsets]
            mask = np.frombuffer(block.active, 'u1')[offsets].astype(bool) & (flat > last)
            mask &= np.isin(rows['state'], states)
            if frontier:
                mask &= np.frombuffer(block.frontier, 'u1')[offsets].astype(bool)
            if region is not None:
                xyz = np.column_stack((flat % nx, flat//nx % ny, flat//(nx*ny)))
                low = np.asarray(self.grid.origin) + xyz*self.grid.resolution
                # Compute upper face independently, avoiding low+r drift at
                # exact grid boundaries; both inequalities are strict.
                high = np.asarray(self.grid.origin) + (xyz+1)*self.grid.resolution
                mask &= ((high > region[0]) & (low < region[1])).all(axis=1)
            match = offsets[mask][:page.limit+1]
            selected.extend((int(block.flat_indices[o]), block.block_id, int(o)) for o in match)
            selected.sort()
            del selected[page.limit+1:]
        complete = len(selected) <= page.limit
        selected = selected[:page.limit]
        by_block = {}
        for position, (_, block_id, offset) in enumerate(selected):
            by_block.setdefault(block_id, []).append((position, offset))
        cells = [None]*len(selected)
        for block_id, entries in by_block.items():
            records = self._cell_records(data.blocks[block_id], [offset for _, offset in entries])
            for (position, _), record in zip(entries, records):
                cells[position] = record
        cells = tuple(cells)
        cursor = '' if complete else self._encode_cursor(data, query_hash, selected[-1][0])
        return CellPage('OK', '', self.identity, self.grid, complete=complete,
                        next_cursor=cursor, returned_count=len(cells), examined_cells=examined_cells,
                        examined_blocks=examined_blocks, elapsed_ns=time.perf_counter_ns()-started,
                        cells=cells)

    def count_frontiers(self, region=None):
        start = time.perf_counter_ns()
        data = self._data()
        region = normalize_region(region)
        count = examined = blocks = 0
        if region is None:
            count = data.summary_values['frontier_count']
        else:
            nx, ny, _ = self.grid.shape
            for block in data.blocks:
                low, high = block_bounds(block, self.grid)
                if not block.frontier_count or not region_intersects(low, high, region):
                    continue
                blocks += 1
                if np.all(low >= region[0]) and np.all(high <= region[1]):
                    count += block.frontier_count
                    continue
                offsets = np.flatnonzero(np.frombuffer(block.frontier, 'u1'))
                flat = block.flat_indices[offsets]
                xyz = np.column_stack((flat % nx, flat//nx % ny, flat//(nx*ny)))
                low = np.asarray(self.grid.origin) + xyz*self.grid.resolution
                high = np.asarray(self.grid.origin) + (xyz+1)*self.grid.resolution
                count += int(((high > region[0]) & (low < region[1])).all(axis=1).sum())
                examined += len(offsets)
        return CountResult('OK', '', self.identity, self.grid, count=count,
                           examined_cells=examined, examined_blocks=blocks,
                           elapsed_ns=time.perf_counter_ns()-start)

    def _encode_cursor(self, data, query_hash, last):
        body = json.dumps((tuple(self.identity), query_hash, last), separators=(',', ':')).encode()
        signature = hmac.digest(data.cursor_secret, body, 'sha256')
        return base64.urlsafe_b64encode(signature+body).decode()

    def _decode_cursor(self, data, cursor, query_hash):
        try:
            if len(cursor) > 2048:
                raise ValueError()
            raw = base64.b64decode(cursor, altchars=b'-_', validate=True)
            signature, body = raw[:32], raw[32:]
            if not hmac.compare_digest(signature, hmac.digest(data.cursor_secret, body, 'sha256')):
                raise ValueError()
            identity, query, last = json.loads(body)
            if tuple(identity) != tuple(self.identity) or query != query_hash or type(last) is not int or last < 0:
                raise ValueError()
            return last
        except (ValueError, TypeError, json.JSONDecodeError):
            raise QueryError('INVALID_CURSOR') from None
