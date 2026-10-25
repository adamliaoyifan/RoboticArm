"""Store-owned derived counts and frontier membership (schema 1).

Only initialize/apply_delta/reset mutate the owner. Queries use detached,
deeply immutable summaries. Native array operations never change primary OCC.
"""
from collections import Counter
from dataclasses import dataclass
from types import MappingProxyType
import hashlib
import json
import secrets

import numpy as np

from luggage_perception.cargo_grid_projection import occupancy_counts
from luggage_perception.cargo_voxel_delta import EVIDENCE_DTYPE, capture_evidence, immutable_array
from luggage_perception.cargo_map_query import (
    BLOCK_EDGE, CELL_DTYPE, GridInfo, QueryIdentity, QuerySnapshotData, VoxelBlock,
)


def _geometry_key(record):
    return (tuple(record['center']), tuple(record['size']), float(record['yaw']))


def _block_ids(indices, mapper):
    nx, ny = mapper.nx, mapper.ny
    bx, by = (nx+BLOCK_EDGE-1)//BLOCK_EDGE, (ny+BLOCK_EDGE-1)//BLOCK_EDGE
    return (indices % nx)//BLOCK_EDGE + bx*((indices//nx % ny)//BLOCK_EDGE
           + by*((indices//(nx*ny))//BLOCK_EDGE))


def _freeze_block(block_id, start, shape, keys, facts, active, frontier, box_members):
    # Only the owner retains full canonical evidence for incremental hashes.
    # Read generations retain the compact public cell fields; old snapshots
    # cannot multiply private log-odds/shadow copies or expose that evidence.
    canonical = np.frombuffer(facts, dtype=EVIDENCE_DTYPE)
    records = np.empty(len(canonical), dtype=CELL_DTYPE)
    for name in CELL_DTYPE.names:
        records[name] = canonical[name]
    allowed = np.frombuffer(active, 'u1').astype(bool)
    counts = tuple(int(v) for v in np.bincount(records['state'][allowed], minlength=3))
    live_sensor = allowed & (records['state'] == 2) & (records['instance'] > 0)
    sensor_members = {}
    for identifier in np.unique(records['instance'][live_sensor]):
        offsets = np.flatnonzero(live_sensor & (records['instance'] == identifier))
        sensor_members[int(identifier)] = offsets.astype('<u2').tobytes()
    boxes = MappingProxyType(dict(box_members))
    digest = hashlib.sha256(facts+active)
    for identifier in sorted(boxes):
        encoded = identifier.encode('utf-8')
        digest.update(len(encoded).to_bytes(4, 'little'))
        digest.update(encoded)
        digest.update(len(boxes[identifier]).to_bytes(4, 'little'))
        digest.update(boxes[identifier])
    return VoxelBlock(block_id, start, shape, keys, records.tobytes(), active, frontier,
                      counts, int(np.frombuffer(frontier, 'u1').sum()),
                      MappingProxyType(sensor_members), boxes, digest.hexdigest())


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
        self._blocks = ()
        self._fact_blocks = ()
        self._bindings = {}
        self._box_indices = {}
        self._geometry_members = {}
        self._box_blocks = {}
        self._known_box_ids = set()
        self._ambiguous_box_ids = set()
        self._sensor_blocks = {}
        self._cursor_secret = secrets.token_bytes(32)

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
        self._initialize_blocks(mapper)

    def reset(self, mapper):
        self.initialize(mapper)

    def apply_delta(self, mapper, delta, update=None, committed_now=False):
        if delta.rebuild:
            existing = set(_geometry_key(r) for r in mapper._placed_boxes)
            bindings = {k: v for k, v in self._bindings.items() if k in existing}
            self.reset(mapper)
            self._apply_bindings(mapper, delta, update, committed_now, bindings)
            self._update_blocks(mapper, delta, np.empty(0, dtype=np.int64))
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
        self._apply_bindings(mapper, delta, update, committed_now)
        self._update_blocks(mapper, delta, candidates)

    def _initialize_blocks(self, mapper):
        self._bindings = {}
        self._box_indices = {}
        self._geometry_members = {}
        self._box_blocks = {}
        self._known_box_ids = set()
        self._ambiguous_box_ids = set()
        self._sensor_blocks = {}
        self._cursor_secret = secrets.token_bytes(32)
        blocks = []
        fact_blocks = []
        nx, ny, nz = mapper.nx, mapper.ny, mapper.nz
        active = np.frombuffer(mapper._active, 'u1')
        for z in range(0, nz, BLOCK_EDGE):
            for y in range(0, ny, BLOCK_EDGE):
                for x in range(0, nx, BLOCK_EDGE):
                    shape = (min(BLOCK_EDGE, nx-x), min(BLOCK_EDGE, ny-y), min(BLOCK_EDGE, nz-z))
                    ix = np.arange(x, x+shape[0], dtype=np.int32)
                    iy = np.arange(y, y+shape[1], dtype=np.int32)
                    iz = np.arange(z, z+shape[2], dtype=np.int32)
                    keys = immutable_array((ix[None, None, :]+nx*(iy[None, :, None]+ny*iz[:, None, None])).ravel())
                    facts = capture_evidence(mapper, keys).tobytes()
                    fact_blocks.append(facts)
                    block = _freeze_block(len(blocks), (x, y, z), shape, keys,
                        facts, active[keys].tobytes(),
                        self._frontiers[keys].astype('u1').tobytes(), {})
                    blocks.append(block)
                    for identifier in block.sensor_members:
                        self._sensor_blocks.setdefault(identifier, set()).add(block.block_id)
        self._blocks = tuple(blocks)
        self._fact_blocks = tuple(fact_blocks)
        self._geometry_dirty_blocks = set()

    def _apply_bindings(self, mapper, delta, update, committed_now, restored=None):
        old_bindings = self._bindings
        existing = {_geometry_key(r) for r in mapper._placed_boxes}
        bindings = {k: v for k, v in (restored if restored is not None else old_bindings).items()
                    if k in existing}
        if committed_now and update is not None and update.box_id:
            bindings[_geometry_key(mapper._placed_boxes[-1])] = str(update.box_id)
            self._known_box_ids.add(str(update.box_id))
        if restored:
            self._known_box_ids.update(restored.values())
        for raster in delta.geometry_rasters:
            if raster.key in existing:
                self._box_indices[raster.key] = raster.indices
        self._box_indices = {k: v for k, v in self._box_indices.items() if k in existing}
        changed = {k for k in old_bindings.keys() | bindings.keys()
                   if old_bindings.get(k) != bindings.get(k)}
        old_ids = {old_bindings[k] for k in changed if k in old_bindings}
        new_ids = {bindings[k] for k in changed if k in bindings}
        # A repeated purported ID on distinct ledger boxes is ambiguous. Do
        # not repair the ledger or invent an identity; close that query.
        id_counts = Counter(bindings.values())
        ambiguous = {v for v, count in id_counts.items() if count != 1}
        new_ids |= {v for v in bindings.values() if v not in self._box_blocks}
        new_ids -= ambiguous
        old_ids |= ambiguous & self._box_blocks.keys()
        dirty = set()
        for identifier in old_ids:
            for bid in self._box_blocks.pop(identifier, ()):
                members = dict(self._geometry_members.get(bid, {}))
                members.pop(identifier, None)
                self._geometry_members[bid] = members
                dirty.add(bid)
        for key, identifier in bindings.items():
            if identifier not in new_ids:
                continue
            indices = self._box_indices.get(key)
            if indices is None:
                continue
            ids = _block_ids(indices, mapper)
            bids = tuple(int(v) for v in np.unique(ids))
            self._box_blocks[identifier] = bids
            for bid in bids:
                offsets = np.searchsorted(self._blocks[bid].flat_indices, indices[ids == bid])
                members = dict(self._geometry_members.get(bid, {}))
                members[identifier] = np.unique(offsets).astype('<u2').tobytes()
                self._geometry_members[bid] = members
                dirty.add(bid)
        self._bindings = bindings
        self._ambiguous_box_ids = ambiguous
        self._geometry_dirty_blocks = dirty

    def _update_blocks(self, mapper, delta, frontier_candidates):
        ids = _block_ids(delta.indices, mapper)
        dirty = set(int(v) for v in np.unique(_block_ids(frontier_candidates, mapper)))
        dirty.update(int(v) for v in np.unique(ids))
        dirty.update(self._geometry_dirty_blocks)
        blocks = list(self._blocks)
        fact_blocks = list(self._fact_blocks)
        copied = 0
        for bid in sorted(dirty):
            old = blocks[bid]
            selected = np.flatnonzero(ids == bid)
            facts = fact_blocks[bid]
            if len(selected):
                records = np.frombuffer(facts, dtype=EVIDENCE_DTYPE).copy()
                offsets = np.searchsorted(old.flat_indices, delta.indices[selected])
                records[offsets] = delta.after[selected]
                facts = records.tobytes()
                fact_blocks[bid] = facts
                copied += len(facts)
            block = _freeze_block(bid, old.start, old.shape, old.flat_indices, facts,
                old.active, self._frontiers[old.flat_indices].astype('u1').tobytes(),
                self._geometry_members.get(bid, {}))
            for identifier in old.sensor_members.keys() - block.sensor_members.keys():
                members = self._sensor_blocks[identifier]
                members.discard(bid)
                if not members:
                    del self._sensor_blocks[identifier]
            for identifier in block.sensor_members.keys() - old.sensor_members.keys():
                self._sensor_blocks.setdefault(identifier, set()).add(bid)
            blocks[bid] = block
        self._blocks = tuple(blocks)
        self._fact_blocks = tuple(fact_blocks)
        self._geometry_dirty_blocks = set()
        work = dict(self._last_work)
        work.update(dirty_blocks=len(dirty), copied_evidence_bytes=copied)
        self._last_work = MappingProxyType(work)

    def freeze_query(self, mapper, map_snapshot, summary):
        """Read-only construction; references only immutable block values."""
        origin = (-mapper.inner_l/2, -mapper.inner_w/2, mapper._hull.floor_z)
        grid = GridInfo((mapper.nx, mapper.ny, mapper.nz), origin,
                        mapper.resolution, 'container_link', mapper.geometry_hash, mapper._hull)
        root = hashlib.sha256(json.dumps((grid.shape, grid.origin, grid.resolution,
                                         grid.geometry_hash), separators=(',', ':')).encode())
        for block in self._blocks:
            root.update(bytes.fromhex(block.fact_digest))
        # Include missing/ambiguous binding status as well as member hashes.
        root.update(json.dumps(sorted((k, v) for k, v in self._bindings.items()),
                               separators=(',', ':')).encode())
        root.update(json.dumps(sorted(self._known_box_ids), separators=(',', ':')).encode())
        # Legacy content_digest intentionally omits publication provenance.
        # Bind that provenance here so a pinned query cannot silently switch
        # acquisition/source/confirmation details under equal legacy legs.
        root.update(json.dumps((map_snapshot.acquisition_stamp_ns, map_snapshot.map_source,
            map_snapshot.cargo_observed, map_snapshot.change_token, map_snapshot.pending_details,
            map_snapshot.last_settle, map_snapshot.ledger_details),
            separators=(',', ':'), sort_keys=True).encode())
        identity = QueryIdentity(*map_snapshot.identity, root.hexdigest())
        return QuerySnapshotData(identity, grid, self._blocks,
            MappingProxyType({k: tuple(sorted(v)) for k, v in self._sensor_blocks.items()}),
            MappingProxyType({k: self._box_blocks.get(k, ()) for k in self._known_box_ids
                              if k not in self._ambiguous_box_ids}), summary.values, map_snapshot,
            self._cursor_secret, owner_evidence_bytes=sum(len(b) for b in self._fact_blocks),
            unbound_geometry_count=len(mapper._placed_boxes)-len(self._box_blocks))

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
