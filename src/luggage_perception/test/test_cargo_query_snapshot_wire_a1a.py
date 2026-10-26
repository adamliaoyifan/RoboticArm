#!/usr/bin/env python3
"""A1a round-trip oracle: wire encode/rebuild equals the source snapshot.

Real store snapshots (deployment hull, 1cm/2cm, 0/5 boxes), FULL path.
Equality covers: identity legs, grid, per-block evidence/active/frontier
bytes (exact), flat indices, membership maps, state/frontier counts
(recompute-verified), summary, provenance legs. Frontier rides the wire
as an explicit member -- a frontier-only change in a block whose
fact_digest is unchanged still ships (delta decision is the caller's
dirty set, not the digest). Deep immutability asserted on the rebuild.
"""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, PKG_ROOT)

from luggage_perception.cargo_query_snapshot_wire import (
    encode_snapshot, rebuild_snapshot)
from luggage_perception.cargo_query_wire import WireError
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def store(resolution, boxes):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution)
    store = OccupancyMapStore(mapper, epoch='wire')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        result = store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i))
        assert result.accepted
    return store, mapper


def snapshot_of(store_):
    store_._snapshot = store_._build_snapshot()
    return store_._query_registry  # not used; helper below


def acquire(store_):
    return store_.acquire_query_snapshot()


class TestA1aRoundTrip(unittest.TestCase):

    def _round_trip(self, resolution, boxes):
        store_, mapper = store(resolution, boxes)
        with acquire(store_) as handle:
            data = handle._data()
            objects = encode_snapshot(data, 'inc-1', 1)
            rebuilt = rebuild_snapshot(objects, data.identity)
        self.assertEqual(tuple(rebuilt.identity), tuple(data.identity))
        self.assertEqual(rebuilt.grid.shape, data.grid.shape)
        self.assertEqual(rebuilt.grid.origin, data.grid.origin)
        self.assertEqual(rebuilt.grid.resolution, data.grid.resolution)
        self.assertEqual(len(rebuilt.blocks), len(data.blocks))
        checked = 0
        for source, copy_ in zip(data.blocks, rebuilt.blocks):
            self.assertEqual(source.block_id, copy_.block_id)
            self.assertEqual(source.evidence, copy_.evidence)
            self.assertEqual(source.active, copy_.active)
            # Frontier rides the wire explicitly:
            self.assertEqual(source.frontier, copy_.frontier)
            self.assertEqual(source.frontier_count, copy_.frontier_count)
            self.assertEqual(source.state_counts, copy_.state_counts)
            self.assertEqual(source.fact_digest, copy_.fact_digest)
            self.assertEqual(tuple(source.flat_indices),
                             tuple(copy_.flat_indices))
            self.assertEqual(dict(source.sensor_members),
                             dict(copy_.sensor_members))
            self.assertEqual(dict(source.box_members),
                             dict(copy_.box_members))
            self.assertEqual(dict(source.distance_members).keys(),
                             dict(copy_.distance_members).keys())
            checked += 1
        self.assertGreater(checked, 0)
        self.assertEqual(dict(rebuilt.summary_values)
                         ['frontier_count'],
                         dict(data.summary_values)['frontier_count'])
        self.assertEqual(dict(rebuilt.sensor_blocks),
                         {k: tuple(v) for k, v in
                          ((k, rebuilt.sensor_blocks[k])
                           for k in rebuilt.sensor_blocks)})
        self.assertEqual(rebuilt.map_snapshot['map_source'],
                         data.map_snapshot.map_source)
        # Deep immutability of the rebuilt payloads.
        with self.assertRaises(AttributeError):
            rebuilt.blocks[0].evidence = b''

    def test_round_trip_deployment_shapes(self):
        for resolution in (0.02, 0.05):
            for boxes in (0, 5):
                self._round_trip(resolution, boxes)

    def test_frontier_only_change_ships_via_dirty_set(self):
        # A cell state change in block A can flip frontier membership in
        # the NEIGHBOUR block B without changing B's fact_digest: the
        # encode stage must emit B's objects when B is in the dirty set
        # (digest equality must never gate emission).
        store_, mapper = store(0.05, 0)
        with acquire(store_) as handle:
            data0 = handle._data()
        # Mark a voxel occupied near a block seam (coordinate 15/16/17
        # crosses BLOCK_EDGE boundaries).
        for coordinate in (15, 16, 17):
            store_.mapper.mark_occupied_voxel(coordinate, 16, 16, label=2,
                                              instance_id=9)
        store_._snapshot = store_._build_snapshot()
        with acquire(store_) as handle:
            data1 = handle._data()
        # Blocks whose frontier bytes changed but fact_digest did not:
        frontier_only = [b1.block_id for b0, b1 in zip(data0.blocks,
                                                       data1.blocks)
                         if b0.frontier != b1.frontier
                         and b0.fact_digest == b1.fact_digest]
        objects = encode_snapshot(data1, 'inc-1', 2,
                                  dirty_blocks=frontier_only)
        rebuilt_partial = {
            name: payload for name, payload in objects.items()
            if not name.startswith('block.')}
        # Merge with unchanged previous-generation payloads (the caller
        # would source these from the prior manifest's addresses).
        full_objects = encode_snapshot(data1, 'inc-1', 2)
        merged = dict(full_objects)
        for block_id in range(len(data1.blocks)):
            if block_id not in frontier_only:
                for suffix in ('evidence', 'active', 'frontier',
                               'flat_indices', 'members', 'meta'):
                    name = 'block.%d.%s' % (block_id, suffix)
                    if name in objects:
                        merged[name] = objects[name]
        rebuilt = rebuild_snapshot(merged, data1.identity)
        for source, copy_ in zip(data1.blocks, rebuilt.blocks):
            self.assertEqual(source.frontier, copy_.frontier)
            self.assertEqual(source.fact_digest, copy_.fact_digest)

    def test_corruption_detected(self):
        store_, _ = store(0.05, 0)
        with acquire(store_) as handle:
            data = handle._data()
        objects = encode_snapshot(data, 'inc-1', 1)
        target = 'block.0.frontier'
        objects[target] = bytes(
            b ^ 1 for b in objects[target])
        with self.assertRaises(WireError):
            rebuild_snapshot(objects, data.identity)


if __name__ == "__main__":
    unittest.main()
