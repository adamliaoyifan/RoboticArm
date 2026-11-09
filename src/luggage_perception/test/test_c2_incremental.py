#!/usr/bin/env python3
"""C2 gates: dirty-first encoding + incremental receive assembly.

- Encode-side: a DELTA with an empty/small dirty set constructs only
  the dirty blocks' payloads (C0 baseline: 2609 JSON constructions per
  metadata-only generation at 1cm; the corrected target is ~11).
- Receive-side: DELTA assembly reuses unchanged immutable block
  objects (identity equality) and only decodes changed objects
  (cache_misses grows by the changed set, not the whole table); the
  assembled snapshot equals a full rebuild of the same generation
  field-by-field (identity, blocks incl. frontier bytes, summary).
"""

import os
import sys
import tempfile
import shutil
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, PKG_ROOT)

from luggage_perception.cargo_query_export import QueryExportPublisher
from luggage_perception.cargo_query_replica import QueryReplica, ReplicaState
from luggage_perception.cargo_query_snapshot_wire import (
    encode_snapshot, rebuild_snapshot)
from luggage_perception import cargo_query_snapshot_wire as snapshot_wire
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def make_store(boxes, resolution=0.05, confirmation=False):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='c2',
                              require_confirmation=confirmation)
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


def snapshot_of(store):
    store._snapshot = store._build_snapshot()
    with store.acquire_query_snapshot() as handle:
        return handle._data()


class JsonCounter:
    def __init__(self):
        self.count = 0

    def __enter__(self):
        self._original = snapshot_wire.encode_json_object
        outer = self
        original = self._original

        def counting(value):
            outer.count += 1
            return original(value)
        snapshot_wire.encode_json_object = counting
        return self

    def __exit__(self, *_):
        snapshot_wire.encode_json_object = self._original
        return False


class TestC2DirtyFirstEncode(unittest.TestCase):

    def test_empty_dirty_constructs_only_toplevel(self):
        store, _ = make_store(5)
        data = snapshot_of(store)
        with JsonCounter() as counter:
            objects = encode_snapshot(data, 'inc', 1, dirty_blocks=set())
        # 12 blocks skipped entirely: only the ~11 top-level JSON
        # payloads are constructed.
        self.assertLessEqual(counter.count, 12)
        self.assertNotIn('block.0.meta', objects)


class TestC2IncrementalReceive(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='c2-')
        self.addCleanup(shutil.rmtree, self.root, True)

    def test_delta_assembly_reuses_and_equals_full(self):
        store, mapper = make_store(5)
        publisher = QueryExportPublisher(self.root)
        self.addCleanup(publisher.close)
        replica = QueryReplica(self.root)

        data0 = snapshot_of(store)
        publisher.offer_committed(data0, dirty_blocks=None)
        self.assertEqual(replica.poll(), ReplicaState.READY)
        misses_after_full = replica.cache_misses
        replica_blocks_before = replica.snapshot.blocks

        # Metadata-only delta: identity advances, no block changes.
        store.apply_update(MapUpdate(
            operation_id='pose-1', kind=UpdateKind.CONTAINER_POSE,
            event_stamp_ns=1_234_567))
        data1 = snapshot_of(store)
        result = publisher.offer_committed(
            data1, dirty_blocks=set(),
            provenance_changed=tuple(data1.identity)
            != tuple(data0.identity))
        self.assertEqual(result['status'], 'OK')
        self.assertEqual(result['kind'], 'DELTA')
        self.assertEqual(replica.poll(), ReplicaState.READY)

        # Unchanged blocks reused WITHIN the replica: object identity is
        # preserved across generations for blocks outside the dirty set
        # (cross-process identity is meaningless by construction).
        reused = sum(1 for a, b in zip(replica_blocks_before,
                                       replica.snapshot.blocks)
                     if a is b)
        self.assertEqual(reused, len(data0.blocks))
        # Only the changed top-level objects were decoded.
        self.assertLess(replica.cache_misses - misses_after_full, 12)

        # Field-by-field equality with a FULL rebuild of data1.
        full_objects = encode_snapshot(data1, 'inc', 99)
        rebuilt = rebuild_snapshot(full_objects, data1.identity)
        self.assertEqual(tuple(rebuilt.identity),
                         tuple(replica.snapshot.identity))
        for source, received in zip(rebuilt.blocks,
                                    replica.snapshot.blocks):
            self.assertEqual(source.evidence, received.evidence)
            self.assertEqual(source.frontier, received.frontier)
            self.assertEqual(source.fact_digest, received.fact_digest)
            self.assertEqual(source.state_counts, received.state_counts)

    def test_delta_with_block_changes_rebuilds_only_changed(self):
        store, mapper = make_store(5)
        publisher = QueryExportPublisher(self.root)
        self.addCleanup(publisher.close)
        replica = QueryReplica(self.root)
        data0 = snapshot_of(store)
        publisher.offer_committed(data0, dirty_blocks=None)
        replica.poll()
        replica_prev_blocks = list(replica.snapshot.blocks)

        store.apply_update(MapUpdate(
            operation_id='box-9', kind=UpdateKind.ADD_BOX_V1,
            box=((0.45, 0.45, mapper._hull.floor_z + .11),
                 (.35, .53, .22), 0.), box_id='box-9'))
        data1 = snapshot_of(store)
        dirty = {b1.block_id for b0, b1 in zip(data0.blocks, data1.blocks)
                 if b0.evidence != b1.evidence
                 or b0.frontier != b1.frontier
                 or b0.fact_digest != b1.fact_digest}
        self.assertGreater(len(dirty), 0)
        publisher.offer_committed(data1, dirty_blocks=dirty)
        self.assertEqual(replica.poll(), ReplicaState.READY)
        reused = sum(1 for b0, b1 in zip(replica_prev_blocks,
                                         replica.snapshot.blocks)
                     if b0 is b1)
        self.assertEqual(reused, len(data0.blocks) - len(dirty))
        # All changed blocks present with fresh content.
        for block_id in dirty:
            self.assertEqual(
                replica.snapshot.blocks[block_id].fact_digest,
                data1.blocks[block_id].fact_digest)


if __name__ == "__main__":
    unittest.main()
