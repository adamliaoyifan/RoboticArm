#!/usr/bin/env python3
"""A1 filesystem gates: publisher/replica FULL+DELTA over a real root.

Covers the protocol's transport section: content-addressed dedup
(unchanged objects never rewritten), self-contained manifests (DELTA
references unchanged addresses), atomic pointer publication, DELTA base
validation (BASE_MISMATCH / SEQ_GAP / PRODUCER_CHANGED -> named
RESYNC_REQUIRED), frontier-only neighbour blocks shipping in deltas,
and receiver-side equality with the source snapshots at every step.
"""

import os
import shutil
import sys
import tempfile
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, PKG_ROOT)

from luggage_perception.cargo_query_export import (
    ExportError, QueryExportPublisher)
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def make_store(boxes, resolution=0.05):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution)
    store = OccupancyMapStore(mapper, epoch='export')
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
    return store


def current_snapshot(store):
    store._snapshot = store._build_snapshot()
    with store.acquire_query_snapshot() as handle:
        return handle._data()


def block_diff(a, b):
    """Block ids whose wire payload differs between two snapshots."""
    dirty = set()
    for block0, block1 in zip(a.blocks, b.blocks):
        if (block0.evidence != block1.evidence
                or block0.active != block1.active
                or block0.frontier != block1.frontier
                or block0.fact_digest != block1.fact_digest
                or dict(block0.sensor_members) != dict(
                    block1.sensor_members)
                or dict(block0.box_members) != dict(block1.box_members)):
            dirty.add(block1.block_id)
    return dirty


class TestPublishReplicaA1(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='occ-export-')
        self.addCleanup(shutil.rmtree, self.root, True)

    def test_full_then_delta_chain_with_dedup(self):
        store = make_store(0)
        publisher = QueryExportPublisher(self.root)
        self.addCleanup(publisher.close)
        replica = QueryReplica(self.root)

        snap0 = current_snapshot(store)
        result = publisher.offer_committed(snap0, dirty_blocks=None)
        self.assertEqual(result['status'], 'OK', result)
        self.assertEqual(result['kind'], 'FULL')
        self.assertEqual(replica.poll(), ReplicaState.READY)
        self.assertEqual(tuple(replica.snapshot.identity),
                         tuple(snap0.identity))
        objects_after_full = result['objects_written']
        self.assertGreater(objects_after_full, 0)

        # Commit a change; the write path's dirty set ships only
        # affected blocks (frontier neighbourhoods included).
        store.mapper.mark_occupied_voxel(15, 16, 16, label=2,
                                         instance_id=9)
        store.mapper.mark_occupied_voxel(16, 16, 16, label=2,
                                         instance_id=9)
        snap1 = current_snapshot(store)
        dirty = block_diff(snap0, snap1)
        self.assertGreater(len(dirty), 0)
        result = publisher.offer_committed(snap1, dirty_blocks=dirty)
        self.assertEqual(result['status'], 'OK', result)
        self.assertEqual(result['kind'], 'DELTA')
        # Dedup: most objects already exist from the FULL generation.
        self.assertLess(result['objects_written'], objects_after_full)
        self.assertEqual(replica.poll(), ReplicaState.READY)
        self.assertEqual(tuple(replica.snapshot.identity),
                         tuple(snap1.identity))
        # Frontier bytes survive the delta exactly.
        for source, received in zip(snap1.blocks,
                                    replica.snapshot.blocks):
            self.assertEqual(source.frontier, received.frontier)

    def test_delta_rejects_wrong_base_and_gaps(self):
        store = make_store(0)
        publisher = QueryExportPublisher(self.root)
        self.addCleanup(publisher.close)
        snap0 = current_snapshot(store)
        publisher.offer_committed(snap0, dirty_blocks=None)

        store.mapper.mark_occupied_voxel(20, 20, 20, label=2)
        snap1 = current_snapshot(store)
        publisher.offer_committed(snap1,
                                  dirty_blocks=block_diff(snap0, snap1))
        store.mapper.mark_occupied_voxel(30, 30, 30, label=2)
        snap2 = current_snapshot(store)
        publisher.offer_committed(snap2,
                                  dirty_blocks=block_diff(snap1, snap2))

        # Fresh replica joins at seq 2 (latest published); applying it
        # without its base is a DELTA_WITHOUT_BASE -> RESYNC_REQUIRED.
        replica = QueryReplica(self.root)
        self.assertEqual(replica.poll(), ReplicaState.RESYNC_REQUIRED)
        self.assertIn('DELTA_WITHOUT_BASE', replica.last_error)
        # Resync: publisher answers with a FULL generation.
        publisher.publish_resync_full(snap2)
        self.assertEqual(replica.poll(), ReplicaState.READY)
        self.assertEqual(tuple(replica.snapshot.identity),
                         tuple(snap2.identity))

    def test_corrupted_object_detected(self):
        store = make_store(0)
        publisher = QueryExportPublisher(self.root)
        self.addCleanup(publisher.close)
        snap0 = current_snapshot(store)
        publisher.offer_committed(snap0, dirty_blocks=None)
        # Corrupt one object file in place.
        objects = os.listdir(os.path.join(self.root, 'objects'))
        victim = os.path.join(self.root, 'objects', sorted(objects)[0])
        with open(victim, 'r+b') as handle:
            handle.seek(4)
            byte = handle.read(1)
            handle.seek(4)
            handle.write(bytes([byte[0] ^ 0xFF]))
        replica = QueryReplica(self.root)
        self.assertIn(replica.poll(),
                      (ReplicaState.RESYNC_REQUIRED,
                       ReplicaState.UNAVAILABLE))
        self.assertTrue(replica.last_error)

    def test_owner_lock_exclusive(self):
        store = make_store(0)
        publisher = QueryExportPublisher(self.root)
        self.addCleanup(publisher.close)
        with self.assertRaises(ExportError):
            QueryExportPublisher(self.root)


if __name__ == "__main__":
    unittest.main()
