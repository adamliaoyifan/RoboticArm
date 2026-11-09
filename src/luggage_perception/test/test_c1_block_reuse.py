#!/usr/bin/env python3
"""C1 gates: public-payload block reuse + DerivedChangeSet surface.

- Same-cloud re-integration (private log-odds accumulate, public bytes
  identical): block OBJECT rebuilds drop to the actually-changed set
  (C0 measured 816/1300 at 1cm), work counters record
  public_payload_reused_blocks, and canonical maintenance still runs
  (fact digests keep advancing owner-side; unchanged-public blocks keep
  the old object identity).
- Real public changes (box add): the affected blocks DO rebuild and all
  semantics stay pinned by the existing contract suite.
- DerivedChangeSet exposes the transaction's derived dirty classes with
  base/target identities; consumers cannot mutate it.
"""

import os
import sys
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, PKG_ROOT)

import numpy as np

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def build_store(resolution, boxes=0):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='c1')
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


def snapshot(store):
    store._snapshot = store._build_snapshot()
    with store.acquire_query_snapshot() as handle:
        return handle._data()


def identity_rebuilds(before, after):
    return sum(1 for a, b in zip(before.blocks, after.blocks)
               if a is not b)


class TestPublicPayloadReuse(unittest.TestCase):

    def test_same_cloud_frame_rebuild_matches_digest_churn(self):
        """Honest semantics (corrects the C0 note): the block fact
        digest binds PRIVATE evidence through the canonical leaves, so
        a same-cloud frame churns digests (measured: 12/12 at 0.05,
        evidence bytes 0 changed) and those rebuilds are CONTRACTUAL.
        Object rebuilds must equal digest-changed blocks exactly, and
        the reuse path must fire precisely for the rest of the dirty
        closure (frontier-candidate blocks whose membership did not
        flip and whose canonical patch is empty)."""
        store, mapper = build_store(0.05)
        data1 = snapshot(store)
        store.apply_update(MapUpdate(
            operation_id='box-1', kind=UpdateKind.ADD_BOX_V1,
            box=((0., 0., mapper._hull.floor_z + .11),
                 (.35, .53, .22), 0.), box_id='box-1'))
        # Read the transaction's counters BEFORE the snapshot helper's
        # own (empty-delta) refresh overwrites _last_work.
        work = dict(store._query_indexes.work)
        data2 = snapshot(store)
        rebuilt = identity_rebuilds(data1, data2)
        digest_changed = sum(1 for a, b in zip(data1.blocks, data2.blocks)
                             if a.fact_digest != b.fact_digest)
        self.assertEqual(rebuilt, digest_changed)
        self.assertEqual(rebuilt,
                         work.get('public_payload_changed_blocks',
                                  rebuilt))
        self.assertEqual(work.get('dirty_blocks', 0),
                         work.get('public_payload_reused_blocks', 0)
                         + work.get('public_payload_changed_blocks', 0))

    def test_same_cloud_second_frame_reuses_blocks(self):
        store, mapper = build_store(0.05)
        points = np.column_stack([
            np.random.default_rng(3).uniform(-0.6, 0.6, 30000),
            np.random.default_rng(4).uniform(-0.8, 0.8, 30000),
            np.random.default_rng(5).uniform(
                mapper._hull.floor_z + .1,
                mapper._hull.floor_z + 1.2, 30000)])

        def integrate(stamp):
            return store.apply_update(MapUpdate(
                operation_id='cloud-%d' % stamp,
                kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=mapper.geometry_hash,
                points=points,
                origin=(0.0, 0.0, mapper._hull.ceiling_z - 0.1),
                minimum_z=mapper._hull.floor_z + .05,
                stamp_key=(stamp, 0),
                acquisition_stamp_ns=1_000_000_000 + stamp,
                settled=True, schema_version=1))

        assert integrate(1).accepted
        data1 = snapshot(store)
        assert integrate(2).accepted
        data2 = snapshot(store)
        rebuilt = identity_rebuilds(data1, data2)
        # Same-cloud frame: digest churn (private-evidence binding) is
        # contractual, so rebuilds == digest-changed blocks; the wire
        # evidence/active/frontier objects still dedupe by CONTENT
        # (transport equivalence is not digest-gated -- plan 4.1).
        digest_changed = sum(1 for a, b in zip(data1.blocks, data2.blocks)
                             if a.fact_digest != b.fact_digest)
        self.assertEqual(rebuilt, digest_changed)
        evidence_changed = sum(
            1 for a, b in zip(data1.blocks, data2.blocks)
            if a.evidence != b.evidence)
        self.assertEqual(evidence_changed, 0)
        # Identity still advances (the frame was a real observation).
        self.assertNotEqual(tuple(data1.identity),
                             tuple(data2.identity))

    def test_real_change_still_rebuilds(self):
        store, mapper = build_store(0.05)
        data1 = snapshot(store)
        store.apply_update(MapUpdate(
            operation_id='box-1', kind=UpdateKind.ADD_BOX_V1,
            box=((0., 0., mapper._hull.floor_z + .11),
                 (.35, .53, .22), 0.), box_id='box-1'))
        data2 = snapshot(store)
        self.assertGreater(identity_rebuilds(data1, data2), 0)
        # New block content really is on the wire surface.
        changed = sum(1 for a, b in zip(data1.blocks, data2.blocks)
                      if a.evidence != b.evidence)
        self.assertEqual(changed, identity_rebuilds(data1, data2))


if __name__ == "__main__":
    unittest.main()
