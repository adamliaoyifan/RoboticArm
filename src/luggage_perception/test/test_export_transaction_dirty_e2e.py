#!/usr/bin/env python3
"""End-to-end transaction dirty-set regression (review finding 1,
2026-10-27): a real Store.apply_update -> wired export hook -> DELTA
generation -> QueryReplica must agree on CONTENT, not just identity.

Defect being pinned: one accepted INTEGRATE_VIEW refreshes the query
indexes TWICE (mutate-time refresh with the real delta, then the
snapshot-build refresh with an empty delta). The per-refresh
_last_dirty_blocks only held the LAST (empty) set, so the exported
DELTA dropped the changed blocks and the replica kept stale evidence
under the NEW identity while still reporting READY.

Scenarios (review-required coverage):
- initial changed observation after the first export,
- repeated observation of the same cloud,
- metadata-only update (NOTE_EVENT: no voxel change, identity legs
  change, empty dirty set is CORRECT there),
- cross-block frontier changes (two clusters straddling the x=16
  block edge).

No ordinary-DELTA-full-rebuild workaround: every generation here is
produced by the real wired hook path and asserted by manifest kind.
"""

import os
import sys
import tempfile
import time
import json
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_description"))
sys.path.insert(0, PKG_ROOT)

import numpy as np

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

# Hull wide enough that nx > BLOCK_EDGE(16): two blocks along x at
# 0.05 (nx=29), so clusters at world x=0.0 and x=+0.4 straddle the
# block edge at column 16 (local x ~= +0.055).
HULL = (1.49, 1.97, 1.48)
CENTER = (0, 0, 0.74)
RES = 0.05


def make_wired_store():
    mapper = CargoVolumeMapper(HULL, CENTER, 0, RES,
                               max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='e2e')
    root = tempfile.mkdtemp(prefix='occ-dirty-e2e-')
    hook = wire_store_export(store, root)
    store._snapshot = store._build_snapshot()
    assert hook.wait_pending(10)
    return store, mapper, hook, root


def source_data(store):
    with store.acquire_query_snapshot() as handle:
        return handle._data()


def changed_block_ids(before, after):
    """Blocks whose wire-visible payload differs between generations."""
    changed = []
    for b0, b1 in zip(before.blocks, after.blocks):
        if (b0.evidence != b1.evidence or b0.frontier != b1.frontier
                or b0.fact_digest != b1.fact_digest):
            changed.append(b1.block_id)
    return changed


def public_payload(b):
    return (b.block_id, b.start, b.shape,
            tuple(b.flat_indices.tolist()), b.evidence, b.active,
            b.frontier, b.state_counts, b.frontier_count,
            b.fact_digest, dict(b.sensor_members),
            dict(b.box_members))


class TestExportTransactionDirtyE2E(unittest.TestCase):

    def setUp(self):
        self.store, self.mapper, self.hook, self.root = \
            make_wired_store()
        self.replica = QueryReplica(self.root)
        self.addCleanup(self.hook.close)
        self._clock = [0]
        # Settle the replica at the FIRST generation before the test
        # body: otherwise a DELTA published by the test can arrive
        # while the replica still lacks the base, triggering the
        # (correct, self-healing) FULL-resync path instead of the DELTA
        # path this suite pins.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if (self.replica.poll() == ReplicaState.READY
                    and self.replica.snapshot is not None):
                break
            time.sleep(0.02)
        self.assertEqual(self.replica.state, ReplicaState.READY)
        self.assertEqual(
            tuple(self.replica.snapshot.identity),
            tuple(source_data(self.store).identity))

    def _integrate(self, points, op):
        stamp = self._clock[0] = self._clock[0] + 1
        return self.store.apply_update(MapUpdate(
            operation_id=op, kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=self.store.identity,
            geometry_hash=self.mapper.geometry_hash,
            points=points,
            origin=(0., 0., self.mapper._hull.ceiling_z - 0.1),
            minimum_z=self.mapper._hull.floor_z + .05,
            stamp_key=(stamp, 0),
            acquisition_stamp_ns=1_000_000_000 + stamp,
            settled=True, schema_version=1))

    def _drain(self, expected_identity, timeout_s=10.0,
               min_manifest_seq=None):
        """Wait for the encoder AND poll the replica until it reaches
        the expected committed identity (READY is not enough: it may
        still sit one generation behind). min_manifest_seq forces
        waiting for a specific generation when the identity does not
        move (e.g. empty-DELTA generations)."""
        self.assertTrue(self.hook.wait_pending(timeout_s))
        if min_manifest_seq is None:
            last = self.hook.status_history[-1]
            min_manifest_seq = int(last.get('seq', 0))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            state = self.replica.poll()
            snap = self.replica.snapshot
            if (state == ReplicaState.READY and snap is not None
                    and tuple(snap.identity) == tuple(expected_identity)
                    and self.replica.manifest is not None
                    and int(self.replica.manifest['seq'])
                    >= min_manifest_seq):
                return snap
            time.sleep(0.02)
        self.fail('replica never reached identity %r seq>=%r (state=%s '
                  'err=%s seq=%r)'
                  % (tuple(expected_identity)[:2], min_manifest_seq,
                     self.replica.state, self.replica.last_error,
                     self.replica.manifest
                     and self.replica.manifest['seq']))

    def _assert_content_matches(self, data):
        snap = self.replica.snapshot
        self.assertEqual(tuple(snap.identity), tuple(data.identity))
        self.assertEqual(len(snap.blocks), len(data.blocks))
        for source_block, replica_block in zip(data.blocks,
                                               snap.blocks):
            self.assertEqual(public_payload(source_block),
                             public_payload(replica_block),
                             'block %d payload differs between source '
                             'and replica' % source_block.block_id)
        self.assertEqual(data.summary_values, snap.summary_values)

    def test_changed_observation_reaches_replica_content(self):
        before = source_data(self.store)
        points = np.array([[.0, .0, CENTER[2] - .4],
                           [.02, .02, CENTER[2] - .4]])
        result = self._integrate(points, 'view-1')
        self.assertTrue(result.accepted, result.reason)
        after = source_data(self.store)
        self.assertTrue(changed_block_ids(before, after),
                        'precondition: the observation must change at '
                        'least one block on the source side')
        self._drain(after.identity)
        # THE regression: identical identity is not enough -- the
        # replica's blocks must carry the new content.
        self._assert_content_matches(after)
        self.assertEqual(self.replica.manifest['kind'], 'DELTA')

    def test_repeated_observation_same_cloud(self):
        points = np.array([[.0, .0, CENTER[2] - .4],
                           [.02, .02, CENTER[2] - .4]])
        self.assertTrue(self._integrate(points, 'view-1').accepted)
        first = source_data(self.store)
        self._drain(first.identity)
        self._assert_content_matches(first)
        # Same cloud again: private evidence churns, public payload may
        # or may not change -- either way the replica must converge to
        # the new generation's CONTENT.
        self.assertTrue(self._integrate(points, 'view-2').accepted)
        second = source_data(self.store)
        self._drain(second.identity)
        self._assert_content_matches(second)

    def test_metadata_only_update_keeps_blocks(self):
        """Identity-legs-only boundary. NOTE_EVENT and CONTAINER_POSE
        are accepted but do NOT move the snapshot identity at all
        (verified 2026-10-27); the real no-voxel-change generation in
        the current contract is the idempotent re-add: accepted, same
        identity, EMPTY dirty set. That empty DELTA is legal here and
        the replica must stay content-equal through it (an empty dirty
        set is only a defect when blocks DID change)."""
        box = ((0., 0., self.mapper._hull.floor_z + .11),
               (.35, .53, .22), 0.)
        first_add = self.store.apply_update(MapUpdate(
            operation_id='b1', kind=UpdateKind.ADD_BOX_V1,
            box=box, box_id='e2e-box'))
        self.assertTrue(first_add.accepted, first_add.reason)
        first = source_data(self.store)
        self._drain(first.identity)

        result = self.store.apply_update(MapUpdate(
            operation_id='b1-again', kind=UpdateKind.ADD_BOX_V1,
            box=box, box_id='e2e-box'))
        self.assertTrue(result.accepted, result.reason)
        after = source_data(self.store)
        self.assertEqual(tuple(after.identity), tuple(first.identity),
                         'precondition: idempotent re-add must not '
                         'move the identity')
        self.assertEqual([public_payload(b) for b in first.blocks],
                         [public_payload(b) for b in after.blocks],
                         'precondition: idempotent re-add must not '
                         'touch voxel blocks')
        self._drain(after.identity)
        self._assert_content_matches(after)

    def test_cross_block_frontier_changes(self):
        before = source_data(self.store)
        # Cluster A near column 10 (block 0), cluster B near column
        # 22+ (block 1): the block edge sits at column 16.
        points = np.array([
            [-.25, .0, CENTER[2] - .4], [-.23, .02, CENTER[2] - .4],
            [.40, .0, CENTER[2] - .4], [.42, .02, CENTER[2] - .4]])
        self.assertTrue(self._integrate(points, 'view-x').accepted)
        after = source_data(self.store)
        changed = changed_block_ids(before, after)
        self.assertGreaterEqual(len(changed), 2,
                                'precondition: clusters must straddle '
                                'the block edge (changed=%r)' % changed)
        self._drain(after.identity)
        self._assert_content_matches(after)

    def test_coalesced_back_to_back_commits_content_matches(self):
        """Commits arriving faster than the encoder: intermediate
        generations ride along (approved coalescing). The ledger must
        classify them coalesced_into / exact_visible, and the final
        replica content must equal the source snapshot block-by-block."""
        base = source_data(self.store)
        ledger_before = len(
            self.hook._publisher.ledger_snapshot())
        for index in range(4):
            points = np.array([[.0 + .01 * index, .0, CENTER[2] - .4]])
            self.assertTrue(
                self._integrate(points, 'burst-%d' % index).accepted)
        final = source_data(self.store)
        self._drain(final.identity)
        self._assert_content_matches(final)
        ledger = self.hook._publisher.ledger_snapshot()
        self.assertGreater(len(ledger), ledger_before)
        terminal = {entry['status'] for entry in ledger}
        self.assertIn('exact_visible', terminal)
        self.assertTrue(
            terminal <= {'exact_visible', 'coalesced_into'},
            'unexpected terminal statuses: %r' % terminal)
        # The final fact is exactly visible; at least one intermediate
        # fact rode along (or all shipped exactly -- both legal, but a
        # coalesced fact must NEVER claim exact_visible).
        for entry in ledger:
            if entry['status'] == 'exact_visible':
                self.assertEqual(
                    list(entry['identity'] or [])[:1],
                    [final.identity[0]])

    def test_delta_apply_updates_derived_surface_and_distance(self):
        """Gen3 defect pin (found by the fault gate's full-snapshot wire
        fingerprint): _apply_delta used to carry the PREVIOUS
        generation's surface_projection/distance_index forward, serving
        stale derived data under the new identity. After any DELTA
        apply, the replica's COMPLETE wire encoding must equal the
        source's -- every object, including surface/distance."""
        from luggage_perception.cargo_query_snapshot_wire import (
            encode_snapshot)
        from luggage_perception.cargo_query_wire import sha256_hex

        def fingerprint(snapshot):
            payloads = encode_snapshot(
                snapshot, 'pin', 0, dirty_blocks=None,
                provenance_changed=True)
            table = {name: sha256_hex(payload)
                     for name, payload in sorted(payloads.items())}
            import hashlib
            return hashlib.sha256(json.dumps(
                table, sort_keys=True).encode()).hexdigest()

        points = np.array([[.0, .0, CENTER[2] - .4],
                           [.02, .02, CENTER[2] - .4]])
        for index in range(3):
            self.assertTrue(
                self._integrate(points, 'derived-%d' % index).accepted)
            data = source_data(self.store)
            self._drain(data.identity)
            self._assert_content_matches(data)
            assert self.replica.snapshot is not None
            self.assertEqual(fingerprint(self.replica.snapshot),
                             fingerprint(data),
                             'full wire fingerprint diverged after '
                             'DELTA %d (stale derived object?)' % index)

    def test_replica_distance_queries_match_source(self):
        """Gen3 A9-pairing find: wire-rebuilt snapshots used to carry
        a hash-only hull placeholder, so nearest_obstacles CRASHED on
        the replica path (AttributeError half_x). The grid object now
        transports the hull descriptor; replica distance results must
        equal the source's."""
        points = [(.1, .1, .8), (-.3, .2, .9), (.5, -.5, 1.2),
                  (0., 0., .6)]
        before = source_data(self.store)
        result = self._integrate(
            np.array([[.0, .0, CENTER[2] - .4]]), 'view-dist')
        self.assertTrue(result.accepted)
        after = source_data(self.store)
        self._drain(after.identity)
        with self.store.acquire_query_snapshot() as handle:
            source = handle.nearest_obstacles(points, 0.5, 'separate')
        from luggage_perception.cargo_map_read_facade import (
            QueryMapReadFacade)
        facade = QueryMapReadFacade(self.replica)
        replica_handle = facade.acquire_query_snapshot('CURRENT')
        replica = replica_handle.nearest_obstacles(points, 0.5,
                                                   'separate')
        self.assertEqual(replica.status, source.status)
        self.assertEqual(len(replica.results), len(source.results))
        for a, b in zip(source.results, replica.results):
            self.assertEqual(a.point, b.point)
            self.assertEqual(a.status, b.status)
            self.assertEqual(
                (a.occupied.exact, a.occupied.distance_m
                 if a.occupied else None),
                (b.occupied.exact, b.occupied.distance_m
                 if b.occupied else None))
        self.assertNotEqual(before.identity, after.identity)

    def test_replica_surface_queries_match_source(self):
        """Gen3 A9-pairing find #2: read_surface reads the surface
        CONTRACT columns from map_snapshot; the wire never transported
        them, so surface queries crashed on the replica path. The
        'surface.contract' object now ships with every generation;
        replica surface results must equal the source's."""
        result = self._integrate(
            np.array([[.05, .05, CENTER[2] - .4]]), 'view-surf')
        self.assertTrue(result.accepted)
        data = source_data(self.store)
        self._drain(data.identity)
        with self.store.acquire_query_snapshot() as handle:
            source = handle.surface()
        from luggage_perception.cargo_map_read_facade import (
            QueryMapReadFacade)
        facade = QueryMapReadFacade(self.replica)
        replica_handle = facade.acquire_query_snapshot('CURRENT')
        replica = replica_handle.surface()
        self.assertEqual(replica.status, source.status)
        self.assertEqual(replica.shape_xy, source.shape_xy)
        self.assertEqual(replica.origin_index_xy, source.origin_index_xy)
        for name in ('surface_z_container', 'height_above_floor',
                     'contract_confidence'):
            self.assertEqual(source.columns[name],
                             replica.columns[name], name)
        self.assertEqual(replica.floor_z_container,
                         source.floor_z_container)

    def test_accumulator_consumed_only_on_accepted_offer(self):
        """A failed offer must keep the accumulated dirt so it ships
        with the NEXT accepted generation. (gen3: the async path can no
        longer hit EXPORT_BACKLOG_FULL -- coalescing precedes capacity
        -- so the named failure injected here is EXPORT_CLOSING.)"""
        owner = self.store._query_indexes
        publisher = self.hook._publisher
        with publisher._state_lock:
            publisher._closing = True      # force a named offer failure
        try:
            points = np.array([[.0, .0, CENTER[2] - .4]])
            self.assertTrue(self._integrate(points, 'view-b1').accepted)
            # The offer failed: the dirt must still be pending.
            self.assertEqual(self.hook.last_status['status'],
                             'EXPORT_CLOSING')
            self.assertNotEqual(owner._dirty_since_export, set(),
                                'a failed offer must retain the '
                                'accumulated dirty set')
        finally:
            with publisher._state_lock:
                publisher._closing = False
        # Next accepted generation ships the retained dirt.
        self.assertTrue(self._integrate(points, 'view-b2').accepted)
        after = source_data(self.store)
        snap = self._drain(after.identity)
        self._assert_content_matches(after)
        self.assertIsNotNone(snap)


if __name__ == "__main__":
    unittest.main()
