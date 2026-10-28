"""G4-A gen3 bounded catch-up matrix (spec
OCC_SEQ_GAP_CATCHUP_SPEC_20261028 section 5; review_5c5c1842 Finding 2).

T1 chain-complete / T2 chain-missing / T3 chain-corrupted /
T4 same-seq conflict / T5 world change / T6 cleanup race /
T7 continuous publishing, plus the deterministic barrier micro-test
(hold the receiver, publish known generations, release: catch-up --
NOT a full rebuild -- must close the gap).
"""

import json
import os
import shutil
import tempfile
import threading
import time
import unittest

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

import numpy as np


def fingerprint(snapshot):
    """Complete wire-encoding digest (same as the fault-gate finger)."""
    import hashlib
    from luggage_perception.cargo_query_snapshot_wire import (
        encode_snapshot)
    from luggage_perception.cargo_query_wire import sha256_hex
    payloads = encode_snapshot(snapshot, 'fp', 0, dirty_blocks=None,
                               provenance_changed=True)
    table = {name: sha256_hex(payload)
             for name, payload in sorted(payloads.items())}
    return hashlib.sha256(json.dumps(table, sort_keys=True)
                          .encode()).hexdigest()


def make_wired(resolution=0.05, epoch='catchup'):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=500)
    store = OccupancyMapStore(mapper, epoch=epoch)
    root = tempfile.mkdtemp(prefix='catchup-')
    hook = wire_store_export(store, root)
    store._snapshot = store._build_snapshot()
    hook.wait_pending(60.0)
    return store, mapper, hook, root


def cloud(mapper, seed):
    return np.column_stack([
        np.random.default_rng(seed).uniform(-0.5, 0.5, 1500),
        np.random.default_rng(seed + 1).uniform(-0.6, 0.6, 1500),
        np.random.default_rng(seed + 2).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.0, 1500)])


def commit(store, mapper, seed):
    result = store.apply_update(MapUpdate(
        operation_id='c%d' % seed, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud(mapper, seed),
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(seed, 0),
        acquisition_stamp_ns=1_000_000_000 + seed,
        settled=True, schema_version=1))
    assert result.accepted
    return result


def settle(replica, identity2, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        replica.poll()
        snap = replica.snapshot
        if (replica.state == ReplicaState.READY and snap is not None
                and tuple(snap.identity)[:2] == tuple(identity2)[:2]):
            return True
        time.sleep(0.02)
    return False


class CatchupMatrix(unittest.TestCase):

    def setUp(self):
        self.store, self.mapper, self.hook, self.root = make_wired()
        self.replica = QueryReplica(self.root)
        self.assertTrue(settle(self.replica, self.store.identity))
        self.base_seq = int(self.replica.manifest['seq'])

    def tearDown(self):
        self.hook.close()
        shutil.rmtree(self.root, ignore_errors=True)

    def _gap(self, count):
        """Commit `count` generations while the receiver is parked;
        returns the parked replica's expected catch-up window."""
        parked_seq = int(self.replica.manifest['seq'])
        for seed in range(100, 100 + count):
            commit(self.store, self.mapper, seed)
        self.hook.wait_pending(60.0)
        return parked_seq

    def _source_fp(self):
        with self.store.acquire_query_snapshot() as handle:
            return fingerprint(handle._data()), list(handle.identity)

    def _events(self, name):
        return [e for e in self.replica.events()
                if e.get('event') == name]

    def test_t1_chain_complete_catches_up_without_full(self):
        self._gap(3)
        state = self.replica.poll()
        self.assertEqual(state, ReplicaState.READY)
        catchups = self._events('catchup')
        self.assertTrue(catchups
                        and catchups[-1]['status'] == 'applied')
        self.assertEqual(catchups[-1]['links'], 3)
        # no FULL resync was requested
        self.assertFalse(os.path.exists(
            os.path.join(self.root, 'control', 'resync')))
        src_fp, ident = self._source_fp()
        self.assertEqual(fingerprint(self.replica.snapshot), src_fp)
        self.assertEqual(tuple(self.replica.snapshot.identity)[:2],
                         tuple(ident)[:2])

    def test_t2_chain_missing_falls_back_named(self):
        self._gap(3)
        producer = self.replica.manifest['producer_incarnation']
        missing = os.path.join(self.root, 'generations', producer,
                               str(self.base_seq + 2), 'manifest.json')
        os.remove(missing)
        state = self.replica.poll()
        self.assertEqual(state, ReplicaState.RESYNC_REQUIRED)
        self.assertIn('SEQ_GAP', self.replica.last_error)
        catchups = self._events('catchup')
        self.assertEqual(catchups[-1]['status'], 'missing')
        # recovery via a further commit still converges
        commit(self.store, self.mapper, 200)
        self.hook.wait_pending(60.0)
        self.assertTrue(settle(self.replica, self.store.identity, 30))
        src_fp, _ = self._source_fp()
        self.assertEqual(fingerprint(self.replica.snapshot), src_fp)

    def test_t3_chain_corrupted_falls_back_named(self):
        self._gap(3)
        producer = self.replica.manifest['producer_incarnation']
        target = os.path.join(self.root, 'generations', producer,
                              str(self.base_seq + 2), 'manifest.json')
        body = json.load(open(target))
        body['target_identity'][1] += 1        # tamper, keep checksum
        # (checksum recompute would still fail base chain at the next
        # link; without recompute validate_manifest fails first)
        with open(target, 'w') as handle:
            json.dump(body, handle)
        state = self.replica.poll()
        self.assertEqual(state, ReplicaState.RESYNC_REQUIRED)
        commit(self.store, self.mapper, 201)
        self.hook.wait_pending(60.0)
        self.assertTrue(settle(self.replica, self.store.identity, 30))
        src_fp, _ = self._source_fp()
        self.assertEqual(fingerprint(self.replica.snapshot), src_fp)

    def test_t4_same_seq_conflict_still_rejected(self):
        self._gap(2)
        producer = self.replica.manifest['producer_incarnation']
        target = os.path.join(self.root, 'generations', producer,
                              str(self.base_seq + 1), 'manifest.json')
        body = json.load(open(target))
        body['target_identity'][1] += 3
        from luggage_perception.cargo_query_wire import (
            canonical_json_bytes, manifest_checksum, sha256_hex)
        body['checksum'] = manifest_checksum(body)
        with open(target, 'wb') as handle:
            handle.write(canonical_json_bytes(body))
        # poll sees latest seq (gap) -> catch-up reads the forged link;
        # its base no longer matches the anchor -> named fallback
        state = self.replica.poll()
        self.assertEqual(state, ReplicaState.RESYNC_REQUIRED)
        catchups = self._events('catchup')
        self.assertEqual(catchups[-1]['status'], 'base_mismatch')

    def test_t5_world_change_never_catches_up(self):
        self._gap(2)
        producer = self.replica.manifest['producer_incarnation']
        # forge an intermediate link from ANOTHER epoch
        src = os.path.join(self.root, 'generations', producer,
                           str(self.base_seq + 1), 'manifest.json')
        body = json.load(open(src))
        body['map_epoch'] = 'other-world'
        from luggage_perception.cargo_query_wire import (
            canonical_json_bytes, manifest_checksum)
        body['checksum'] = manifest_checksum(body)
        with open(src, 'wb') as handle:
            handle.write(canonical_json_bytes(body))
        state = self.replica.poll()
        self.assertEqual(state, ReplicaState.RESYNC_REQUIRED)
        catchups = self._events('catchup')
        self.assertEqual(catchups[-1]['status'], 'world_change')

    def test_t6_cleanup_race_falls_back(self):
        self._gap(3)
        original = self.replica._load_payload
        counter = [0]

        def racing(digest, name):
            counter[0] += 1
            if counter[0] == 2:
                # A file vanished mid-apply: the loader converts this
                # to a named WireError in production; inject at the
                # same boundary.
                from luggage_perception.cargo_query_wire import WireError
                raise WireError('OBJECT_MISSING', name)
            return original(digest, name)
        self.replica._load_payload = racing
        state = self.replica.poll()
        self.assertEqual(state, ReplicaState.RESYNC_REQUIRED)
        self.replica._load_payload = original
        commit(self.store, self.mapper, 202)
        self.hook.wait_pending(60.0)
        self.assertTrue(settle(self.replica, self.store.identity, 30))
        src_fp, _ = self._source_fp()
        self.assertEqual(fingerprint(self.replica.snapshot), src_fp)

    def test_t7_continuous_publishing_during_catchup(self):
        parked = self._gap(2)
        # a writer thread keeps publishing while the receiver catches up
        stop = threading.Event()
        errors = []

        def writer():
            seed = 300
            while not stop.is_set():
                try:
                    commit(self.store, self.mapper, seed)
                    seed += 1
                except Exception as error:  # noqa: BLE001
                    errors.append(error)
                    return
                time.sleep(0.05)
        thread = threading.Thread(target=writer, daemon=True)
        thread.start()
        try:
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                self.replica.poll()
                if (self.replica.state == ReplicaState.READY
                        and self.replica.snapshot is not None
                        and int(self.replica.manifest['seq'])
                        > parked + 1):
                    break
                time.sleep(0.02)
            self.assertFalse(errors)
            self.assertEqual(self.replica.state, ReplicaState.READY)
            self.assertGreater(int(self.replica.manifest['seq']),
                               parked + 1)
        finally:
            stop.set()
            thread.join(timeout=10)
            self.hook.wait_pending(60.0)
        self.assertTrue(settle(self.replica, self.store.identity, 30))
        src_fp, _ = self._source_fp()
        self.assertEqual(fingerprint(self.replica.snapshot), src_fp)

    def test_barrier_microtest_amplification_removed(self):
        """The review's deterministic micro-test: hold the receiver,
        publish three known generations, release. Catch-up closes the
        gap; the recovery path must NOT be a FULL rebuild (no resync
        request file) and content must equal the source."""
        parked = self._gap(3)
        t0 = time.monotonic()
        state = self.replica.poll()
        elapsed = time.monotonic() - t0
        self.assertEqual(state, ReplicaState.READY)
        self.assertLess(elapsed, 30.0)
        self.assertFalse(os.path.exists(
            os.path.join(self.root, 'control', 'resync')),
            'catch-up must not request a FULL resync for a retained '
            'complete chain')
        self.assertGreater(int(self.replica.manifest['seq']), parked)
        src_fp, _ = self._source_fp()
        self.assertEqual(fingerprint(self.replica.snapshot), src_fp)


if __name__ == '__main__':
    unittest.main()
