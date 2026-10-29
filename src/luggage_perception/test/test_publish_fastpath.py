"""Unchanged-generation reuse, without weakening the check we still run.

Pointer reuse is keyed by producer + relative path + manifest digest,
never by seq. A new receiver and a resync hash the body again.
Payload digests are reused only when the encoded bytes compare equal.
"""

import json
import os
import shutil
import sys
import tempfile
import time
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, 'src', 'luggage_perception'))

from luggage_perception.cargo_query_export import (  # noqa: E402
    QueryExportPublisher)
from luggage_perception.cargo_query_replica import (  # noqa: E402
    QueryReplica, ReplicaState)
from luggage_perception.cargo_query_snapshot_wire import (  # noqa: E402
    encode_snapshot)
from luggage_perception.cargo_query_wire import sha256_hex  # noqa: E402
from luggage_perception.cargo_volume_mapper import (  # noqa: E402
    CargoVolumeMapper)
from luggage_perception.occupancy_map_store import (  # noqa: E402
    MapUpdate, OccupancyMapStore, UpdateKind)


def make_store():
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.05)
    store = OccupancyMapStore(mapper, epoch='fastpath')
    result = store.apply_update(MapUpdate(
        operation_id='b0', kind=UpdateKind.ADD_BOX_V1,
        box=((0.0, 0.18, mapper._hull.floor_z + 0.2),
             (0.35, 0.53, 0.22), 0.0), box_id='box-0'))
    assert result.accepted
    return store


def snapshot(store):
    store._snapshot = store._build_snapshot()
    with store.acquire_query_snapshot() as handle:
        return handle._data()


class FastPathTests(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='occ-fastpath-')
        self.addCleanup(shutil.rmtree, self.root, True)
        self.store = make_store()
        self.publisher = QueryExportPublisher(self.root)
        self.addCleanup(self.publisher.close)
        self.snap = snapshot(self.store)
        self.publisher.offer_committed(self.snap, dirty_blocks=None)
        self.replica = QueryReplica(self.root)
        self.assertEqual(self.replica.poll(), ReplicaState.READY)
        self.assertEqual(self.replica.manifest_body_reads, 1)

    def test_unchanged_pointer_does_not_reread_manifest(self):
        self.replica.poll()
        self.replica.poll()
        self.assertEqual(self.replica.manifest_body_reads, 1)
        self.assertEqual(self.replica.state, ReplicaState.READY)

    def test_same_seq_with_a_new_digest_is_reverified(self):
        pointer_path = os.path.join(self.root, 'published.json')
        with open(pointer_path, 'rb') as handle:
            pointer = json.loads(handle.read().decode('utf-8'))
        pointer['manifest_sha256'] = '0' * 64
        with open(pointer_path, 'w') as handle:
            json.dump(pointer, handle)
        self.assertEqual(self.replica.poll(), ReplicaState.UNAVAILABLE)
        self.assertIn('POINTER_MISMATCH', self.replica.last_error)
        self.assertEqual(self.replica.manifest_body_reads, 2)

    def test_restart_rehashes_and_catches_a_replaced_body(self):
        self.replica.poll()
        self.assertEqual(self.replica.manifest_body_reads, 1)
        with open(os.path.join(self.root, 'published.json'), 'rb') as handle:
            pointer = json.loads(handle.read())
        manifest_path = os.path.join(self.root, pointer['manifest'])
        with open(manifest_path, 'wb') as handle:
            handle.write(b'{"replaced":true}')
        self.replica.poll()
        self.assertEqual(self.replica.manifest_body_reads, 1)
        self.assertEqual(self.replica.state, ReplicaState.READY)
        fresh = QueryReplica(self.root)
        self.assertEqual(fresh.poll(), ReplicaState.UNAVAILABLE)
        self.assertIn('POINTER_MISMATCH', fresh.last_error)
        self.assertEqual(fresh.manifest_body_reads, 1)

    def test_resync_rehashes_the_same_pointer(self):
        self.replica._resync('TEST')
        self.assertEqual(self.replica.poll(), ReplicaState.READY)
        self.assertEqual(self.replica.manifest_body_reads, 2)

    def test_head_error_updates_without_rereading_manifest(self):
        self.replica.refresh_control_freshness(2.0)
        head_path = os.path.join(self.root, 'head.json')
        with open(head_path, 'rb') as handle:
            head = json.loads(handle.read().decode('utf-8'))
        head['error'] = 'WRITER_STOPPED'
        head['published_monotonic_ns'] = time.monotonic_ns()
        with open(head_path, 'w') as handle:
            json.dump(head, handle)
        self.replica.poll()
        self.assertEqual(self.replica.manifest_body_reads, 1)
        self.assertEqual(self.replica.head_error, 'WRITER_STOPPED')
        self.assertFalse(self.replica.control_healthy)

    def test_reused_payload_digest_matches_a_fresh_hash(self):
        import luggage_perception.cargo_query_export as export
        hashed = []
        original = export.sha256_hex

        def wrapped(data):
            if isinstance(data, (bytes, bytearray)):
                hashed.append(bytes(data))
            return original(data)

        export.sha256_hex = wrapped
        try:
            result = self.publisher.offer_committed(
                self.snap, dirty_blocks=set())
        finally:
            export.sha256_hex = original
        self.assertEqual(result['status'], 'OK')
        payloads = encode_snapshot(
            self.snap, self.publisher.incarnation, result['seq'],
            dirty_blocks=set(), provenance_changed=True)
        for name, payload in payloads.items():
            self.assertNotIn(payload, hashed)
            self.assertEqual(
                sha256_hex(payload),
                self._entry_digest(result['seq'], name))

    def test_changed_payload_is_hashed_again(self):
        mapper = self.store.mapper
        self.store.apply_update(MapUpdate(
            operation_id='occ', kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=self.store.identity,
            geometry_hash=mapper.geometry_hash,
            points=__import__('numpy').array([[0.1, 0.2, 0.9]]),
            origin=(0.0, 0.0, mapper._hull.ceiling_z - 0.1),
            minimum_z=mapper._hull.floor_z + 0.05,
            stamp_key=(2, 0), acquisition_stamp_ns=2,
            settled=True, schema_version=1))
        changed = snapshot(self.store)
        import luggage_perception.cargo_query_export as export
        hashed = []
        original = export.sha256_hex

        def wrapped(data):
            if isinstance(data, (bytes, bytearray)):
                hashed.append(bytes(data))
            return original(data)

        export.sha256_hex = wrapped
        try:
            result = self.publisher.offer_committed(
                changed, dirty_blocks=None, force_full=False)
        finally:
            export.sha256_hex = original
        self.assertEqual(result['status'], 'OK')
        payloads = encode_snapshot(
            changed, self.publisher.incarnation, result['seq'],
            dirty_blocks=None, provenance_changed=True)
        hashed_payloads = [payload for payload in payloads.values()
                           if payload in hashed]
        self.assertTrue(hashed_payloads)
        for payload in hashed_payloads:
            self.assertEqual(
                sha256_hex(payload),
                self._entry_digest(
                    result['seq'],
                    next(name for name, body in payloads.items()
                         if body == payload)))

    def _entry_digest(self, seq, name):
        path = os.path.join(
            self.root, 'generations', self.publisher.incarnation,
            str(seq), 'manifest.json')
        with open(path, 'rb') as handle:
            manifest = json.loads(handle.read().decode('utf-8'))
        return manifest['entries'][name]['sha256']


if __name__ == '__main__':
    unittest.main()
