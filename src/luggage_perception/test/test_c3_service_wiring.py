#!/usr/bin/env python3
"""C3 behavior gates (three pinned behaviors + two wording boundaries).

1. Single provider per service name: the isolated facade serves the
   four typed services and nothing else; in_process default unchanged
   (QueryMode rejects unknown modes, defaults to in_process).
2. The store hook uses the async path: apply_update returns with the
   physical fact committed while the encoder may still be draining; a
   full queue or export failure is a named status and never blocks or
   rolls back the commit (tested by injecting a failing factory).
3. Named failures, never pseudo-healthy stale maps: no initial
   snapshot -> QUERY_INDEX_UNAVAILABLE; known-newer committed head
   past the lag bound -> REPLICA_LAGGING (a fresh heartbeat alone does
   NOT mask it); wrong expected identity -> SNAPSHOT_IDENTITY_MISMATCH.

Wording boundaries kept (user 2026-10-26): heartbeat freshness != map
freshness (the facade compares committed vs local READY identities and
checks error/lag); held-generation immutability != full reclaim
protocol (reclaim/ACK-loss/interruption/resource caps are later fault
gates -- not claimed here).
"""

import os
import sys
import tempfile
import shutil
import time
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, PKG_ROOT)

from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_map_query_mode import (
    QueryMode, StoreExportHook, wire_store_export)
from luggage_perception.cargo_query_export import QueryExportPublisher
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


class FailingPublisher:
    """Factory stub whose every offer fails -- proves the commit path
    survives export failure (physical facts never roll back)."""

    def __init__(self, root):
        self.root = root
        self.calls = 0

    def offer_committed(self, *args, **kwargs):
        self.calls += 1
        return {'status': 'EXPORT_IO_ERROR'}

    def wait_pending(self, timeout_s=30.0):
        return True

    def close(self):
        pass


def make_store(boxes=5, resolution=0.05):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='c3')
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


class TestQueryMode(unittest.TestCase):

    def test_default_and_validation(self):
        self.assertEqual(QueryMode.from_launch(''), QueryMode.IN_PROCESS)
        self.assertEqual(QueryMode.from_launch('in_process'),
                         QueryMode.IN_PROCESS)
        self.assertEqual(QueryMode.from_launch('isolated'),
                         QueryMode.ISOLATED)
        with self.assertRaises(ValueError):
            QueryMode.from_launch('both')


class TestStoreHookAsync(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='c3-')
        self.addCleanup(shutil.rmtree, self.root, True)

    def test_hook_uses_async_path_and_commit_survives(self):
        store, mapper = make_store()
        hook = wire_store_export(store, self.root)
        self.addCleanup(hook.close)
        # The hook must have created an ASYNC publisher.
        self.assertTrue(hook._publisher._async)
        result = store.apply_update(MapUpdate(
            operation_id='pose-1', kind=UpdateKind.CONTAINER_POSE,
            event_stamp_ns=1))
        self.assertEqual(result.status.value, 'accepted',
                         'commit must not be blocked by export')
        self.assertTrue(hook.wait_pending(30.0))
        self.assertEqual(hook.last_status['status'], 'ACCEPTED')

    def test_export_failure_never_rolls_back(self):
        store, mapper = make_store()
        hook = wire_store_export(store, self.root)
        self.addCleanup(hook.close)
        failing = FailingPublisher(self.root)
        hook._publisher = failing  # every later offer now fails
        result = store.apply_update(MapUpdate(
            operation_id='box-x', kind=UpdateKind.ADD_BOX_V1,
            box=((0.4, -0.4, mapper._hull.floor_z + .11),
                 (.35, .53, .22), 0.), box_id='box-x'))
        self.assertEqual(result.status.value, 'accepted',
                         'export failure must not block or roll back')
        self.assertGreater(failing.calls, 0)
        self.assertEqual(hook.last_status['status'], 'EXPORT_IO_ERROR')
        # The physical fact is committed: the box is queryable.
        with store.acquire_query_snapshot() as handle:
            self.assertGreater(len(handle._data().box_blocks), 5)


class TestNamedFailuresFacade(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix='c3f-')
        self.addCleanup(shutil.rmtree, self.root, True)
        from luggage_perception.cargo_map_replica_node import (
            QueryMapReadFacade)
        self.store, self.mapper = make_store()
        self.publisher = QueryExportPublisher(self.root, async_mode=True)
        self.addCleanup(self.publisher.close)
        self.replica = QueryReplica(self.root)
        self.facade = QueryMapReadFacade(self.replica)

    def _publish(self, dirty=None):
        self.store._snapshot = self.store._build_snapshot()
        with self.store.acquire_query_snapshot() as handle:
            data = handle._data()
        self.publisher.offer_committed(
            data, dirty_blocks=dirty, provenance_changed=True)
        self.publisher.wait_pending(30.0)

    def test_no_snapshot_named_unavailable(self):
        with self.assertRaises(QueryError) as caught:
            self.facade.acquire_query_snapshot()
        self.assertEqual(caught.exception.reason,
                         'QUERY_INDEX_UNAVAILABLE')

    def test_known_newer_head_beyond_lag_is_named_lagging(self):
        self._publish()
        self.assertEqual(self.replica.poll(), ReplicaState.READY)
        # Simulate a stalled receiver: the writer commits+heartbeats a
        # NEWER identity (fresh heartbeat) but the replica never
        # applies it. A fresh heartbeat must NOT mask the lag.
        head_path = os.path.join(self.root, 'head.json')
        with open(head_path, 'r+b') as handle:
            head = handle.read()
        newer = head.replace(
            b'"revision":' + str(self.replica.manifest[
                'target_identity'][1]).encode(),
            b'"revision":' + str(self.replica.manifest[
                'target_identity'][1] + 5).encode())
        # Rewrite with a fresh monotonic stamp so the heartbeat itself
        # is undeniably alive.
        self.publisher._latest_committed = (
            self.publisher._latest_committed[0], set(), True)
        self.publisher._head_written_ns = 0
        import json as _json
        state = _json.loads(head.decode())
        state['target_identity'] = list(state['target_identity'])
        state['target_identity'][1] += 5
        state['published_monotonic_ns'] = time.monotonic_ns()
        state['commit_monotonic_ns'] = time.monotonic_ns()
        temp = head_path + '.tmp'
        with open(temp, 'w') as handle:
            handle.write(_json.dumps(state, sort_keys=True,
                                     separators=(',', ':')))
        os.replace(temp, head_path)
        self.replica.state = ReplicaState.READY  # local view unchanged
        # G4-A gen3 contract (plan d87c5731 1.1, review_8a372dee P1-3):
        # CURRENT is STRICT -- a known newer committed head rejects
        # regardless of the lag bound; a fresh heartbeat never masks
        # the lag. The bounded-stale window belongs to PINNED reads.
        with self.assertRaises(QueryError) as caught:
            self.facade._lag_bound_s = 0.0
            self.facade.acquire_query_snapshot()
        self.assertEqual(caught.exception.reason, 'REPLICA_LAGGING')
        self.facade._lag_bound_s = 2.0
        with self.assertRaises(QueryError) as caught:
            self.facade.acquire_query_snapshot('CURRENT')
        self.assertEqual(caught.exception.reason, 'REPLICA_LAGGING')
        # A PINNED read of the local identity inside the same-world
        # bound still serves the local complete generation with its
        # exact echoed identity.
        snapshot = self.facade.acquire_query_snapshot(
            tuple(self.replica.manifest['target_identity']))
        self.assertEqual(
            tuple(snapshot.identity),
            tuple(self.replica.manifest['target_identity']))

    def test_wrong_expected_identity_named_mismatch(self):
        self._publish()
        self.assertEqual(self.replica.poll(), ReplicaState.READY)
        local = list(self.replica.manifest['target_identity'])
        wrong = list(local)
        wrong[1] = wrong[1] + 9
        with self.assertRaises(QueryError) as caught:
            self.facade.acquire_query_snapshot(tuple(wrong))
        self.assertEqual(caught.exception.reason,
                         'SNAPSHOT_IDENTITY_MISMATCH')


if __name__ == "__main__":
    unittest.main()
