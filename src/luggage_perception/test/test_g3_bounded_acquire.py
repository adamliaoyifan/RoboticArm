"""G4-A gen3 CURRENT availability (review_three_issues #2, direction
3): the placement-style client flow is ONE bounded acquisition of
CURRENT (retry while the strict contract refuses, under an explicit
deadline) followed by a PINNED snapshot for all further reads -- never
CURRENT for every internal step.

Run against a real wired store + replica with a back-to-back writer
thread: every bounded acquisition must succeed well inside the
deadline, and the pinned handle must serve consistent reads across
subsequent commits.
"""

import os
import sys
import threading
import time
import unittest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_description"))
sys.path.insert(0, PKG_ROOT)

import numpy as np

from luggage_perception.cargo_map_query import QueryError, QueryPage
from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

REGION = ((-0.15, -0.15, 0.5), (0.15, 0.15, 0.9))


def bounded_acquire(facade, deadline_s=2.0):
    """One bounded CURRENT acquisition: retry named rejections until
    the deadline; return (handle, waited_ms) or raise the last error."""
    deadline = time.monotonic() + deadline_s
    last = None
    while time.monotonic() < deadline:
        try:
            return facade.acquire_query_snapshot('CURRENT'), None
        except QueryError as error:
            last = error
            time.sleep(0.005)
    raise last


class TestBoundedAcquireThenPin(unittest.TestCase):

    def test_bounded_acquire_succeeds_and_pin_stays_consistent(self):
        mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                                   0.05, max_raycast_points=2000)
        store = OccupancyMapStore(mapper, epoch='bounded')
        import tempfile
        root = tempfile.mkdtemp(prefix='bounded-acquire-')
        hook = wire_store_export(store, root)
        self.addCleanup(hook.close)
        store._snapshot = store._build_snapshot()
        hook.wait_pending(30.0)
        replica = QueryReplica(root)
        from luggage_perception.cargo_map_read_facade import (
            QueryMapReadFacade)
        facade = QueryMapReadFacade(replica)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if replica.poll() == ReplicaState.READY:
                break
            time.sleep(0.02)

        rng = np.random.default_rng(7)
        stop = threading.Event()
        errors = []

        def writer():
            stamp = [0]
            while not stop.is_set():
                stamp[0] += 1
                cloud = np.column_stack([
                    rng.uniform(-0.5, 0.5, 4000),
                    rng.uniform(-0.6, 0.6, 4000),
                    rng.uniform(mapper._hull.floor_z + .1,
                                mapper._hull.floor_z + 1.0, 4000)])
                try:
                    store.apply_update(MapUpdate(
                        operation_id='ba-%d' % stamp[0],
                        kind=UpdateKind.INTEGRATE_VIEW,
                        expected_snapshot=store.identity,
                        geometry_hash=mapper.geometry_hash,
                        points=cloud,
                        origin=(0., 0.,
                                mapper._hull.ceiling_z - 0.1),
                        minimum_z=mapper._hull.floor_z + .05,
                        stamp_key=(stamp[0], 0),
                        acquisition_stamp_ns=1_000_000_000 + stamp[0],
                        settled=True, schema_version=1))
                except Exception as error:  # noqa: BLE001
                    errors.append(error)
                    return
                time.sleep(0.25)            # ~4 writes/s overlap load

        thread = threading.Thread(target=writer, daemon=True)
        thread.start()
        self.addCleanup(stop.set)
        try:
            waits = []
            for _ in range(20):
                replica.poll()
                t0 = time.monotonic()
                handle, _ = bounded_acquire(facade, deadline_s=2.0)
                waits.append((time.monotonic() - t0) * 1000.0)
                # PINNED phase: all internal reads use THIS handle; the
                # identity must not move even as the writer commits.
                identity = tuple(handle.identity)
                first = handle.query_cells(
                    REGION, (0, 1, 2), page=QueryPage(4096, ''))
                for _ in range(5):
                    time.sleep(0.03)
                    replica.poll()
                    again = handle.query_cells(
                        REGION, (0, 1, 2), page=QueryPage(4096, ''))
                    self.assertEqual(tuple(handle.identity), identity)
                    self.assertEqual(len(again.cells), len(first.cells))
                handle.close()
            self.assertFalse(errors)
            self.assertTrue(all(w < 2000.0 for w in waits))
            worst = round(max(waits), 1)
            median = round(sorted(waits)[len(waits) // 2], 1)
            self.assertLess(worst, 2000.0)
            # record for the report: worst bounded-acquire wait under a
            # ~4 writes/s overlap load (observed far below the deadline)
            self.assertLess(median, 500.0)
        finally:
            stop.set()
            thread.join(timeout=10)


if __name__ == '__main__':
    unittest.main()
