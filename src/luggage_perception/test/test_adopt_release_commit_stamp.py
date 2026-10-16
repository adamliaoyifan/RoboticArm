"""Commit-clock labelling for placed-box commits (B1: inside the store).

The node's _adopt_release moved into OccupancyMapStore._adopt_release and
runs inside apply_update. Contract unchanged: with the confirmation gate
on, a commit records a pending change, labels a map that has no stamp yet
with the commit clock, and never refreshes an existing sensor stamp.
Offline: a real store with an injected clock, no ROS graph.
"""
import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
)


class _Clock(object):
    def __init__(self):
        self.now = 7_000_000_000

    def __call__(self):
        return self.now


def _store(gate):
    return OccupancyMapStore(
        CargoVolumeMapper(inner_size=[1.0, 1.0, 1.0],
                          center_base=[0.0, 0.0, 0.5], yaw=0.0,
                          resolution=0.1),
        require_confirmation=gate, clock_ns=_Clock(), epoch="e1")


def _add(store, op, center=(0.1, 0.1, 0.15)):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.ADD_BOX_V1,
        box=(list(center), [0.2, 0.2, 0.2], 0.0)))


def _integrate(store, op, stamp_ns):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.INTEGRATE_VIEW,
        geometry_hash=store.mapper.geometry_hash,
        expected_snapshot=SnapshotIdentity("", store.map_revision, ""),
        settled=True, schema_version=1,
        points=[[0.0, 0.0, 0.8]], origin=(2.0, 0.0, 1.0),
        stamp_key=(int(stamp_ns // 1_000_000_000), 0),
        acquisition_stamp_ns=stamp_ns))


class TestAdoptReleaseCommitStamp(unittest.TestCase):

    def test_gate_on_first_commit_keeps_stamp_absent_and_records_commit_clock(self):
        store = _store(gate=True)
        result = _add(store, "op-1")
        self.assertTrue(result.accepted)
        snap = store.snapshot()
        self.assertEqual(snap.acquisition_stamp_ns, 0)
        self.assertEqual(snap.map_source, "release_integrated")
        self.assertEqual(snap.pending_changes, (("box_placed", 1),))
        # Frozen pending record: (token, event, stamp_ns, confirmable,
        # identity).
        self.assertEqual(snap.pending_details[0],
                         (1, "box_placed", 7_000_000_000, True, ""))

    def test_gate_on_commit_never_refreshes_an_existing_sensor_stamp(self):
        store = _store(gate=True)
        _integrate(store, "i1", stamp_ns=3_000_000_000)
        self.assertEqual(store.snapshot().acquisition_stamp_ns,
                         3_000_000_000)
        _add(store, "op-1", center=(0.4, 0.4, 0.15))
        snap = store.snapshot()
        self.assertEqual(snap.acquisition_stamp_ns, 3_000_000_000)
        self.assertEqual(snap.map_source, "release_integrated")
        self.assertEqual(snap.pending_changes, (("box_placed", 1),))

    def test_gate_on_duplicate_commit_records_nothing_and_keeps_stamp(self):
        store = _store(gate=True)
        _add(store, "op-1")
        pending_after_first = store.snapshot().pending_changes
        result = _add(store, "op-2")  # same box, new operation id
        self.assertTrue(result.accepted)
        self.assertFalse(result.details["committed_now"])
        snap = store.snapshot()
        self.assertEqual(snap.pending_changes, pending_after_first)
        self.assertEqual(snap.acquisition_stamp_ns, 0)

    def test_gate_off_commit_stamps_the_clock(self):
        store = _store(gate=False)
        _add(store, "op-1")
        snap = store.snapshot()
        self.assertEqual(snap.acquisition_stamp_ns, 7_000_000_000)
        self.assertEqual(snap.map_source, "release_integrated")
        self.assertTrue(snap.occupancy_confirmed)  # nothing recorded


if __name__ == "__main__":
    unittest.main()
