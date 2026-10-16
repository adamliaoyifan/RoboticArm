#!/usr/bin/env python3
"""Pending-vs-observation acceptance rows at the store level (Plan B / B1).

未确认 → 有效观测 → 已确认；已确认 → 释放/移除/碰动/位姿变化 → 待确认；
待确认 → 覆盖变化区域的有效观测 → 已确认。遮挡和"没有看到"不是空闲证据；
时间流逝本身不使已确认地图失效。
"""
import itertools
import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
    UpdateStatus,
)


class _Clock(object):
    def __init__(self):
        self.now = 1_000_000_000_000000000

    def __call__(self):
        return self.now

    def advance(self, ns):
        self.now += ns


_BOX = ((0.0, 0.0, 0.15), (0.4, 0.4, 0.2), 0.0)
#: Three points on the box's upper half in distinct voxels (>= 0.1 m
#: apart: apply_integrate downsamples to the map resolution).
_UPPER = [[0.0, 0.0, 0.20], [0.15, 0.0, 0.20], [-0.15, 0.0, 0.20]]
_ELSEWHERE = [[0.4, 0.4, 0.60], [0.45, 0.4, 0.60], [0.4, 0.45, 0.60]]


def _store():
    return OccupancyMapStore(
        CargoVolumeMapper(inner_size=[1.0, 1.0, 1.0],
                          center_base=[0.0, 0.0, 0.5], yaw=0.0,
                          resolution=0.1),
        require_confirmation=True, clock_ns=_Clock(), epoch="e1")


def _add(store, op="a1"):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.ADD_BOX_V2,
        expected_snapshot=SnapshotIdentity("e1", store.map_revision, ""),
        geometry_hash=store.mapper.geometry_hash, box=_BOX,
        measured_source="verify_placed_box",
        verified_center=_BOX[0], verify_tolerance_m=0.10))


_SEQ = itertools.count(10)


def _observe(store, points, op="i1", stamp_ns=2_000_000_000_000000000,
             settled=True, stamp_key=None):
    if stamp_key is None:
        stamp_key = (next(_SEQ), 0)
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=SnapshotIdentity("", store.map_revision, ""),
        geometry_hash=store.mapper.geometry_hash,
        settled=settled, schema_version=1, points=points,
        origin=(2.0, 0.0, 1.0),
        stamp_key=stamp_key,
        acquisition_stamp_ns=stamp_ns))


def _note(store, event, op, stamp_ns=None):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.NOTE_EVENT, event=event,
        event_stamp_ns=stamp_ns if stamp_ns is not None else _Clock().now))


def _remove(store, op="rel-1", stale_identity=None):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.REMOVE_BOX_V2,
        expected_snapshot=stale_identity, geometry_hash="",
        box=_BOX))


class TestConfirmationMachine(unittest.TestCase):

    def test_unconfirmed_until_valid_observation(self):
        store = _store()
        r_add = _add(store)
        self.assertTrue(r_add.accepted)
        self.assertFalse(store.snapshot().occupancy_confirmed)
        # An observation that does not see the box's upper half does not
        # confirm (occlusion / "did not look" is not free-space evidence).
        r_off = _observe(store, _ELSEWHERE, op="i0")
        self.assertTrue(r_off.accepted)
        self.assertFalse(store.snapshot().occupancy_confirmed)
        # Covering observation confirms.
        r_on = _observe(store, _UPPER, op="i1")
        self.assertTrue(r_on.accepted)
        self.assertTrue(store.snapshot().occupancy_confirmed)

    def test_not_settled_view_never_confirms(self):
        """An unsettled view is refused at the premise (existing OCC
        contract) — it never reaches the map, so it can never confirm."""
        store = _store()
        _add(store)
        r = _observe(store, _UPPER, op="i1", settled=False)
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "NOT_SETTLED")
        self.assertFalse(store.snapshot().occupancy_confirmed)

    def test_disturbance_moves_confirmed_to_pending(self):
        store = _store()
        _add(store)
        _observe(store, _UPPER, op="i1")
        self.assertTrue(store.snapshot().occupancy_confirmed)
        r = _note(store, "box_disturbed", op="n1")
        self.assertTrue(r.accepted)
        snap = store.snapshot()
        self.assertFalse(snap.occupancy_confirmed)
        self.assertEqual(snap.change_event, "box_disturbed")

    def test_removal_stays_pending_fail_closed(self):
        """No observation rule for removal yet: a removed box's region
        must be re-observed before the map claims confirmed again —
        first version fails closed (stays pending)."""
        store = _store()
        _add(store)
        _observe(store, _UPPER, op="i1")
        r = _remove(store)  # matching premise, box exists
        self.assertTrue(r.accepted)
        snap = store.snapshot()
        self.assertFalse(snap.occupancy_confirmed)
        self.assertEqual(snap.placed_count, 0)
        # Even a covering observation of the region cannot confirm a
        # removal in this version (no observed-free rule).
        r2 = _observe(store, _UPPER, op="i2",
                      stamp_ns=3_000_000_000_000000000)
        self.assertTrue(r2.accepted)
        self.assertFalse(store.snapshot().occupancy_confirmed)

    def test_disturbance_stays_pending_fail_closed_b1(self):
        """B1 semantics: disturbance has no observation rule yet (an
        observed disturbance means the box moved somewhere unknown), so it
        stays pending until reset even under a covering observation.
        The spec's full pending-recovery rule (footprint-carrying
        disturbance events) is deferred to B2."""
        store = _store()
        _add(store)
        _observe(store, _UPPER, op="i1")
        self.assertTrue(store.snapshot().occupancy_confirmed)
        _note(store, "box_disturbed", op="n1")
        self.assertFalse(store.snapshot().occupancy_confirmed)
        r = _observe(store, _UPPER, op="i2",
                     stamp_ns=3_000_000_000_000000000)
        self.assertTrue(r.accepted)
        self.assertFalse(store.snapshot().occupancy_confirmed)
        # The one recovery path in B1: reset clears everything.
        r_reset = store.apply_update(
            MapUpdate(operation_id="reset", kind=UpdateKind.RESET))
        self.assertTrue(r_reset.accepted)
        self.assertTrue(store.snapshot().occupancy_confirmed)

    def test_old_frame_does_not_confirm(self):
        """A frame acquired before the change cannot clear it (the
        integration must also have begun after the change)."""
        store = _store()
        _add(store)
        # Frame stamped BEFORE the commit clock (clock sits at 1e18).
        r = _observe(store, _UPPER, op="i1",
                     stamp_ns=500_000_000_000000000)
        self.assertTrue(r.accepted)
        self.assertFalse(store.snapshot().occupancy_confirmed)

    def test_time_alone_does_not_invalidate(self):
        store = _store()
        _add(store)
        _observe(store, _UPPER, op="i1")
        identity = store.identity
        confirmed = store.snapshot().occupancy_confirmed
        clock = store._clock_ns
        clock.advance(10_000_000_000)  # 10 s of nothing happening
        snap = store.snapshot()
        self.assertEqual(snap.identity, identity)
        self.assertEqual(snap.occupancy_confirmed, confirmed)

    def test_duplicate_observation_does_not_invalidate(self):
        """Same-content re-observation (a second settled view of the same
        box) keeps the map confirmed and the digest stable."""
        store = _store()
        _add(store)
        _observe(store, _UPPER, op="i1", stamp_key=(2, 0))
        digest = store.identity.content_digest
        r = _observe(store, _UPPER, op="i2", stamp_key=(3, 0),
                     stamp_ns=3_000_000_000_000000000)
        self.assertTrue(r.accepted)
        self.assertTrue(store.snapshot().occupancy_confirmed)
        self.assertEqual(store.identity.content_digest, digest)


class TestFactRecording(unittest.TestCase):

    def test_container_pose_change_via_fact_kind(self):
        store = _store()
        _add(store)
        _observe(store, _UPPER, op="i1")
        r = store.apply_update(MapUpdate(
            operation_id="pose-1", kind=UpdateKind.CONTAINER_POSE))
        self.assertTrue(r.accepted)
        snap = store.snapshot()
        self.assertFalse(snap.occupancy_confirmed)
        self.assertEqual(snap.change_event, "container_pose_changed")
        self.assertIn(("container_pose_changed", 2),
                      snap.pending_changes)

    def test_fact_with_foreign_hull_rejected(self):
        store = _store()
        r = _remove(store, stale_identity=None)  # not used here
        self.assertTrue(r.accepted or r.status is UpdateStatus.REJECTED)
        r2 = store.apply_update(MapUpdate(
            operation_id="pose-x", kind=UpdateKind.CONTAINER_POSE,
            geometry_hash="foreign-hull"))
        self.assertIs(r2.status, UpdateStatus.REJECTED)
        self.assertEqual(r2.reason, "GEOMETRY_HASH_MISMATCH")


if __name__ == "__main__":
    unittest.main()
