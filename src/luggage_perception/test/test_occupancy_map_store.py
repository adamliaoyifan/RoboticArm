#!/usr/bin/env python3
"""OccupancyMapStore transaction contract (Plan B / B1, offline Gate 1).

Covers the acceptance-matrix rows testable without consumers: duplicate
and concurrent updates, restart/out-of-order identity conflicts, pending
vs observation, snapshot freezing, cache cap, and the content-digest
properties (column-height detection, confirmation detection, re-observation
stability).
"""
import threading
import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    PROCESSED_CAP,
    MapSnapshot,
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
    UpdateStatus,
    _content_digest,
)


class _Clock(object):
    def __init__(self):
        self.now = 1_000_000_000_000000000

    def __call__(self):
        return self.now

    def advance(self, ns):
        self.now += ns


def _mapper():
    return CargoVolumeMapper(
        inner_size=[1.0, 1.0, 1.0],
        center_base=[0.0, 0.0, 0.5],
        yaw=0.0,
        resolution=0.1,
    )


def _store(require_confirmation=False, cap=PROCESSED_CAP):
    return OccupancyMapStore(
        _mapper(), require_confirmation=require_confirmation,
        clock_ns=_Clock(), epoch="e1", processed_cap=cap)


class _StoreCase(unittest.TestCase):
    """Fixture: helpers carry the mapper's geometry hash and derive the
    verify anchor from the slot itself (matching the v2 adapter)."""

    require_confirmation = False

    def setUp(self):
        self.store = _store(
            require_confirmation=self.require_confirmation)

    @property
    def _gh(self):
        return self.store.mapper.geometry_hash

    def _add(self, op, center, rev=None, digest="", epoch="e1", tol=0.10,
             verified=None, size=(0.2, 0.2, 0.2)):
        if rev is None:
            rev = self.store.map_revision
        if verified is None:
            verified = center
        return MapUpdate(
            operation_id=op, kind=UpdateKind.ADD_BOX_V2,
            expected_snapshot=SnapshotIdentity(epoch, rev, digest),
            geometry_hash=self._gh, box=(tuple(center), tuple(size), 0.0),
            measured_source="verify_placed_box",
            verified_center=tuple(verified), verify_tolerance_m=tol)

    def _integrate(self, op, points, rev=None,
                   stamp_ns=2_000_000_000_000000000,
                   stamp_key=None, settled=True):
        if rev is None:
            rev = self.store.map_revision
        return MapUpdate(
            operation_id=op, kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=SnapshotIdentity("", rev, ""),
            geometry_hash=self._gh,
            settled=settled, schema_version=1,
            points=[list(p) for p in points], origin=(2.0, 0.0, 1.0),
            stamp_key=stamp_key or (int(stamp_ns // 1_000_000_000), 0),
            acquisition_stamp_ns=stamp_ns)


class TestContentDigest(_StoreCase):

    def test_same_content_reobservation_digest_stable(self):
        """Re-observing identical content must not invalidate routes: the
        revision moves, the digest does not."""
        r0 = self.store.apply_update(self._integrate(
            "i1", [(0.0, 0.0, 0.5), (0.1, 0.0, 0.5)], stamp_key=(1, 0)))
        self.assertTrue(r0.accepted)
        r1 = self.store.apply_update(self._integrate(
            "i2", [(0.0, 0.0, 0.5), (0.1, 0.0, 0.5)],
            stamp_ns=2_000_000_000, stamp_key=(2, 0)))
        self.assertTrue(r1.accepted)
        self.assertEqual(r0.identity.content_digest,
                         r1.identity.content_digest)
        self.assertGreater(r1.identity.revision, r0.identity.revision)

    def test_column_height_change_detected(self):
        r_short = self.store.apply_update(
            self._add("a1", (0.0, 0.0, 0.10), verified=(0.0, 0.0, 0.10)))
        r_tall = self.store.apply_update(self._add(
            "a2", (0.5, 0.5, 0.20), verified=(0.5, 0.5, 0.20),
            size=(0.2, 0.2, 0.4)))
        self.assertTrue(r_short.accepted)
        self.assertTrue(r_tall.accepted)
        self.assertNotEqual(r_short.identity.content_digest,
                            r_tall.identity.content_digest)

    def test_sensor_vs_geometry_flip_detected(self):
        """A sensor top later locked to geometry changes the digest even at
        an equal column height (confidence is digest input)."""
        r_int = self.store.apply_update(self._integrate(
            "i1", [(0.0, 0.0, 0.5)], stamp_key=(1, 0)))
        digest_sensor = r_int.identity.content_digest
        r_box = self.store.apply_update(self._add(
            "a1", (0.0, 0.0, 0.25), verified=(0.0, 0.0, 0.25),
            size=(0.2, 0.2, 0.5)))
        self.assertNotEqual(digest_sensor, r_box.identity.content_digest)

    def test_confirmation_state_in_digest(self):
        store = _store(require_confirmation=True)
        base = store.map_revision
        r_add = store.apply_update(MapUpdate(
            operation_id="a1", kind=UpdateKind.ADD_BOX_V2,
            expected_snapshot=SnapshotIdentity("e1", base, ""),
            geometry_hash=store.mapper.geometry_hash,
            box=((0.0, 0.0, 0.15), (0.4, 0.4, 0.2), 0.0),
            measured_source="verify_placed_box",
            verified_center=(0.0, 0.0, 0.15), verify_tolerance_m=0.10))
        self.assertFalse(r_add.snapshot.occupancy_confirmed)
        # A confirming observation: >=3 points on the box's upper half in
        # DISTINCT voxels (apply_integrate downsamples to the map
        # resolution), frame strictly after the change, settled.
        r_obs = store.apply_update(MapUpdate(
            operation_id="i1", kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=SnapshotIdentity("", r_add.identity.revision, ""),
            geometry_hash=store.mapper.geometry_hash,
            settled=True, schema_version=1,
            points=[[0.0, 0.0, 0.20], [0.15, 0.0, 0.20], [-0.15, 0.0, 0.20]],
            origin=(2.0, 0.0, 1.0), stamp_key=(5, 0),
            acquisition_stamp_ns=2_000_000_000_000000000))
        self.assertTrue(r_obs.accepted, r_obs.details)
        self.assertTrue(r_obs.snapshot.occupancy_confirmed)
        self.assertNotEqual(r_add.identity.content_digest,
                            r_obs.identity.content_digest)

    def test_digest_deterministic_across_rebuild(self):
        self.store.apply_update(self._add("a1", (0.0, 0.0, 0.15)))
        first = self.store.snapshot()
        self.assertEqual(first.identity.content_digest,
                         _content_digest(first))


class TestIdempotency(_StoreCase):

    def test_duplicate_operation_id_exact_once(self):
        update = self._add("op-1", (0.0, 0.0, 0.15))
        r1 = self.store.apply_update(update)
        self.assertTrue(r1.accepted)
        replay = self.store.apply_update(update)  # literal retry, same bytes
        self.assertIs(replay.status, UpdateStatus.ALREADY_PROCESSED)
        self.assertEqual(replay.identity, r1.identity)
        self.assertTrue(replay.details["replay"])
        self.assertEqual(replay.details["original_status"], "accepted")
        self.assertEqual(self.store.map_revision, r1.identity.revision)

    def test_same_operation_id_different_content_rejected(self):
        self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        clash = self.store.apply_update(self._add("op-1", (0.5, 0.5, 0.15)))
        self.assertIs(clash.status, UpdateStatus.REJECTED)
        self.assertEqual(clash.reason, "OPERATION_ID_CONFLICT")
        self.assertEqual(self.store.snapshot().placed_count, 1)

    def test_empty_operation_id_rejected(self):
        r = self.store.apply_update(self._add("", (0.0, 0.0, 0.15)))
        self.assertIs(r.status, UpdateStatus.REJECTED)

    def test_processed_cache_cap_eviction_is_duplicate_safe(self):
        store = _store(cap=3)
        gh = store.mapper.geometry_hash
        updates = []

        def add(op, center):
            update = MapUpdate(
                operation_id=op, kind=UpdateKind.ADD_BOX_V2,
                expected_snapshot=SnapshotIdentity(
                    "e1", store.map_revision, ""),
                geometry_hash=gh, box=(tuple(center), (0.2, 0.2, 0.2), 0.0),
                measured_source="verify_placed_box",
                verified_center=tuple(center), verify_tolerance_m=0.10)
            updates.append(update)
            return store.apply_update(update)

        add("op-1", (0.0, 0.0, 0.15))
        add("op-2", (0.3, 0.3, 0.15))
        add("op-3", (-0.3, 0.3, 0.15))
        add("op-4", (0.3, -0.3, 0.15))
        self.assertEqual(len(store._processed), 3)
        # op-1 was evicted. A literal retry WITH its stale premise fails
        # closed (the documented caller rule: premise-first, never rebase).
        replay = store.apply_update(updates[0])
        self.assertIs(replay.status, UpdateStatus.REJECTED)
        self.assertEqual(replay.reason, "REVISION_MISMATCH")
        # Without a premise the retry re-executes and the mapper's own
        # ledger matching keeps it duplicate-safe (no double box).
        replay_open = store.apply_update(MapUpdate(
            operation_id="op-1", kind=UpdateKind.ADD_BOX_V2,
            geometry_hash=gh, box=((0.0, 0.0, 0.15), (0.2, 0.2, 0.2), 0.0),
            measured_source="verify_placed_box",
            verified_center=(0.0, 0.0, 0.15), verify_tolerance_m=0.10))
        self.assertTrue(replay_open.accepted)
        self.assertFalse(replay_open.details["committed_now"])
        self.assertEqual(store.snapshot().placed_count, 4)

    def test_replay_survives_reset_without_reapplying(self):
        """A retried pre-reset write must not re-apply onto the fresh
        epoch (idempotency-first: exactly-once beats premise replay)."""
        update = self._add("op-1", (0.0, 0.0, 0.15))
        self.store.apply_update(update)
        self.store.apply_update(
            MapUpdate(operation_id="reset", kind=UpdateKind.RESET))
        self.assertEqual(self.store.snapshot().placed_count, 0)
        replay = self.store.apply_update(update)
        self.assertIs(replay.status, UpdateStatus.ALREADY_PROCESSED)
        self.assertEqual(self.store.snapshot().placed_count, 0)


class TestPremiseAndEpoch(_StoreCase):

    def test_version_conflict_no_autoretry(self):
        r1 = self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        stale = self.store.apply_update(
            self._add("op-2", (0.5, 0.5, 0.15), rev=r1.identity.revision - 1))
        self.assertIs(stale.status, UpdateStatus.REJECTED)
        self.assertEqual(stale.reason, "REVISION_MISMATCH")
        # The current identity rides in the result for the caller re-read.
        self.assertEqual(stale.identity, self.store.identity)
        self.assertEqual(self.store.snapshot().placed_count, 1)

    def test_epoch_mismatch_after_reset(self):
        self.store.apply_update(
            MapUpdate(operation_id="reset", kind=UpdateKind.RESET))
        old = self.store.apply_update(
            self._add("op-x", (0.0, 0.0, 0.15), epoch="e1", rev=1))
        self.assertIs(old.status, UpdateStatus.REJECTED)
        self.assertEqual(old.reason, "EPOCH_MISMATCH")

    def test_epoch_mismatch_after_restart(self):
        first = _store()
        second = OccupancyMapStore(_mapper(), clock_ns=_Clock(), epoch="e2")
        stale = second.apply_update(MapUpdate(
            operation_id="op-1", kind=UpdateKind.ADD_BOX_V2,
            expected_snapshot=SnapshotIdentity(first.epoch, 1, ""),
            geometry_hash=second.mapper.geometry_hash,
            box=((0.0, 0.0, 0.15), (0.2, 0.2, 0.2), 0.0),
            measured_source="verify_placed_box",
            verified_center=(0.0, 0.0, 0.15), verify_tolerance_m=0.10))
        self.assertIs(stale.status, UpdateStatus.REJECTED)
        self.assertEqual(stale.reason, "EPOCH_MISMATCH")

    def test_content_digest_leg_mismatch(self):
        r = self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        clash = self.store.apply_update(self._add(
            "op-2", (0.5, 0.5, 0.15), rev=r.identity.revision,
            digest="0" * 64))
        self.assertIs(clash.status, UpdateStatus.REJECTED)
        self.assertEqual(clash.reason, "CONTENT_DIGEST_MISMATCH")

    def test_late_fact_on_stale_premise_recorded_pending(self):
        """A release/removal fact arriving late is recorded, not dropped:
        the map stops claiming confirmed-unchanged."""
        self.require_confirmation = True
        self.setUp()
        r_add = self.store.apply_update(self._add(
            "op-1", (0.0, 0.0, 0.15), size=(0.4, 0.4, 0.2)))
        r_obs = self.store.apply_update(self._integrate(
            "i1", [(0.0, 0.0, 0.20), (0.15, 0.0, 0.20), (-0.15, 0.0, 0.20)],
            stamp_key=(5, 0)))
        self.assertTrue(r_obs.accepted, r_obs.details)
        self.assertTrue(r_obs.snapshot.occupancy_confirmed)
        late = self.store.apply_update(MapUpdate(
            operation_id="rel-1", kind=UpdateKind.REMOVE_BOX_V2,
            expected_snapshot=r_add.identity,  # stale on purpose
            geometry_hash=self._gh,
            box=((0.0, 0.0, 0.15), (0.4, 0.4, 0.2), 0.0),
            event="box_removed"))
        self.assertIs(late.status, UpdateStatus.RECORDED_PENDING)
        self.assertEqual(late.reason, "LATE_FACT_RECORDED_PENDING")
        self.assertFalse(late.snapshot.occupancy_confirmed)
        self.assertEqual(late.snapshot.pending_changes,
                         (("box_removed", 2),))
        self.assertGreater(late.identity.revision, r_obs.identity.revision)
        self.assertEqual(late.snapshot.placed_count, 1)  # not removed

    def test_late_fact_gate_off_fails_closed(self):
        """Without the confirmation gate there is nothing to record, so a
        stale-premise fact is refused (pre-B1 node behavior)."""
        r_add = self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        self.assertTrue(r_add.accepted)
        stale = self.store.apply_update(MapUpdate(
            operation_id="rel-1", kind=UpdateKind.REMOVE_BOX_V2,
            expected_snapshot=SnapshotIdentity("e1", 1, ""),
            geometry_hash=self._gh,
            box=((0.0, 0.0, 0.15), (0.2, 0.2, 0.2), 0.0)))
        self.assertIs(stale.status, UpdateStatus.REJECTED)
        self.assertEqual(stale.reason, "REVISION_MISMATCH")
        self.assertEqual(self.store.snapshot().placed_count, 1)

    def test_remove_without_match_rejected(self):
        self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        r = self.store.apply_update(MapUpdate(
            operation_id="rel-x", kind=UpdateKind.REMOVE_BOX_V2,
            geometry_hash=self._gh,
            box=((0.8, 0.8, 0.15), (0.2, 0.2, 0.2), 0.0)))
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "NO_MATCHING_BOX")


class TestSnapshotFreeze(_StoreCase):

    def test_snapshot_is_frozen(self):
        self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        snap = self.store.snapshot()
        self.assertIsInstance(snap, MapSnapshot)
        with self.assertRaises(Exception):
            snap.placed_count = 5
        with self.assertRaises(TypeError):
            snap.height[0][0] = 9.0

    def test_snapshot_survives_later_mutation(self):
        self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        held = self.store.snapshot()
        held_identity = held.identity
        held_digest = _content_digest(held)
        self.store.apply_update(self._add("op-2", (0.5, 0.5, 0.15)))
        self.assertEqual(held.identity, held_identity)
        self.assertEqual(_content_digest(held), held_digest)
        self.assertEqual(held.placed_count, 1)
        self.assertEqual(self.store.snapshot().placed_count, 2)


class TestConcurrency(_StoreCase):
    """Gate-1 requirement: both orderings plus the simultaneous race,
    20 repetitions each, zero invariant violations."""

    REPS = 20

    def test_concurrent_same_operation_id(self):
        for _ in range(self.REPS):
            store = _store()
            barrier = threading.Barrier(2)
            results = []

            def worker():
                barrier.wait()
                results.append(store.apply_update(MapUpdate(
                    operation_id="op-1", kind=UpdateKind.ADD_BOX_V2,
                    expected_snapshot=SnapshotIdentity("e1", 1, ""),
                    geometry_hash=store.mapper.geometry_hash,
                    box=((0.0, 0.0, 0.15), (0.2, 0.2, 0.2), 0.0),
                    measured_source="verify_placed_box",
                    verified_center=(0.0, 0.0, 0.15),
                    verify_tolerance_m=0.10)))

            threads = [threading.Thread(target=worker) for _ in range(2)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            statuses = sorted(r.status.value for r in results)
            self.assertEqual(
                statuses, ["accepted", "already_processed"],
                "exactly one ACCEPTED per id per content, got %s" % statuses)
            self.assertEqual(store.snapshot().placed_count, 1)

    def test_concurrent_conflicting_premise(self):
        """Two writers against the same expected identity: at most one
        lands; the loser sees the current identity."""
        for _ in range(self.REPS):
            store = _store()
            gh = store.mapper.geometry_hash
            barrier = threading.Barrier(2)
            results = []

            def worker(op, center):
                barrier.wait()
                results.append(store.apply_update(MapUpdate(
                    operation_id=op, kind=UpdateKind.ADD_BOX_V2,
                    expected_snapshot=SnapshotIdentity("e1", 1, ""),
                    geometry_hash=gh,
                    box=(tuple(center), (0.2, 0.2, 0.2), 0.0),
                    measured_source="verify_placed_box",
                    verified_center=tuple(center),
                    verify_tolerance_m=0.10)))

            threads = [
                threading.Thread(target=worker, args=("a", (0.0, 0.0, 0.15))),
                threading.Thread(target=worker, args=("b", (0.5, 0.5, 0.15))),
            ]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            accepted = [r for r in results if r.accepted]
            rejected = [r for r in results
                        if r.status is UpdateStatus.REJECTED]
            self.assertEqual(len(accepted), 1)
            self.assertEqual(len(rejected), 1)
            self.assertEqual(rejected[0].reason, "REVISION_MISMATCH")
            self.assertEqual(rejected[0].identity, store.identity)

    def test_serial_orderings_converge(self):
        """A->B and B->A over disjoint boxes converge to one digest."""
        for _ in range(self.REPS):
            forward = _store()
            reverse = _store()

            def add(store, op, center):
                return store.apply_update(MapUpdate(
                    operation_id=op, kind=UpdateKind.ADD_BOX_V2,
                    expected_snapshot=SnapshotIdentity(
                        "e1", store.map_revision, ""),
                    geometry_hash=store.mapper.geometry_hash,
                    box=(tuple(center), (0.2, 0.2, 0.2), 0.0),
                    measured_source="verify_placed_box",
                    verified_center=tuple(center),
                    verify_tolerance_m=0.10))

            r1 = add(forward, "a", (0.0, 0.0, 0.15))
            add(forward, "b", (0.5, 0.5, 0.15))
            r2 = add(reverse, "b", (0.5, 0.5, 0.15))
            add(reverse, "a", (0.0, 0.0, 0.15))
            self.assertTrue(r1.accepted and r2.accepted)
            self.assertEqual(forward.identity.content_digest,
                             reverse.identity.content_digest)
            self.assertEqual(forward.map_revision, reverse.map_revision)


class TestResetSemantics(_StoreCase):

    def test_reset_rotates_epoch_and_clears_state(self):
        r_add = self.store.apply_update(self._add("op-1", (0.0, 0.0, 0.15)))
        epoch_before = self.store.epoch
        r_reset = self.store.apply_update(
            MapUpdate(operation_id="reset", kind=UpdateKind.RESET))
        self.assertTrue(r_reset.accepted)
        self.assertNotEqual(self.store.epoch, epoch_before)
        snap = r_reset.snapshot
        self.assertEqual(snap.placed_count, 0)
        self.assertTrue(snap.occupancy_confirmed)  # nothing pending anymore
        self.assertEqual(snap.map_source, "static_empty")
        self.assertEqual(snap.acquisition_stamp_ns, 0)
        self.assertGreater(r_reset.identity.revision,
                           r_add.identity.revision)

    def test_note_event_bumps_only_when_recorded(self):
        store = _store(require_confirmation=True)
        rev = store.map_revision
        r = store.apply_update(MapUpdate(
            operation_id="note-1", kind=UpdateKind.NOTE_EVENT,
            event="box_disturbed"))
        self.assertTrue(r.accepted)
        self.assertEqual(store.map_revision, rev + 1)
        self.assertFalse(store.snapshot().occupancy_confirmed)
        # Gate off: nothing recorded, no bump.
        plain = _store(require_confirmation=False)
        rev0 = plain.map_revision
        r0 = plain.apply_update(MapUpdate(
            operation_id="note-1", kind=UpdateKind.NOTE_EVENT,
            event="box_disturbed"))
        self.assertTrue(r0.accepted)
        self.assertFalse(r0.details["recorded"])
        self.assertEqual(plain.map_revision, rev0)


if __name__ == "__main__":
    unittest.main()
