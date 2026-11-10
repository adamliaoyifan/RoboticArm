#!/usr/bin/env python3
"""MapValidityLedger + store integration contract (OCC-GRID-CONSUMER-COMPAT G1).

Covers the reset-validity lifecycle testable without ROS: boot/reset
transitions and their latched event log, every BUILDING->VALID premise
individually (including the explicit integrity predicate — known_cells>0
alone is not integrity), reset_id idempotency riding the reset ledger,
the entry-epoch transaction race check, lossless epoch strings on the
projections, and the retired-handle boundary after an epoch rotation.
"""
import unittest

from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_lifecycle import (
    STATE_BUILDING,
    STATE_INVALID,
    STATE_VALID,
    MapValidityLedger,
)
from luggage_perception.occupancy_map_store import (
    REASON_EPOCH_MISMATCH_UNDER_RACE,
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
    UpdateStatus,
)

try:
    from luggage_perception.occupancy_map_store import (
        REASON_EPOCH_MISMATCH,
        REASON_OPERATION_ID_CONFLICT,
    )
except ImportError:  # pragma: no cover - re-exported names moved
    from luggage_perception.cargo_view_integration import (
        REASON_EPOCH_MISMATCH,
        REASON_OPERATION_ID_CONFLICT,
    )


class _Clock(object):
    def __init__(self, now=1_000_000_000_000_000_000):
        self.now = now

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


class _LifecycleCase(unittest.TestCase):

    def setUp(self):
        self.clock = _Clock()
        self.store = OccupancyMapStore(
            _mapper(), clock_ns=self.clock, epoch="e1")

    @property
    def _gh(self):
        return self.store.mapper.geometry_hash

    def _integrate(self, op, points=((0.0, 0.0, 0.5), (0.1, 0.0, 0.5)),
                   stamp_ns=None, stamp_key=None, settled=True,
                   store=None):
        target = store if store is not None else self.store
        stamp = (self.clock.now if stamp_ns is None else int(stamp_ns))
        rev = target.map_revision
        return MapUpdate(
            operation_id=op, kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=SnapshotIdentity("", rev, ""),
            geometry_hash=target.mapper.geometry_hash,
            settled=settled, schema_version=1,
            points=[list(p) for p in points], origin=(2.0, 0.0, 1.0),
            stamp_key=stamp_key or (int(stamp // 1_000_000_000), 0),
            acquisition_stamp_ns=stamp)

    def _reset(self, op="r", reason="test_reset", expected_epoch=None,
               event=None):
        return MapUpdate(
            operation_id=op, kind=UpdateKind.RESET,
            expected_snapshot=(SnapshotIdentity(expected_epoch, 0, "")
                               if expected_epoch else None),
            geometry_hash="", event=event if event is not None else reason)


class TestBootAndReset(_LifecycleCase):

    def test_boot_is_building_generation_one(self):
        record, _identity, events = self.store.validity_bundle()
        self.assertEqual(record.state, STATE_BUILDING)
        self.assertEqual(record.generation, 1)
        self.assertEqual(record.epoch, "e1")
        self.assertEqual(record.reason, "boot")
        self.assertEqual([e.state for e in events], [STATE_BUILDING])

    def test_reset_emits_two_events_and_rotates_epoch(self):
        old_epoch = self.store.epoch
        result = self.store.apply_update(self._reset())
        self.assertTrue(result.accepted)
        record, identity, events = self.store.validity_bundle()
        self.assertNotEqual(record.epoch, old_epoch)
        self.assertEqual(identity.epoch, record.epoch)
        self.assertEqual(record.state, STATE_BUILDING)
        # boot(1) + INVALID(2) + BUILDING(3): two generations per reset,
        # matching the observer's commit_reset + begin_building pair.
        self.assertEqual(record.generation, 3)
        self.assertEqual([e.state for e in events],
                         [STATE_BUILDING, STATE_INVALID, STATE_BUILDING])
        self.assertEqual(events[-2].reset_id, "r")
        self.assertEqual(events[-2].reason, "test_reset")
        self.assertEqual(events[-1].reason, "store_ready")

    def test_reset_id_idempotent_same_content(self):
        first = self.store.apply_update(self._reset(op="reset-v2:r1"))
        epoch_after = self.store.epoch
        replay = self.store.apply_update(self._reset(op="reset-v2:r1"))
        self.assertEqual(replay.status, UpdateStatus.ALREADY_PROCESSED)
        self.assertEqual(self.store.epoch, epoch_after)
        # No second rotation: the two-event pattern appears exactly once.
        _, _, events = self.store.validity_bundle()
        self.assertEqual([e.state for e in events].count(STATE_INVALID), 1)

    def test_reset_id_conflict_same_id_different_reason(self):
        self.store.apply_update(self._reset(op="reset-v2:r1",
                                            reason="one"))
        conflict = self.store.apply_update(self._reset(
            op="reset-v2:r1", reason="two"))
        self.assertEqual(conflict.status, UpdateStatus.REJECTED)
        self.assertEqual(conflict.reason, REASON_OPERATION_ID_CONFLICT)

    def test_reset_stale_expected_epoch_refused(self):
        self.store.apply_update(self._reset(op="r1"))
        stale = self.store.apply_update(self._reset(
            op="r2", expected_epoch="deadbeef"))
        self.assertEqual(stale.status, UpdateStatus.REJECTED)
        self.assertEqual(stale.reason, REASON_EPOCH_MISMATCH)


class TestValidPromotion(_LifecycleCase):

    def _prime_observation(self):
        result = self.store.apply_update(self._integrate("i1"))
        self.assertTrue(result.accepted)
        return result

    def test_accepted_integrate_promotes_to_valid(self):
        self._prime_observation()
        record, _identity, events = self.store.validity_bundle()
        self.assertEqual(record.state, STATE_VALID)
        self.assertEqual(record.reason, "observed")
        self.assertEqual(record.generation, 2)
        self.assertEqual(events[-1].state, STATE_VALID)
        self.assertEqual(record.geometry_hash, self._gh)
        self.assertGreater(record.last_integrated_stamp_ns, 0)

    def test_no_observation_never_valid(self):
        # A transaction that is not an observation (box fact) leaves the
        # BUILDING premise unmet: floor priors and empty receipts can
        # never promote.
        self.store.apply_update(MapUpdate(
            operation_id="b1", kind=UpdateKind.ADD_BOX_V1,
            box=((0.0, 0.0, 0.5), (0.2, 0.2, 0.2), 0.0)))
        record, _, _ = self.store.validity_bundle()
        self.assertEqual(record.state, STATE_BUILDING)

    def test_stale_observation_stays_building(self):
        result = self.store.apply_update(self._integrate(
            "i1", stamp_ns=self.clock.now - 2_000_000_000))
        self.assertTrue(result.accepted)
        record, _, _ = self.store.validity_bundle()
        self.assertEqual(record.state, STATE_BUILDING)

    def test_integrity_failure_stays_building(self):
        original = self.store._snapshot_integrity_ok
        self.store._snapshot_integrity_ok = lambda: False
        try:
            result = self.store.apply_update(self._integrate("i1"))
            self.assertTrue(result.accepted)
            record, _, _ = self.store.validity_bundle()
            self.assertEqual(record.state, STATE_BUILDING)
        finally:
            self.store._snapshot_integrity_ok = original
        # Same content under a new id once integrity passes: a later
        # observation promotes (premises are re-evaluated per txn).
        result = self.store.apply_update(self._integrate(
            "i2", stamp_ns=self.clock.now + 1_000_000,
            stamp_key=(999, 0)))
        self.assertTrue(result.accepted)
        record, _, _ = self.store.validity_bundle()
        self.assertEqual(record.state, STATE_VALID)

    def test_reset_after_valid_rebuilds_from_scratch(self):
        self._prime_observation()
        self.store.apply_update(self._reset(op="r1"))
        record, _, _ = self.store.validity_bundle()
        self.assertEqual(record.state, STATE_BUILDING)
        self.assertEqual(record.last_integrated_stamp_ns, 0)
        # The pre-reset observation does not carry over: a NEW epoch's
        # observation is required (I1 recovery premise).
        result = self.store.apply_update(self._integrate("i2"))
        self.assertTrue(result.accepted)
        record, _, _ = self.store.validity_bundle()
        self.assertEqual(record.state, STATE_VALID)

    def test_valid_pinned_identity_matches_snapshot(self):
        self._prime_observation()
        record, identity, _ = self.store.validity_bundle()
        self.assertEqual(record.epoch, identity.epoch)
        self.assertEqual(record.map_revision, identity.revision)


class TestEntryEpochRace(_LifecycleCase):

    def test_queued_update_rejected_under_race(self):
        # The authoritative I3 boundary: a command captured against an
        # epoch that a reset replaced before the transaction ran.
        update = self._integrate("i1")
        result = self.store._apply_locked(update, epoch_at_entry="old")
        self.assertEqual(result.status, UpdateStatus.REJECTED)
        self.assertEqual(result.reason, REASON_EPOCH_MISMATCH_UNDER_RACE)

    def test_reset_itself_exempt_from_race_check(self):
        update = self._reset(op="r1")
        result = self.store._apply_locked(update, epoch_at_entry="old")
        self.assertTrue(result.accepted)

    def test_matching_entry_epoch_passes(self):
        result = self.store._apply_locked(
            self._integrate("i1"), epoch_at_entry=self.store.epoch)
        self.assertTrue(result.accepted)


class TestLosslessIdentity(_LifecycleCase):

    def _uuid_store(self):
        import uuid
        return OccupancyMapStore(
            _mapper(), clock_ns=self.clock,
            epoch=uuid.uuid4().hex)

    def test_v1_projection_carries_uuid_epoch_verbatim_and_generation(self):
        store = self._uuid_store()
        store.apply_update(self._integrate("i1", store=store))
        projected = store.v1_surface_dict()
        epoch = projected["epoch"]
        # The UUID string is carried verbatim — no truncation, hashing or
        # renumbering anywhere on the wire path.
        self.assertIsInstance(epoch, str)
        self.assertEqual(epoch, store.epoch)
        self.assertRegex(epoch, r"^[0-9a-f]{32}$")
        self.assertGreaterEqual(int(projected["validity_generation"]), 2)

    def test_v1_projection_epoch_all_digit_uuid_untouched(self):
        # A pathological-but-legal all-digit UUID epoch with a leading
        # zero must survive verbatim (the rejected numeric normalization
        # would corrupt identities like this).
        epoch = "012345678901234567890123456789ab"
        store = OccupancyMapStore(_mapper(), clock_ns=self.clock,
                                  epoch=epoch)
        store.apply_update(self._integrate("i1", store=store))
        self.assertEqual(store.v1_surface_dict()["epoch"], epoch)
        self.assertEqual(store.lifecycle.record.epoch, epoch)

    def test_v2_record_carries_generation(self):
        self.store.apply_update(self._integrate("i1"))
        record = self.store.v2_record("scene-1", "req-1")
        self.assertEqual(record["epoch"], self.store.epoch)
        self.assertGreaterEqual(int(record["validity_generation"]), 2)

    def test_ledger_record_json_schema(self):
        self.store.apply_update(self._integrate("i1"))
        record = self.store.validity_bundle()[0]
        payload = record.as_dict()
        for key in ("state", "epoch", "generation", "reason", "reset_id",
                    "transition_mono_ns", "transition_stamp_ns",
                    "map_revision", "observed_revision",
                    "last_integrated_stamp_ns", "geometry_hash",
                    "config_digest", "clock_domain", "backend"):
            self.assertIn(key, payload)
        self.assertEqual(payload["backend"], "grid_g")
        self.assertEqual(payload["clock_domain"], "ros")


class TestRetiredHandles(_LifecycleCase):

    def test_old_handle_cannot_resolve_after_rotation(self):
        self.store.apply_update(self._integrate("i1"))
        handle = self.store.acquire_query_snapshot()
        try:
            self.store.apply_update(self._reset(op="r1"))
            # Already-resolved data stays immutable for whoever holds it
            # (page semantics); a NEW resolve of the retired generation
            # must fail closed (response-delivery boundary).
            with self.assertRaises(QueryError):
                handle._data()
        finally:
            handle.close()

    def test_resume_old_token_after_rotation_expired(self):
        self.store.apply_update(self._integrate("i1"))
        handle = self.store.acquire_query_snapshot()
        token, identity = handle._token, handle.identity
        handle.close()
        self.store.apply_update(self._reset(op="r1"))
        with self.assertRaises(QueryError):
            self.store.acquire_query_snapshot(
                token=token, expected_identity=identity)


class TestLedgerUnit(unittest.TestCase):
    """Direct ledger unit checks that need no store."""

    def test_events_cap_bounds_latched_log(self):
        clock = _Clock()
        ledger = MapValidityLedger(clock, events_cap=3)
        ledger.on_boot("e1")
        ledger.note_reset("e1", "e2", "r1", "explicit")
        ledger.note_reset("e2", "e3", "r2", "explicit")
        states = [e.state for e in ledger.events()]
        self.assertEqual(len(states), 3)  # capped, oldest dropped
        self.assertEqual(ledger.record.epoch, "e3")

    def test_observation_from_foreign_epoch_ignored(self):
        clock = _Clock()
        ledger = MapValidityLedger(clock)
        ledger.on_boot("e1")
        foreign = SnapshotIdentity("other-epoch", 5, "digest")
        ledger.note_observation(foreign, clock.now)
        # Guard: premises untouched by a foreign-epoch identity.
        ledger.evaluate_valid(
            foreign, "gh", clock.now, lambda: True)
        self.assertEqual(ledger.record.state, STATE_BUILDING)


if __name__ == "__main__":
    unittest.main()


class TestWireSchemaAlignment(unittest.TestCase):
    """G's validity JSON must be a field-superset of the observer's wire.

    The common consumer parses BOTH streams with one code path; the
    observer's validity_json keys (observer_node.cpp) must all resolve,
    with identical JSON types except epoch (observer uint64 vs G UUID
    string — normalized losslessly on the consumer side per backend).
    """

    OBSERVER_KEYS = {
        "state": str, "epoch": str, "generation": int, "reason": str,
        "reset_id": str, "transition_mono_ns": int,
        "map_revision": int, "observed_revision": int,
        "last_integrated_stamp_ns": int, "geometry_hash": str,
        "config_digest": str, "clock_domain": str,
    }

    def test_record_covers_observer_wire_schema(self):
        import uuid
        clock = _Clock()
        store = OccupancyMapStore(
            _mapper(), clock_ns=clock, epoch=uuid.uuid4().hex)
        payload = store.validity_bundle()[0].as_dict()
        for key, kind in self.OBSERVER_KEYS.items():
            self.assertIn(key, payload, key)
            self.assertIsInstance(payload[key], kind, key)
        # G-only additions (backend binding + ROS-domain twin stamp).
        self.assertEqual(payload["backend"], "grid_g")
        self.assertIsInstance(payload["transition_stamp_ns"], int)
        self.assertIsInstance(store.lifecycle.events_json(), str)
