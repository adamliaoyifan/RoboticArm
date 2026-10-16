#!/usr/bin/env python3
"""Server-side verify-anchor gate at the store level (Plan B / B1).

The Plan-A node-level gate moves into apply_update's evidence step; these
cases pin the same behaviors against the real store, including the
ledger's verify_anchor trail now carrying the operation id.
"""
import math
import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
    UpdateStatus,
)


def _store():
    return OccupancyMapStore(
        CargoVolumeMapper(inner_size=[1.0, 1.0, 1.0],
                          center_base=[0.0, 0.0, 0.5], yaw=0.0,
                          resolution=0.1),
        epoch="e1")


def _add(store, op, slot, verified, tol, size=(0.2, 0.2, 0.2)):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.ADD_BOX_V2,
        expected_snapshot=SnapshotIdentity("", store.map_revision, ""),
        geometry_hash=store.mapper.geometry_hash,
        box=(tuple(slot), tuple(size), 0.0),
        measured_source="verify_placed_box",
        verified_center=None if verified is None else tuple(verified),
        verify_tolerance_m=tol))


class TestStoreAnchorGate(unittest.TestCase):

    def setUp(self):
        self.store = _store()

    def _ledger_anchor(self):
        ledger = self.store.snapshot().ledger
        self.assertEqual(len(ledger), 1)
        return ledger[0][4]  # (center, size, yaw, source, anchor)

    def test_anchored_match_commits_with_trail(self):
        r = _add(self.store, "op-1", (0.1, 0.1, 0.15),
                 (0.1, 0.1, 0.19), 0.10)
        self.assertTrue(r.accepted, r.details)
        anchor = self._ledger_anchor()
        # Frozen trail record: (request_id, delta_m, tolerance_m).
        self.assertEqual(anchor[0], "op-1")
        self.assertAlmostEqual(anchor[1], 0.04, places=6)
        self.assertEqual(anchor[2], 0.10)
        # The committed projection reconstructs the dict shape.
        committed = self.store.committed_dict("container_link", {})
        exported = committed["boxes"][0]["verify_anchor"]
        self.assertEqual(exported["request_id"], "op-1")
        self.assertAlmostEqual(exported["delta_m"], 0.04, places=6)
        self.assertEqual(exported["tolerance_m"], 0.10)

    def test_witness02_phantom_is_rejected(self):
        """The exact witness_02 geometry: a slot 0.65 m from the verified
        pose must never reach the ledger (anchor gate fires before any
        mutation, so the out-of-container slot never rasterizes)."""
        r = _add(self.store, "op-1", (0.53, -0.51, 0.15),
                 (0.53, 0.14, 0.64), 0.10)
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_ANCHOR_MISMATCH")
        self.assertEqual(self.store.snapshot().placed_count, 0)

    def test_zero_voxel_box_is_a_rejection_not_an_exception(self):
        """A box outside the container fails the mapper's payload
        validation; the transaction returns REJECTED (fail closed),
        never raises through the adapter boundary."""
        r = _add(self.store, "op-1", (0.9, 0.9, 0.15), (0.9, 0.9, 0.15),
                 0.10)
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "INVALID_PAYLOAD")
        self.assertEqual(self.store.snapshot().placed_count, 0)

    def test_legacy_unanchored_commit_unchanged(self):
        r = _add(self.store, "op-1", (0.1, 0.1, 0.15), None, 0.0)
        self.assertTrue(r.accepted)
        self.assertIsNone(self._ledger_anchor())

    def test_nonfinite_verified_center_rejected(self):
        r = _add(self.store, "op-1", (0.53, 0.14, 0.15),
                 (float("nan"), 0.0, 0.0), 0.10)
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_NOT_FINITE")

    def test_missing_verified_center_rejected_when_tolerance_set(self):
        r = _add(self.store, "op-1", (0.53, 0.14, 0.15), None, 0.10)
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_MISSING")

    def test_nan_tolerance_rejected_not_silently_legacy(self):
        r = _add(self.store, "op-1", (0.53, -0.51, 0.15),
                 (0.53, 0.14, 0.64), float("nan"))
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_NOT_FINITE")
        self.assertEqual(self.store.snapshot().placed_count, 0)

    def test_infinite_tolerance_rejected(self):
        r = _add(self.store, "op-1", (0.53, -0.51, 0.15),
                 (0.53, 0.14, 0.64), float("inf"))
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_NOT_FINITE")

    def test_negative_tolerance_rejected(self):
        """tolerance=-0.1 used to fall through the `> 0` gate into the
        legacy un-anchored commit; only == 0 is compat."""
        r = _add(self.store, "op-1", (0.53, -0.51, 0.15),
                 (0.53, 0.14, 0.64), -0.1)
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_NOT_FINITE")
        self.assertEqual(self.store.snapshot().placed_count, 0)

    def test_tolerance_above_ceiling_rejected(self):
        from luggage_perception.occupancy_map_store import (
            VERIFY_ANCHOR_TOLERANCE_MAX_M)
        r = _add(self.store, "op-1", (0.53, -0.51, 0.15),
                 (0.53, 0.14, 0.64), VERIFY_ANCHOR_TOLERANCE_MAX_M + 0.01)
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_ANCHOR_MISMATCH")

    def test_unknown_measured_source_rejected(self):
        r = self.store.apply_update(MapUpdate(
            operation_id="op-1", kind=UpdateKind.ADD_BOX_V2,
            expected_snapshot=SnapshotIdentity("", self.store.map_revision, ""),
            geometry_hash=self.store.mapper.geometry_hash,
            box=((0.53, 0.14, 0.15), (0.2, 0.2, 0.2), 0.0),
            measured_source="planned",
            verified_center=(0.53, 0.14, 0.64), verify_tolerance_m=0.10))
        self.assertIs(r.status, UpdateStatus.REJECTED)
        self.assertEqual(r.reason, "EVIDENCE_MISSING")

    def test_witness_geometry_delta_is_0_65(self):
        """Guard the geometry the phantom case relies on."""
        from luggage_perception.cargo_volume_mapper import (
            verify_anchor_delta_m)
        delta = verify_anchor_delta_m(
            [0.53, -0.51, 0.639], [0.53, 0.14, 0.639])
        self.assertTrue(math.isclose(delta, 0.65, abs_tol=1e-6))


if __name__ == "__main__":
    unittest.main()
