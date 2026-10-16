#!/usr/bin/env python3
"""Single-snapshot projections (Plan B / B1): one snapshot → three
deterministic exports (v1 JSON dict, v2 record, committed ledger).

Pins the commit_revision fix (the downstream default-to-1 bug), the
byte-stability of the v1 JSON across rebuilds, and identity agreement
across all three projections.
"""
import json
import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
)


def _store(require_confirmation=True):
    return OccupancyMapStore(
        CargoVolumeMapper(inner_size=[1.0, 1.0, 1.0],
                          center_base=[0.0, 0.0, 0.5], yaw=0.0,
                          resolution=0.1),
        require_confirmation=require_confirmation, epoch="e1")


def _add(store, op="a1"):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.ADD_BOX_V2,
        expected_snapshot=SnapshotIdentity("e1", store.map_revision, ""),
        geometry_hash=store.mapper.geometry_hash,
        box=((0.1, 0.1, 0.15), (0.2, 0.2, 0.2), 0.0),
        measured_source="verify_placed_box",
        verified_center=(0.1, 0.1, 0.15), verify_tolerance_m=0.10))


class TestProjections(unittest.TestCase):

    def test_v1_dict_carries_commit_revision_equal_to_map_revision(self):
        """planning_snapshot_from_surface reads
        surface_map.get('commit_revision', 1) — the mapper never emitted
        it, so every downstream PlanningSnapshot carried commit_revision
        1. The projection now pins it to the map revision."""
        store = _store()
        _add(store)
        surface = store.v1_surface_dict()
        self.assertEqual(surface["commit_revision"],
                         surface["map_revision"])
        self.assertEqual(surface["commit_revision"], store.map_revision)
        self.assertEqual(surface["epoch"], "e1")
        self.assertEqual(surface["content_digest"],
                         store.identity.content_digest)
        self.assertIn("occupancy_confirmed", surface)
        self.assertIn("pending_changes", surface)
        self.assertIn("map_source", surface)

    def test_v2_record_carries_identity_and_confirmation(self):
        store = _store()
        _add(store)
        record = store.v2_record("scene-1", "map-rev-%d"
                                 % store.map_revision)
        self.assertEqual(record["map_revision"], store.map_revision)
        self.assertEqual(record["epoch"], "e1")
        self.assertEqual(record["content_digest"],
                         store.identity.content_digest)
        self.assertFalse(record["occupancy_confirmed"])
        self.assertEqual(record["state"][1][1], "unknown")  # sanity: grid

    def test_committed_dict_carries_identity(self):
        store = _store()
        _add(store)
        payload = store.committed_dict("container_link",
                                       {"translation": [1, 0, 0]})
        self.assertEqual(payload["map_revision"], store.map_revision)
        self.assertEqual(payload["epoch"], "e1")
        self.assertEqual(payload["content_digest"],
                         store.identity.content_digest)
        self.assertEqual(len(payload["boxes"]), 1)
        self.assertEqual(payload["boxes"][0]["verify_anchor"]
                         ["request_id"], "a1")

    def test_v1_json_byte_stable_across_rebuilds(self):
        store = _store()
        _add(store)
        first = json.dumps(store.v1_surface_dict(), sort_keys=True)
        # Force a rebuild path: same content re-observed (a duplicate add
        # with a new op id re-executes the duplicate-safe branch).
        _add(store, op="a2")
        second = json.dumps(store.v1_surface_dict(), sort_keys=True)
        self.assertEqual(first, second)

    def test_projections_agree_on_identity(self):
        store = _store()
        _add(store)
        v1 = store.v1_surface_dict()
        v2 = store.v2_record("scene-1", "r")
        committed = store.committed_dict("container_link", {})
        identity_fields = {
            "revision": (v1["map_revision"], v2["map_revision"],
                         committed["map_revision"]),
            "epoch": (v1["epoch"], v2["epoch"], committed["epoch"]),
            "digest": (v1["content_digest"], v2["content_digest"],
                       committed["content_digest"]),
        }
        for name, values in identity_fields.items():
            self.assertEqual(len(set(values)), 1,
                             "%s diverged across projections: %r"
                             % (name, values))
        # The grids the two surface projections carry describe the same
        # column states.
        self.assertEqual(
            [cell for row in v1["state"] for cell in row],
            [cell for row in v2["state"] for cell in row])


if __name__ == "__main__":
    unittest.main()
