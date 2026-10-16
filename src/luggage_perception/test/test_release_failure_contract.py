#!/usr/bin/env python3
"""Release-failure contract at the map (offline behavior tests).

释放事件发生就通知 occ 进入 pending；锚定受琴提交即确认（锚=实测占用
证据）；无锚提交保持 legacy box_placed pending；观测可确认 box_released。
"""
import importlib.util
import os
import unittest
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
_NODE = os.path.join(_HERE, "..", "scripts", "cargo_volume_mapper_node.py")

import sys  # noqa: E402
for _entry in (
        os.path.join(_ROOT, "src", "luggage_planning"),
        os.path.join(_ROOT, "src", "luggage_description"),
        os.path.join(_ROOT, "src", "luggage_perception"), _ROOT):
    if _entry not in sys.path:
        sys.path.append(_entry)

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper  # noqa: E402
from luggage_perception.occupancy_map_store import (  # noqa: E402
    MapUpdate,
    OccupancyMapStore,
    SnapshotIdentity,
    UpdateKind,
)


class _Clock(object):
    now = 1_000_000_000_000000000

    def __call__(self):
        return self.now


def _store():
    return OccupancyMapStore(
        CargoVolumeMapper(inner_size=[1.0, 1.0, 1.0],
                          center_base=[0.0, 0.0, 0.5], yaw=0.0,
                          resolution=0.1),
        require_confirmation=True, clock_ns=_Clock(), epoch="e1")


def _release_fact(store, center=(0.1, 0.1, 0.15), op="rel-1"):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.NOTE_EVENT, event="box_released",
        box=((center[0], center[1], center[2]), (0.4, 0.4, 0.2), 0.0)))


def _anchored_add(store, center=(0.1, 0.1, 0.15), tol=0.10, op="a1"):
    return store.apply_update(MapUpdate(
        operation_id=op, kind=UpdateKind.ADD_BOX_V2,
        expected_snapshot=SnapshotIdentity("e1", store.map_revision, ""),
        geometry_hash=store.mapper.geometry_hash,
        box=((center[0], center[1], center[2]), (0.4, 0.4, 0.2), 0.0),
        measured_source="verify_placed_box",
        verified_center=(center[0], center[1], center[2] + tol * 0.4),
        verify_tolerance_m=tol))


def _observe_upper_half(store, tag, stamp_ns):
    return store.apply_update(MapUpdate(
        operation_id=tag, kind=UpdateKind.INTEGRATE_VIEW,
        geometry_hash=store.mapper.geometry_hash,
        expected_snapshot=SnapshotIdentity("", store.map_revision, ""),
        settled=True, schema_version=1,
        points=[[0.1, 0.1, 0.20], [0.25, 0.1, 0.20], [-0.05, 0.1, 0.20]],
        origin=(2.0, 0.0, 1.0), stamp_key=(int(stamp_ns // 10 ** 9), 0),
        acquisition_stamp_ns=stamp_ns))


class TestReleaseFailureContract(unittest.TestCase):

    def test_release_fact_pends_immediately_with_footprint(self):
        """Item 1: the map learns the world changed at vacuum-off, before
        any acceptance decision — pending carries the commanded footprint
        (confirmable) and blocks the confirmation headline."""
        store = _store()
        self.assertTrue(store.snapshot().occupancy_confirmed)
        result = _release_fact(store)
        self.assertTrue(result.accepted, result.details)
        snap = store.snapshot()
        self.assertFalse(snap.occupancy_confirmed)
        self.assertEqual(snap.pending_changes,
                         (("box_released", 1),))
        self.assertEqual(snap.pending_details[0],
                         (1, "box_released", _Clock.now, True))

    def test_anchored_commit_confirms_and_clears_release_fact(self):
        """Anchor = measured-occupancy evidence: an accepted anchored
        commit clears the matching box_released fact without adding a
        box_placed pending (no camera view needed on the success path)."""
        store = _store()
        _release_fact(store)
        result = _anchored_add(store)
        self.assertTrue(result.accepted, result.details)
        snap = store.snapshot()
        self.assertTrue(snap.occupancy_confirmed)
        self.assertEqual(snap.pending_changes, ())
        self.assertEqual(snap.placed_count, 1)

    def test_unanchored_commit_keeps_legacy_pending(self):
        """Legacy (un-anchored) commits keep box_placed pending; the
        release fact stays too — observation resolves both."""
        store = _store()
        _release_fact(store)
        result = store.apply_update(MapUpdate(
            operation_id="a-legacy", kind=UpdateKind.ADD_BOX_V1,
            box=((0.1, 0.1, 0.15), (0.4, 0.4, 0.2), 0.0)))
        self.assertTrue(result.accepted)
        snap = store.snapshot()
        self.assertFalse(snap.occupancy_confirmed)
        self.assertEqual(snap.pending_changes,
                         (("box_released", 1), ("box_placed", 2)))
        # A covering observation confirms both footprint-carrying facts.
        obs = _observe_upper_half(store, "i1", 2_000_000_000_000000000)
        self.assertTrue(obs.accepted, obs.details)
        self.assertTrue(store.snapshot().occupancy_confirmed)

    def test_verify_failed_box_stays_pending_and_blocks_next_box(self):
        """The acceptance-failure path: no commit, the release fact
        remains — the confirmation headline the next-box gate reads
        stays False (the landed obstacle is not free space)."""
        store = _store()
        _release_fact(store)
        # (no commit: verify failed downstream)
        self.assertFalse(store.snapshot().occupancy_confirmed)
        self.assertEqual(store.snapshot().pending_changes,
                         (("box_released", 1),))

    def test_nonmatching_anchor_does_not_clear_foreign_fact(self):
        """The anchor clears only the footprint it matches."""
        store = _store()
        _release_fact(store, center=(0.1, 0.1, 0.15), op="rel-a")
        result = _anchored_add(store, center=(0.5, -0.5, 0.15), op="a-b")
        self.assertTrue(result.accepted, result.details)
        snap = store.snapshot()
        self.assertFalse(snap.occupancy_confirmed)
        self.assertIn(("box_released", 1), snap.pending_changes)


def _load_node():
    try:
        import rclpy  # noqa: F401
        import luggage_msgs.srv  # noqa: F401
    except ImportError as exc:
        raise unittest.SkipTest("node deps unavailable: %s" % exc)
    spec = importlib.util.spec_from_file_location("mapper_node_rfc", _NODE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestNoteEventAdapter(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mod = _load_node()

    def _node(self, require=True):
        node = SimpleNamespace()
        node._store = OccupancyMapStore(
            CargoVolumeMapper(inner_size=[1.0, 1.0, 1.0],
                              center_base=[0.0, 0.0, 0.5], yaw=0.0,
                              resolution=0.1),
            require_confirmation=require, clock_ns=_Clock(), epoch="e1")
        node._node_clock_ns = lambda: _Clock.now
        return node

    def _request(self, event="box_released", center=(0.1, 0.1, 0.15),
                 size=(0.4, 0.4, 0.2), yaw=0.0):
        return SimpleNamespace(
            event=event,
            box_center_container=list(center),
            box_size_wdh=list(size),
            box_yaw=float(yaw),
            source="place_smoke_driver")

    def _response(self):
        return SimpleNamespace(status="", reason="", epoch="",
                               content_digest="", occupancy_confirmed=None)

    def test_release_notification_maps_through_the_transaction(self):
        node = self._node()
        response = self._response()
        self.mod.CargoVolumeMapperNode._on_note_event(
            node, self._request(), response)
        self.assertEqual(response.status, "accepted")
        self.assertTrue(response.epoch)
        self.assertFalse(response.occupancy_confirmed)
        self.assertEqual(node._store.snapshot().pending_changes,
                         (("box_released", 1),))

    def test_anchored_commit_after_notification_reconfirms(self):
        node = self._node()
        self.mod.CargoVolumeMapperNode._on_note_event(
            node, self._request(), self._response())
        result = node._store.apply_update(MapUpdate(
            operation_id="a1", kind=UpdateKind.ADD_BOX_V2,
            expected_snapshot=SnapshotIdentity(
                "e1", node._store.map_revision, ""),
            geometry_hash=node._store.mapper.geometry_hash,
            box=((0.1, 0.1, 0.15), (0.4, 0.4, 0.2), 0.0),
            measured_source="verify_placed_box",
            verified_center=(0.1, 0.1, 0.19), verify_tolerance_m=0.10))
        self.assertTrue(result.accepted)
        self.assertTrue(node._store.snapshot().occupancy_confirmed)

    def test_nonfinite_footprint_records_event_without_footprint(self):
        node = self._node()
        response = self._response()
        request = self._request(center=(float("nan"), 0.0, 0.0))
        self.mod.CargoVolumeMapperNode._on_note_event(
            node, request, response)
        self.assertEqual(response.status, "accepted")
        # Non-confirmable (no footprint) but recorded: the map still
        # stops claiming confirmed-unchanged.
        self.assertFalse(response.occupancy_confirmed)


if __name__ == "__main__":
    unittest.main()
