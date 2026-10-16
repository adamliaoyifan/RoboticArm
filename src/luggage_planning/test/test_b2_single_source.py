#!/usr/bin/env python3
"""B2 single-source migration, first wave (offline).

1. The selector must not override a B1 surface's own confirmation fields
   with its v1 latch (two verdict sources; the req-2b freshness race).
2. The route-admission fence (_same_admitted_map) must honor the
   publishing map's epoch + content digest when either side declares
   them, and the identity must flow: v2 contract -> surface dict ->
   PlanningSnapshot -> latest-snapshot factory input shape.
"""
import importlib.util
import os
import sys
import unittest
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", "..", ".."))
for _entry in (
        os.path.join(_ROOT, "src", "luggage_planning"),
        os.path.join(_ROOT, "src", "luggage_description"),
        os.path.join(_ROOT, "src", "luggage_perception"),
        _ROOT):
    if _entry not in sys.path:
        sys.path.append(_entry)

from luggage_planning.route_producer import (  # noqa: E402
    _same_admitted_map,
    surface_map_from_contract,
)
from luggage_perception.cargo_view_integration import (  # noqa: E402
    planning_snapshot_from_surface,
)


def _selector_module():
    path = os.path.join(_ROOT, "src", "luggage_planning", "scripts",
                        "trajectory_selector_node.py")
    spec = importlib.util.spec_from_file_location("selector_t", path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except ImportError as exc:
        raise unittest.SkipTest("selector node imports unavailable: %s" % exc)
    return module


def _surface_map(**extra):
    base = {
        "resolution": 0.1,
        "origin_local": [0.0, 0.0],
        "nx": 2,
        "ny": 2,
        "state": [["free", "occupied"], ["unknown", "free"]],
        "inner_size": [1.0, 1.0, 1.0],
        "geometry_hash": "gh",
        "map_revision": 4,
        "acquisition_stamp_ns": 7,
    }
    base.update(extra)
    return base


class TestSelectorVerdictSource(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.mod = _selector_module()

    def test_b1_declared_surface_is_not_overridden(self):
        request = SimpleNamespace(surface=SimpleNamespace(epoch="e9"))
        self.assertIsNone(self.mod.TrajectorySelectorNode._surface_validity(
            request, {"occupancy_confirmed": False,
                      "change_event": "box_placed"}))

    def test_pre_b1_surface_falls_back_to_latch(self):
        request = SimpleNamespace(surface=SimpleNamespace(epoch=""))
        validity = self.mod.TrajectorySelectorNode._surface_validity(
            request, {"occupancy_confirmed": False,
                      "change_event": "box_placed"})
        self.assertEqual(validity, {"occupancy_confirmed": False,
                                    "change_event": "box_placed"})

    def test_missing_surface_message_falls_back_to_latch(self):
        request = SimpleNamespace()
        validity = self.mod.TrajectorySelectorNode._surface_validity(
            request, {})
        self.assertEqual(validity, {"occupancy_confirmed": True,
                                    "change_event": ""})


class TestAdmissionFenceIdentity(unittest.TestCase):

    def _snap(self, **fields):
        base = {
            "acquisition_stamp_ns": 7, "map_revision": 4,
            "column_height_digest": "chd", "epoch": "e9",
            "content_digest": "d9",
        }
        base.update(fields)
        return SimpleNamespace(**base)

    def test_declared_identity_must_match(self):
        admitted = self._snap()
        self.assertTrue(_same_admitted_map(admitted, self._snap()))
        self.assertFalse(_same_admitted_map(
            admitted, self._snap(epoch="e10")))
        self.assertFalse(_same_admitted_map(
            admitted, self._snap(content_digest="d10")))

    def test_one_sided_declaration_fails_closed(self):
        admitted = self._snap()
        self.assertFalse(_same_admitted_map(
            admitted, self._snap(epoch="", content_digest="")))

    def test_legacy_fields_only_keeps_legacy_behavior(self):
        admitted = self._snap(epoch="", content_digest="")
        self.assertTrue(_same_admitted_map(
            admitted, self._snap(epoch="", content_digest="")))
        self.assertFalse(_same_admitted_map(
            admitted, self._snap(epoch="", content_digest="",
                                 map_revision=5)))


class TestIdentityFlow(unittest.TestCase):

    def test_contract_to_surface_to_snapshot(self):
        contract = {
            "frame_id": "container_link",
            "resolution_m": 0.1,
            "origin_xy_container": [0.0, 0.0],
            "state": [["free", "occupied"], ["unknown", "free"]],
            "stamp_ns": 7,
            "occupancy_confirmed": False,
            "change_event": "box_placed",
            "epoch": "e9",
            "content_digest": "d9",
        }
        request = {"map_revision": 4, "geometry_hash": "gh",
                   "inner_size": [1.0, 1.0, 1.0]}
        bridged = surface_map_from_contract(contract, request)
        self.assertEqual(bridged["epoch"], "e9")
        self.assertEqual(bridged["content_digest"], "d9")
        self.assertFalse(bridged["occupancy_confirmed"])

        snapshot = planning_snapshot_from_surface(
            bridged, scene_revision="s", request_id="r")
        self.assertEqual(snapshot.epoch, "e9")
        self.assertEqual(snapshot.content_digest, "d9")
        self.assertFalse(snapshot.occupancy_confirmed)
        self.assertEqual(snapshot.change_event, "box_placed")

        # The fence sees the declared identity.
        other = planning_snapshot_from_surface(
            _surface_map(**bridged), scene_revision="s", request_id="r2")
        self.assertTrue(_same_admitted_map(snapshot, other))
        rotated = planning_snapshot_from_surface(
            _surface_map(**dict(bridged, epoch="e10")),
            scene_revision="s", request_id="r3")
        self.assertFalse(_same_admitted_map(snapshot, rotated))

    def test_pre_b1_snapshot_leaves_identity_empty(self):
        snapshot = planning_snapshot_from_surface(
            _surface_map(), scene_revision="s", request_id="r")
        self.assertEqual(snapshot.epoch, "")
        self.assertEqual(snapshot.content_digest, "")


if __name__ == "__main__":
    unittest.main()
