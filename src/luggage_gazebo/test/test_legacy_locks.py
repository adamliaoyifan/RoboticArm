#!/usr/bin/env python3
"""[ST09-I S5.7-5] legacy surfaces are locked to their lineages.

- the fixed floor-center slot is a TEST FIXTURE (pack-eval lineage); on
  the typed production chain it is a named refusal that can never mask
  a placement failure, and the evidence names slot_source;
- the world-X zone guess survives only for non-frame-runtime callers
  (the typed chain refuses SCENE_IDENTITY_MISSING instead);
- place_only_eval_driver has no import edge anywhere in the package.
"""

import os
import re
import sys
import unittest
from types import SimpleNamespace

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
_ROOT = os.path.normpath(os.path.join(_SCRIPTS, "..", ".."))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from place_smoke_driver import PlaceSmokeDriver  # noqa: E402


class _FixtureDriver(object):
    """Only what _place_slot_for/_container_zone touch."""

    def __init__(self, typed=True, scene=None):
        self._typed = SimpleNamespace() if typed else None
        self._scene_config = scene

    _place_slot_for = PlaceSmokeDriver._place_slot_for
    _container_zone = PlaceSmokeDriver._container_zone

    def suction_xyz(self):
        return (1.9, 0.0, 1.2), ""

    def _aperture_corridor_depth(self):
        return 0.41, "test"


class TestFixedSlotLock(unittest.TestCase):

    def test_typed_chain_routes_to_certified_place_slot(self):
        trial = SimpleNamespace(fail_code="", extras={})
        driver = _FixtureDriver(typed=True)
        sentinel = (object(), {"slot_source": "certified_route"})
        driver._typed_place_slot = lambda _pick, _trial: sentinel
        slot, meta = driver._place_slot_for(
            SimpleNamespace(height_valid=True), None, trial)
        self.assertIs(slot, sentinel[0])
        self.assertEqual(meta["slot_source"], "certified_route")

    def test_fixed_slot_stays_a_named_test_fixture_for_legacy(self):
        # Pack-eval lineage (no typed chain): the fixture still works
        # and the evidence NAMES it as a fixture.
        trial = SimpleNamespace(fail_code="", extras={})
        driver = _FixtureDriver(typed=False, scene={"container": {
            "outer": {"length": 0.6, "width": 0.6},
            "inner": {"length": 0.52, "width": 0.52},
        }})
        driver._fixed_slot = lambda box: (SimpleNamespace(), {"source":
                                                "fixed_floor_center"})
        slot, meta = driver._place_slot_for(
            SimpleNamespace(height_valid=True), None, trial)
        self.assertIsNotNone(slot)
        self.assertEqual(meta["slot_source"], "test_fixture")
        self.assertEqual(trial.fail_code, "")


class TestWorldXFallbackLock(unittest.TestCase):

    def test_world_x_guess_survives_only_without_the_typed_chain(self):
        typed = _FixtureDriver(typed=True, scene=None)
        zone, reason = typed._container_zone()
        self.assertEqual(zone, "unknown")
        self.assertEqual(reason, "SCENE_IDENTITY_MISSING")
        legacy = _FixtureDriver(typed=False, scene=None)
        zone, reason = legacy._container_zone()
        self.assertIn(zone, ("opening_corridor", "outside_container"))
        self.assertEqual(reason, "")


class TestPlaceOnlyNoImportEdge(unittest.TestCase):

    def test_no_module_imports_place_only_eval_driver(self):
        package_root = os.path.join(_ROOT, "src", "luggage_gazebo")
        pattern = re.compile(
            r"(from|import)\s+place_only_eval_driver")
        offenders = []
        for base, _dirs, files in os.walk(package_root):
            for name in files:
                if not name.endswith(".py"):
                    continue
                path = os.path.join(base, name)
                if name == "place_only_eval_driver.py":
                    continue
                with open(path, encoding="utf-8",
                          errors="replace") as handle:
                    if pattern.search(handle.read()):
                        offenders.append(
                            os.path.relpath(path, package_root))
        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
