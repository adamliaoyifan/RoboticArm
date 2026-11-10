"""Sim-object contract regression (G3 wave 3): the pipeline owns assets.

Drives the real pipeline validators over the committed models directory:
every REGISTERED tier (catalog + inch) must have a model dir whose
collision STL and visual DAE vertex extents equal the registered size,
and ``sized_suitcases.json`` must equal the disk state exactly (the
manifest also carries the placed-body primitive_collision_size and the
asset hashes). Registering a new tier and running the pipeline makes
this test pass; hand-committed model dirs make it fail.
"""

import json
import os
import unittest

from luggage_description.suitcase_visual import (
    SIZED_MANIFEST_NAME,
    check_sized_assets,
    iter_size_tiers,
    sized_asset_records,
    sized_suitcases_manifest_path,
)

MODELS_ROOT = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "luggage_gazebo", "models"))


class TestSizedAssetConsistency(unittest.TestCase):
    def test_every_registered_tier_passes_the_pipeline_check(self):
        failures = check_sized_assets(MODELS_ROOT)
        self.assertEqual(failures, [])

    def test_manifest_matches_disk_state(self):
        on_disk = json.load(open(sized_suitcases_manifest_path(MODELS_ROOT)))
        self.assertEqual(on_disk, sized_asset_records(MODELS_ROOT))

    def test_manifest_covers_every_registered_tier_with_primitive(self):
        manifest = json.load(open(os.path.join(
            MODELS_ROOT, SIZED_MANIFEST_NAME)))
        for tier, size, _mass, _catalog in iter_size_tiers():
            for visual in ("suitcase_loafbrr", "suitcase_vintage"):
                entry = manifest.get("%s_%s" % (visual, tier))
                self.assertIsNotNone(
                    entry, "manifest missing %s_%s" % (visual, tier))
                self.assertEqual(
                    entry["primitive_collision_size"], list(size))
                self.assertEqual(entry["size"], list(size))
                self.assertTrue(entry.get("stl_sha256"))


if __name__ == "__main__":
    unittest.main()
