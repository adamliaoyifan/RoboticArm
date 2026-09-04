#!/usr/bin/env python3
"""Unit tests for continuous box size and mass sampling.

Continuous sizes are what force perception to measure a box rather than
recognise one of three shapes, so the envelope and the mass model both have to
hold under sampling, not just on the reference sizes.
"""

import os
import random
import unittest

import yaml

from luggage_description import box_catalog_utils as UTILS

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

CONFIG = os.path.join(PKG_ROOT, "config", "box_catalog.yaml.example")
CATALOG = yaml.safe_load(open(CONFIG))

REFERENCE_SIZES = {
    "carryon": (0.55, 0.40, 0.25),
    "standard": (0.70, 0.45, 0.28),
    "large": (0.80, 0.50, 0.32),
}


class TestSizeRange(unittest.TestCase):
    def test_envelope_spans_the_reference_entries(self):
        ranges = UTILS.box_size_range(CATALOG)
        for size in REFERENCE_SIZES.values():
            for axis, value in enumerate(size):
                low, high = ranges[axis]
                self.assertGreaterEqual(value, low - 1e-9)
                self.assertLessEqual(value, high + 1e-9)

    def test_samples_stay_inside_the_envelope(self):
        ranges = UTILS.box_size_range(CATALOG)
        rng = random.Random(7)
        for _ in range(200):
            size = UTILS.sample_box_size(ranges, rng)
            for axis, value in enumerate(size):
                low, high = ranges[axis]
                self.assertGreaterEqual(value, low - 1e-9)
                self.assertLessEqual(value, high + 1e-9)

    def test_sampling_is_deterministic_for_a_seed(self):
        ranges = UTILS.box_size_range(CATALOG)
        first = [UTILS.sample_box_size(ranges, random.Random(3))
                 for _ in range(1)]
        second = [UTILS.sample_box_size(ranges, random.Random(3))
                  for _ in range(1)]
        self.assertEqual(first, second)

    def test_samples_are_rarely_catalog_sizes(self):
        """Otherwise perception could still get by on recognition."""
        ranges = UTILS.box_size_range(CATALOG)
        rng = random.Random(11)
        catalog = set(REFERENCE_SIZES.values())
        hits = sum(
            1 for _ in range(200)
            if tuple(UTILS.sample_box_size(ranges, rng)) in catalog)
        self.assertEqual(hits, 0)

    def test_falls_back_to_entry_envelope_without_size_range(self):
        config = dict(CATALOG)
        config.pop("size_range")
        ranges = UTILS.box_size_range(config)
        self.assertAlmostEqual(ranges[0][0], 0.55)
        self.assertAlmostEqual(ranges[0][1], 0.80)


class TestMassModel(unittest.TestCase):
    def test_reference_sizes_land_near_catalog_masses(self):
        model = UTILS.box_mass_model(CATALOG)
        masses = {
            name: UTILS.nominal_box_mass(size, model)
            for name, size in REFERENCE_SIZES.items()
        }
        self.assertAlmostEqual(masses["carryon"], 8.0, delta=0.3)
        self.assertAlmostEqual(masses["standard"], 15.0, delta=1.0)
        self.assertAlmostEqual(masses["large"], 23.0, delta=0.3)

    def test_mass_increases_with_volume(self):
        model = UTILS.box_mass_model(CATALOG)
        small = UTILS.nominal_box_mass(REFERENCE_SIZES["carryon"], model)
        large = UTILS.nominal_box_mass(REFERENCE_SIZES["large"], model)
        self.assertGreater(large, small)

    def test_sampled_mass_never_exceeds_rated_payload(self):
        """The vacuum model is only validated up to max_kg."""
        model = UTILS.box_mass_model(CATALOG)
        ranges = UTILS.box_size_range(CATALOG)
        rng = random.Random(5)
        for _ in range(500):
            size = UTILS.sample_box_size(ranges, rng)
            mass = UTILS.sample_box_mass(size, model, rng)
            self.assertLessEqual(mass, model["max_kg"] + 1e-9)
            self.assertGreaterEqual(mass, model["min_kg"] - 1e-9)

    def test_jitter_actually_varies_mass(self):
        model = UTILS.box_mass_model(CATALOG)
        rng = random.Random(9)
        size = REFERENCE_SIZES["standard"]
        masses = {round(UTILS.sample_box_mass(size, model, rng), 4)
                  for _ in range(50)}
        self.assertGreater(len(masses), 10)

    def test_zero_jitter_reproduces_nominal(self):
        model = dict(UTILS.box_mass_model(CATALOG))
        model["density_jitter"] = 0.0
        size = REFERENCE_SIZES["large"]
        self.assertAlmostEqual(
            UTILS.sample_box_mass(size, model, random.Random(1)),
            UTILS.nominal_box_mass(size, model))


class TestCatalogSpawnSampling(unittest.TestCase):
    def test_catalog_mode_only_hits_three_sizes(self):
        entries = UTILS.box_catalog_entries(CATALOG)
        catalog_sizes = {tuple(e["size"]) for e in entries}
        self.assertEqual(catalog_sizes, set(REFERENCE_SIZES.values()))
        rng = random.Random(13)
        seen = set()
        for _ in range(100):
            _entry, size, _mass, generated = UTILS.sample_pickup_box(
                entries, rng, size_mode="catalog")
            self.assertFalse(generated)
            seen.add(tuple(size))
        self.assertEqual(seen, catalog_sizes)

    def test_catalog_mode_is_default(self):
        entries = UTILS.box_catalog_entries(CATALOG)
        _entry, size, _mass, generated = UTILS.sample_pickup_box(
            entries, random.Random(1))
        self.assertFalse(generated)
        self.assertIn(tuple(size), set(REFERENCE_SIZES.values()))


if __name__ == "__main__":
    unittest.main()
