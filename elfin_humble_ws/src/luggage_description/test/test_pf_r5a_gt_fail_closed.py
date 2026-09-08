#!/usr/bin/env python3
"""PF-R5A: fail-closed mesh-observable GT and deterministic identity.

docs/plans/platform_free_height_closure.md PF-R5A. The mesh-observable
GT reference must never substitute catalog dimensions: missing,
truncated, malformed/non-binary, or unknown-tier assets raise
MeshReferenceError, and the six valid sized assets resolve to pinned
observable values with stable SHA-256 and version identity.
"""

import os
import struct
import tempfile
import unittest

from luggage_description.suitcase_visual import (
    OBSERVABLE_REFERENCE_VERSION,
    MeshReferenceError,
    resolve_observable_reference,
    stl_sha256,
)

_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
GAZEBO_MODELS = os.path.normpath(os.path.join(
    _PKG_ROOT, "..", "luggage_gazebo", "models"))

SIZED = [
    ("suitcase_loafbrr", "small", (0.55, 0.40, 0.25)),
    ("suitcase_loafbrr", "medium", (0.70, 0.45, 0.28)),
    ("suitcase_loafbrr", "large", (0.80, 0.50, 0.32)),
    ("suitcase_vintage", "small", (0.55, 0.40, 0.25)),
    ("suitcase_vintage", "medium", (0.70, 0.45, 0.28)),
    ("suitcase_vintage", "large", (0.80, 0.50, 0.32)),
]

#: Pinned observable references (run8-era computation, 2026-09-04;
#: see docs/status/evidence/platform_free_height/
#: 2026-09-04_2110_pfr5-g4s-run8-official/). Any change here means the
#: valid-path GT changed and a new 30-trial run is mandatory.
PINNED = {
    ("suitcase_loafbrr", "small"): (0.5159, 0.3900, 0.2375),
    ("suitcase_loafbrr", "medium"): (0.6566, 0.4388, 0.2660),
    ("suitcase_loafbrr", "large"): (0.7504, 0.4875, 0.3040),
    ("suitcase_vintage", "small"): (0.5343, 0.3618, 0.2346),
    ("suitcase_vintage", "medium"): (0.6819, 0.4091, 0.2636),
    ("suitcase_vintage", "large"): (0.7772, 0.4522, 0.3004),
}


def _sized_dir(tmp, visual, tier):
    d = os.path.join(tmp, "%s_%s" % (visual, tier), "meshes")
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, "suitcase.stl")


def _make_binary_stl(path, triangles=4):
    """Minimal valid binary STL: ``triangles`` flat facet records."""
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80)
        fh.write(struct.pack("<I", triangles))
        for _ in range(triangles):
            fh.write(struct.pack("<12fH", *([0.0] * 12), 0))


class TestFailClosed(unittest.TestCase):

    def test_unknown_tier_raises(self):
        with self.assertRaises(MeshReferenceError) as ctx:
            resolve_observable_reference(
                GAZEBO_MODELS, (0.61, 0.44, 0.27), "suitcase_loafbrr")
        self.assertIn("unknown size tier", str(ctx.exception))

    def test_missing_asset_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(MeshReferenceError) as ctx:
                resolve_observable_reference(
                    tmp, (0.55, 0.40, 0.25), "suitcase_loafbrr")
            self.assertIn("missing STL asset", str(ctx.exception))

    def test_truncated_stl_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _sized_dir(tmp, "suitcase_loafbrr", "small")
            _make_binary_stl(path, triangles=4)
            # truncate mid-triangle
            with open(path, "rb") as fh:
                data = fh.read()
            with open(path, "wb") as fh:
                fh.write(data[:80 + 4 + 25])
            with self.assertRaises(MeshReferenceError) as ctx:
                resolve_observable_reference(
                    tmp, (0.55, 0.40, 0.25), "suitcase_loafbrr")
            self.assertIn("unusable STL", str(ctx.exception))

    def test_malformed_nonfinite_stl_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _sized_dir(tmp, "suitcase_vintage", "medium")
            with open(path, "wb") as fh:
                fh.write(b"\0" * 80)
                fh.write(struct.pack("<I", 1))
                # NaN vertex floats decode "successfully" but poison the
                # observable values; the resolver must reject them.
                nan = float("nan")
                inf = float("inf")
                fh.write(struct.pack(
                    "<12fH",
                    0.0, 0.0, 1.0,           # normal
                    nan, nan, 0.1,           # v1
                    0.2, 0.2, inf,           # v2
                    -0.2, -0.2, 0.1,         # v3
                    0))
            with self.assertRaises(MeshReferenceError) as ctx:
                resolve_observable_reference(
                    tmp, (0.70, 0.45, 0.28), "suitcase_vintage")
            self.assertIn(
                "non-finite observable values", str(ctx.exception))

    def test_non_binary_stl_raises(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _sized_dir(tmp, "suitcase_loafbrr", "large")
            with open(path, "wb") as fh:
                fh.write(b"solid ascii\nendsolid\n")
            with self.assertRaises(MeshReferenceError) as ctx:
                resolve_observable_reference(
                    tmp, (0.80, 0.50, 0.32), "suitcase_loafbrr")
            self.assertIn("unusable STL", str(ctx.exception))


class TestPinnedReferences(unittest.TestCase):

    def test_all_six_assets_resolve_to_pinned_values(self):
        for visual, tier, size in SIZED:
            ref = resolve_observable_reference(GAZEBO_MODELS, size, visual)
            w, d, h = PINNED[(visual, tier)]
            self.assertAlmostEqual(ref["width"], w, places=3,
                                   msg=(visual, tier, "width"))
            self.assertAlmostEqual(ref["depth"], d, places=3,
                                   msg=(visual, tier, "depth"))
            self.assertAlmostEqual(ref["height"], h, places=3,
                                   msg=(visual, tier, "height"))

    def test_reference_identity_fields(self):
        ref = resolve_observable_reference(
            GAZEBO_MODELS, (0.55, 0.40, 0.25), "suitcase_loafbrr")
        self.assertEqual(ref["version"], OBSERVABLE_REFERENCE_VERSION)
        self.assertIn("top_band_frac=0.25", ref["version"])
        self.assertIn("z_bin=0.001", ref["version"])
        self.assertRegex(ref["stl_sha256"], r"^[0-9a-f]{64}$")
        self.assertTrue(ref["stl_path"].endswith(".stl"))

    def test_sha256_is_deterministic_and_content_addressed(self):
        a = stl_sha256(os.path.join(
            GAZEBO_MODELS, "suitcase_loafbrr_small", "meshes",
            "suitcase.stl"))
        b = stl_sha256(os.path.join(
            GAZEBO_MODELS, "suitcase_loafbrr_small", "meshes",
            "suitcase.stl"))
        self.assertEqual(a, b)
        c = stl_sha256(os.path.join(
            GAZEBO_MODELS, "suitcase_vintage_small", "meshes",
            "suitcase.stl"))
        self.assertNotEqual(a, c)

    def test_resolution_is_deterministic(self):
        for visual, _tier, size in SIZED:
            r1 = resolve_observable_reference(GAZEBO_MODELS, size, visual)
            r2 = resolve_observable_reference(GAZEBO_MODELS, size, visual)
            self.assertEqual(r1, r2)

    def test_version_string_is_pinned(self):
        """Bumping semantics without bumping the version is the silent-GT
        hazard; pin the exact current version so any change fails here."""
        self.assertEqual(
            OBSERVABLE_REFERENCE_VERSION,
            "mesh_observable_reference/v1(top_band_frac=0.25,"
            "z_bin=0.001,lid=densest-bin-median,extent=plateau_band)")


if __name__ == "__main__":
    unittest.main()
