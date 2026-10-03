#!/usr/bin/env python3
"""[ST09-I S5.7-6] semantic assets resolve from the install overlay.

Formal runs must not depend on the main workspace, the source tree, or
the user cache: empty launch flags fall back to the colcon-installed
package share (never a bare filename — ultralytics' not-found fallback
is a network download), and an absent asset is a NAMED launch error.
"""

import importlib.util
import os
import unittest
from unittest import mock

_LAUNCH = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "launch",
    "sim_world.launch.py"))


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "sim_world_launch_under_test", _LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestSemanticAssetPaths(unittest.TestCase):

    def setUp(self):
        self.launch = _load_launch_module()

    def test_empty_flags_resolve_the_package_share(self):
        share = "/overlay/luggage_perception/share/luggage_perception"
        with mock.patch.object(self.launch, "get_package_share_directory",
                               return_value=share):
            vendor, model = self.launch._semantic_asset_paths(
                {"semantic_clip_vendor_dir": "",
                 "semantic_model_name": ""})
        self.assertEqual(vendor, os.path.join(share, "vendor"))
        # NEVER the bare filename default: its ultralytics fallback is
        # a network download.
        self.assertEqual(
            model, os.path.join(share, "models", "yolov8s-world.pt"))
        self.assertTrue(os.path.isabs(model))

    def test_explicit_overrides_win(self):
        share = "/overlay/luggage_perception/share/luggage_perception"
        with mock.patch.object(self.launch, "get_package_share_directory",
                               return_value=share):
            vendor, model = self.launch._semantic_asset_paths(
                {"semantic_clip_vendor_dir": " /explicit/vendor ",
                 "semantic_model_name": "/explicit/yolo.pt"})
        self.assertEqual(vendor, "/explicit/vendor")
        self.assertEqual(model, "/explicit/yolo.pt")

    def test_missing_assets_are_named_launch_errors(self):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            vendor = os.path.join(tmp, "vendor")
            model = os.path.join(tmp, "yolo.pt")
            with self.assertRaises(RuntimeError) as ctx:
                self.launch._require_semantic_assets(vendor, model)
            self.assertIn("SEMANTIC_VENDOR_MISSING", str(ctx.exception))
            os.makedirs(os.path.join(vendor, "clip_pkg"))
            with self.assertRaises(RuntimeError) as ctx:
                self.launch._require_semantic_assets(vendor, model)
            self.assertIn("SEMANTIC_WEIGHTS_MISSING", str(ctx.exception))
            with open(model, "w", encoding="utf-8") as handle:
                handle.write("x")
            self.assertIsNone(
                self.launch._require_semantic_assets(vendor, model))


class TestSemanticPydeps(unittest.TestCase):
    """[ST09-I S5.7-6] interpreter deps come from the frozen prefix."""

    def setUp(self):
        self.launch = _load_launch_module()

    def test_empty_flag_resolves_ws_root_and_missing_is_named(self):
        import tempfile

        ws = "/tmp/candidate-ws"
        with mock.patch.object(
                self.launch, "get_package_prefix",
                return_value=os.path.join(
                    ws, "install-st09", "luggage_gazebo")):
            # absent prefix -> NAMED failure (the user site is never a
            # fallback source)
            with self.assertRaises(RuntimeError) as ctx:
                self.launch._semantic_pydeps_env(
                    {"semantic_pydeps_dir": ""})
            self.assertIn("SEMANTIC_PYDEPS_MISSING", str(ctx.exception))
            # deployed prefix -> (path, nvidia lib dirs)
            with tempfile.TemporaryDirectory() as tmp:
                prefix = os.path.join(tmp, "pydeps-frozen")
                nvidia = os.path.join(prefix, "nvidia", "cublas", "lib")
                os.makedirs(nvidia)
                with open(os.path.join(prefix, "DEPLOYED.json"),
                          "w", encoding="utf-8") as handle:
                    handle.write("{}")
                resolved, libs = self.launch._semantic_pydeps_env(
                    {"semantic_pydeps_dir": prefix})
                self.assertEqual(resolved, prefix)
                self.assertEqual(libs, [nvidia])

    def test_explicit_flag_wins(self):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            prefix = os.path.join(tmp, "explicit-pydeps")
            os.makedirs(prefix)
            with open(os.path.join(prefix, "DEPLOYED.json"), "w",
                      encoding="utf-8") as handle:
                handle.write("{}")
            resolved, _libs = self.launch._semantic_pydeps_env(
                {"semantic_pydeps_dir": prefix})
            self.assertEqual(resolved, prefix)


if __name__ == "__main__":
    unittest.main()
