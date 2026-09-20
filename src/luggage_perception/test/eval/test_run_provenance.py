#!/usr/bin/env python3
"""Unit tests for eval-run provenance and the install-staleness guard."""
import os
import tempfile
import time
import unittest

from luggage_perception.eval.run_provenance import (
    check_install_staleness,
    collect_provenance,
    staleness_warning,
)


class TestCollectProvenance(unittest.TestCase):
    def test_config_hash_is_sha256_and_order_insensitive(self):
        a = collect_provenance({"backend": "stub", "prompts": ["a", "b"]})
        b = collect_provenance({"prompts": ["a", "b"], "backend": "stub"})
        self.assertEqual(len(a["config_hash"]), 64)
        self.assertEqual(a["config_hash"], b["config_hash"])
        other = collect_provenance({"backend": "other"})
        self.assertNotEqual(a["config_hash"], other["config_hash"])

    def test_revision_shape(self):
        prov = collect_provenance({})
        revision = prov["code_revision"]
        self.assertTrue(revision == "unknown" or len(revision) == 40)
        self.assertIn("python", prov)
        self.assertTrue(prov["module_path"].endswith("luggage_perception"))

    def test_no_config_still_hashes(self):
        prov = collect_provenance()
        self.assertEqual(len(prov["config_hash"]), 64)


class TestCheckInstallStaleness(unittest.TestCase):
    def test_source_tree_run_is_never_stale(self):
        # The suite imports the source tree (PYTHONPATH=src), so the
        # running module is a source copy; when run against an install
        # copy instead, staleness legitimately depends on build state.
        report = check_install_staleness()
        if "%sinstall%s" % (os.sep, os.sep) not in report["module_path"]:
            self.assertEqual(report["mode"], "source")
            self.assertFalse(report["stale"])
            self.assertEqual(report["newer_files"], [])

    def test_staleness_detection_on_temp_trees(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "src", "pkg")
            inst = os.path.join(tmp, "ws", "install", "pkg", "local",
                                "lib", "python3.10", "dist-packages", "pkg")
            os.makedirs(src)
            os.makedirs(inst)
            # Same-age copies: fresh.
            for name in ("__init__.py", "node.py"):
                for root in (src, inst):
                    path = os.path.join(root, name)
                    with open(path, "w") as handle:
                        handle.write("# copy\n")
                    past = time.time() - 1000.0
                    os.utime(path, (past, past))
            # Re-touch the source copy: now newer than the install copy.
            os.utime(os.path.join(src, "node.py"), None)
            report = _staleness_for_paths(src, inst)
            self.assertEqual(report["mode"], "install")
            self.assertEqual(report["checked"], 2)
            self.assertTrue(report["stale"])
            self.assertEqual(report["newer_files"], ["node.py"])

    def test_missing_installed_file_counts_as_stale(self):
        with tempfile.TemporaryDirectory() as tmp:
            src = os.path.join(tmp, "src", "pkg")
            inst = os.path.join(tmp, "ws", "install", "pkg", "local",
                                "lib", "python3.10", "dist-packages", "pkg")
            os.makedirs(src)
            os.makedirs(inst)
            with open(os.path.join(src, "a.py"), "w") as handle:
                handle.write("# a\n")
            with open(os.path.join(inst, "b.py"), "w") as handle:
                handle.write("# b\n")
            report = _staleness_for_paths(src, inst)
            self.assertTrue(report["stale"])
            self.assertIn("missing:a.py", report["newer_files"])


def _staleness_for_paths(src_dir, inst_dir):
    """check_install_staleness against explicit trees (test seam)."""
    import luggage_perception.eval.run_provenance as rp

    original_dir = rp._module_dir
    original_source = rp.source_package_dir
    rp._module_dir = lambda: inst_dir
    rp.source_package_dir = lambda: src_dir
    try:
        return rp.check_install_staleness()
    finally:
        rp._module_dir = original_dir
        rp.source_package_dir = original_source


class TestStalenessWarning(unittest.TestCase):
    def test_fresh_install_gives_no_warning(self):
        self.assertEqual(staleness_warning({"stale": False}), "")

    def test_stale_install_names_files(self):
        text = staleness_warning({
            "stale": True,
            "newer_files": ["eval/replay_evaluate.py",
                            "eval/bag_tf.py"],
        })
        self.assertIn("stale", text)
        self.assertIn("replay_evaluate.py", text)
        self.assertIn("colcon build", text)


if __name__ == "__main__":
    unittest.main()
