#!/usr/bin/env python3
"""PF-A1: online geometry nodes must not consume GetCurrentBox."""

from __future__ import division

import os
import unittest

from luggage_perception.eval.pf_a1_static_audit import (
    ONLINE_GEOMETRY_NODES,
    audit_tree,
)


def _workspace_root():
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


class TestPFA1StaticAudit(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = _workspace_root()
        cls.report = audit_tree(cls.root)

    def test_workspace_looks_right(self):
        detector = os.path.join(
            self.root, "src/luggage_perception/scripts/luggage_detector_node.py")
        self.assertTrue(os.path.isfile(detector), detector)

    def test_height_geometry_pass(self):
        self.assertTrue(
            self.report["height_geometry_pass"], self.report["blockers"])
        self.assertEqual(self.report["blockers"], [])

    def test_online_nodes_have_no_getcurrentbox(self):
        for row in self.report["online_geometry_nodes"]:
            self.assertEqual(
                row["getcurrentbox_lines"], [], row["file"])
            self.assertEqual(
                row["spawned_geometry_parser_lines"], [], row["file"])

    def test_online_node_list_is_the_pickup_path(self):
        self.assertIn(
            "src/luggage_perception/scripts/luggage_detector_node.py",
            ONLINE_GEOMETRY_NODES)
        self.assertIn(
            "src/luggage_planning/scripts/waypoint_generator_node.py",
            ONLINE_GEOMETRY_NODES)

    def test_perception_uses_epoch_parser_not_geometry_parser(self):
        by_file = {
            row["file"]: row for row in self.report["online_geometry_nodes"]}
        detector = by_file[
            "src/luggage_perception/scripts/luggage_detector_node.py"]
        self.assertTrue(detector["epoch_parser_lines"])
        self.assertFalse(detector["spawned_geometry_parser_lines"])

    def test_eval_drivers_keep_getcurrentbox(self):
        eval_files = {
            row["file"]: row for row in self.report["eval_providers"]}
        gate4 = eval_files.get("scripts/platform_free_height_gate4_eval.py")
        self.assertIsNotNone(gate4)
        self.assertTrue(gate4["getcurrentbox_lines"])

    def test_task_state_has_hardware_provider(self):
        provider = self.report["task_state_hardware_provider"]
        self.assertTrue(provider["credible"])

    def test_vacuum_geometry_is_classified_sim_backend(self):
        backend = self.report["sim_backend"]
        self.assertEqual(backend["class"], "sim_backend_only")
        self.assertTrue(backend["spawned_geometry_parser_lines"])

    def test_inventory_covers_required_inputs(self):
        names = [row["input"] for row in self.report["inventory"]]
        joined = " | ".join(names)
        self.assertIn("GetCurrentBox", joined)
        self.assertIn("current_box id+generation", joined)
        self.assertIn("platform_z", joined)
        self.assertIn("set_pose", joined)


if __name__ == "__main__":
    unittest.main()
