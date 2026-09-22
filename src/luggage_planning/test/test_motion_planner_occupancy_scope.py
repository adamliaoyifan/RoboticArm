#!/usr/bin/env python3
"""Occupancy-injection scope and failure capture wiring in the node.

Pure-logic tests (no ROS runtime): the ``occupancy_segments`` scope
predicate, and the replay copy of node-side failure capture artifacts.
The scope is what widened occupancy from carry-only to every segment;
the copy keeps the capture visible in the eval replay bundle.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

_GAZEBO_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..",
    "luggage_gazebo", "scripts"))
if _GAZEBO_SCRIPTS not in sys.path:
    sys.path.insert(0, _GAZEBO_SCRIPTS)

from luggage_gazebo.place_metrics import PlaceTrial  # noqa: E402

import motion_planner_node as node_mod  # noqa: E402
import place_smoke_driver as smoke_mod  # noqa: E402


class OccupancyScopeTest(unittest.TestCase):
    def test_wildcard_covers_every_segment(self):
        for name in ("transit", "traverse", "insert", "descend", "retreat",
                     "place_exit", "stage_mid", "pre_grasp", "attach",
                     "place_probe", "pick_retreat"):
            self.assertTrue(node_mod.occupancy_scope("*", name), name)

    def test_list_scope_narrows_back_to_carry_only(self):
        csv = " transit , traverse "
        self.assertTrue(node_mod.occupancy_scope(csv, "transit"))
        self.assertTrue(node_mod.occupancy_scope(csv, "traverse"))
        for name in ("insert", "descend", "retreat", "place_exit", ""):
            self.assertFalse(node_mod.occupancy_scope(csv, name), name)

    def test_empty_scope_disables_injection(self):
        for name in ("transit", "anything"):
            self.assertFalse(node_mod.occupancy_scope("", name))
            self.assertFalse(node_mod.occupancy_scope(None, name))


class _ReplayShell(object):
    """Only the collaborators `_freeze_motion_replay` touches."""

    def __init__(self, scene):
        self._scene_snapshot = lambda: scene
        self._joint_ring = ()
        self._segment_exec_t0 = None
        self._segments_log = []
        self._last_pick_detection = None
        self._surface_2d = None

    def suction_xyz(self):
        return [0.0, 0.0, 0.0], None


class ReplayCaptureCopyTest(unittest.TestCase):
    """Node failure-capture artifacts land in the trial replay bundle."""

    def setUp(self):
        self._tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self._tmp, True)
        self._env = os.environ.get("MOTION_BOUNDARY_DUMP")
        self._orig_env_set = "MOTION_BOUNDARY_DUMP" in os.environ

    def tearDown(self):
        if self._orig_env_set:
            os.environ["MOTION_BOUNDARY_DUMP"] = self._env
        else:
            os.environ.pop("MOTION_BOUNDARY_DUMP", None)

    def _boundary_with_capture(self):
        root = os.path.join(self._tmp, "boundaries")
        os.makedirs(root, exist_ok=True)
        surface_path = os.path.join(
            root, "fail_1000_descend_surface_2d.json")
        with open(surface_path, "w", encoding="utf-8") as handle:
            json.dump({"map_revision": 3}, handle)
        cloud_path = os.path.join(root, "fail_1000_descend_cloud.ply")
        with open(cloud_path, "wb") as handle:
            handle.write(b"ply\n")
        record = {
            "name": "descend",
            "trajectory": {"points": [{"t": 0.0, "q": [0.0]}]},
            "capture": {
                "surface_2d_dump": surface_path,
                "sidecar": os.path.join(root, "fail_1000_descend_capture.json"),
                "cloud": {"path": cloud_path},
                "depth": {"path": os.path.join(root, "absent.npy")},
                "missing": [],
            },
        }
        with open(os.path.join(root, "latest.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(record, handle)
        os.environ["MOTION_BOUNDARY_DUMP"] = root
        return root

    def test_capture_artifacts_copied_into_replay(self):
        self._boundary_with_capture()
        driver = _ReplayShell(scene={"world_object_ids": []})
        trial = PlaceTrial(index=1, fail_code="PLACE_PLAN_descend")
        dest = os.path.join(self._tmp, "trial_01")
        os.makedirs(dest, exist_ok=True)
        manifest = smoke_mod.PlaceSmokeDriver._freeze_motion_replay(
            driver, dest, trial)
        replay = os.path.join(dest, "replay")
        copied = sorted(
            name for name in os.listdir(replay) if name.startswith("capture_"))
        self.assertEqual(copied, [
            "capture_fail_1000_descend_cloud.ply",
            "capture_fail_1000_descend_surface_2d.json",
        ])
        self.assertIn("capture_surface_2d_dump", manifest["artifacts"])
        self.assertIn("capture_cloud", manifest["artifacts"])
        # The absent depth must not be listed and must not gate replay.
        self.assertNotIn("capture_depth", manifest["artifacts"])
        self.assertEqual(
            [m for m in manifest["missing"] if m.startswith("capture")], [])

    def test_capture_missing_surfaces_in_extras_not_missing(self):
        root = self._boundary_with_capture()
        record = json.load(open(os.path.join(root, "latest.json")))
        record["capture"]["missing"] = ["depth_absent"]
        with open(os.path.join(root, "latest.json"), "w",
                  encoding="utf-8") as handle:
            json.dump(record, handle)
        driver = _ReplayShell(scene={"world_object_ids": []})
        trial = PlaceTrial(index=2, fail_code="PLACE_PLAN_descend")
        dest = os.path.join(self._tmp, "trial_02")
        os.makedirs(dest, exist_ok=True)
        manifest = smoke_mod.PlaceSmokeDriver._freeze_motion_replay(
            driver, dest, trial)
        # depth_absent is visible in extras, never in manifest missing.
        self.assertNotIn("depth_absent", manifest["missing"])
        self.assertEqual(trial.extras["replay"]["capture_missing"],
                         ["depth_absent"])


if __name__ == "__main__":
    unittest.main()
