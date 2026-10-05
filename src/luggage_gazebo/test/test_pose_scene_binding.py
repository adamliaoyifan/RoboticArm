"""The rehearsal pose file must already cite the launched scene revision."""

import os
import sys
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "scripts"))

from frame_migration_st09_rehearsal import cell_binding  # noqa: E402


class PoseSceneBinding(unittest.TestCase):
    def test_installed_sim_deploy_pair_is_bound(self):
        binding = cell_binding(os.path.join(REPO, "install-st09"))
        self.assertTrue(binding["bound"], binding)
        self.assertEqual(
            binding["pose_file_revision"], binding["scene_file_revision"])
        self.assertEqual(binding["pose_name"], "pickup_observe_sim")
        self.assertTrue(all(
            value == binding["scene_file_revision"]
            for value in binding["capture_revisions"].values()))


if __name__ == "__main__":
    unittest.main()
