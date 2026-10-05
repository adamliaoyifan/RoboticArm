#!/usr/bin/env python3
"""Round-trip and depth-order tests for the task grid arrays."""

import os
import tempfile
import unittest

import numpy as np

from luggage_planning.reachable_task_grid import (
    MARGINAL,
    REACHABLE,
    UNREACHABLE,
    build_by_depth,
    cell_indices,
    load_task_grid,
    load_workspace,
    reduce_pose_seeds,
    save_task_grid,
    save_workspace,
    select_branch,
    unpack_index,
    usable,
)


class TestReachableTaskGrid(unittest.TestCase):
    def test_cell_index_uses_floor(self):
        self.assertEqual(cell_indices(-0.74, -0.98, 0.54, -0.745, -0.985, 0.53, 0.05), (0, 0, 0))
        self.assertEqual(cell_indices(0.70, 0.90, 1.90, -0.745, -0.985, 0.53, 0.05)[0] > 0, True)

    def test_selects_branch_nearest_observe_seed(self):
        far = np.array([3.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        near = np.array([0.0, -1.57, 1.57, np.pi, 1.57, 0.0])
        chosen, count = select_branch(np.vstack([far, near]))
        self.assertEqual(count, 2)
        self.assertTrue(np.allclose(chosen, near))

    def test_reduce_keeps_one_seed_and_the_branch_count(self):
        positions = np.array([[0.1, 0.2, 0.3], [0.1, 0.2, 0.3], [0.4, 0.0, 0.2]], np.float32)
        yaws = np.array([0, 0, 1], np.uint8)
        joints = np.array([
            [3.0, 0, 0, 0, 0, 0],
            [0.0, -1.57, 1.57, np.pi, 1.57, 0.0],
            [0.2, -1.0, 1.0, 0.0, 1.0, 0.1],
        ])
        counts = np.array([2, 2, 1], np.uint8)
        pose_id = np.array([0, 0, 1], np.uint32)
        out_p, out_y, out_j, out_n = reduce_pose_seeds(positions, yaws, joints, counts, pose_id)
        self.assertEqual(out_p.shape[0], 2)
        self.assertTrue(np.allclose(out_j[0], joints[1]))
        self.assertEqual(int(out_n[0]), 2)
        self.assertEqual(int(out_y[1]), 1)

    def test_by_depth_is_back_wall_then_low(self):
        status = np.zeros((3, 2, 3, 2), np.uint8)
        status[2, 0, 2, 0] = REACHABLE
        status[2, 0, 0, 1] = MARGINAL
        status[0, 1, 1, 0] = REACHABLE
        status[1, 0, 0, 0] = UNREACHABLE
        bands = build_by_depth(status)
        self.assertEqual(len(bands[1]), 0)
        back = [unpack_index(v, status.shape) for v in bands[2]]
        self.assertEqual([cell[2] for cell in back], [0, 2])
        self.assertEqual(back[0][3], 1)

    def test_roundtrip_and_session_arrays_start_clear(self):
        status = np.zeros((2, 1, 2, 2), np.uint8)
        status[1, 0, 0, 0] = REACHABLE
        contact = np.full(status.shape + (6,), np.nan)
        contact[1, 0, 0, 0] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        meta = {
            "resolution": 0.05,
            "origin_d": -0.1,
            "origin_a": -0.05,
            "origin_z": 0.5,
            "shape": [2, 1, 2, 2],
            "yaw_bins": [0.0, 1.5707963267948966],
            "opening_at_high_index": False,
        }
        with tempfile.TemporaryDirectory() as tmp:
            npz = os.path.join(tmp, "task_reachable.npz")
            yml = os.path.join(tmp, "task_reachable.yaml")
            save_task_grid(npz, yml, status, contact, contact.copy(), meta)
            loaded = load_task_grid(npz, yml)
            ws = os.path.join(tmp, "workspace_ikfast.npz")
            save_workspace(ws, [[0.1, 0.2, 0.3]], [0], [[0.1] * 6], [2])
            workspace = load_workspace(ws)
        self.assertEqual(int(loaded["status"][1, 0, 0, 0]), REACHABLE)
        self.assertEqual(int(loaded["occupied_count"].sum()), 0)
        self.assertTrue(np.all(loaded["opening_hops"] == np.uint16(0xFFFF)))
        self.assertEqual(len(loaded["by_depth"][1]), 1)
        self.assertFalse(usable(
            loaded["status"], loaded["occupied_count"], loaded["opening_hops"],
            (1, 0, 0, 0)))
        self.assertEqual(int(workspace["n_solutions"][0]), 2)


if __name__ == "__main__":
    unittest.main()
