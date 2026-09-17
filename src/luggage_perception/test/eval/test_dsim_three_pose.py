#!/usr/bin/env python3
"""Unit tests for the three-pose depth+stamped-TF oracle. No ROS."""

import unittest

import numpy as np

from luggage_perception.eval.dsim_three_pose import (
    gt_planes,
    measure_depth_world,
    optical_from_world_rt,
    quat_xyzw_to_R,
    score_three_pose,
    world_from_optical,
)


def _sample(pose_name, err_top=0.005, err_support=0.004, err_xy=0.01, **extra):
    rec = {
        "pose_name": pose_name,
        "ok": True,
        "tf_source": "stamp",
        "used_camera_cloud": False,
        "err_top_m": err_top,
        "err_support_m": err_support,
        "err_xy_m": err_xy,
    }
    rec.update(extra)
    return rec


class TestThreePose(unittest.TestCase):
    def test_roundtrip_transform(self):
        rot = quat_xyzw_to_R(0.0, 0.0, 0.0, 1.0)
        trans = np.array([1.0, 2.0, 3.0])
        pts = np.array([[0.1, -0.2, 0.5]])
        world = world_from_optical(pts, rot, trans)
        rot_ow, trans_ow = optical_from_world_rt(rot, trans)
        back = world_from_optical(world, rot_ow, trans_ow)
        np.testing.assert_allclose(back, pts, atol=1e-9)

    def test_gt_planes(self):
        planes = gt_planes((-1.0, 0.0, 0.4), (0.5, 0.35, 0.56))
        self.assertAlmostEqual(planes["top_z"], 0.68)
        self.assertAlmostEqual(planes["support_z"], 0.12)

    def test_score_requires_three_stamped_poses(self):
        two = [_sample("a"), _sample("b")]
        verdict = score_three_pose(two)
        self.assertFalse(verdict["pass"])
        latest = [
            _sample("a"), _sample("b"),
            _sample("c", tf_source="latest"),
        ]
        self.assertFalse(score_three_pose(latest)["pass"])
        cloud = [
            _sample("a"), _sample("b"),
            _sample("c", used_camera_cloud=True),
        ]
        self.assertFalse(score_three_pose(cloud)["pass"])

    def test_score_passes_three_good_samples(self):
        verdict = score_three_pose([
            _sample("pickup_observe"),
            _sample("pickup_j1_plus"),
            _sample("pickup_j1_minus"),
        ])
        self.assertTrue(verdict["pass"], verdict["failures"])

    def test_measure_synthetic_nadir_box(self):
        rot_wo = np.diag([1.0, -1.0, -1.0])
        trans_wo = np.array([-1.0, 0.0, 1.9])
        gt_xyz = (-1.0, 0.0, 0.40)
        gt_size = (0.50, 0.36, 0.56)
        gt_quat = (0.0, 0.0, 0.0, 1.0)
        k = (323.1775, 322.8994, 317.7526, 178.0294)
        h, w = 360, 640
        top_z = 0.40 + 0.5 * 0.56
        support_z = 0.40 - 0.5 * 0.56
        top_mm = int(round((1.9 - top_z) * 1000.0))
        support_mm = int(round((1.9 - support_z) * 1000.0))
        depth = np.zeros((h, w), dtype=np.uint16)
        # Coarse GT projection to seed the cargo rectangle.
        rec = measure_depth_world(
            np.full((h, w), top_mm, dtype=np.uint16),
            w, h, k, rot_wo, trans_wo, gt_xyz, gt_quat, gt_size, stride=2)
        self.assertIsNotNone(rec.get("bbox"))
        u0, v0, u1, v1 = rec["bbox"]
        ua, ub = int(u0), int(u1)
        va, vb = int(v0), int(v1)
        pad = 40
        depth[max(0, va - pad):min(h, vb + pad + 1),
              max(0, ua - pad):min(w, ub + pad + 1)] = support_mm
        depth[va:vb + 1, ua:ub + 1] = top_mm
        measured = measure_depth_world(
            depth, w, h, k, rot_wo, trans_wo, gt_xyz, gt_quat, gt_size,
            stride=2)
        self.assertTrue(measured["ok"], measured)
        self.assertLess(measured["err_top_m"], 0.02)
        self.assertLess(measured["err_xy_m"], 0.03)
        self.assertFalse(measured["used_camera_cloud"])
        self.assertEqual(measured["tf_source"], "stamp")


if __name__ == "__main__":
    unittest.main()
