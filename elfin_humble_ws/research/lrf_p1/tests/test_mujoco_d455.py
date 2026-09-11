"""MuJoCo D455 back-projection and observation-contract tests."""

from __future__ import division

import math
import os
import unittest

import numpy as np

from research.lrf_p1.contracts import walk_forbidden
from research.lrf_p1.mujoco_d455.camera import (
    depth_to_cloud,
    mj_camera_opencv_rotation,
)
from research.lrf_p1.mujoco_d455.d455 import depth_intrinsics, hfov_note
from research.lrf_p1.mujoco_d455.scene import lookat_xyaxes


class TestD455Intrinsics(unittest.TestCase):
    def test_square_pixels_match_vfov(self):
        k = depth_intrinsics()
        self.assertEqual(k["width"], 848)
        self.assertEqual(k["height"], 480)
        fy = (480 / 2.0) / math.tan(math.radians(58.0) / 2.0)
        self.assertAlmostEqual(k["fy"], fy, places=6)
        self.assertAlmostEqual(k["fx"], k["fy"], places=6)
        self.assertIn("87", hfov_note())

    def test_lookat_right_handed(self):
        right, yup, look = lookat_xyaxes((0, -1, 0.5), (0, 0, 0.1))
        r = np.column_stack([right, yup, -look])
        self.assertAlmostEqual(float(np.linalg.det(r)), 1.0, places=6)


class TestBackprojectBox(unittest.TestCase):
    def test_known_box_cloud_in_aabb(self):
        try:
            import mujoco as mj  # noqa: F401
        except ImportError:
            self.skipTest("mujoco not installed")
        os.environ.setdefault("MUJOCO_GL", "egl")
        from research.lrf_p1.mujoco_d455.camera import camera_state, render_rgbd
        from research.lrf_p1.mujoco_d455.scene import lookat_xyaxes

        pos = np.array([0.0, -1.0, 0.5])
        target = np.array([0.0, 0.0, 0.08])
        right, yup, _look = lookat_xyaxes(pos, target)
        xyaxes = " ".join(str(v) for v in np.concatenate([right, yup]))
        xml = """
        <mujoco>
          <visual><global offwidth="848" offheight="480"/></visual>
          <worldbody>
            <light pos="0 0 3" dir="0 0 -1"/>
            <geom type="box" size="0.1 0.15 0.08" pos="0 0 0.08" rgba="0.2 0.5 0.9 1"/>
            <camera name="d455_depth" pos="0 -1 0.5" xyaxes="%s" fovy="58"/>
          </worldbody>
        </mujoco>
        """ % xyaxes
        model = mj.MjModel.from_xml_string(xml)
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        _rgb, depth = render_rgbd(model, data)
        cam = camera_state(model, data)
        k = depth_intrinsics()
        cloud = depth_to_cloud(depth, k, cam["R_cv"], cam["t"])
        self.assertGreater(len(cloud), 1000)
        inside = (
            (np.abs(cloud[:, 0]) < 0.12)
            & (np.abs(cloud[:, 1]) < 0.17)
            & (cloud[:, 2] > -0.01)
            & (cloud[:, 2] < 0.18)
        )
        self.assertGreater(float(inside.mean()), 0.95)

    def test_observation_has_no_privileged_keys(self):
        obs = {
            "points": np.zeros((4, 3)),
            "roi_center_xy": (0.0, 0.0),
            "frame_id": "world",
            "stamp": 0.0,
        }
        self.assertEqual(list(walk_forbidden(obs)), [])
        dirty = dict(obs)
        dirty["gazebo_state"] = {}
        self.assertTrue(list(walk_forbidden(dirty)))


class TestCvRotation(unittest.TestCase):
    def test_look_is_positive_z(self):
        # MJ xmat columns: X, Y, Z_mj with Z_mj = -look
        look = np.array([0.0, 1.0, 0.0])
        right = np.array([1.0, 0.0, 0.0])
        yup = np.array([0.0, 0.0, 1.0])
        r_mj = np.column_stack([right, yup, -look])
        r_cv = mj_camera_opencv_rotation(r_mj)
        # OpenCV +Z should be look
        np.testing.assert_allclose(r_cv[:, 2], look, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
