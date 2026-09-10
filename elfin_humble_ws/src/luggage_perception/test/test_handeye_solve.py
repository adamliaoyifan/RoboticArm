"""Offline hand-eye solve on synthetic AX=XB."""

from __future__ import division

import json
import os
import tempfile
import unittest

import numpy as np

import sys
SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import handeye_solve  # noqa: E402


def _rand_R(rng):
    u = rng.normal(size=3)
    u = u / np.linalg.norm(u)
    ang = rng.uniform(-0.8, 0.8)
    k = np.array([
        [0.0, -u[2], u[1]],
        [u[2], 0.0, -u[0]],
        [-u[1], u[0], 0.0],
    ])
    return np.eye(3) + math_sin(ang) * k + (1.0 - math_cos(ang)) * k.dot(k)


def math_sin(a):
    return float(np.sin(a))


def math_cos(a):
    return float(np.cos(a))


class HandeyeSolveTests(unittest.TestCase):
    def test_synthetic_methods_agree(self):
        rng = np.random.default_rng(3)
        R_cam = _rand_R(rng)
        t_cam = np.array([0.04, -0.02, 0.08])
        T_fc = np.eye(4)
        T_fc[:3, :3] = R_cam
        T_fc[:3, 3] = t_cam
        poses = []
        K = np.array([[646.0, 0.0, 640.0], [0.0, 646.0, 360.0], [0.0, 0.0, 1.0]])
        square = 0.051
        import cv2
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        board = cv2.aruco.CharucoBoard((10, 8), square, 0.0375, dictionary)
        obj = board.getChessboardCorners()
        ids = list(range(min(20, len(obj))))
        for i in range(16):
            R_ct = _rand_R(rng)
            t_ct = np.array([
                0.02 * rng.normal(), 0.02 * rng.normal(), 0.55 + 0.05 * rng.normal(),
            ])
            T_ct = np.eye(4)
            T_ct[:3, :3] = R_ct
            T_ct[:3, 3] = t_ct
            T_gf = np.linalg.inv(T_ct).dot(np.linalg.inv(T_fc))
            img = []
            behind = False
            for ident in ids:
                X = obj[ident]
                xcam = R_ct.dot(X) + t_ct
                if xcam[2] <= 0.05:
                    behind = True
                    break
                u = K[0, 0] * xcam[0] / xcam[2] + K[0, 2]
                v = K[1, 1] * xcam[1] / xcam[2] + K[1, 2]
                img.append([u, v])
            if behind:
                continue
            poses.append({
                "pose_id": "p%d" % i,
                "T_base_flange": T_gf.tolist(),
                "camera_matrix": K.tolist(),
                "detection": {
                    "count": len(ids),
                    "corner_ids": ids,
                    "corners_px": img,
                },
            })
        result = handeye_solve.solve(poses, K, np.zeros(5), square, 0.0375)
        self.assertGreaterEqual(result["n"], 8)
        est = np.asarray(result["methods"]["PARK"]["t_m"])
        self.assertLess(np.linalg.norm(est - t_cam), 0.03)
        self.assertIsNotNone(result["spread"])
        self.assertLess(result["spread"]["translation_mm"], 10.0)

    def test_replay_index_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            blob = {"square_length_m": 0.0497, "poses": []}
            path = os.path.join(tmp, "capture_index.json")
            with open(path, "w", encoding="utf-8") as handle:
                json.dump(blob, handle)
            poses, square = handeye_solve.load_poses(path, 0.05)
            self.assertEqual(square, 0.0497)
            self.assertEqual(poses, [])


if __name__ == "__main__":
    unittest.main()
