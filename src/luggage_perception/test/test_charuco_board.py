"""calib.io CC600 Coarse board spec and detection."""

from __future__ import division

import os
import sys
import unittest

import numpy as np

SCRIPTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from luggage_perception.charuco_board import (
    CC600_CALIBIO_COARSE,
    apply_overrides,
    default_config_path,
    detect_charuco,
    interior_corner_count,
    load_spec,
    make_board,
)


class CharucoCc600Tests(unittest.TestCase):
    def test_yaml_matches_calibio_coarse(self):
        spec = load_spec(default_config_path())
        self.assertEqual(spec["squares_x"], 14)
        self.assertEqual(spec["squares_y"], 9)
        self.assertAlmostEqual(spec["square_length_m"], 0.040)
        self.assertAlmostEqual(spec["marker_length_m"], 0.030)
        self.assertEqual(spec["dictionary"], "DICT_5X5_100")
        self.assertFalse(spec["legacy_pattern"])
        self.assertEqual(interior_corner_count(spec), 13 * 8)
        self.assertAlmostEqual(spec["squares_x"] * spec["square_length_m"], 0.560)
        self.assertAlmostEqual(spec["squares_y"] * spec["square_length_m"], 0.360)
        self.assertTrue(os.path.isfile(default_config_path()))

    def test_caliper_override(self):
        spec = apply_overrides(CC600_CALIBIO_COARSE, square_length_m=0.0398)
        self.assertAlmostEqual(spec["square_length_m"], 0.0398)
        self.assertAlmostEqual(spec["marker_length_m"], 0.030)

    def test_detect_generated_board(self):
        spec = dict(CC600_CALIBIO_COARSE)
        board, _dictionary = make_board(spec)
        image = board.generateImage((1400, 900))
        if image.ndim == 2:
            import cv2
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        det = detect_charuco(image, spec)
        self.assertIsNotNone(det)
        self.assertGreaterEqual(det["count"], 80)
        self.assertGreaterEqual(det["marker_count"], 40)
        chess = board.getChessboardCorners()
        self.assertEqual(len(chess), interior_corner_count(spec))
        pitch = float(np.linalg.norm(np.asarray(chess[1]) - np.asarray(chess[0])))
        self.assertAlmostEqual(pitch, spec["square_length_m"], places=6)

    def test_capture_probe_cli(self):
        import cv2
        import tempfile
        from handeye_capture import main as capture_main

        spec = dict(CC600_CALIBIO_COARSE)
        board, _dictionary = make_board(spec)
        image = board.generateImage((1400, 900))
        if image.ndim == 2:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        with tempfile.TemporaryDirectory() as tmp:
            png = os.path.join(tmp, "board.png")
            overlay = os.path.join(tmp, "overlay.png")
            cv2.imwrite(png, image)
            rc = capture_main(["--probe", "--image", png, "--overlay", overlay])
            self.assertEqual(rc, 0)
            self.assertTrue(os.path.isfile(overlay))

    def test_builtin_matches_yaml_when_file_missing_keys(self):
        spec = load_spec("/nonexistent/cc600.yaml")
        self.assertEqual(spec["squares_x"], 14)
        self.assertEqual(spec["name"], "cc600_calibio_coarse")


if __name__ == "__main__":
    unittest.main()
