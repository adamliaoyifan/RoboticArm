#!/usr/bin/env python3
"""DS1-C4 adapter contract: 32FC1 metres to 16UC1 millimetres, fail-closed."""

from __future__ import division

import unittest

import numpy as np

from luggage_gazebo.depth_image_adapter import convert_depth_metres_to_mm


def _pack(metres, step=None):
    metres = np.asarray(metres, dtype=np.float32)
    height, width = metres.shape
    row = metres.tobytes()
    min_step = width * 4
    if step is None:
        step = min_step
        data = row
    else:
        pad = bytearray(height * step)
        for i in range(height):
            start = i * step
            pad[start:start + min_step] = metres[i].tobytes()
        data = bytes(pad)
    return width, height, step, data


class TestDepthImageAdapter(unittest.TestCase):
    def test_finite_metres_nearest_millimetre(self):
        metres = np.array([[0.261, 1.2345], [2.0, 0.5004]], dtype=np.float32)
        width, height, step, data = _pack(metres)
        out, reason = convert_depth_metres_to_mm(
            "32FC1", width, height, step, data, near_m=0.26, max_m=3.0)
        self.assertIsNone(reason)
        mm = np.frombuffer(out, dtype="<u2").reshape(height, width)
        self.assertEqual(int(mm[0, 0]), 261)
        self.assertEqual(int(mm[0, 1]), 1234)
        self.assertEqual(int(mm[1, 0]), 2000)
        self.assertEqual(int(mm[1, 1]), 500)
        self.assertLessEqual(abs(mm[0, 1] / 1000.0 - 1.2345), 0.001)

    def test_invalid_values_become_zero(self):
        metres = np.array(
            [[np.inf, -np.inf, np.nan, 0.0],
             [-0.1, 0.25, 0.26, 3.1]],
            dtype=np.float32)
        width, height, step, data = _pack(metres)
        out, reason = convert_depth_metres_to_mm(
            "32FC1", width, height, step, data, near_m=0.26, max_m=3.0)
        self.assertIsNone(reason)
        mm = np.frombuffer(out, dtype="<u2").reshape(height, width)
        self.assertEqual(int(mm[0, 0]), 0)
        self.assertEqual(int(mm[0, 1]), 0)
        self.assertEqual(int(mm[0, 2]), 0)
        self.assertEqual(int(mm[0, 3]), 0)
        self.assertEqual(int(mm[1, 0]), 0)
        self.assertEqual(int(mm[1, 1]), 0)
        self.assertEqual(int(mm[1, 2]), 260)
        self.assertEqual(int(mm[1, 3]), 0)

    def test_output_layout_16uc1(self):
        metres = np.full((3, 5), 1.0, dtype=np.float32)
        width, height, step, data = _pack(metres)
        out, reason = convert_depth_metres_to_mm(
            "32FC1", width, height, step, data)
        self.assertIsNone(reason)
        self.assertEqual(len(out), width * height * 2)
        mm = np.frombuffer(out, dtype="<u2").reshape(height, width)
        self.assertTrue((mm == 1000).all())

    def test_padded_step_preserved_geometry(self):
        metres = np.array([[0.5, 0.6]], dtype=np.float32)
        width, height, step, data = _pack(metres, step=16)
        self.assertEqual(step, 16)
        out, reason = convert_depth_metres_to_mm(
            "32FC1", width, height, step, data)
        self.assertIsNone(reason)
        mm = np.frombuffer(out, dtype="<u2").reshape(height, width)
        self.assertEqual(int(mm[0, 0]), 500)
        self.assertEqual(int(mm[0, 1]), 600)

    def test_truncated_payload_fails_closed(self):
        metres = np.ones((2, 4), dtype=np.float32)
        width, height, step, data = _pack(metres)
        out, reason = convert_depth_metres_to_mm(
            "32FC1", width, height, step, data[:-3])
        self.assertIsNone(out)
        self.assertEqual(reason, "truncated_payload")

    def test_unsupported_encoding_fails_closed(self):
        out, reason = convert_depth_metres_to_mm(
            "16UC1", 2, 2, 4, b"\x00" * 8)
        self.assertIsNone(out)
        self.assertEqual(reason, "unsupported_encoding")

    def test_malformed_step_fails_closed(self):
        out, reason = convert_depth_metres_to_mm(
            "32FC1", 4, 1, 4, b"\x00" * 16)
        self.assertIsNone(out)
        self.assertEqual(reason, "malformed_step")


if __name__ == "__main__":
    unittest.main()
