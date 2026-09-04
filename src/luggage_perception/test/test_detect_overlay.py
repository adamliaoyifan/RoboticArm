#!/usr/bin/env python3
"""Unit tests for detect_overlay (no roscore required).

Covers the two things that silently produce a wrong-but-plausible picture:
the OBB corner construction and the pinhole projection. Drawing tests run
when cv2 is installed; they skip otherwise so geometry still tests in a
bare environment.
"""

import math
import os
import sys
import unittest

import numpy as np

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

from luggage_perception.detect_overlay import (  # noqa: E402
    COLOR_FAILURE_BGR,
    COLOR_GT_FALLBACK_BGR,
    COLOR_PERCEPTION_BGR,
    OBB_EDGES,
    draw_detection_overlay,
    draw_failure_overlay,
    draw_timestamp_banner,
    format_detection_label,
    format_stamp_sec,
    obb_corners_world,
    parse_detection_record,
    project_detection,
    project_points,
    rotation_from_quaternion,
    source_color_bgr,
    stamp_alignment,
    timestamp_banner_lines,
    transform_points,
)


def _yaw_quat(yaw):
    return [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)]


class TestObbCorners(unittest.TestCase):
    def test_axis_aligned_box_spans_half_extents(self):
        corners = obb_corners_world(
            position=[1.0, 2.0, 3.0], quat_xyzw=_yaw_quat(0.0),
            size=[0.8, 0.4, 0.2])

        self.assertEqual(corners.shape, (8, 3))
        np.testing.assert_allclose(corners.min(axis=0), [0.6, 1.8, 2.9])
        np.testing.assert_allclose(corners.max(axis=0), [1.4, 2.2, 3.1])
        np.testing.assert_allclose(corners.mean(axis=0), [1.0, 2.0, 3.0])

    def test_yaw_90_swaps_xy_extents(self):
        corners = obb_corners_world(
            position=[0.0, 0.0, 0.0], quat_xyzw=_yaw_quat(math.pi / 2),
            size=[0.8, 0.4, 0.2])

        span = corners.max(axis=0) - corners.min(axis=0)
        np.testing.assert_allclose(span, [0.4, 0.8, 0.2], atol=1e-9)

    def test_every_edge_connects_adjacent_corners(self):
        # An edge must change exactly one axis; a wrong corner ordering shows
        # up as a diagonal across a face.
        size = np.array([0.8, 0.4, 0.2])
        corners = obb_corners_world([0.0, 0.0, 0.0], _yaw_quat(0.0), size)
        for a, b in OBB_EDGES:
            delta = np.abs(corners[a] - corners[b])
            changed = delta > 1e-9
            self.assertEqual(
                int(changed.sum()), 1,
                "edge (%d,%d) is not an axis-aligned box edge" % (a, b))
            np.testing.assert_allclose(delta[changed], size[changed])


class TestRotationFromQuaternion(unittest.TestCase):
    def test_identity(self):
        np.testing.assert_allclose(
            rotation_from_quaternion([0.0, 0.0, 0.0, 1.0]), np.eye(3))

    def test_degenerate_quaternion_falls_back_to_identity(self):
        np.testing.assert_allclose(
            rotation_from_quaternion([0.0, 0.0, 0.0, 0.0]), np.eye(3))

    def test_yaw_90_maps_x_to_y(self):
        rot = rotation_from_quaternion(_yaw_quat(math.pi / 2))
        np.testing.assert_allclose(rot.dot([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0],
                                   atol=1e-9)


class TestProjectPoints(unittest.TestCase):
    INTRINSICS = (600.0, 600.0, 320.0, 240.0)

    def test_point_on_optical_axis_hits_principal_point(self):
        uv, valid = project_points([[0.0, 0.0, 2.0]], self.INTRINSICS)
        self.assertTrue(bool(valid[0]))
        np.testing.assert_allclose(uv[0], [320.0, 240.0])

    def test_offset_point_matches_pinhole_model(self):
        uv, valid = project_points([[0.1, -0.2, 2.0]], self.INTRINSICS)
        self.assertTrue(bool(valid[0]))
        np.testing.assert_allclose(uv[0], [320.0 + 30.0, 240.0 - 60.0])

    def test_points_behind_camera_are_invalid_and_nan(self):
        uv, valid = project_points(
            [[0.0, 0.0, -1.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            self.INTRINSICS)
        np.testing.assert_array_equal(valid, [False, False, True])
        self.assertTrue(np.all(np.isnan(uv[:2])))
        self.assertTrue(np.all(np.isfinite(uv[2])))

    def test_empty_input(self):
        uv, valid = project_points(np.zeros((0, 3)), self.INTRINSICS)
        self.assertEqual(uv.shape, (0, 2))
        self.assertEqual(valid.shape, (0,))


class TestTransformPoints(unittest.TestCase):
    def test_applies_rotation_then_translation(self):
        rot = rotation_from_quaternion(_yaw_quat(math.pi / 2))
        out = transform_points([[1.0, 0.0, 0.0]], rot, [0.0, 0.0, 5.0])
        np.testing.assert_allclose(out[0], [0.0, 1.0, 5.0], atol=1e-9)


class TestProjectDetection(unittest.TestCase):
    INTRINSICS = (600.0, 600.0, 320.0, 240.0)

    def test_box_in_front_of_camera_projects_around_centre(self):
        # Camera looks down +z of its own optical frame at a box 2 m away,
        # with world and optical axes aligned for an easy hand-check.
        centre_uv, corner_uv, corner_valid, centre_ok = project_detection(
            position=[0.0, 0.0, 2.0], quat_xyzw=_yaw_quat(0.0),
            size=[0.8, 0.4, 0.2],
            rotation=np.eye(3), translation=[0.0, 0.0, 0.0],
            intrinsics=self.INTRINSICS)

        self.assertTrue(centre_ok)
        np.testing.assert_allclose(centre_uv, [320.0, 240.0])
        self.assertTrue(bool(np.all(corner_valid)))
        # Corners straddle the principal point in both axes.
        self.assertLess(corner_uv[:, 0].min(), 320.0)
        self.assertGreater(corner_uv[:, 0].max(), 320.0)
        self.assertLess(corner_uv[:, 1].min(), 240.0)
        self.assertGreater(corner_uv[:, 1].max(), 240.0)

    def test_box_behind_camera_yields_no_centre(self):
        centre_uv, _, corner_valid, centre_ok = project_detection(
            position=[0.0, 0.0, -2.0], quat_xyzw=_yaw_quat(0.0),
            size=[0.8, 0.4, 0.2],
            rotation=np.eye(3), translation=[0.0, 0.0, 0.0],
            intrinsics=self.INTRINSICS)

        self.assertFalse(centre_ok)
        self.assertIsNone(centre_uv)
        self.assertFalse(bool(np.any(corner_valid)))


class TestParseDetectionRecord(unittest.TestCase):
    SUCCESS = {
        "success": True,
        "source": "perception",
        "reason": "ok",
        "confidence": 0.93,
        "detected": {
            "id": "detected_box",
            "position": [-1.0, 0.0, 0.6],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "size": [0.74, 0.44, 0.27],
        },
    }

    def test_success_record(self):
        record = parse_detection_record(self.SUCCESS)
        self.assertTrue(record.success)
        self.assertEqual(record.source, "perception")
        np.testing.assert_allclose(record.position, [-1.0, 0.0, 0.6])
        np.testing.assert_allclose(record.size, [0.74, 0.44, 0.27])
        self.assertAlmostEqual(record.confidence, 0.93)

    def test_failure_record_has_no_pose(self):
        record = parse_detection_record({
            "success": False,
            "source": "perception",
            "reason": "DETECT_TOO_FEW_POINTS",
            "confidence": 0.0,
        })
        self.assertFalse(record.success)
        self.assertIsNone(record.position)
        self.assertEqual(record.reason, "DETECT_TOO_FEW_POINTS")

    def test_success_without_pose_is_demoted_to_failure(self):
        # Otherwise the overlay would keep showing the previous box.
        record = parse_detection_record({"success": True, "reason": "ok"})
        self.assertFalse(record.success)

    def test_malformed_pose_is_demoted_to_failure(self):
        payload = dict(self.SUCCESS)
        payload["detected"] = {"position": [1.0, 2.0], "orientation": [],
                               "size": []}
        record = parse_detection_record(payload)
        self.assertFalse(record.success)

    def test_non_record_payloads(self):
        self.assertIsNone(parse_detection_record(None))
        self.assertIsNone(parse_detection_record([1, 2, 3]))
        self.assertIsNone(parse_detection_record({"no_success_key": 1}))

    def test_label_and_colour_distinguish_gt_fallback(self):
        record = parse_detection_record(self.SUCCESS)
        self.assertIn("0.74x0.44x0.27", format_detection_label(record))
        self.assertEqual(source_color_bgr(record.source), COLOR_PERCEPTION_BGR)
        self.assertEqual(source_color_bgr("gt_fallback"),
                         COLOR_GT_FALLBACK_BGR)


def _cv2_or_skip():
    try:
        import cv2  # noqa: F401
        return True
    except ImportError:
        return False


class TestDrawOverlay(unittest.TestCase):
    """Success draws a box; failure draws a banner and no box.

    This is the contract a reviewer sees in RViz. Geometry tests above do not
    exercise cv2, so a broken draw path would ship as a blank Image panel.
    """

    def setUp(self):
        if not _cv2_or_skip():
            self.skipTest("cv2 not installed")

    def test_success_draws_on_a_copy(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        centre, corners, valid, _ = project_detection(
            position=[0.0, 0.0, 2.0], quat_xyzw=_yaw_quat(0.0),
            size=[0.74, 0.44, 0.27],
            rotation=np.eye(3), translation=[0.0, 0.0, 0.0],
            intrinsics=(600.0, 600.0, 320.0, 240.0))
        out = draw_detection_overlay(
            image, centre, corners, valid, "perception 0.74x0.44x0.27 conf=0.93",
            COLOR_PERCEPTION_BGR)
        self.assertEqual(out.shape, image.shape)
        self.assertEqual(int(image.sum()), 0)
        self.assertGreater(int((out.sum(axis=2) > 0).sum()), 0)
        # Centre cross lands on the principal point.
        self.assertTrue(np.any(out[240, 308:333] > 0))

    def test_failure_shows_reason_and_no_box(self):
        # The node draws failure on the latest raw RGB, not the previous
        # overlay, so a failed detect cannot leave a cyan box on screen.
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        failure = draw_failure_overlay(image, "DETECT_TOO_FEW_POINTS")
        self.assertEqual(int(image.sum()), 0)
        cyan = np.all(failure == COLOR_PERCEPTION_BGR, axis=2)
        self.assertFalse(bool(np.any(cyan)))
        red = np.all(failure == COLOR_FAILURE_BGR, axis=2)
        self.assertTrue(bool(np.any(red)))

    def test_timestamp_banner_marks_match_and_mismatch(self):
        image = np.zeros((80, 160, 3), dtype=np.uint8)
        lines, meta = timestamp_banner_lines(
            3.0, 3.0, dump_stamp=3.2, infer_ms=15.0, cargo_stamp=3.0)
        self.assertTrue(meta["aligned"])
        self.assertIn("MATCH", lines[2])
        self.assertEqual(format_stamp_sec(3.0), "3.000")
        out = draw_timestamp_banner(image, lines, aligned=True)
        self.assertEqual(int(image.sum()), 0)
        self.assertGreater(int(out.sum()), 0)
        aligned, delta = stamp_alignment(3.0, 3.4)
        self.assertFalse(aligned)
        self.assertAlmostEqual(delta, 0.4)
        bad_lines, bad_meta = timestamp_banner_lines(3.0, 3.4)
        self.assertFalse(bad_meta["aligned"])
        self.assertIn("MISMATCH", bad_lines[2])
        bad = draw_timestamp_banner(image, bad_lines, aligned=False)
        self.assertGreater(int(bad.sum()), 0)


if __name__ == "__main__":
    unittest.main()
