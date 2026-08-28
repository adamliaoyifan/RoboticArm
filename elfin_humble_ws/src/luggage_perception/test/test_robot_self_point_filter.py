#!/usr/bin/env python3
"""Unit tests for RobotSelfPointFilter geometric self-filter."""

import os
import sys
import unittest


from luggage_perception.robot_self_point_filter import RobotSelfPointFilter, _validate_body  # noqa: E402


class _FakeTransform(object):
    def __init__(self, tx=0.0, ty=0.0, tz=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        self.transform = type(
            "T",
            (),
            {
                "translation": type("Tr", (), {"x": tx, "y": ty, "z": tz})(),
                "rotation": type("R", (), {"x": qx, "y": qy, "z": qz, "w": qw})(),
            },
        )()


class _FakeTfBuffer(object):
    def __init__(self, transforms=None, missing=None):
        self._transforms = transforms or {}
        self._missing = set(missing or [])

    def lookup_transform(self, target, source, stamp, timeout=None):
        key = (target, source)
        if source in self._missing:
            raise Exception("missing TF %s -> %s" % (target, source))
        if key not in self._transforms:
            return _FakeTransform()
        return self._transforms[key]


class TestValidateBody(unittest.TestCase):
    def test_sphere_body(self):
        body = _validate_body(
            {"name": "wrist", "frame": "elfin_link6", "type": "sphere",
             "center": [0, 0, 0], "radius": 0.1},
            0.02,
        )
        self.assertEqual(body["type"], "sphere")
        self.assertAlmostEqual(body["radius"], 0.12)

    def test_box_body(self):
        body = _validate_body(
            {"name": "panel", "frame": "suction_panel", "type": "box",
             "center": [0, 0, 0], "size": [0.2, 0.3, 0.2]},
            0.03,
        )
        self.assertEqual(body["type"], "box")
        self.assertAlmostEqual(body["half_size"][0], 0.13)


class TestRobotSelfPointFilter(unittest.TestCase):
    def test_disabled_passthrough(self):
        filt = RobotSelfPointFilter(enabled=False)
        points = [(1.0, 2.0, 3.0), (0.5, 0.5, 0.5)]
        out = filt.filter_points(points, _FakeTfBuffer())
        self.assertEqual(out, points)
        self.assertEqual(filt.last_stats["dropped_self"], 0)

    def test_sphere_filters_center_point(self):
        filt = RobotSelfPointFilter(
            enabled=True,
            base_frame="base",
            bodies=[{
                "name": "wrist",
                "frame": "elfin_link6",
                "type": "sphere",
                "center": [0, 0, 0],
                "radius": 0.15,
            }],
            padding=0.0,
        )
        tf = _FakeTfBuffer({("base", "elfin_link6"): _FakeTransform()})
        points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        out = filt.filter_points(points, tf)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0], (1.0, 0.0, 0.0))
        self.assertEqual(filt.last_stats["dropped_self"], 1)
        indexed_out, kept_indices = filt.filter_points_with_indices(points, tf)
        self.assertEqual(indexed_out, out)
        self.assertEqual(kept_indices, [1])

    def test_box_filters_interior(self):
        filt = RobotSelfPointFilter(
            enabled=True,
            base_frame="base",
            bodies=[{
                "name": "box",
                "frame": "eef",
                "type": "box",
                "center": [0, 0, 0],
                "size": [0.2, 0.2, 0.2],
            }],
            padding=0.0,
        )
        tf = _FakeTfBuffer({("base", "eef"): _FakeTransform()})
        points = [(0.05, 0.05, 0.05), (0.5, 0.5, 0.5)]
        out = filt.filter_points(points, tf)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0], (0.5, 0.5, 0.5))

    def test_tf_missing_skips_body_keeps_points(self):
        filt = RobotSelfPointFilter(
            enabled=True,
            base_frame="base",
            bodies=[{
                "name": "wrist",
                "frame": "missing_link",
                "type": "sphere",
                "center": [0, 0, 0],
                "radius": 0.5,
            }],
            padding=0.0,
        )
        tf = _FakeTfBuffer(missing=["missing_link"])
        points = [(0.0, 0.0, 0.0)]
        out = filt.filter_points(points, tf)
        self.assertEqual(len(out), 1)
        self.assertIn("missing_link", filt.last_stats["tf_missing_links"])

    def test_point_intersection_supports_obstacle_extent(self):
        filt = RobotSelfPointFilter(
            enabled=True,
            base_frame="base",
            bodies=[{
                "name": "link1", "frame": "link1", "type": "sphere",
                "center": [0, 0, 0], "radius": 0.15,
            }],
            padding=0.0,
        )
        tf = _FakeTfBuffer({("base", "link1"): _FakeTransform()})
        point = (0.20, 0.0, 0.0)
        intersects, missing = filt.point_intersects_robot(point, tf)
        self.assertFalse(intersects)
        self.assertEqual(missing, [])
        intersects, missing = filt.point_intersects_robot(
            point, tf, extra_padding=0.07)
        self.assertTrue(intersects)
        self.assertEqual(missing, [])

    def test_point_intersection_reports_missing_tf(self):
        filt = RobotSelfPointFilter(
            enabled=True,
            base_frame="base",
            bodies=[{
                "name": "wrist", "frame": "missing_link", "type": "sphere",
                "center": [0, 0, 0], "radius": 0.15,
            }],
            padding=0.0,
        )
        intersects, missing = filt.point_intersects_robot(
            (1.0, 0.0, 0.0), _FakeTfBuffer(missing=["missing_link"]))
        self.assertFalse(intersects)
        self.assertEqual(missing, ["missing_link"])

    def test_does_not_modify_input_list(self):
        filt = RobotSelfPointFilter(
            enabled=True,
            base_frame="base",
            bodies=[{
                "name": "wrist",
                "frame": "link",
                "type": "sphere",
                "center": [0, 0, 0],
                "radius": 0.2,
            }],
            padding=0.0,
        )
        tf = _FakeTfBuffer({("base", "link"): _FakeTransform()})
        points = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]
        original_len = len(points)
        out = filt.filter_points(points, tf)
        self.assertEqual(len(points), original_len)
        self.assertIsNot(out, points)

    def test_from_config_dict(self):
        filt = RobotSelfPointFilter.from_config_dict({
            "self_filter": {
                "enabled": True,
                "base_frame": "elfin_base_link",
                "padding": 0.01,
                "bodies": [{
                    "name": "cam",
                    "frame": "camera_link",
                    "type": "box",
                    "center": [0, 0, 0],
                    "size": [0.1, 0.1, 0.1],
                }],
            }
        })
        self.assertTrue(filt.enabled)
        self.assertEqual(len(filt.bodies), 1)


if __name__ == "__main__":
    unittest.main()
