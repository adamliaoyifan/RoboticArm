#!/usr/bin/env python3
"""Unit tests for DSI-C1 scoring. No ROS."""

import unittest

from luggage_perception.eval.dsim_c1_scoring import (
    camera_native_clouds,
    detector_hit_ratio,
    parse_publisher_count,
    score_c1,
)


def _ok_payload(**overrides):
    payload = {
        "rates": {
            "color": {"hz": 15.0},
            "depth_mm": {"hz": 15.1},
        },
        "identity": {"four_product_exact_ratio": 0.99},
        "d34": {
            "d3_emission_over_rgb": 0.92,
            "d4_paired_depth_over_emitted": 0.98,
            "d4_filter": {
                "exact_join_joined_over_depth": 0.97,
                "stale_drop_ratio": 0.01,
            },
        },
        "detector_stream_stats": [
            {"raw_lookup": {"raw_lookup_status": "hit"}} for _ in range(20)
        ],
        "pointcloud2_topics": [
            "/livox/lidar [sensor_msgs/msg/PointCloud2]",
            "/luggage/semantic/cargo_points [sensor_msgs/msg/PointCloud2]",
            "/luggage/semantic/obstacle_points [sensor_msgs/msg/PointCloud2]",
        ],
        "clock_info": "Type: rosgraph_msgs/msg/Clock\nPublisher count: 1\n",
    }
    payload.update(overrides)
    return payload


class TestC1Scoring(unittest.TestCase):
    def test_pass_payload(self):
        verdict = score_c1(_ok_payload())
        self.assertTrue(verdict["pass"], verdict["failures"])
        self.assertEqual(verdict["clock_publishers"], 1)
        self.assertEqual(verdict["detector"]["ratio"], 1.0)

    def test_camera_cloud_fails(self):
        payload = _ok_payload(pointcloud2_topics=[
            "/camera/depth/points [sensor_msgs/msg/PointCloud2]",
            "/livox/lidar [sensor_msgs/msg/PointCloud2]",
        ])
        verdict = score_c1(payload)
        self.assertFalse(verdict["pass"])
        self.assertTrue(any("PointCloud2" in f for f in verdict["failures"]))
        self.assertEqual(
            camera_native_clouds(payload["pointcloud2_topics"]),
            ["/camera/depth/points"])

    def test_rate_and_join_bars(self):
        payload = _ok_payload()
        payload["rates"]["color"]["hz"] = 12.0
        self.assertFalse(score_c1(payload)["pass"])
        payload = _ok_payload()
        payload["d34"]["d3_emission_over_rgb"] = 0.50
        self.assertFalse(score_c1(payload)["pass"])
        payload = _ok_payload()
        payload["d34"]["d4_filter"]["stale_drop_ratio"] = 0.08
        self.assertFalse(score_c1(payload)["pass"])

    def test_detector_ratio(self):
        stats = (
            [{"raw_lookup": {"raw_lookup_status": "hit"}}] * 19
            + [{"raw_lookup": {"raw_lookup_status": "miss"}}]
        )
        rec = detector_hit_ratio(stats)
        self.assertEqual(rec["joined_cargo"], 20)
        self.assertAlmostEqual(rec["ratio"], 0.95)
        self.assertIsNone(detector_hit_ratio([])["ratio"])
        payload = _ok_payload(detector_stream_stats=[])
        self.assertFalse(score_c1(payload)["pass"])

    def test_clock_count(self):
        self.assertEqual(parse_publisher_count("Publisher count: 2"), 2)
        payload = _ok_payload(clock_info="Publisher count: 2\n")
        self.assertFalse(score_c1(payload)["pass"])


if __name__ == "__main__":
    unittest.main()
