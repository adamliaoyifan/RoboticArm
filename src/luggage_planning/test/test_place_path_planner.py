#!/usr/bin/env python3
"""A0 multi-slot trajectory selector (launch-free)."""

import unittest

from luggage_description.container_geometry import cuboid_from_inner_size
from luggage_planning.occupancy_place_paths import (
    DEFAULT_WEIGHTS,
    OccupancyMismatch,
    PLACE_PATH_INFEASIBLE,
    select_trajectory,
)
from luggage_planning.place_path_planner import PlacePathPlanner
from luggage_planning.pose import MotionSegment, Point, Pose, Quaternion
from luggage_planning.place_path_planner import apply_variant_to_segment


def _surface(nx=20, ny=10, geom=None, rev=3, height=0.0, state="free"):
    inner = [1.0, 0.5, 1.0]
    desc = cuboid_from_inner_size(inner, floor_z=0.0).descriptor()
    return {
        "geometry_hash": desc["geometry_hash"] if geom is None else geom,
        "geometry_descriptor": desc,
        "map_revision": rev,
        "resolution": 0.05,
        "nx": nx,
        "ny": ny,
        "inner_size": inner,
        "floor_z": 0.0,
        "center_base": [0.0, 0.0, 0.5],
        "yaw": 0.0,
        "height": [[height] * ny for _ in range(nx)],
        "state": [[state] * ny for _ in range(nx)],
        "confidence": [["none"] * ny for _ in range(nx)],
    }


def _occupy_ix(surface, ix, height=0.40):
    for iy in range(surface["ny"]):
        surface["state"][ix][iy] = "occupied"
        surface["height"][ix][iy] = height
        surface["confidence"][ix][iy] = "sensor"


class TestPlacePathPlanner(unittest.TestCase):
    def test_raised_wins_over_blocked_direct(self):
        surface = _surface()
        _occupy_ix(surface, 10, 0.40)
        planner = PlacePathPlanner(
            weights=DEFAULT_WEIGHTS, inflate_m=0.0, arm_radius=0.0,
            max_candidates_per_slot=4)
        winner, rows, reason, _snap = planner.plan(
            surface,
            slots=[{"index": 0, "score": 1.0, "target": (0.35, 0.0, 0.50)}],
            portal=(-0.35, 0.0, 0.50),
            payload_wdh=(0.08, 0.08, 0.15),
            expected_hash=surface["geometry_hash"],
            expected_revision=3,
        )
        self.assertEqual(reason, "")
        self.assertIsNotNone(winner)
        self.assertTrue(
            winner["method"].startswith("clear"), winner)
        methods = [row["method"] for row in rows]
        self.assertIn("direct", methods)
        direct = next(row for row in rows if row["method"] == "direct")
        self.assertFalse(direct["feasible"])

    def test_second_slot_when_first_blocked(self):
        surface = _surface()
        _occupy_ix(surface, 10, 0.90)
        planner = PlacePathPlanner(
            weights=DEFAULT_WEIGHTS, inflate_m=0.0, arm_radius=0.0,
            max_candidates_per_slot=3)
        winner, rows, reason, _snap = planner.plan(
            surface,
            slots=[
                {"index": 0, "score": 0.9, "target": (0.35, 0.0, 0.50)},
                {"index": 1, "score": 0.4, "target": (-0.20, 0.0, 0.80)},
            ],
            portal=(-0.35, 0.0, 0.80),
            payload_wdh=(0.08, 0.08, 0.15),
        )
        self.assertEqual(reason, "")
        self.assertIsNotNone(winner)
        self.assertEqual(winner["slot_index"], 1)
        tried = {row["slot_index"] for row in rows}
        self.assertEqual(tried, {0, 1})

    def test_w_placement_flips_winner(self):
        cheap_bad = {
            "slot_index": 1,
            "slot_score": 0.2,
            "method": "direct",
            "waypoints": [(0.0, 0.0, 1.0), (0.4, 0.0, 1.0)],
            "feasible": True,
            "min_clearance": 0.10,
            "cartesian_fraction": 1.0,
        }
        longer_good = {
            "slot_index": 0,
            "slot_score": 1.0,
            "method": "offset",
            "waypoints": [(0.0, 0.0, 1.0), (0.3, 0.2, 1.0), (0.6, 0.0, 1.0)],
            "feasible": True,
            "min_clearance": 0.10,
            "cartesian_fraction": 1.0,
        }
        w_on = dict(DEFAULT_WEIGHTS)
        w_on["w_placement"] = 1.0
        w_on["w_efficiency"] = 0.5
        winner_on, _, _ = select_trajectory(
            [dict(cheap_bad), dict(longer_good)], w_on)
        self.assertEqual(winner_on["slot_index"], 0)
        w_off = dict(DEFAULT_WEIGHTS)
        w_off["w_placement"] = 0.0
        w_off["w_efficiency"] = 5.0
        winner_off, _, _ = select_trajectory(
            [dict(cheap_bad), dict(longer_good)], w_off)
        self.assertEqual(winner_off["slot_index"], 1)

    def test_probe_low_fraction_emits_second_method(self):
        surface = _surface()
        seen = []

        def probe(row):
            seen.append(row["method"])
            row["ik_ok"] = True
            row["cartesian_fraction"] = (
                0.50 if row["method"] == "direct" else 1.0)

        planner = PlacePathPlanner(
            weights=DEFAULT_WEIGHTS, inflate_m=0.0, arm_radius=0.0,
            cartesian_min_fraction=0.95)
        winner, rows, reason, _snap = planner.plan(
            surface,
            slots=[{"index": 0, "score": 1.0, "target": (0.3, 0.0, 0.6)}],
            portal=(-0.3, 0.0, 0.6),
            payload_wdh=(0.08, 0.08, 0.10),
            probe_fn=probe,
        )
        self.assertGreaterEqual(len(rows), 2)
        self.assertEqual(reason, "")
        self.assertIsNotNone(winner)
        self.assertNotEqual(winner["method"], "direct")
        direct = next(row for row in rows if row["method"] == "direct")
        self.assertFalse(direct["feasible"])

    def test_mismatch_before_sweep(self):
        planner = PlacePathPlanner(inflate_m=0.0)
        with self.assertRaises(OccupancyMismatch):
            planner.plan(
                _surface(geom="g1"),
                slots=[{"index": 0, "score": 1.0, "target": (0.2, 0.0, 0.5)}],
                portal=(-0.2, 0.0, 0.5),
                payload_wdh=(0.08, 0.08, 0.10),
                expected_hash="nope",
            )

    def test_apply_variant_sets_waypoints(self):
        segment = MotionSegment(
            name="traverse", type="cartesian",
            target_pose=Pose(
                position=Point(x=1.0, y=0.0, z=1.0),
                orientation=Quaternion(x=1.0, w=0.0)))
        apply_variant_to_segment(
            segment, [(0.0, 0.0, 1.2), (0.5, 0.1, 1.2), (1.0, 0.0, 1.0)])
        self.assertEqual(len(segment.waypoints), 2)
        self.assertAlmostEqual(segment.waypoints[0].position.x, 0.0)
        self.assertAlmostEqual(segment.target_pose.position.x, 1.0)
        self.assertAlmostEqual(segment.target_pose.position.z, 1.0)

    def test_apply_variant_frame_convert(self):
        # Map-frame variant onto a world-frame segment: the consumer's
        # converter runs before anything is written, orientation kept.
        segment = MotionSegment(
            name="traverse", type="cartesian",
            target_pose=Pose(
                position=Point(x=9.0, y=9.0, z=9.0),
                orientation=Quaternion(x=1.0, w=0.0)))

        def map_to_world(xyz):
            return (xyz[0] + 10.0, xyz[1] - 1.0, xyz[2] + 0.5)

        apply_variant_to_segment(
            segment, [(0.0, 0.0, 1.2), (1.0, 2.0, 1.0)],
            frame_convert=map_to_world)
        self.assertEqual(len(segment.waypoints), 1)
        self.assertAlmostEqual(segment.waypoints[0].position.x, 10.0)
        self.assertAlmostEqual(segment.waypoints[0].position.y, -1.0)
        self.assertAlmostEqual(segment.waypoints[0].position.z, 1.7)
        self.assertAlmostEqual(segment.target_pose.position.x, 11.0)
        self.assertAlmostEqual(segment.target_pose.position.y, 1.0)
        self.assertAlmostEqual(segment.target_pose.position.z, 1.5)
        self.assertAlmostEqual(segment.target_pose.orientation.x, 1.0)


if __name__ == "__main__":
    unittest.main()
