#!/usr/bin/env python3
"""ROS-free tests for cargo-view integrate selection and occupancy scoring."""

import time
import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_view_integration import (
    DEFAULT_FRESHNESS_WINDOW_SEC,
    REASON_GEOMETRY_HASH_MISMATCH,
    REASON_NOT_SETTLED,
    REASON_REVISION_MISMATCH,
    REASON_SCHEMA_MISMATCH,
    REASON_STAMP_ALREADY_INTEGRATED,
    REASON_STAMP_STALE,
    REASON_VIEW_EMPTY,
    apply_integrate,
    floor_through_count,
    footprint_sensor_coverage,
    occupied_sensor_aabb,
    prepare_points,
    select_fresh_record,
    stamp_key,
    stamp_to_sec,
    untracked_xyz,
    validate_integrate_request,
)


def _mapper(res=0.05):
    return CargoVolumeMapper(
        inner_size=[1.0, 1.0, 1.0],
        center_base=[0.0, 0.0, 0.5],
        yaw=0.0,
        resolution=res,
        max_raycast_points=400,
    )


def _record(sec, nsec=0, n_points=10):
    return {
        "stamp_key": (int(sec), int(nsec)),
        "stamp_sec": float(sec) + float(nsec) * 1e-9,
        "n_points": int(n_points),
    }


class TestRequestValidation(unittest.TestCase):

    def test_settled_false(self):
        self.assertEqual(
            validate_integrate_request(
                False, 1, "h", 1, "h", 1),
            REASON_NOT_SETTLED)

    def test_schema_mismatch(self):
        self.assertEqual(
            validate_integrate_request(
                True, 99, "h", 1, "h", 1),
            REASON_SCHEMA_MISMATCH)

    def test_hash_mismatch_and_empty(self):
        self.assertEqual(
            validate_integrate_request(
                True, 1, "a", 1, "b", 1),
            REASON_GEOMETRY_HASH_MISMATCH)
        self.assertEqual(
            validate_integrate_request(
                True, 1, "", 1, "h", 1),
            REASON_GEOMETRY_HASH_MISMATCH)

    def test_revision_mismatch(self):
        self.assertEqual(
            validate_integrate_request(
                True, 1, "h", 1, "h", 2),
            REASON_REVISION_MISMATCH)

    def test_ok(self):
        self.assertIsNone(
            validate_integrate_request(True, 1, "h", 3, "h", 3))
        self.assertIsNone(
            validate_integrate_request(True, 0, "h", 3, "h", 3))


class TestFreshness(unittest.TestCase):

    def test_stale_when_outside_window(self):
        recs = [_record(1.0)]
        record, reason = select_fresh_record(
            recs, 1.5, DEFAULT_FRESHNESS_WINDOW_SEC, ())
        self.assertIsNone(record)
        self.assertEqual(reason, REASON_STAMP_STALE)

    def test_picks_nearest_inside_window(self):
        recs = [_record(1.00), _record(1.05, n_points=3)]
        record, reason = select_fresh_record(recs, 1.06, 0.20, ())
        self.assertIsNone(reason)
        self.assertEqual(record["stamp_key"], recs[1]["stamp_key"])
        self.assertEqual(record["n_points"], 3)

    def test_already_integrated(self):
        recs = [_record(2.0)]
        record, reason = select_fresh_record(
            recs, 2.0, 0.20, {recs[0]["stamp_key"]})
        self.assertIsNone(record)
        self.assertEqual(reason, REASON_STAMP_ALREADY_INTEGRATED)

    def test_stamp_helpers(self):
        self.assertEqual(stamp_key((3, 4)), (3, 4))
        self.assertAlmostEqual(stamp_to_sec((1, 500000000)), 1.5)


class TestPrepareAndIntegrate(unittest.TestCase):

    def test_empty_points(self):
        kept, n = prepare_points([], 0.05)
        self.assertEqual(n, 0)
        self.assertEqual(kept.shape[0], 0)

    def test_view_empty(self):
        mapper = _mapper()
        before = mapper.stats()["map_revision"]
        result = apply_integrate(mapper, [], origin=(0.0, 0.0, 1.0))
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason_code"], REASON_VIEW_EMPTY)
        self.assertEqual(mapper.stats()["map_revision"], before)

    def test_sensor_occupied_and_latency(self):
        mapper = _mapper()
        pts = [(0.0, 0.0, 0.5) for _ in range(20)]
        result = apply_integrate(
            mapper, pts, origin=(0.0, 0.0, 1.5))
        self.assertTrue(result["ok"], result)
        surface = mapper.surface_map_2d()
        occupied = [
            (ix, iy)
            for ix in range(surface["nx"])
            for iy in range(surface["ny"])
            if (surface["state"][ix][iy] == "occupied"
                and surface["confidence"][ix][iy] == "sensor")
        ]
        self.assertTrue(occupied)

    def test_two_clusters_accumulate(self):
        mapper = _mapper()
        apply_integrate(
            mapper, [( -0.3, 0.0, 0.5)] * 30, origin=(0.0, 0.0, 1.5))
        apply_integrate(
            mapper, [( 0.3, 0.0, 0.5)] * 30, origin=(0.0, 0.0, 1.5))
        surface = mapper.surface_map_2d()
        left, _, _ = footprint_sensor_coverage(
            surface, (-0.45, -0.1, -0.15, 0.1))
        right, _, _ = footprint_sensor_coverage(
            surface, (0.15, -0.1, 0.45, 0.1))
        self.assertGreater(left, 0.0)
        self.assertGreater(right, 0.0)

    def test_geometry_lock_survives_integrate(self):
        mapper = _mapper()
        mapper.mark_placed_box([0.0, 0.0, 0.5], [0.2, 0.2, 0.2])
        apply_integrate(
            mapper, [(0.0, 0.0, 0.6)] * 10, origin=(0.0, 0.0, 1.5))
        surface = mapper.surface_map_2d()
        confs = {
            surface["confidence"][ix][iy]
            for ix in range(surface["nx"])
            for iy in range(surface["ny"])
            if surface["state"][ix][iy] == "occupied"
        }
        self.assertTrue(confs & {"geometry", "sensor"})

    def test_latency_100k_under_two_seconds(self):
        mapper = _mapper(res=0.05)
        import numpy as np
        rng = np.random.RandomState(0)
        pts = rng.uniform(-0.2, 0.2, size=(100000, 3))
        pts[:, 2] = 0.5 + rng.uniform(-0.05, 0.05, size=100000)
        t0 = time.monotonic()
        result = apply_integrate(
            mapper, pts, origin=(0.0, 0.0, 1.5), voxel_size=0.05)
        elapsed = time.monotonic() - t0
        self.assertTrue(result["ok"], result)
        self.assertLess(elapsed, 2.0)
        self.assertLess(result["n_kept"], 20000)

    def test_floor_through_zero_on_occupied_patch(self):
        mapper = _mapper()
        apply_integrate(
            mapper,
            [(x, y, 0.4) for x in (-0.15, 0.0, 0.15)
             for y in (-0.1, 0.0, 0.1) for _ in range(8)],
            origin=(0.0, 0.0, 1.5))
        surface = mapper.surface_map_2d()
        patch = occupied_sensor_aabb(surface)
        self.assertIsNotNone(patch)
        count, summary = floor_through_count(
            surface, [0.2, 0.2, 0.2], patch)
        self.assertEqual(count, 0, summary)
        self.assertTrue(summary["synthetic_plan_only"])


class TestUntrackedConcat(unittest.TestCase):

    def test_cargo_only(self):
        xyz = untracked_xyz([(1.0, 0.0, 0.0)], [(9.0, 0.0, 0.0)], False)
        self.assertEqual(len(xyz), 1)
        self.assertAlmostEqual(xyz[0, 0], 1.0)

    def test_include_obstacle(self):
        xyz = untracked_xyz([(1.0, 0.0, 0.0)], [(9.0, 0.0, 0.0)], True)
        self.assertEqual(len(xyz), 2)


if __name__ == "__main__":
    unittest.main()
