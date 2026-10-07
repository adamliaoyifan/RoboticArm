#!/usr/bin/env python3
"""ROS-free tests for cargo-view integrate selection and occupancy scoring."""

import math
import time
import unittest

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.cargo_view_integration import (
    MAP_MEASURED,
    MAP_RELEASE_INTEGRATED,
    measured_identity,
    release_identity,
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
    box_point_mask,
    footprint_sensor_coverage,
    footprint_sensor_height,
    occupancy_height_matches,
    optical_axis_angle_rad,
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


class TestOccupancyHeight(unittest.TestCase):

    def _surface(self, height_rows, confidence="sensor"):
        return {
            "resolution": 0.05,
            "nx": 1,
            "ny": 2,
            "origin_local": [0.0, 0.0],
            "state": [["occupied", "occupied"]],
            "confidence": [[confidence, confidence]],
            "height": [[0.0, 0.0]],
            "height_above_floor": height_rows,
        }

    def test_median_within_one_voxel_matches_box_height(self):
        surface = self._surface([[0.25, 0.30]])
        height = footprint_sensor_height(surface, (0.0, 0.0, 0.2, 0.2))
        self.assertAlmostEqual(height, 0.275)
        self.assertTrue(occupancy_height_matches(height, 0.25, 0.05))

    def test_hover_above_one_voxel_does_not_match(self):
        surface = self._surface([[0.35, 0.35]])
        height = footprint_sensor_height(surface, (0.0, 0.0, 0.2, 0.2))
        self.assertFalse(occupancy_height_matches(height, 0.25, 0.05))

    def test_v2_height_above_floor_beats_v1_height(self):
        surface = self._surface([[0.25, 0.25]])
        surface["height"] = [[0.90, 0.90]]
        height = footprint_sensor_height(surface, (0.0, 0.0, 0.2, 0.2))
        self.assertAlmostEqual(height, 0.25)

    def test_optical_axis_straight_ahead_is_zero(self):
        import math
        angle, distance = optical_axis_angle_rad(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 2.0))
        self.assertAlmostEqual(angle, 0.0)
        self.assertAlmostEqual(distance, 2.0)
        side, _ = optical_axis_angle_rad(
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(side, math.pi / 2.0)

    def test_geometry_cells_are_ignored(self):
        surface = self._surface([[0.25, 0.25]], confidence="geometry")
        self.assertIsNone(
            footprint_sensor_height(surface, (0.0, 0.0, 0.2, 0.2)))


class TestBoxPointMask(unittest.TestCase):

    def test_keeps_the_lid_and_drops_the_wall(self):
        import numpy as np
        center = (0.2, -0.1, 0.655)
        size = (0.55, 0.40, 0.25)
        lid = np.array([[0.2, -0.1, 0.78], [0.40, 0.0, 0.70]])
        wall = np.array([[1.4, -0.1, 0.78]])
        points = np.vstack((lid, wall))
        mask = box_point_mask(points, center, size, yaw=0.0, margin=0.0)
        self.assertEqual(mask.tolist(), [True, True, False])

    def test_yaw_swaps_the_narrow_axis(self):
        import numpy as np
        center = (0.0, 0.0, 0.0)
        size = (0.40, 0.20, 0.25)
        point = np.array([[0.0, 0.15, 0.0]])
        self.assertFalse(bool(box_point_mask(point, center, size, yaw=0.0)[0]))
        self.assertTrue(bool(box_point_mask(
            point, center, size, yaw=math.pi / 2.0)[0]))

    def test_margin_admits_a_near_miss_only(self):
        import numpy as np
        center = (0.0, 0.0, 0.0)
        size = (0.40, 0.40, 0.40)
        point = np.array([[0.22, 0.0, 0.0]])
        self.assertFalse(bool(box_point_mask(point, center, size)[0]))
        self.assertTrue(bool(box_point_mask(
            point, center, size, margin=0.05)[0]))

    def test_empty_cloud_stays_empty(self):
        import numpy as np
        mask = box_point_mask([], (0.0, 0.0, 0.0), (0.5, 0.4, 0.25))
        self.assertEqual(mask.shape, (0,))
        self.assertEqual(mask.dtype, np.dtype(bool))


class TestUntrackedConcat(unittest.TestCase):

    def test_cargo_only(self):
        xyz = untracked_xyz([(1.0, 0.0, 0.0)], [(9.0, 0.0, 0.0)], False)
        self.assertEqual(len(xyz), 1)
        self.assertAlmostEqual(xyz[0, 0], 1.0)

    def test_include_obstacle(self):
        xyz = untracked_xyz([(1.0, 0.0, 0.0)], [(9.0, 0.0, 0.0)], True)
        self.assertEqual(len(xyz), 2)


class TestMapIdentity(unittest.TestCase):

    def test_measured_keeps_the_sensor_stamp(self):
        stamp, source = measured_identity(38_000_000_000)
        self.assertEqual(stamp, 38_000_000_000)
        self.assertEqual(source, MAP_MEASURED)

    def test_measured_does_not_invent_a_stamp(self):
        stamp, source = measured_identity(0)
        self.assertEqual(stamp, 0)
        self.assertEqual(source, MAP_MEASURED)

    def test_release_uses_the_commit_clock(self):
        stamp, source = release_identity(38_350_000_000)
        self.assertEqual(stamp, 38_350_000_000)
        self.assertEqual(source, MAP_RELEASE_INTEGRATED)

    def test_release_before_the_clock_starts_stays_unstamped(self):
        stamp, source = release_identity(0)
        self.assertEqual(stamp, 0)
        self.assertEqual(source, MAP_RELEASE_INTEGRATED)


class TestContainerPoseMoved(unittest.TestCase):

    def test_small_drift_is_not_a_change_and_a_shift_is(self):
        from luggage_perception.cargo_view_integration import (
            container_pose_moved,
        )

        self.assertFalse(container_pose_moved(
            {"translation": [0, 0, 0], "rotation_rpy": [0, 0, 0]},
            {"translation": [0.001, 0, 0], "rotation_rpy": [0, 0, 0]}))
        self.assertTrue(container_pose_moved(
            {"translation": [-1.5, 0.6, 0], "rotation_rpy": [0, 0, 3.14]},
            {"translation": [-1.4, 0.6, 0], "rotation_rpy": [0, 0, 3.14]}))


if __name__ == "__main__":
    unittest.main()
