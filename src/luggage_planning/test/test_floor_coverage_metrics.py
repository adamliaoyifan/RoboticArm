#!/usr/bin/env python3
"""Deterministic tests for container inner-floor FOV coverage metrics."""

from __future__ import division

import os
import sys
import unittest


TEST_ROOT = os.path.dirname(os.path.abspath(__file__))

from luggage_planning.interior_view_scorer import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    CameraIntrinsics,
    ContainerFloor,
    RaycastConfig,
    SparseOccupancyGrid,
    floor_coverage_metrics,
)


def wide_intrinsics():
    """Return a 9x9 pinhole camera whose frustum spans the test container."""
    return CameraIntrinsics(width=9, height=9, fx=9.0, fy=9.0)


def container_floor():
    """Inner floor of a 0.4 m cube centred at z=0.5 (floor plane at z=0.3)."""
    return ContainerFloor(
        center_base=(0.0, 0.0, 0.5),
        yaw=0.0,
        inner_size=(0.4, 0.4, 0.4),
        resolution=0.1,
    )


def box_grid(cells=None, default_state=FREE):
    """Occupancy grid sharing the container_floor() cell indexing."""
    return SparseOccupancyGrid(
        origin=(-0.2, -0.2, 0.3),
        shape=(4, 4, 4),
        resolution=0.1,
        cells=cells,
        default_state=default_state,
    )


def overhead_candidate():
    return {
        "camera_xyz": (0.0, 0.0, 1.0),
        "look_at": (0.0, 0.0, 0.0),
        "camera_up": (0.0, 0.0, 1.0),
    }


def metrics(candidate, occupancy=None, floor=None):
    return floor_coverage_metrics(
        candidate,
        occupancy if occupancy is not None else box_grid(),
        wide_intrinsics(),
        floor if floor is not None else container_floor(),
        RaycastConfig(max_range=2.0, pixel_stride=1),
    )


class TestContainerFloor(unittest.TestCase):
    def test_cell_grid_matches_inner_dimensions(self):
        floor = container_floor()
        self.assertEqual((floor.nx, floor.ny), (4, 4))
        self.assertEqual(floor.cell_count, 16)
        self.assertAlmostEqual(floor.plane_z, 0.3)

    def test_points_outside_the_inner_box_have_no_cell(self):
        floor = container_floor()
        self.assertIsNone(floor.world_to_cell((0.5, 0.0, 0.3)))
        self.assertEqual(floor.world_to_cell((0.0, 0.0, 0.3)), (2, 2))

    def test_cell_center_round_trips_through_world_to_cell(self):
        floor = container_floor()
        for ix in range(floor.nx):
            for iy in range(floor.ny):
                center = floor.cell_center_base(ix, iy)
                self.assertEqual(floor.world_to_cell(center), (ix, iy))

    def test_yaw_rotates_the_cell_lattice(self):
        floor = ContainerFloor(
            center_base=(0.0, 0.0, 0.5),
            yaw=1.5707963267948966,
            inner_size=(0.4, 0.4, 0.4),
            resolution=0.1,
        )
        center = floor.cell_center_base(0, 0)
        # Local (-x, -y) corner maps to base (+y, -x) under a +90 deg yaw.
        self.assertAlmostEqual(center[0], 0.15)
        self.assertAlmostEqual(center[1], -0.15)


class TestFloorCoverage(unittest.TestCase):
    def test_overhead_view_covers_the_entire_floor(self):
        result = metrics(overhead_candidate())
        self.assertEqual(result["floor_cells_total"], 16)
        self.assertEqual(result["floor_cells_covered"], 16)
        self.assertAlmostEqual(result["floor_xy_coverage"], 1.0)
        self.assertAlmostEqual(result["inside_container_fov_ratio"], 1.0)
        self.assertAlmostEqual(result["outside_container_ratio"], 0.0)

    def test_oblique_view_covers_less_than_overhead(self):
        overhead = metrics(overhead_candidate())
        oblique = metrics({
            "camera_xyz": (0.0, -0.55, 0.62),
            "look_at": (0.0, 0.0, 0.32),
            "camera_up": (0.0, 0.0, 1.0),
        })
        self.assertLess(
            oblique["floor_xy_coverage"], overhead["floor_xy_coverage"])

    def test_camera_pointing_away_sees_nothing(self):
        result = metrics({
            "camera_xyz": (0.0, 0.0, 1.0),
            "look_at": (0.0, 0.0, 2.0),
            "camera_up": (0.0, 1.0, 0.0),
        })
        self.assertAlmostEqual(result["floor_xy_coverage"], 0.0)
        self.assertAlmostEqual(result["inside_container_fov_ratio"], 0.0)
        self.assertAlmostEqual(result["outside_container_ratio"], 1.0)
        self.assertEqual(result["rays_hit_floor"], 0)

    def test_occupancy_in_a_foreign_column_blocks_coverage(self):
        ceiling = {
            (ix, iy, 3): OCCUPIED
            for ix in range(4) for iy in range(4)
        }
        clear = metrics(overhead_candidate())
        occluded = metrics(overhead_candidate(), occupancy=box_grid(ceiling))
        self.assertLess(
            occluded["floor_xy_coverage"], clear["floor_xy_coverage"])
        self.assertTrue(occluded["blocked_cells"])
        self.assertFalse(
            set(occluded["blocked_cells"])
            & set(occluded["covered_cells"]))

    def test_occupancy_in_its_own_column_still_counts_as_covered(self):
        # The ray straight below the camera is stopped by cargo standing in
        # the very cell it targets; observing that stack still observes the
        # column, so the cell must stay covered.
        stack = {(2, 2, iz): OCCUPIED for iz in range(4)}
        result = metrics(overhead_candidate(), occupancy=box_grid(stack))
        self.assertIn((2, 2), result["covered_cells"])

    def test_unknown_columns_are_reported_as_gain(self):
        unknown = metrics(
            overhead_candidate(),
            occupancy=box_grid(default_state=UNKNOWN))
        self.assertAlmostEqual(
            unknown["floor_unknown_gain"], unknown["floor_xy_coverage"])
        known = metrics(overhead_candidate())
        self.assertAlmostEqual(known["floor_unknown_gain"], 0.0)

    def test_results_are_deterministic(self):
        first = metrics(overhead_candidate())
        second = metrics(overhead_candidate())
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
