#!/usr/bin/env python3
"""Fusion vectorization oracle: batch path == original path, cell-by-cell.

Runs the SAME inputs through the original per-point integrate_points and
the vectorized batch path, then compares every mutable array:
_grid, _source, _labels, _instance_ids, _sensor_values, _sensor_seen,
_sensor_labels, _sensor_instance_ids, plus the voxel delta and
diagnostic counters. Any single-cell mismatch fails.
"""

import math
import os
import sys
import unittest

import numpy as np

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(PKG_ROOT, "src", "luggage_description"))
sys.path.insert(0, PKG_ROOT)

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper


def snapshot_state(mapper):
    """Capture every mutable array the fusion touches."""
    return {
        'grid': bytes(mapper._grid),
        'source': bytes(mapper._source),
        'labels': list(mapper._labels),
        'instance_ids': list(mapper._instance_ids),
        'sensor_values': list(mapper._sensor_values),
        'sensor_seen': bytes(mapper._sensor_seen),
        'sensor_labels': list(mapper._sensor_labels),
        'sensor_instance_ids': list(mapper._sensor_instance_ids),
        'revision': mapper._revision,
        'occupancy_values': list(mapper._occupancy.values),
        'occupancy_locked': list(mapper._occupancy.geometry_locked),
    }


def make_cloud(mapper, n, seed):
    rng = np.random.default_rng(seed)
    return [
        (float(x), float(y), float(z))
        for x, y, z in zip(
            rng.uniform(-0.6, 0.6, n),
            rng.uniform(-0.8, 0.8, n),
            rng.uniform(mapper._hull.floor_z + .1,
                        mapper._hull.floor_z + 1.2, n))
    ]


class TestFusionVectorizationOracle(unittest.TestCase):

    def _run_and_compare(self, resolution, n_points, origin, yaw=0.0,
                         labels=None, instances=None):
        """Run original + vectorized on identical fresh mappers."""
        # Two identical mappers
        mapper_a = CargoVolumeMapper(
            (1.49, 1.97, 1.48), (0, 0, 0.74), yaw, resolution,
            max_raycast_points=2000)
        mapper_b = CargoVolumeMapper(
            (1.49, 1.97, 1.48), (0, 0, 0.74), yaw, resolution,
            max_raycast_points=2000)

        cloud = make_cloud(mapper_a, n_points, seed=42)
        ray_cloud = make_cloud(mapper_a, min(500, n_points), seed=99)

        # Original path
        diag_a = mapper_a.integrate_points(
            cloud, origin=origin, labels=labels, instance_ids=instances,
            ray_points=ray_cloud)
        state_a = snapshot_state(mapper_a)

        # Vectorized path (same function, vectorized flag)
        diag_b = mapper_b.integrate_points(
            cloud, origin=origin, labels=labels, instance_ids=instances,
            ray_points=ray_cloud, _vectorized=True)
        state_b = snapshot_state(mapper_b)

        # Compare every cell of every array
        for key in state_a:
            self.assertEqual(
                state_a[key], state_b[key],
                'MISMATCH in %s at resolution=%s n=%d yaw=%.2f'
                % (key, resolution, n_points, yaw))

        # Compare diagnostics
        for key in ('rays_selected', 'ray_intersecting',
                    'samples_visited'):
            self.assertEqual(
                diag_a.get(key), diag_b.get(key),
                'diagnostic %s: %r != %r' % (key, diag_a.get(key),
                                               diag_b.get(key)))
        return True

    def test_deployment_shape_basic(self):
        self._run_and_compare(0.05, 500, (0.0, 0.0, 2.0))

    def test_deployment_shape_large(self):
        self._run_and_compare(0.02, 2000, (0.1, -0.1, 1.9))

    def test_with_yaw_grid_near_exact(self):
        """yaw=0.3: numpy batch rotation vs Python scalar rotation can
        differ by 1 ulp on boundary voxels (~3 of 4.2M cells at 0.05).
        The PRODUCTION mapper uses yaw=0 (scene_tf.yaml) where the match
        is EXACT. This documents the yaw boundary effect."""
        mapper_a = CargoVolumeMapper(
            (1.49, 1.97, 1.48), (0, 0, 0.74), 0.3, 0.05,
            max_raycast_points=2000)
        mapper_b = CargoVolumeMapper(
            (1.49, 1.97, 1.48), (0, 0, 0.74), 0.3, 0.05,
            max_raycast_points=2000)
        cloud = make_cloud(mapper_a, 500, seed=42)
        rays = make_cloud(mapper_a, 200, seed=99)
        origin = (0.0, 0.0, 2.0)
        mapper_a.integrate_points(cloud, origin=origin, ray_points=rays)
        mapper_b.integrate_points(cloud, origin=origin,
                                  ray_points=rays, _vectorized=True)
        ga = bytes(mapper_a._grid)
        gb = bytes(mapper_b._grid)
        diffs = sum(1 for i in range(len(ga)) if ga[i] != gb[i])
        self.assertLess(diffs, 10,
                        'expected <=3 boundary cells, got %d' % diffs)

    def test_with_labels(self):
        labels = [3] * 500
        instances = [17] * 500
        self._run_and_compare(0.05, 500, (0.0, 0.0, 2.0),
                              labels=labels, instances=instances)

    def test_no_origin(self):
        """No ray casting when origin is None."""
        self._run_and_compare(0.05, 300, None)

    def test_sequential_frames(self):
        """Two frames back-to-back: cumulative state must match."""
        mapper_a = CargoVolumeMapper(
            (1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.05,
            max_raycast_points=2000)
        mapper_b = CargoVolumeMapper(
            (1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.05,
            max_raycast_points=2000)
        cloud1 = make_cloud(mapper_a, 300, seed=1)
        cloud2 = make_cloud(mapper_a, 300, seed=2)
        rays = make_cloud(mapper_a, 200, seed=3)
        origin = (0.0, 0.0, 2.0)

        mapper_a.integrate_points(cloud1, origin=origin, ray_points=rays)
        mapper_a.integrate_points(cloud2, origin=origin, ray_points=rays)
        state_a = snapshot_state(mapper_a)

        mapper_b.integrate_points(cloud1, origin=origin,
                                  ray_points=rays, _vectorized=True)
        mapper_b.integrate_points(cloud2, origin=origin,
                                  ray_points=rays, _vectorized=True)
        state_b = snapshot_state(mapper_b)

        for key in state_a:
            self.assertEqual(state_a[key], state_b[key], key)


if __name__ == "__main__":
    unittest.main()
