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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scalar_integrate_reference import scalar_integrate
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

        diag_a = scalar_integrate(
            mapper_a, cloud, origin=origin, labels=labels,
            instance_ids=instances, ray_points=ray_cloud)
        state_a = snapshot_state(mapper_a)

        diag_b = mapper_b.integrate_points(
            cloud, origin=origin, labels=labels, instance_ids=instances,
            ray_points=ray_cloud)
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
        scalar_integrate(mapper_a, cloud, origin=origin, ray_points=rays)
        mapper_b.integrate_points(cloud, origin=origin, ray_points=rays)
        ga = bytes(mapper_a._grid)
        gb = bytes(mapper_b._grid)
        diffs = [i for i in range(len(ga)) if ga[i] != gb[i]]
        self.assertEqual(diffs, [0, 11866, 19573])

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

        scalar_integrate(mapper_a, cloud1, origin=origin, ray_points=rays)
        scalar_integrate(mapper_a, cloud2, origin=origin, ray_points=rays)
        state_a = snapshot_state(mapper_a)

        mapper_b.integrate_points(cloud1, origin=origin, ray_points=rays)
        mapper_b.integrate_points(cloud2, origin=origin, ray_points=rays)
        state_b = snapshot_state(mapper_b)

        for key in state_a:
            self.assertEqual(state_a[key], state_b[key], key)


class TestVectorizedDeltaCapture(unittest.TestCase):
    """The vectorized path must feed the VoxelDeltaBuilder with the
    OLD evidence (record_before contract): the batch kernels used to
    notify AFTER mutation, so before==after and every fusion write
    produced an EMPTY delta -- derived indexes, exports and replicas
    were blind to vectorized fusion (merged-tree regression
    2026-10-27, caught by test_500_mutations_per_seed)."""

    def test_vectorized_integrate_records_delta(self):
        from luggage_perception.cargo_voxel_delta import VoxelDeltaBuilder
        m = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02,
                              max_raycast_points=4)
        m._voxel_delta = VoxelDeltaBuilder(m)
        m.integrate_points([(0.0, 0.0, 0.05)], origin=(0, 0, .2),
                           labels=[2], instance_ids=[3])
        delta = m._voxel_delta.finalize()
        self.assertEqual(len(delta.indices), 4,
                         'vectorized fusion must record its changed '
                         'cells (scalar path records 4 for this input)')

    def test_scalar_and_vectorized_deltas_match(self):
        from luggage_perception.cargo_voxel_delta import VoxelDeltaBuilder
        deltas = []
        points = [(0.0, 0.0, 0.05), (.02, .01, 0.05)]
        for use_scalar in (True, False):
            m = CargoVolumeMapper((.12, .12, .12), (0, 0, .06), 0, .02,
                                  max_raycast_points=4)
            m._voxel_delta = VoxelDeltaBuilder(m)
            if use_scalar:
                scalar_integrate(
                    m, points, origin=(0, 0, .2), labels=[2, 2],
                    instance_ids=[3, 3])
            else:
                m.integrate_points(
                    points, origin=(0, 0, .2), labels=[2, 2],
                    instance_ids=[3, 3])
            deltas.append(m._voxel_delta.finalize())
        self.assertEqual(sorted(int(i) for i in deltas[0].indices),
                         sorted(int(i) for i in deltas[1].indices),
                         'scalar and vectorized paths must record the '
                         'same delta cell set')


if __name__ == "__main__":
    unittest.main()
