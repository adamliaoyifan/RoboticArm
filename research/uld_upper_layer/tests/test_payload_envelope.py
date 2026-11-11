from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from payload_envelope import rotation_projection_range, box_projection_range, separation_lower_bound


class PayloadEnvelopeContracts(unittest.TestCase):
    def test_spherical_cap_extrema_and_full_rotation(self):
        vectors = np.eye(3)
        low, high = rotation_projection_range(vectors, [1., 0., 0.], .2)
        np.testing.assert_allclose(low, [np.cos(.2), -np.sin(.2), -np.sin(.2)])
        np.testing.assert_allclose(high, [1., np.sin(.2), np.sin(.2)])
        low, high = rotation_projection_range(vectors, [1., 0., 0.], np.pi)
        np.testing.assert_allclose(low, -np.ones(3)); np.testing.assert_allclose(high, np.ones(3))

    def test_contains_independent_random_rigid_rotations(self):
        rng = np.random.default_rng(720)
        vectors = rng.normal(size=(100, 3))
        axis = np.array([1., 2., 3.]); axis /= np.linalg.norm(axis)
        low, high = rotation_projection_range(vectors, axis, .3)
        for _ in range(250):
            direction = rng.normal(size=3); direction /= np.linalg.norm(direction)
            rotated = Rotation.from_rotvec(direction * rng.uniform(0., .3)).apply(vectors) @ axis
            self.assertTrue(np.all(rotated >= low - 1e-12))
            self.assertTrue(np.all(rotated <= high + 1e-12))

    def test_zero_error_is_exact_and_transverse_error_has_no_longitudinal_expansion(self):
        pose = np.eye(4); pose[:3, 3] = [3., 4., 5.]
        self.assertEqual(box_projection_range(pose, [2, 4, 6], [1, 0, 0], 0., 0.), (2., 4.))
        self.assertEqual(box_projection_range(pose, [2, 4, 6], [1, 0, 0], .1, 0., [1, 0, 0]), (2., 4.))
        np.testing.assert_allclose(box_projection_range(pose, [2, 4, 6], [0, 1, 0], .1, 0., [1, 0, 0]), [1.9, 6.1])

    def test_bound_can_be_tighter_than_isotropic_inflation_without_false_collision_claim(self):
        pose = np.eye(4); size = [.55, .4, .25]
        obstacle = np.array([[x, y, z] for x in (.2877, .29) for y in (-.3, .3) for z in (-.3, .3)])
        result = separation_lower_bound(pose, size, obstacle, np.eye(3), .005, np.deg2rad(1), [1, 0, 0])
        self.assertGreater(result['lower_m'], .008)
        self.assertLess(.0127 - .005 - 2 * np.linalg.norm(size) / 2 * np.sin(np.deg2rad(.5)), .005)
        overlapping = separation_lower_bound(pose, size, obstacle - [.1, 0, 0], np.eye(3), .005, .01)
        self.assertEqual(overlapping['lower_m'], 0.)

    def test_common_rigid_transform_preserves_bounds(self):
        pose = np.eye(4); obstacle = np.array([[1., -.2, -.2], [1., .2, .2]])
        axes = np.eye(3); direction = np.array([1., 0., 0.])
        old = separation_lower_bound(pose, [.5, .4, .3], obstacle, axes, .005, .02, direction)
        transform = np.eye(4); transform[:3, :3] = Rotation.from_euler('xyz', [.2, .4, .6]).as_matrix()
        transform[:3, 3] = [2., -3., 4.]
        new = separation_lower_bound(transform @ pose, [.5, .4, .3],
            obstacle @ transform[:3, :3].T + transform[:3, 3], axes @ transform[:3, :3].T,
            .005, .02, transform[:3, :3] @ direction)
        self.assertAlmostEqual(old['lower_m'], new['lower_m'])

    def test_invalid_inputs_cannot_produce_a_bound(self):
        for angle in (-.1, np.nan, 4.):
            with self.assertRaises(ValueError):rotation_projection_range(np.eye(3), [1, 0, 0], angle)
        with self.assertRaises(ValueError):box_projection_range(np.eye(4), [1, 1, 1], [1, 0, 0], .01, .1, [2, 0, 0])


if __name__ == '__main__':
    unittest.main()
