from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from edge_contact_diagnostic import diagnose_support_edge
from geometry import pose


class EdgeContactDiagnosticContracts(unittest.TestCase):
    def diagnose(self, point=(.5, 0., 1.), normal=(-1., 0., 0.), inverted=False,
                 signed=True, depth=.00001, direction=(1., 0., 0.), tolerance=.001):
        rotation = Rotation.from_euler('x', 180, degrees=True).as_matrix() if inverted else np.eye(3)
        return diagnose_support_edge(pose([0, 0, 1.125], rotation), (1, 1, .25),
            pose([1., 0, .5]), [[-.5, -.5, -.5], [.5, .5, .5]], point, normal, direction,
            point_tolerance_m=tolerance, penetration_m=.0005, normal_alignment_deg=1.,
            depth_m=depth, normal_toward_box=signed)

    def test_edge_cone_candidate_preserves_strict_rejection(self):
        for inverted in (False, True):
            for normal in ((-1., 0., 0.), (-2**-.5, 0., 2**-.5)):
                result = self.diagnose(normal=normal, inverted=inverted)
                self.assertEqual(result['classification'], 'GEOMETRIC_EDGE_TRANSITION_CANDIDATE')
                self.assertFalse(result['current_point_predicate_passed'])
                self.assertTrue(result['diagnostic_only'])

    def test_flat_support_keeps_its_existing_classification(self):
        result = self.diagnose(normal=(0., 0., 1.))
        self.assertEqual(result['classification'], 'TOP_BOTTOM_SUPPORT')
        self.assertTrue(result['current_point_predicate_passed'])

    def test_native_signed_normal_and_fcl_unsigned_order_are_distinct(self):
        self.assertFalse(self.diagnose(normal=(1., 0., 0.))['edge_candidate'])
        self.assertTrue(self.diagnose(normal=(1., 0., 0.), signed=False)['edge_candidate'])
        self.assertFalse(self.diagnose(normal=(0., 1., 0.))['edge_candidate'])

    def test_side_face_wrong_material_depth_and_back_edge_are_not_edge_candidates(self):
        for change in (dict(point=(.5, 0., .99)), dict(point=(.49, 0., 1.)),
                       dict(point=(.5, .6, 1.)), dict(depth=.0006), dict(direction=(-1., 0., 0.))):
            with self.subTest(change=change):
                result = self.diagnose(**change)
                self.assertFalse(result['edge_candidate']); self.assertTrue(result['failed_checks'])
                self.assertFalse(result['current_point_predicate_passed'])

    def test_off_axis_push_is_unsupported_and_point_band_is_configured(self):
        result = self.diagnose(direction=(2**-.5, 2**-.5, 0.))
        self.assertIn('face_aligned_push', result['failed_checks'])
        self.assertTrue(self.diagnose(point=(.5, 0., .9995))['edge_candidate'])
        self.assertFalse(self.diagnose(point=(.5, 0., .9995), tolerance=.0001)['edge_candidate'])

    def test_translation_and_common_yaw_preserve_geometry_classification(self):
        transform = pose([2., -3., 4.], Rotation.from_euler('z', .7).as_matrix())
        point = (transform @ [.5, 0., 1., 1.])[:3]
        result = diagnose_support_edge(transform @ pose([0, 0, 1.125]), (1, 1, .25),
            transform @ pose([1., 0, .5]), [[-.5, -.5, -.5], [.5, .5, .5]],
            point, transform[:3, :3] @ [-1., 0., 0.], transform[:3, :3] @ [1., 0., 0.],
            point_tolerance_m=.001, penetration_m=.0005, normal_alignment_deg=1.,
            depth_m=.00001, normal_toward_box=True)
        self.assertEqual(result['classification'], self.diagnose()['classification'])

    def test_malformed_direction_is_not_normalized_into_validity(self):
        with self.assertRaises(ValueError):self.diagnose(direction=(2., 0., 0.))


if __name__ == '__main__':
    unittest.main()
