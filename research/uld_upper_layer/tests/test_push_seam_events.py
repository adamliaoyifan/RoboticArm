import unittest
import numpy as np
from planning_types import SurfacePatch
from push_seam_events import trace_push_seams


class PushSeamEventsTests(unittest.TestCase):
    def run_trace(self, start, end, patches, size=(.4, .2, .2)):
        return trace_push_seams(start, end, size, patches)

    def poses(self):
        start = np.eye(4); start[2, 3] = 1.1
        end = start.copy(); end[0, 3] = 1.
        return start, end

    def test_narrow_patch_continuous_interval_and_actual_height(self):
        start, end = self.poses()
        report = self.run_trace(start, end, [SurfacePatch('next', (.5, -.3, .50001, .3), 1.002)])
        self.assertEqual([e['kind'] for e in report['events']], ['enter', 'leave'])
        self.assertAlmostEqual(report['events'][0]['progress_m'], .3)
        self.assertAlmostEqual(report['events'][1]['progress_m'], .70001)
        self.assertAlmostEqual(report['patches'][0]['nominal_bottom_minus_top_m'], -.002)

    def test_initial_overlap_has_exit_without_invented_entry(self):
        start, end = self.poses()
        report = self.run_trace(start, end, [SurfacePatch('current', (-.3, -.3, .3, .3), 1.)])
        self.assertEqual([e['kind'] for e in report['events']], ['leave'])
        self.assertEqual(report['patches'][0]['overlap_start_m'], 0.)
        self.assertAlmostEqual(report['events'][0]['progress_m'], .5)

    def test_grazing_and_outside_are_not_positive_area_support(self):
        start, end = self.poses()
        patches = [SurfacePatch('grazing', (.5, .1, .8, .4), 1.),
                   SurfacePatch('outside', (.5, .2, .8, .4), 1.)]
        self.assertEqual(self.run_trace(start, end, patches)['patches'], [])

    def test_rotated_and_inverted_box_uses_real_footprint(self):
        start, end = self.poses()
        start[:3, :3] = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        end[:3, :3] = start[:3, :3]
        report = self.run_trace(start, end, [SurfacePatch('next', (.5, -.3, .8, .3), 1.)])
        self.assertAlmostEqual(report['events'][0]['progress_m'], .4)
        self.assertAlmostEqual(report['patches'][0]['nominal_bottom_minus_top_m'], 0.)

    def test_diagonal_sat_matches_independent_polygon_overlap(self):
        from shapely.geometry import Polygon, box
        start, end = self.poses(); end[1, 3] = .7
        angle = .37
        start[:2, :2] = [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
        end[:3, :3] = start[:3, :3]
        patches = [SurfacePatch('next', (.5, .25, .8, .6), 1.)]
        report = self.run_trace(start, end, patches)
        row = report['patches'][0]
        offsets = np.array([[-.2, -.1], [.2, -.1], [.2, .1], [-.2, .1]]) @ start[:2, :2].T
        direction = np.array(report['direction_xy'])
        for s in np.linspace(0., report['length_m'], 301):
            actual = Polygon(offsets + s * direction).intersection(box(*patches[0].bounds_xy)).area > 1e-12
            self.assertEqual(actual, row['overlap_start_m'] < s < row['overlap_end_m'])

    def test_rejects_unsupported_motion_and_invalid_sources(self):
        start, end = self.poses(); end[2, 3] += .001
        with self.assertRaisesRegex(ValueError, 'HORIZONTAL'):
            self.run_trace(start, end, [])
        start, end = self.poses()
        patch = SurfacePatch('same', (.5, -.3, .8, .3), 1.)
        with self.assertRaisesRegex(ValueError, 'DUPLICATE'):
            self.run_trace(start, end, [patch, patch])


if __name__ == '__main__':
    unittest.main()
