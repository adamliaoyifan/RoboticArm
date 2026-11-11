"""Behavioural geometry and dynamic-body contracts for the synthetic fixture."""
import json
from copy import deepcopy
from unittest.mock import patch
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from assets import load
from check_scene import measure
from scene import ROOT, fixture, intersecting_triangles, write_scene


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets = load(ROOT)
        cls.spec = fixture(cls.assets)

    def test_dense_aligned_top_has_box_clearance_but_not_panel_clearance(self):
        spec = self.spec
        self.assertEqual(len(spec['boxes']),24)
        self.assertGreater(spec['lower_fill_fraction'],.9)
        self.assertAlmostEqual(spec['upper_headroom_m'],.28)
        self.assertLess(spec['payload_ceiling_gap_m'],spec['suction_panel_depth_from_contact_m'])
        tops = [b['center_container_m'][2]+b['size_m'][2]/2 for b in spec['boxes'] if b['layer']==3]
        np.testing.assert_allclose(tops,1.73,atol=1e-9)
        self.assertTrue(all(b['pose_world'][3:]==[0.,0.,0.] for b in spec['boxes']))

    def test_overhanging_chamfer_rows_keep_center_inside_underlying_support(self):
        for layer in (1,2,3):
            upper = next(b for b in self.spec['boxes'] if b['layer']==layer and b['name'].endswith('x0_y1'))
            lower = next(b for b in self.spec['boxes'] if b['layer']==layer-1 and b['name'].endswith('x0_y1'))
            lower_edge = lower['center_container_m'][1]+lower['size_m'][1]/2
            self.assertLess(upper['center_container_m'][1]+.02,lower_edge)

    def test_large_gap_does_not_satisfy_this_fixture(self):
        with self.assertRaises(ValueError):
            fixture(self.assets,ceiling_gap=.4)

    def test_standalone_and_overlay_have_real_dynamic_boxes(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'new_scene'
            write_scene(ROOT,output)
            for name in ('overlay.sdf','standalone.sdf'):
                world = ET.parse(output/name).getroot().find('world')
                boxes = [m for m in world.findall('model') if m.get('name').startswith('lower_')]
                self.assertEqual(len(boxes),24)
                for model in boxes:
                    self.assertEqual(model.findtext('static'),'false')
                    self.assertIsNone(model.find('joint'))
                    self.assertEqual(model.findtext('link/gravity'),'true')
                    self.assertIsNotNone(model.find('link/collision/geometry/box'))
                self.assertIsNotNone(world.find("model[@name='upper_layer_payload']"))
            self.assertIsNotNone(ET.parse(output/'standalone.sdf').find("world/model[@name='airport_container_real']"))
            self.assertIsNone(ET.parse(output/'overlay.sdf').find("world/model[@name='airport_container_real']"))
            with self.assertRaises(ValueError):
                write_scene(ROOT,output)

    def test_mesh_guard_rejects_rib_intersection_inside_semantic_hull(self):
        bad = deepcopy(self.spec)
        box = next(b for b in bad['boxes'] if b['name']=='lower_l3_x1_y1')
        # Still within semantic Y=0.985, but intersects the real inward rail.
        box['center_container_m'][1] += .04
        with tempfile.TemporaryDirectory() as directory, patch('scene.fixture',return_value=bad):
            with self.assertRaisesRegex(ValueError,'intersects source collision mesh'):
                write_scene(ROOT,Path(directory)/'invalid')

    def test_triangle_guard_uses_more_than_axis_aligned_bounds(self):
        triangles = np.array([[[.9,2,0],[2,.9,0],[2,2,0]],
                              [[0,0,0],[2,0,0],[0,2,0]]])
        self.assertEqual(intersecting_triangles([0,0,0],[1,1,1],triangles),[1])

    def test_scoring_detects_transient_motion_and_missing_capture(self):
        spec = dict(boxes=[dict(name='lower_1',pose_world=[0,0,0,0,0,0])])
        def frame(t,x):
            return dict(header=dict(stamp=dict(sec=t)),pose=[dict(name='lower_1',
                        position=dict(x=x,y=0,z=0),orientation=dict(w=1))])
        messages = [frame(0,0)]*20+[frame(3,.01),frame(5,0)]
        result = measure(spec,messages)
        self.assertFalse(result['pass_'])
        self.assertAlmostEqual(result['models']['lower_1']['horizontal_m'],.01)
        self.assertFalse(measure(spec,[])['capture_complete'])
        self.assertTrue(measure(spec,[frame(0,0)]*20+[frame(5,0)])['pass_'])


if __name__ == '__main__':
    unittest.main()
