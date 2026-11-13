"""Behavioral tests independent of candidate enumeration implementation."""
from dataclasses import FrozenInstanceError, replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from geometry import Box, axis_box, pose
from planning_types import OccupiedBox, SceneSnapshot, SurfacePatch, UpperConfig
from support_geometry import measure_support
from ideal_snapshot import query_box


class SupportContracts(unittest.TestCase):
    def setUp(self):self.config=UpperConfig.load()

    def test_gap_is_not_contact_but_com_can_be_supported_over_it(self):
        patches=(SurfacePatch('left',(-.3,-.2,-.002,.2),1.),
                 SurfacePatch('right',(.002,-.2,.3,.2),1.))
        result=measure_support(pose([0,0,1.125]),(.55,.4,.25),patches,self.config)
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.contact_area_ratio,1-.004/.55)
        self.assertAlmostEqual(result.com_margin_m,.2)
        self.assertIn('MULTIPOLYGON',result.contact_wkt)
        self.assertIn('POLYGON',result.support_polygon_wkt)

    def test_config_margin_changes_same_input_decision(self):
        patch=(SurfacePatch('p',(-.015,-.2,.3,.2),1.),)
        # 65% area passes independently; COM is 15mm from the support edge.
        body=pose([0,0,1.125])
        low=measure_support(body,(.10,.4,.25),patch,replace(self.config,support_com_margin_m=.01))
        high=measure_support(body,(.10,.4,.25),patch,self.config)
        self.assertTrue(low.accepted)
        self.assertFalse(high.accepted)
        self.assertEqual(high.reason,'COM_SUPPORT_MARGIN')

    def test_com_outside_and_area_gate_independent(self):
        # Full longitudinal support but too narrow laterally.
        patch=(SurfacePatch('p',(-.3,-.1,.3,.1),1.),)
        high=measure_support(pose([0,0,1.125]),(.55,.4,.25),patch,self.config)
        low=measure_support(pose([0,0,1.125]),(.55,.4,.25),patch,
                            replace(self.config,support_area_ratio_min=.4))
        self.assertEqual(high.reason,'SUPPORT_AREA');self.assertTrue(low.accepted)
        outside=measure_support(pose([0,0,1.125]),(.55,.4,.25),patch,
             replace(self.config,support_area_ratio_min=.4),com_local=(0,.15,0))
        self.assertLess(outside.com_margin_m,0)
        self.assertFalse(outside.accepted)

    def test_voxel_upper_face_is_not_exact_contact(self):
        patch=(SurfacePatch('p',(-1,-1,1,1),1.),)
        result=measure_support(pose([0,0,1.135]),(.55,.4,.25),patch,self.config)
        self.assertFalse(result.accepted)
        self.assertEqual(result.contact_area_m2,0)

    def test_overlapping_patches_not_double_counted(self):
        patch=(SurfacePatch('a',(-1,-1,1,1),1.),SurfacePatch('b',(-1,-1,1,1),1.))
        result=measure_support(pose([0,0,1.125]),(.55,.4,.25),patch,self.config)
        self.assertAlmostEqual(result.contact_area_ratio,1)

    def test_config_is_immutable_and_invalid_changes_rejected(self):
        with self.assertRaises(FrozenInstanceError):self.config.support_com_margin_m=.1
        for values in ({'support_com_margin_m':-1},{'support_area_ratio_min':1.1},
                       {'handoff_step_m':0},{'support_com_margin_m':float('nan')}):
            with self.assertRaises(ValueError):replace(self.config,**values)
        self.assertNotEqual(self.config.revision,replace(self.config,support_com_margin_m=.01).revision)

    def test_unknown_is_not_free(self):
        snapshot=SceneSnapshot('id','geometry','container_link','ideal_sim_fixture',
            (0.,0.,0.),(10,10,10),.1,(),(),(0.,0.,0.,1.,1.,1.),0.,1.)
        self.assertEqual(query_box(snapshot,axis_box('inside',[.1]*3,[.2]*3))[0],'FREE')
        self.assertEqual(query_box(snapshot,axis_box('outside',[-.1]*3,[.2]*3))[0],'UNKNOWN')

    def test_occupancy_broadphase_preserves_rotated_and_margin_collisions(self):
        from scipy.spatial.transform import Rotation
        snapshot=SceneSnapshot('id','geometry','container_link','ideal_sim_fixture',
            (-5.,-5.,-5.),(100,100,100),.1,
            (OccupiedBox('near',(0.,0.,0.),(.2,.2,.2)),
             OccupiedBox('far',(4.,4.,4.),(.2,.2,.2))),(),(-5.,-5.,-5.,5.,5.,5.),0.,1.)
        body=Box('rotated',np.array([.18,0,0]),Rotation.from_euler('z',45,degrees=True).as_matrix(),np.ones(3)*.1)
        self.assertEqual(query_box(snapshot,body,.005),('OCCUPIED',('near',)))
        self.assertEqual(query_box(snapshot,axis_box('close',[.104,-.1,-.1],[.3,.1,.1]),.005),('OCCUPIED',('near',)))
        self.assertEqual(query_box(snapshot,axis_box('clear',[.106,-.1,-.1],[.3,.1,.1]),.005),('FREE',()))
        # An overlapping enclosing AABB is only a possible collision; SAT must
        # still establish that this diagonal slender body clears the obstacle.
        slender=Box('slender',np.array([.23,.23,0]),Rotation.from_euler('z',-45,degrees=True).as_matrix(),np.array([.3,.005,.05]))
        self.assertEqual(query_box(snapshot,slender,.005),('FREE',()))

    def test_tilted_box_is_not_silently_flattened(self):
        from scipy.spatial.transform import Rotation
        body=pose([0,0,1.125],Rotation.from_euler('x',.1).as_matrix())
        with self.assertRaises(ValueError):measure_support(body,(.55,.4,.25),(),self.config)


if __name__=='__main__':unittest.main()
