"""Calibration geometry is source-derived, including below-policy diagnostics."""
from pathlib import Path
from dataclasses import replace,FrozenInstanceError
import json
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assets import load
from candidates import matrix
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig
from release_geometry import release_matrix,release_geometry
from scene import fixture


class ReleaseGeometryContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets=load(Path(__file__).resolve().parents[3]);cls.config=UpperConfig.load()
        cls.temp=tempfile.TemporaryDirectory();data=fixture(cls.assets)
        data['pickup_payload']=dict(name='upper_layer_payload',pose_world=[-1,0,.986,0,0,0])
        (Path(cls.temp.name)/'fixture.json').write_text(json.dumps(data))
        cls.snapshot,cls.state=capture_fixture(cls.temp.name,cls.assets,cls.config)
        cls.cases=release_matrix(cls.assets,cls.snapshot,cls.state,cls.config,
            [.4+i*.05 for i in range(9)],(0.,90.),3)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def test_matrix_preserves_unavailable_and_below_policy_cases(self):
        self.assertEqual(len(self.cases),54)
        self.assertEqual(len({case.case_id for case in self.cases}),54)
        self.assertTrue(any(not case.geometry_available for case in self.cases))
        self.assertTrue(any(case.geometry_available and not case.expected_support.accepted for case in self.cases))
        self.assertTrue(any(case.geometry_available and case.expected_support.accepted for case in self.cases))
        for case in self.cases:
            self.assertAlmostEqual(case.expected_support.contact_area_ratio,case.requested_support_ratio,places=6)
            if case.geometry_available:self.assertIn('PHYSICS_NOT_EVALUATED',case.reason)

    def test_available_footprint_inside_material_face_and_source_tool_clearance(self):
        half=np.array(self.state.dimensions)/2
        footprint=np.array(self.assets['contact']['footprint_size_xy_m'])/2
        edge=self.assets['contact']['boundary_margin_m']
        xmin=min(p.bounds_xy[0] for p in self.snapshot.surfaces)
        for case in self.cases:
            if not case.geometry_available:continue
            box=matrix(case.container_T_box);tool=matrix(case.container_T_tool);relative=np.linalg.inv(box)@tool
            for u,v in ((-1,-1),(-1,1),(1,-1),(1,1)):
                point=relative@np.array([u*footprint[0],v*footprint[1],0,1])
                self.assertTrue(np.all(np.abs(point[:2])<=half[:2]-edge+1e-8))
            forward=max(body.moved(tool).corners()[:,0].max() for body in self.assets['tool'])
            self.assertGreaterEqual(xmin-forward,self.config.collision_margin_m-1e-8)

    def test_geometry_ids_bind_config_and_box_and_cases_are_immutable(self):
        args=(self.assets,self.snapshot,self.state,self.config,.6,0.,0)
        original=release_geometry(*args)
        changed=release_geometry(self.assets,self.snapshot,replace(self.state,dimensions=(.6,.4,.25)),self.config,.6,0.,0)
        self.assertNotEqual(original.source_geometry_id,changed.source_geometry_id)
        with self.assertRaises(FrozenInstanceError):original.reason='stable'


if __name__=='__main__':unittest.main()
