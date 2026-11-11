"""Asset-derived tests: no copied tool dimensions or precomputed successful pose."""
from pathlib import Path
import sys
import unittest
import tempfile
import json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assets import load
from candidates import handoff_windows, matrix, generate_actions, samples
from dataclasses import replace
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig
from scene import fixture


class JointGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets=load(Path(__file__).resolve().parents[3])
        cls.config=UpperConfig.load()
        cls.temp=tempfile.TemporaryDirectory()
        data=fixture(cls.assets)
        data['pickup_payload']=dict(name='upper_layer_payload',pose_world=[-1,0,.986,0,0,0])
        (Path(cls.temp.name)/'fixture.json').write_text(json.dumps(data))
        cls.snapshot,cls.state=capture_fixture(cls.temp.name,cls.assets,cls.config)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def test_all_orientation_windows_and_short_side_rejection(self):
        rows=handoff_windows(self.assets,self.state,self.config)
        self.assertEqual(len(rows),16)
        long=[r for r in rows if r['box_yaw_deg'] in (0,180)]
        short=[r for r in rows if r['box_yaw_deg'] in (90,270)]
        self.assertTrue(any(r['feasible'] for r in long))
        self.assertTrue(all(not r['feasible'] for r in short))
        for r in long:self.assertAlmostEqual(r['insertion_min_m'],.55*.6)

    def test_center_grasp_has_no_window(self):
        length=self.state.dimensions[0]
        forward=max(b.corners()[:,0].max() for b in self.assets['tool'])
        self.assertLess(length/2-forward-self.config.collision_margin_m,
                        self.config.support_area_ratio_min*length)

    def test_narrow_window_includes_legal_high_endpoint(self):
        values=samples(0.,.00999999,.01)
        self.assertEqual(values,(0.,.00999999))
        self.assertLessEqual(max(np.diff(values)),.01)

    def test_copied_immutable_input_and_exact_surface(self):
        self.assertIsInstance(self.snapshot.occupied,tuple)
        self.assertEqual(self.snapshot.source_kind,'ideal_sim_fixture')
        top=max(p.surface_z_container for p in self.snapshot.surfaces)
        self.assertAlmostEqual(top,self.assets['hull'].ceiling_z-.25-.03)
        self.assertAlmostEqual(matrix(self.state.container_T_box)[2,3],.986)

    def test_small_budget_interleaves_orientations_and_leaves_clearance_slack(self):
        config=replace(self.config,max_action_candidates=6)
        actions,_=generate_actions(self.assets,self.snapshot,self.state,config)
        self.assertEqual(len(actions),6)
        self.assertEqual({a.tool_yaw_deg for a in actions},{0,90,180})
        self.assertEqual({round(matrix(a.handoff_box_pose)[0,0]) for a in actions},{-1,1})
        support_x=min(p.bounds_xy[0] for p in self.snapshot.surfaces)
        for action in actions:
            tcp=matrix(action.handoff_tool_pose)
            forward=max(body.moved(tcp).corners()[:,0].max() for body in self.assets['tool'])
            self.assertGreater(support_x-forward,config.collision_margin_m+config.grasp_step_m/2)


if __name__=='__main__':unittest.main()
