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

    def test_outer_search_deadline_is_not_reset_by_candidate_generation(self):
        import time
        config=replace(self.config,max_action_candidates=6)
        actions,report=generate_actions(self.assets,self.snapshot,self.state,config,
                                        deadline=time.monotonic()-1.)
        self.assertEqual(actions,[])
        self.assertEqual(report['termination'],'BUDGET_EXHAUSTED')

    def test_push_screen_cache_ignores_only_unconsumed_grasp(self):
        from action_screen import push_screen_key
        actions,_=generate_actions(self.assets,self.snapshot,self.state,
                                   replace(self.config,max_action_candidates=1))
        action=actions[0];seeds=[[0.]*6]
        key=lambda candidate,state=self.state,config=self.config:push_screen_key(
            self.snapshot,state,candidate,seeds,config)
        original=key(action)
        self.assertEqual(original,key(replace(action,box_T_tool=tuple(np.eye(4).ravel()))))
        moved=np.array(action.handoff_box_pose).reshape(4,4).copy();moved[1,3]+=.02
        self.assertNotEqual(original,key(replace(action,handoff_box_pose=tuple(moved.ravel()))))
        self.assertNotEqual(original,key(action,config=replace(self.config,collision_margin_m=.01)))
        self.assertNotEqual(original,push_screen_key(self.snapshot,self.state,action,[[.1]*6],self.config))

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

    def test_contact_stroke_changes_control_endpoint_without_moving_nominal_box(self):
        from planning_types import action_from_record
        from dataclasses import asdict
        rows=[]
        for stroke in (0., .00005, .0002):
            config=replace(self.config,max_action_candidates=1,support_contact_stroke_m=stroke)
            actions,_=generate_actions(self.assets,self.snapshot,self.state,config)
            action=actions[0];rows.append(action)
            expected=matrix(action.handoff_tool_pose).copy();expected[2,3]-=stroke
            np.testing.assert_allclose(matrix(action.support_contact_tool_pose),expected,atol=1e-12)
            self.assertEqual(action_from_record(asdict(action)),action)
        self.assertEqual(rows[0].handoff_box_pose,rows[-1].handoff_box_pose)
        self.assertEqual(rows[0].box_T_tool,rows[-1].box_T_tool)
        self.assertNotEqual(rows[0].action_id,rows[-1].action_id)
        record=asdict(rows[-1]);record.pop('support_contact_tool_pose')
        with self.assertRaises(KeyError):action_from_record(record)
        with self.assertRaises(ValueError):replace(self.config,support_contact_stroke_m=.001)

    def test_push_control_stroke_preserves_nominal_placement_and_travel(self):
        actions=[]
        for stroke in (0., .00005, .0002):
            config=replace(self.config,max_action_candidates=1,push_contact_stroke_m=stroke)
            rows,_=generate_actions(self.assets,self.snapshot,self.state,config);a=rows[0];actions.append(a)
            for reference,control in ((a.push_start_tool_pose,a.push_start_control_tool_pose),
                                      (a.push_end_tool_pose,a.push_end_control_tool_pose)):
                delta=matrix(control)[:3,3]-matrix(reference)[:3,3]
                np.testing.assert_allclose(delta,np.array(a.push_direction)*stroke,atol=1e-12)
            travel=matrix(a.push_end_control_tool_pose)[:3,3]-matrix(a.push_start_control_tool_pose)[:3,3]
            self.assertAlmostEqual(np.linalg.norm(travel),a.push_distance_m)
        self.assertEqual(actions[0].placement.final_box_pose,actions[-1].placement.final_box_pose)
        self.assertEqual(actions[0].push_start_tool_pose,actions[-1].push_start_tool_pose)
        with self.assertRaises(ValueError):replace(self.config,push_contact_stroke_m=.001)

    def test_push_candidate_binds_source_plate_face_to_box_face_and_roundtrips(self):
        from dataclasses import asdict
        from planning_types import action_from_record
        from tool_contact_geometry import derive_push_surface
        actions,_=generate_actions(self.assets,self.snapshot,self.state,
                                   replace(self.config,max_action_candidates=1))
        action=actions[0];source=derive_push_surface(self.assets)
        self.assertEqual(action.push_contact_geometry_id,source.geometry_id)
        plate=matrix(action.push_start_tool_pose)@matrix(action.tool_T_push_surface)
        box=matrix(action.handoff_box_pose)
        self.assertAlmostEqual(plate[0,3],box[0,3]-self.state.dimensions[0]/2)
        np.testing.assert_allclose(plate[:3,2],action.push_direction,atol=1e-12)
        record=json.loads(json.dumps(asdict(action)))
        self.assertEqual(action_from_record(record),action)
        record.pop('tool_T_push_surface')
        with self.assertRaises(KeyError):action_from_record(record)


if __name__=='__main__':unittest.main()
