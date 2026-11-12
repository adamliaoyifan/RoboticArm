"""Input-driven geometry: real source tool, dimensions, targets and fixed grasp."""
from dataclasses import replace,asdict
from pathlib import Path
import sys,tempfile,json,unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from assets import load
from scene import fixture
from ideal_snapshot import capture_fixture
from planning_types import UpperConfig
from geometry import pose
from placement_request import PlacementRequest,target_actions
from request_planner import request_from_record


class RequestContracts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assets=load(Path(__file__).resolve().parents[3]);cls.tmp=tempfile.TemporaryDirectory()
        data=fixture(cls.assets);data['pickup_payload']=dict(name='upper_layer_payload',pose_world=[-1,0,.986,0,0,0])
        (Path(cls.tmp.name)/'fixture.json').write_text(json.dumps(data))
        cls.config=replace(UpperConfig.load(),max_action_candidates=6,search_budget_s=30.)
        cls.snapshot,cls.box=capture_fixture(cls.tmp.name,cls.assets,cls.config)
        top=max(p.surface_z_container for p in cls.snapshot.surfaces)
        cls.ids=tuple(p.source_id for p in cls.snapshot.surfaces if abs(p.surface_z_container-top)<1e-7)
        cls.target=pose([-.10,-.49,top+cls.box.dimensions[2]/2],Rotation.from_euler('x',np.pi).as_matrix())

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def request(self,**changes):
        return replace(PlacementRequest('test',self.box,self.snapshot,(0.,)*6,
            (tuple(self.target.ravel()),),self.ids),**changes)

    def generate(self,request,config=None):return target_actions(self.assets,request,config or self.config)

    def test_exact_targets_and_changed_dimensions_drive_geometry(self):
        for size in ((.55,.4,.25),(.57,.4,.25)):
            for x in (-.1,.03):
                target=self.target.copy();target[0,3]=x
                request=self.request(box=replace(self.box,dimensions=size),final_box_poses=(tuple(target.ravel()),))
                actions,report=self.generate(request)
                self.assertTrue(actions,report)
                for action in actions:
                    np.testing.assert_allclose(np.array(action.placement.final_box_pose).reshape(4,4),target,atol=1e-12)
                    self.assertGreaterEqual(action.handoff_support.contact_area_ratio,self.config.support_area_ratio_min-1e-10)
                    np.testing.assert_allclose(np.array(action.pick_tool_pose).reshape(4,4),
                        np.array(request.box.container_T_box).reshape(4,4)@np.array(action.box_T_tool).reshape(4,4))

    def test_fixed_attachment_filters_without_regrasp(self):
        actions,_=self.generate(self.request());grasp=actions[0].box_T_tool
        request=self.request(start_attached=True,confirmed_box_T_tool=grasp)
        actions,report=self.generate(request)
        self.assertTrue(actions,report)
        for a in actions:np.testing.assert_allclose(a.box_T_tool,grasp,atol=1e-7)
        displaced=np.array(grasp).reshape(4,4).copy();displaced[1,3]+=.05
        actions,_=self.generate(replace(request,confirmed_box_T_tool=tuple(displaced.ravel())))
        self.assertEqual(actions,[])

    def test_invalid_boundaries_fail_before_planning(self):
        for values in (dict(frame_id='elfin_base_link'),dict(support_source_ids=('missing',)),
                       dict(start_attached=True),dict(start_q=(float('nan'),)*6),
                       dict(box=replace(self.box,dimensions=(-1,.4,.25)))):
            with self.subTest(values=values),self.assertRaises(ValueError):self.request(**values)
        bad=self.target.copy();bad[0,0]=2
        with self.assertRaises(ValueError):self.request(final_box_poses=(tuple(bad.ravel()),))

    def test_floating_or_sideways_target_is_not_silently_moved(self):
        target=self.target.copy();target[2,3]+=.002
        actions,report=self.generate(self.request(final_box_poses=(tuple(target.ravel()),)))
        self.assertEqual(actions,[]);self.assertIn('TARGET_NOT_ON_SUPPORT',report['reasons'])
        target=pose(self.target[:3,3],Rotation.from_euler('y',np.pi/2).as_matrix())
        actions,report=self.generate(self.request(final_box_poses=(tuple(target.ravel()),)))
        self.assertEqual(actions,[]);self.assertIn('TARGET_ORIENTATION_OUTSIDE_ACTION_FAMILY',report['reasons'])

    def test_sub_tolerance_surface_heights_keep_exact_requested_target(self):
        surfaces=tuple(replace(p,surface_z_container=p.surface_z_container-3e-7)
                       if p.source_id in self.ids else p for p in self.snapshot.surfaces)
        request=self.request(snapshot=replace(self.snapshot,surfaces=surfaces))
        actions,report=self.generate(request)
        self.assertTrue(actions,report)
        for a in actions:np.testing.assert_allclose(a.placement.final_box_pose,self.target.ravel(),atol=1e-12)

    def test_runtime_com_margin_changes_real_candidate_admission(self):
        actions,_=self.generate(self.request());self.assertTrue(actions)
        actions,report=self.generate(self.request(),replace(self.config,support_com_margin_m=.30))
        self.assertEqual(actions,[])

    def test_wire_roundtrip_is_immutable_and_identical(self):
        request=self.request();restored=request_from_record(json.loads(json.dumps(asdict(request))))
        self.assertEqual(request,restored);self.assertEqual(request.revision,restored.revision)


if __name__=='__main__':unittest.main()
