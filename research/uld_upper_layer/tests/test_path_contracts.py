"""Behavioural checks for real OMPL wiring and controller sweep rejection."""
from importlib.util import find_spec
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from path_planning import joint_path, certify_path
from planning_types import UpperConfig


class ToyScene:
    def __init__(self,blocked=False):
        self.urdf=SimpleNamespace(joint_limits=lambda:[[-1.,1.]]*6,
                                 joints={'one':{'xyz':np.array([1.,0,0])}})
        self.assets={'tool':[SimpleNamespace(center=np.zeros(3),half=np.ones(3)*.1)]}
        self.blocked=blocked
    def check_state(self,q,margin,contact):
        self.sample_clearance_m=.2
        if self.blocked and -.02<float(q[0])<.02:return dict(reason='MID_PATH_OBSTACLE')
        return None


class PathContracts(unittest.TestCase):
    def test_controller_sweep_rejects_between_valid_endpoints(self):
        result=certify_path(ToyScene(True),dict(positions=[[-.5]*6,[.5]*6]),UpperConfig.load())
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'],'MID_PATH_OBSTACLE')
        self.assertGreater(result['checks'],1)

    def test_interval_bounds_certify_virtual_samples_and_reject_obstacle(self):
        timed=dict(positions=[[-.5]*6,[.5]*6])
        result=certify_path(ToyScene(),timed,UpperConfig.load(),interval_bounds=True)
        self.assertTrue(result['valid'])
        self.assertGreater(result['virtual_1mm_samples'],result['checks'])
        self.assertFalse(certify_path(ToyScene(True),timed,UpperConfig.load(),interval_bounds=True)['valid'])

    @unittest.skipUnless(find_spec('ompl'),'isolated OMPL dependency required')
    def test_real_ompl_api_returns_exact_connected_path(self):
        start=[-.5]*6;goal=[.5]*6
        path,detail=joint_path(ToyScene(),start,goal,UpperConfig.load())
        self.assertIsNotNone(path,detail)
        np.testing.assert_allclose(path[0],start)
        np.testing.assert_allclose(path[-1],goal)
        self.assertTrue(certify_path(ToyScene(),dict(positions=path),UpperConfig.load())['valid'])

    @unittest.skipUnless(find_spec('ompl'),'isolated OMPL dependency required')
    def test_real_rrt_detour_keeps_returned_edge_proofs(self):
        from dataclasses import replace
        class DetourScene(ToyScene):
            def check_state(self,q,margin,contact):
                distance=max(abs(float(q[0]))-.2,abs(float(q[1]))-.3)
                self.sample_clearance_m=distance
                self.sample_distances=[(distance,('point','obstacle'))]
                return dict(reason='DET0UR_OBSTACLE') if distance<margin else None
            def relative_joint_indices(self,pair):return {0,1}
        scene=DetourScene();config=replace(UpperConfig.load(),path_attempt_budget_s=10.)
        path,detail=joint_path(scene,[-.5,0,0,0,0,0],[.5,0,0,0,0,0],config)
        self.assertIsNotNone(path,detail)
        self.assertGreater(len(path),2)
        self.assertTrue(detail['certificate']['valid'])
        self.assertTrue(certify_path(scene,dict(positions=path),config,interval_bounds=True)['valid'])

    @unittest.skipUnless(find_spec('ompl'),'isolated OMPL dependency required')
    def test_invalid_goal_never_returns_solution(self):
        path,detail=joint_path(ToyScene(True),[-.5]*6,[0.]*6,UpperConfig.load())
        self.assertIsNone(path)
        self.assertEqual(detail['reason'],'GOAL_INVALID')


if __name__=='__main__':unittest.main()
