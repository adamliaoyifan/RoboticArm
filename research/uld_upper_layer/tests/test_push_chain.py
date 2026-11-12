"""Push branch selection follows the moving box, not isolated endpoints."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from coupled_chain import push_branches,nearest_path_winding,path_winding_variants
from planning_types import UpperConfig

class PushChain(unittest.TestCase):
    def test_carry_enumerates_both_limit_valid_winding_directions(self):
        path=[[-3.4,-1.5],[-3.1,-1.4]]
        variants=list(path_winding_variants(path,[3.,1.5],[[-6.3,6.3],[-6.3,6.3]]))
        self.assertGreater(variants[0][0][0],0)
        self.assertTrue(any(v[0][1]>4 for v in variants))
        self.assertTrue(any(v[0][1]<0 for v in variants))
        for variant in variants:
            np.testing.assert_allclose(np.sin(variant),np.sin(path))
            np.testing.assert_allclose(np.diff(variant,axis=0),np.diff(path,axis=0))

    def test_whole_path_winding_preserves_angles_without_crossing_limits(self):
        path=[[-3.4,5.9],[-3.1,6.2]]
        shifted=nearest_path_winding(path,[3.,-.2],[[-6.3,6.3],[-6.3,6.3]])
        np.testing.assert_allclose(np.sin(shifted),np.sin(path))
        np.testing.assert_allclose(np.diff(shifted,axis=0),np.diff(path,axis=0))
        self.assertGreater(shifted[0][0],0)
        self.assertLess(shifted[0][1],0)
        self.assertIsNone(nearest_path_winding([[0.],[7.]],[0.],[[-3.,3.]]))

    def test_blocked_end_branch_backtracks_and_reverses_box_and_joint_path(self):
        handoff=np.eye(4);final=np.eye(4);final[0,3]=.3
        action=SimpleNamespace(handoff_box_pose=handoff.ravel(),
            placement=SimpleNamespace(final_box_pose=final.ravel()),
            push_start_control_tool_pose=handoff.ravel(),push_start_tool_pose=handoff.ravel(),
            push_direction=(1,0,0))
        state=SimpleNamespace(dimensions=(.55,.4,.25))
        scene=SimpleNamespace(world_container=np.eye(4),set_payload=lambda *a,**kw:None,
            check_state=lambda *a:None)
        calls=[];events=[]
        def path(scene,q,target,config,contact,motion=None):
            calls.append((q,motion))
            if q==[1]:return None,dict(reason='ROBOT_WORLD_CLEARANCE')
            if motion:
                np.testing.assert_allclose(motion(0),final)
                np.testing.assert_allclose(motion(1),handoff)
                return [[2],[3]],{}
            return [[3],[4]],{}
        with patch('coupled_chain.cartesian_path',side_effect=path):
            branches=list(push_branches(scene,state,action,[[1],[2]],UpperConfig.load(),events.append))
        self.assertEqual(len(branches),1)
        self.assertEqual(branches[0]['push_path'],[[3],[2]])
        self.assertEqual(branches[0]['contact_path'],[[4],[3]])
        self.assertEqual(branches[0]['pre_q'],[4])
        self.assertEqual(events[0]['stage'],'reverse_push')

if __name__=='__main__':unittest.main()
