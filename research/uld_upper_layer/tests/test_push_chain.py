"""Push branch selection follows the moving box, not isolated endpoints."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from coupled_chain import push_branches
from planning_types import UpperConfig

class PushChain(unittest.TestCase):
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
