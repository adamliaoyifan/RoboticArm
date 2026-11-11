"""Whole-action backtracking contract: a reachable pick may still be a dead end."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planning_types import UpperConfig
from stage_planner import plan_action


class BranchSearch(unittest.TestCase):
    def setUp(self):
        pose=tuple(np.eye(4).ravel())
        self.state=SimpleNamespace(dimensions=(.55,.4,.25),container_T_box=pose)
        self.action=SimpleNamespace(pick_tool_pose=pose)
        self.goals=[[.1]*6,[.2]*6,[.3]*6]
        self.scene=SimpleNamespace(world_container=np.eye(4),
            set_payload=lambda *a,**k:None,check_state=lambda *a:None,
            endpoint_solutions=lambda *a:(self.goals,dict(scope='sampled_ik')))
        self.events=[]

    def test_downstream_collision_backtracks_pick_branch_and_stops_at_first_full_plan(self):
        outcomes=[dict(valid=False,reason='NO_CONNECTED_HANDOFF_BRANCH',
                       diagnostic=dict(reason='SELF_COLLISION',pair=['arm','camera'])),
                  dict(valid=True,stages=['pick','carry','release','push','retreat'])]
        with patch('action_screen.screen_push',return_value=dict(valid=True)),patch(
                'stage_planner._plan_from_pick_branch',side_effect=outcomes) as attempt:
            result=plan_action(self.scene,self.state,self.action,[0.]*6,[],UpperConfig.load(),self.events.append)
        self.assertTrue(result['valid'])
        self.assertEqual(result['pick_joint_branches_attempted'],2)
        self.assertEqual(attempt.call_count,2)
        self.assertEqual([call.args[7] for call in attempt.call_args_list],self.goals[:2])
        self.assertEqual([event['valid'] for event in self.events if event['event']=='pick_branch_output'],[False,True])

    def test_all_sampled_pick_branches_fail_without_executable_result(self):
        with patch('action_screen.screen_push',return_value=dict(valid=True)),patch(
                'stage_planner._plan_from_pick_branch',return_value=dict(valid=False,reason='BLOCKED')):
            result=plan_action(self.scene,self.state,self.action,[0.]*6,[],UpperConfig.load(),self.events.append)
        self.assertFalse(result['valid'])
        self.assertEqual(result['pick_joint_branches_attempted'],3)
        self.assertFalse(result['exhaustive'])
        self.assertEqual(result['stages'],[])


if __name__=='__main__':unittest.main()
