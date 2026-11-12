"""An attached request must preserve measured grasp and skip all pickup stages."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from planning_types import UpperConfig
from stage_planner import plan_action
from physical_executor import expected_stages,STAGES,PhysicalExecutor


class AttachedStart(unittest.TestCase):
    def test_measured_relation_is_required_before_any_downstream_planning(self):
        transform=np.eye(4);transform[2,3]=.4
        state=SimpleNamespace(dimensions=(.55,.4,.25),container_T_box=tuple(transform.ravel()))
        action=SimpleNamespace(box_T_tool=tuple(np.eye(4).ravel()))
        scene=SimpleNamespace(container_world=np.eye(4),world_container=np.eye(4),
            urdf=SimpleNamespace(tcp=lambda *a:np.eye(4),path_to=lambda *a:None))
        with patch('action_screen.screen_push') as screen:
            result=plan_action(scene,state,action,[0.]*6,[],UpperConfig.load(),lambda r:None,True)
            self.assertFalse(result['valid']);screen.assert_not_called()
            self.assertEqual(result['reason'],'CONFIRMED_ATTACHMENT_POSE_MISMATCH')

    def test_matching_attachment_routes_from_current_joints_without_pick_branch_search(self):
        flat=tuple(np.eye(4).ravel());payload_calls=[]
        state=SimpleNamespace(dimensions=(.55,.4,.25),container_T_box=flat)
        action=SimpleNamespace(box_T_tool=flat)
        scene=SimpleNamespace(container_world=np.eye(4),world_container=np.eye(4),
            urdf=SimpleNamespace(tcp=lambda *a:np.eye(4),path_to=lambda *a:None),
            set_payload=lambda *a,**k:payload_calls.append((a,k)),check_state=lambda *a:None)
        current=[.1]*6
        with patch('action_screen.screen_handoff',return_value=dict(valid=True,q_solutions=[])),patch('action_screen.screen_push',return_value=dict(valid=True)),patch(
            'stage_planner._plan_from_pick_branch',return_value=dict(valid=True)) as carry:
            result=plan_action(scene,state,action,current,[],UpperConfig.load(),lambda r:None,True)
            self.assertTrue(result['valid'])
            self.assertEqual(carry.call_args.args[3],current)
            self.assertIsNone(carry.call_args.args[7])
            self.assertTrue(carry.call_args.kwargs['start_attached'])
            self.assertEqual(payload_calls[0][0][1],'attached')

    def test_executor_stage_contract_and_missing_live_attachment_fail_closed(self):
        self.assertEqual(expected_stages(dict(start_attached=True)),STAGES[3:])
        self.assertEqual(expected_stages({}),STAGES)
        executor=PhysicalExecutor.__new__(PhysicalExecutor)
        executor._runtime=SimpleNamespace(snapshot=lambda:dict(vacuum=dict(attached=False)))
        plan=dict(valid=True,start_attached=True,
            stages=[dict(name=n,certificate=dict(valid=True)) for n in STAGES[3:]])
        with self.assertRaisesRegex(ValueError,'CONFIRMED_LIVE_ATTACHMENT_REQUIRED'):executor.execute(plan)
        plan['stages']=plan['stages'][1:]
        with self.assertRaisesRegex(ValueError,'COMPLETE_NOMINAL_STAGE_CHAIN_REQUIRED'):executor.execute(plan)


if __name__=='__main__':unittest.main()
