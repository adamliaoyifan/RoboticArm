"""Live planning retains strict state and attachment identity at the hold boundary."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys,unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from live_planning import validate_held_start
from execution_monitor import ExecutionMonitor
from planning_types import UpperConfig
from execution_policy import ExecutionPolicy

class LivePlanning(unittest.TestCase):
    def test_live_hold_drift_uses_existing_joint_and_box_thresholds(self):
        pose=np.eye(4).ravel().tolist()
        req=SimpleNamespace(start_q=[0.]*6,box=SimpleNamespace(container_T_box=pose))
        sample=SimpleNamespace(q=[0.]*6,container_T_box=pose)
        config=UpperConfig.load();policy=ExecutionPolicy.load()
        self.assertEqual(validate_held_start(req,sample,config,policy)['joint_max_error_rad'],0.)
        sample.q[0]=2*policy.initial_joint_error_rad
        with self.assertRaisesRegex(ValueError,'LIVE_START_CHANGED'):validate_held_start(req,sample,config,policy)
        sample.q[0]=0.;sample.container_T_box=pose.copy();sample.container_T_box[3]=2*config.contact_penetration_m
        with self.assertRaisesRegex(ValueError,'LIVE_START_CHANGED'):validate_held_start(req,sample,config,policy)

    def test_action_rebinding_requires_held_attachment_and_checks_measured_relation(self):
        monitor=ExecutionMonitor.__new__(ExecutionMonitor)
        monitor._phase='planning_hold';monitor._sample=SimpleNamespace(attached=True,tool_T_box_at_attach=np.eye(4))
        monitor._config=UpperConfig.load();monitor._policy=ExecutionPolicy.load()
        action=SimpleNamespace(box_T_tool=np.eye(4).ravel())
        with patch('planning_types.action_from_record',return_value=action):monitor.bind_held_action({})
        self.assertIs(monitor._action,action)
        shifted=np.eye(4);shifted[0,3]=.01
        with patch('planning_types.action_from_record',return_value=SimpleNamespace(box_T_tool=shifted.ravel())):
            with self.assertRaisesRegex(ValueError,'LIVE_ACTION_ATTACHMENT_CHANGED'):monitor.bind_held_action({})
        self.assertIs(monitor._action,action)
        monitor._phase='push'
        with self.assertRaisesRegex(ValueError,'ATTACHED_PLANNING_HOLD'):monitor.bind_held_action({})

if __name__=='__main__':unittest.main()
