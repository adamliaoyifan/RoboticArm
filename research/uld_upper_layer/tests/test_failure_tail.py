from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from failure_tail import collect_failure_tail


class FailureTailContracts(unittest.TestCase):
    def setUp(self):
        self.policy=SimpleNamespace(condition_wait_budget_s=.1,monitor_period_s=.01,trigger_post_s=1.)
        self.runtime=SimpleNamespace(node=object(),snapshot=lambda:dict(vacuum=dict(sim_time_s=2.)))

    def test_shutdown_does_not_spin_or_hide_original_failure(self):
        with patch('rclpy.ok',return_value=False),patch('rclpy.spin_once') as spin:
            tail,error=collect_failure_tail(self.runtime,self.policy,1.)
        self.assertEqual((tail,error),([],'ROS_CONTEXT_CLOSED'));spin.assert_not_called()

    def test_shutdown_race_is_reported_instead_of_escaping_finalization(self):
        with patch('rclpy.ok',return_value=True),patch('rclpy.spin_once',side_effect=RuntimeError('context closed')):
            tail,error=collect_failure_tail(self.runtime,self.policy,1.)
        self.assertFalse(tail);self.assertIn('context closed',error)

    def test_complete_post_window_and_unavailable_stamp_are_distinct(self):
        with patch('rclpy.ok',return_value=True),patch('rclpy.spin_once'):
            tail,error=collect_failure_tail(self.runtime,self.policy,1.)
        self.assertIsNone(error);self.assertEqual(tail[0][0],2.)
        self.assertEqual(collect_failure_tail(self.runtime,self.policy,None),([], 'NATIVE_STAMP_UNAVAILABLE'))


if __name__=='__main__':unittest.main()
