"""Clock-control waits must drain observation callbacks instead of blocking them."""
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys,tempfile,unittest,json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime_io import RuntimeIO


class PlanningClock(unittest.TestCase):
    def test_control_response_wait_keeps_monitoring_and_preserves_failure(self):
        for failure in (None,'NATIVE_CHECK_FAILED'):
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as d:
                runtime=RuntimeIO.__new__(RuntimeIO)
                runtime.launch=SimpleNamespace(poll=lambda:None);runtime.world_name='test';runtime.env={}
                runtime.output=Path(d);runtime.node=object();runtime._capture_policy=SimpleNamespace(monitor_period_s=.01)
                runtime.snapshot=lambda:dict(vacuum=dict(sim_time_s=2.))
                observations=[];polls=iter([None,None,0])
                process=SimpleNamespace(poll=lambda:next(polls),communicate=lambda:('data: true\n',''),returncode=0)
                def observe(row):observations.append(row);return failure
                with patch('runtime_io.subprocess.Popen',return_value=process),patch('rclpy.spin_once') as spin:
                    if failure:
                        with self.assertRaisesRegex(RuntimeError,failure):runtime.set_physics_paused(True,observe)
                    else:runtime.set_physics_paused(True,observe)
                self.assertEqual(len(observations),2);self.assertEqual(spin.call_count,2)
                record=json.loads((Path(d)/'planning_clock_control.jsonl').read_text())
                self.assertTrue(record['accepted']);self.assertTrue(record['pause'])

if __name__=='__main__':unittest.main()
