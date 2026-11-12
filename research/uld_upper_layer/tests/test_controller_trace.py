"""Reference/feedback evidence and tracking aborts use measured joint positions."""
from pathlib import Path
from types import SimpleNamespace
import sys,tempfile,unittest,json
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from controller_trace import state_record,check_tracking


def point(values):return SimpleNamespace(positions=values,velocities=[],accelerations=[],effort=[])


class ControllerEvidence(unittest.TestCase):
    def test_deprecated_state_fields_remain_readable_and_order_is_preserved(self):
        msg=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=2,nanosec=3)),
            joint_names=[f'elfin_joint{i}' for i in range(1,7)],reference=point([]),feedback=point([]),
            desired=point([.1]*6),actual=point([.09]*6),error=point([.01]*6),output=point([]))
        row=state_record(msg)
        self.assertEqual(row['reference']['positions'],[.1]*6)
        self.assertEqual(row['feedback']['positions'],[.09]*6)
        self.assertAlmostEqual(row['stamp_s'],2.000000003)

    def test_tracking_failure_and_stale_feedback_fail_closed(self):
        state=dict(stamp_s=2.,joint_names=[f'elfin_joint{i}' for i in range(1,7)],
            reference=dict(positions=[0.]*6),feedback=dict(positions=[0.]*6))
        with tempfile.TemporaryDirectory() as d:
            checks=[]
            runtime=SimpleNamespace(output=Path(d),controller_trace=SimpleNamespace(latest=state,
                record_check=lambda age,error:checks.append((age,error))),
                snapshot=lambda:dict(vacuum=dict(sim_time_s=2.01)))
            check_tracking(runtime,.01,.1)
            state['feedback']['positions'][4]=.02
            with self.assertRaisesRegex(RuntimeError,'CONTROLLER_TRACKING_ERROR'):check_tracking(runtime,.01,.1)
            data=json.loads((Path(d)/'controller_tracking_failure.json').read_text())
            self.assertEqual(data['max_error_rad'],.02)
            self.assertEqual(len(checks),2)
            self.assertAlmostEqual(checks[-1][0],.01)
            state['stamp_s']=1.
            with self.assertRaisesRegex(RuntimeError,'CONTROLLER_FEEDBACK_NOT_CURRENT'):check_tracking(runtime,.01,.1)

if __name__=='__main__':unittest.main()
