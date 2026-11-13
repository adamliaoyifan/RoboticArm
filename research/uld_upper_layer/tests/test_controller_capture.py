"""Actual ROS feedback delivery must survive an unspun command node."""
from pathlib import Path
from types import SimpleNamespace
import sys,tempfile,time,unittest,uuid
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from controller_capture import ControllerFeedbackCapture
from controller_trace import ControllerTrace


class IndependentControllerCapture(unittest.TestCase):
    def test_receives_without_main_spin_and_reports_callback_failure(self):
        import rclpy
        from rclpy.context import Context
        from control_msgs.msg import JointTrajectoryControllerState
        for fail in (False,True):
            with self.subTest(fail=fail),tempfile.TemporaryDirectory() as d:
                context=Context();rclpy.init(args=[],context=context)
                node=rclpy.create_node('controller_capture_test',context=context)
                topic='/controller_capture_test_'+uuid.uuid4().hex
                publisher=node.create_publisher(JointTrajectoryControllerState,topic,1)
                trace=ControllerTrace(Path(d)/'controller.jsonl.gz')
                def broken(*args):raise RuntimeError('sink broke')
                capture=ControllerFeedbackCapture(SimpleNamespace(observe=broken) if fail else trace,
                    topic,context=context)
                try:
                    message=JointTrajectoryControllerState();message.header.stamp.sec=123
                    message.joint_names=[f'elfin_joint{i}' for i in range(1,7)]
                    message.reference.positions=[.1]*6;message.feedback.positions=[.099]*6
                    deadline=time.monotonic()+3.
                    # Publisher needs no executor; only the capture spins. In
                    # production this thread is occupied with full-arm checks.
                    while not (capture.error if fail else trace.count>=3):
                        publisher.publish(message);time.sleep(.02)
                        self.assertLess(time.monotonic(),deadline)
                    if fail:self.assertIn('sink broke',capture.error)
                    else:
                        self.assertIsNone(capture.error)
                        self.assertEqual(trace.latest['stamp_s'],123.)
                        self.assertEqual(trace.latest['feedback']['positions'],[.099]*6)
                finally:
                    capture.stop();trace.close();node.destroy_node();context.shutdown()
                self.assertFalse(capture._thread.is_alive())


if __name__=='__main__':unittest.main()
