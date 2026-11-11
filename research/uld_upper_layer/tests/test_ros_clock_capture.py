"""Real ROS clock delivery while the unrelated main node is not spun."""
from pathlib import Path
import gzip
import json
import sys
import tempfile
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


class ClockCaptureContracts(unittest.TestCase):
    def test_owned_executor_receives_clock_without_controller_node_polling(self):
        import rclpy
        from rosgraph_msgs.msg import Clock
        from ros_clock_capture import RosClockCapture
        rclpy.init()
        main=rclpy.create_node('upper_clock_contract_publisher')
        publisher=main.create_publisher(Clock,'/clock',10)
        received=[]
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'clock.jsonl.gz'
            capture=RosClockCapture(path,received.append,lambda:1.)
            capture.start()
            try:
                deadline=time.monotonic()+5.
                while time.monotonic()<deadline and not received:
                    message=Clock();message.clock.sec=1;publisher.publish(message)
                    # Deliberately no spin_once(main): controller callback
                    # activity cannot be needed for this observer to progress.
                    time.sleep(.01)
                self.assertTrue(received)
                self.assertEqual(received[-1],1.)
                self.assertIsNone(capture.error)
            finally:
                capture.stop();main.destroy_node();rclpy.shutdown()
            rows=[json.loads(line) for line in gzip.open(path,'rt')]
            self.assertTrue(rows)
            self.assertEqual(rows[-1]['native_latest_stamp_s'],1.)


if __name__=='__main__':unittest.main()
