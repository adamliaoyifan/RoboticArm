"""Independent, observation-only ROS clock boundary with owned executor.

Controller action feedback and collision batches must not determine when clock
messages are consumed. This observer changes no simulation state and records
clock/native stamp differences so bridge delay remains visible.
"""
import gzip
import json
import threading
import time


class RosClockCapture:
    def __init__(self,path,accept,native_stamp):
        import rclpy
        from rosgraph_msgs.msg import Clock
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        self._node=rclpy.create_node('upper_layer_clock_observer')
        self._executor=SingleThreadedExecutor();self._executor.add_node(self._node)
        self._trace=gzip.open(path,'wt',compresslevel=3)
        self.error=None;self._closing=False;self._count=0
        def callback(message):
            stamp=message.clock.sec+message.clock.nanosec*1e-9
            accept(stamp)
            self._trace.write(json.dumps(dict(stamp_s=stamp,native_latest_stamp_s=native_stamp(),
                                              callback_wall=time.time()))+'\n')
            self._count+=1
            if self._count%100==0:self._trace.flush()
        self._node.create_subscription(Clock,'/clock',callback,qos_profile_sensor_data)
        self._thread=threading.Thread(target=self._run,daemon=True)

    def _run(self):
        try:self._executor.spin()
        except BaseException as exc:
            if not self._closing:self.error='CLOCK_CAPTURE_ERROR:'+repr(exc)

    def start(self):self._thread.start()

    def stop(self):
        self._closing=True;self._executor.shutdown(timeout_sec=2.)
        self._thread.join(timeout=2.)
        if self._thread.is_alive():self.error='CLOCK_CAPTURE_TEARDOWN_TIMEOUT'
        self._node.destroy_node();self._trace.close()
