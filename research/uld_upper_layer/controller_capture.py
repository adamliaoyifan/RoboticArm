"""Dedicated read-only ROS executor for aligned controller feedback.

Collision/contact validation can take several callbacks on the command thread.
Feedback freshness must not depend on that thread choosing this subscription.
The runtime owns this capture and stops it before closing the trace or context.
"""
import threading


class ControllerFeedbackCapture:
    def __init__(self,trace,topic='/elfin_arm_controller/controller_state',context=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy
        from control_msgs.msg import JointTrajectoryControllerState
        self.error=None
        self._node=rclpy.create_node('upper_layer_controller_feedback',context=context)
        self._executor=SingleThreadedExecutor(context=context)
        self._executor.add_node(self._node)
        self._node.create_subscription(JointTrajectoryControllerState,topic,
            lambda message:trace.observe(message,topic),
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self._thread=threading.Thread(target=self._spin,name='controller-feedback',daemon=True)
        self._thread.start()

    def _spin(self):
        try:self._executor.spin()
        except Exception as exc:self.error='CONTROLLER_FEEDBACK_CAPTURE_FAILED:'+repr(exc)

    def stop(self):
        self._executor.shutdown(timeout_sec=3.)
        self._thread.join(timeout=3.)
        if self._thread.is_alive():raise RuntimeError('CONTROLLER_FEEDBACK_CAPTURE_DID_NOT_STOP')
        self._node.destroy_node()
