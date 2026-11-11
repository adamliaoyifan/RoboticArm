"""Controller boundary: send certified joint trajectories, observe actual state.

This module sends no box poses. The caller owns contact-mode transitions and
must supply a monitor which returns a stop reason on any invariant violation.
"""
import time


class ArmController:
    def __init__(self,runtime,action_name='/elfin_arm_controller/follow_joint_trajectory'):
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        self.runtime=runtime;self.action_type=FollowJointTrajectory
        self.client=ActionClient(runtime.node,FollowJointTrajectory,action_name)

    def _wait(self,future,deadline,monitor=None):
        import rclpy
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('CONTROLLER_RESPONSE_TIMEOUT')
            rclpy.spin_once(self.runtime.node,timeout_sec=.01)
            if monitor:
                reason=monitor(self.runtime.snapshot())
                if reason:raise RuntimeError(reason)
        return future.result()

    def execute(self,trajectory,monitor,wall_budget_s):
        from trajectory_msgs.msg import JointTrajectoryPoint
        if not self.client.wait_for_server(timeout_sec=5.):
            raise RuntimeError('ARM_CONTROLLER_UNAVAILABLE')
        goal=self.action_type.Goal()
        goal.trajectory.joint_names=[f'elfin_joint{i}' for i in range(1,7)]
        for q,velocity,stamp in zip(trajectory['positions'],trajectory['velocities'],
                                    trajectory['time_from_start']):
            point=JointTrajectoryPoint();point.positions=list(q);point.velocities=list(velocity)
            seconds=int(stamp);point.time_from_start.sec=seconds
            point.time_from_start.nanosec=int(round((stamp-seconds)*1e9))
            if point.time_from_start.nanosec==1_000_000_000:
                point.time_from_start.sec+=1;point.time_from_start.nanosec=0
            goal.trajectory.points.append(point)
        deadline=time.monotonic()+wall_budget_s
        handle=self._wait(self.client.send_goal_async(goal),deadline)
        if not handle.accepted:raise RuntimeError('ARM_TRAJECTORY_REJECTED')
        try:
            response=self._wait(handle.get_result_async(),deadline,monitor)
            result=response.result
            if result.error_code!=self.action_type.Result.SUCCESSFUL:
                raise RuntimeError(f'ARM_CONTROLLER_ERROR:{result.error_code}:{result.error_string}')
            return dict(error_code=result.error_code,error_string=result.error_string,
                        terminal_observation=self.runtime.snapshot())
        except BaseException:
            # Keep observations alive while the controller acknowledges stop.
            self._wait(handle.cancel_goal_async(),time.monotonic()+5.)
            raise
