from types import SimpleNamespace as NS
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import MoveItErrorCodes
from luggage_planning.motion.motion_executor import MotionExecutor


def test_native_executor_wait_includes_exact_admitted_duration(monkeypatch):
    from luggage_planning.motion import motion_frame_adapters
    points = [JointTrajectoryPoint(positions=[q]*6, velocities=[0.0]*6,
              time_from_start=Duration(sec=t, nanosec=0)) for q,t in [(0.0,0),(0.1,175)]]
    monkeypatch.setattr(motion_frame_adapters, 'timed_trajectory',
                        lambda admitted: ([f'elfin_joint{i}' for i in range(1,7)],points))
    executor = object.__new__(MotionExecutor)
    goals = []
    executor._execute = NS(wait_for_server=lambda **kwargs: True,
                           send_goal_async=lambda goal: goals.append(goal))
    executor._wait_future = lambda future, timeout: NS(accepted=True)
    executor._notify = lambda *args: None
    waits = []
    def wait(handle, timeout):
        waits.append(timeout)
        # Controller completes later than the configured overhead alone.
        if timeout < 175.2:
            return None
        return NS(result=NS(error_code=NS(val=MoveItErrorCodes.SUCCESS)))
    executor._wait_goal = wait
    executor._result = lambda success, message, fraction, **kwargs: NS(success=success)
    result = executor._execute_timed_joint_v2(NS(name='transit'), None, 90.0)
    assert result.success
    assert waits == [265.0]
    assert goals[0].trajectory.joint_trajectory.points == points
