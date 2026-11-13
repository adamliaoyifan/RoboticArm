"""Bounded candidate search for the diagnostic's empty site-view motion.

MoveGroup still validates and executes each candidate. A terminal invalid plan
with no executed trajectory consumes an attempt, not the entire observation job.
Never retry partial execution, execution errors or uncertain Action transport.
"""
import copy
import json
import time
from action_msgs.msg import GoalStatus
from moveit_msgs.msg import MoveItErrorCodes


def execute_observation_goal(host, goal, name, *, timeout, clock=time.monotonic):
    attempts=goal.request.num_planning_attempts
    if attempts<1:raise ValueError('POSITIVE_OBSERVATION_PLANNING_ATTEMPTS_REQUIRED')
    deadline=clock()+timeout
    candidate=copy.deepcopy(goal)
    candidate.request.num_planning_attempts=1
    for index in range(attempts):
        remaining=deadline-clock()
        if remaining<=0:raise TimeoutError('OBSERVE_PLANNING_DEADLINE')
        wrapped=host.action(host.move,candidate,name,remaining,deadline=deadline)
        result=wrapped.result
        code=result.error_code.val
        executed=result.executed_trajectory
        retry=(wrapped.status==GoalStatus.STATUS_ABORTED and code==MoveItErrorCodes.INVALID_MOTION_PLAN
            and not executed.joint_trajectory.points and not executed.multi_dof_joint_trajectory.points)
        host.record('observation_planning_attempt.json',json.dumps(dict(
            attempt=index+1,max_attempts=attempts,status=wrapped.status,error_code=code,
            joint_points_executed=len(executed.joint_trajectory.points),
            multi_dof_points_executed=len(executed.multi_dof_joint_trajectory.points),
            retryable_empty_invalid_plan=retry,remaining_s=max(0.,deadline-clock()))).encode())
        if clock()>=deadline:raise TimeoutError('OBSERVE_PLANNING_DEADLINE')
        if wrapped.status==GoalStatus.STATUS_SUCCEEDED and code==MoveItErrorCodes.SUCCESS:
            return wrapped
        if not retry or index+1==attempts:
            raise RuntimeError('OBSERVE_MOVEIT:'+str(code)+':ACTION_STATUS:'+str(wrapped.status))
