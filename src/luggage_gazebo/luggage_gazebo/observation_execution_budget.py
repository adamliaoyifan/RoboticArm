"""Response wait for an already-certified trajectory, not a planning limit."""
from luggage_planning.timed_execution_budget import timed_execution_wait


def certified_execution_wait(segment, overhead_s):
    return timed_execution_wait(segment.joint_trajectory.trajectory.points, overhead_s)


observation_execution_wait = certified_execution_wait
