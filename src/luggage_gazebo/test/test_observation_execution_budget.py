from types import SimpleNamespace as NS
import pytest
from luggage_gazebo.observation_execution_budget import observation_execution_wait


def segment(times):
    return NS(joint_trajectory=NS(trajectory=NS(points=[
        NS(time_from_start=NS(sec=s,nanosec=n)) for s,n in times])))


def test_wait_includes_certified_motion_without_changing_trajectory():
    value=segment([(0,0),(77,311627469)])
    assert observation_execution_wait(value,60)==pytest.approx(137.311627469)
    assert value.joint_trajectory.trajectory.points[-1].time_from_start.sec==77


@pytest.mark.parametrize('times',[[],[(1,0),(0,0)],[(0,0),(0,0)],[(-1,0)],[(0,1_000_000_000)]])
def test_refuses_invalid_certified_times(times):
    with pytest.raises(ValueError):observation_execution_wait(segment(times),60)
