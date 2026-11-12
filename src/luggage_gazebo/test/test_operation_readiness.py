from types import SimpleNamespace
import pytest
from luggage_gazebo.operation_readiness import wait_active_controllers


def states(arm, broadcaster='active'):
    return SimpleNamespace(controller=[SimpleNamespace(name='joint_state_broadcaster',state=broadcaster),
        SimpleNamespace(name='elfin_arm_controller',state=arm)])


def test_sensor_early_controller_inactive_is_not_terminal_failure():
    samples=iter([states('unconfigured'),states('inactive'),states('active')]);now=[0.]
    result=wait_active_controllers(lambda:next(samples),deadline=1.,clock=lambda:now[0],
        pause=lambda seconds:now.__setitem__(0,now[0]+seconds))
    assert result.controller[1].state=='active' and now[0]==.4


@pytest.mark.parametrize('sample',[states('inactive'),states('active','inactive'),SimpleNamespace(controller=[])])
def test_neither_half_ready_nor_missing_controllers_authorize_cycle(sample):
    now=[0.]
    with pytest.raises(TimeoutError):
        wait_active_controllers(lambda:sample,deadline=.5,clock=lambda:now[0],
            pause=lambda seconds:now.__setitem__(0,now[0]+seconds))
    assert now[0]==.5
