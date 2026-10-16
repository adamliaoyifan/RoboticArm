import math
import pytest
from luggage_planning.observation_joint_goal import nearest_revolute_goal


def test_wrap_reduces_actual_wrist_motion_without_changing_orientation():
    goal = [-2.326786302484541, -2.648232939126331]
    start = [4.85568723350709, 3.4318731711527826]
    models = {name: dict(type='revolute', lower=-6.28, upper=6.28)
              for name in ['j4', 'j6']}
    result = nearest_revolute_goal(goal, start, ['j4', 'j6'], models)
    for original, adjusted, current in zip(goal, result, start):
        assert math.sin(adjusted) == pytest.approx(math.sin(original))
        assert math.cos(adjusted) == pytest.approx(math.cos(original))
        assert abs(adjusted-current) < 1.
        assert -6.28 <= adjusted <= 6.28


def test_limits_and_prismatic_joint_prevent_invalid_wraps():
    models = {'limited': dict(type='revolute', lower=-3., upper=3.),
              'linear': dict(type='prismatic', lower=-10., upper=10.)}
    assert nearest_revolute_goal([-2., -2.], [2., 5.], ['limited', 'linear'], models) == (-2., -2.)
    with pytest.raises(ValueError):
        nearest_revolute_goal([4.], [2.], ['limited'], models)
