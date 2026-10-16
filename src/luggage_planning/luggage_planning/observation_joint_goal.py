"""Choose an equivalent observation IK goal near the actual start state."""
import math


def nearest_revolute_goal(solution, reference, joint_names, joint_models):
    """Change only complete turns of bounded revolute joints.

    This is a goal representation, not a path shortcut. The edge builder and
    full-route certifier must validate the resulting motion normally. Unknown
    joint models and non-revolute joints retain the original solution.
    """
    if len(solution) != len(reference) or len(solution) != len(joint_names):
        raise ValueError('joint goal and reference lengths differ')
    result = []
    turn = 2 * math.pi
    for value, start, name in zip(solution, reference, joint_names):
        value, start = float(value), float(start)
        if not math.isfinite(value) or not math.isfinite(start):
            raise ValueError('joint goal and reference must be finite')
        model = joint_models.get(name, {})
        if model.get('type') != 'revolute':
            result.append(value)
            continue
        lower, upper = float(model['lower']), float(model['upper'])
        if not math.isfinite(lower) or not math.isfinite(upper) or not lower <= value <= upper:
            raise ValueError('IK solution outside finite joint limits')
        low_turn = math.ceil((lower - value) / turn)
        high_turn = math.floor((upper - value) / turn)
        nearest = min(high_turn, max(low_turn, round((start - value) / turn)))
        result.append(value + nearest * turn)
    return tuple(result)
