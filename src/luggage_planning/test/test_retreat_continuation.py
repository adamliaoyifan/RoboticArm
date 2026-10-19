import math
import pytest
from luggage_planning.retreat_continuation import refine_retreat


def test_large_total_joint_travel_is_continuous_locally():
    seeds = []
    def refine(p, orientation, seed):
        seeds.append(seed)
        return (p[0] * 2, 0, 0, 0, 0, 0)
    q, evidence = refine_retreat((0, 0, 0), (1, 0, 0), (0, 0, 0, 1),
                                (0,) * 6, refine, lambda: False)
    assert q == (2, 0, 0, 0, 0, 0)
    assert len(seeds) == 100
    assert seeds[-1][0] == pytest.approx(1.98)
    assert evidence['reason'] == 'refined'


@pytest.mark.parametrize('jump', [1.1, 2 * math.pi])
def test_local_jump_rejects_even_wrapped_equivalent(jump):
    q, e = refine_retreat((0, 0, 0), (.1, 0, 0), (0, 0, 0, 1), (0,) * 6,
                         lambda *args: (jump, 0, 0, 0, 0, 0), lambda: False)
    assert q is None and e['reason'] == 'branch_change'
    assert len(e['steps']) == 1


def test_budget_and_extent_prevent_solves():
    def forbidden(*args):
        raise AssertionError('must not solve')
    for target, expired, reason in [((2, 0, 0), False, 'step_limit'),
                                    ((.1, 0, 0), True, 'deadline')]:
        q, e = refine_retreat((0, 0, 0), target, (0, 0, 0, 1), (0,) * 6,
                             forbidden, lambda: expired)
        assert q is None and e['reason'] == reason


def test_ik_miss_returns_no_partial_endpoint():
    q, e = refine_retreat((0, 0, 0), (.1, 0, 0), (0, 0, 0, 1), (0,) * 6,
                         lambda *args: None, lambda: False)
    assert q is None and e['reason'] == 'ik_failed'
