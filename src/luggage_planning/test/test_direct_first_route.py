from types import SimpleNamespace
import pytest
from luggage_planning.direct_first_route import DirectFirstRoutePlanner
from luggage_planning.grid_path_search import RoutePlanningError


@pytest.mark.parametrize('direct_success', [True, False])
def test_lazy_order_and_shared_deadline(direct_success):
    now = [0]
    calls = []
    attempts = []

    def factory(staging, deadline):
        calls.append(('prepare', staging, deadline))
        now[0] += 10
        def plan(*args, budget_ns):
            calls.append(('plan', staging, budget_ns))
            now[0] += 20
            if not staging and not direct_success:
                raise RoutePlanningError('NO_FEASIBLE_ROUTE', {})
            return 'route'
        return SimpleNamespace(plan_route=plan)

    planner = DirectFirstRoutePlanner(factory, lambda: now[0], has_staging=True, attempts=attempts)
    assert planner.plan_route(budget_ns=100) == 'route'
    assert calls[:2] == [('prepare', False, 60), ('plan', False, 50)]
    if direct_success:
        assert len(calls) == 2
    else:
        assert calls[2:] == [('prepare', True, 100), ('plan', True, 60)]
    assert attempts[-1]['outcome'] == 'planned'


def test_no_fallback_when_deadline_exhausted():
    now = [0]
    calls = []
    def factory(staging, deadline):
        calls.append(staging)
        def plan(*args, **kwargs):
            now[0] = 101
            raise RoutePlanningError('NO_FEASIBLE_ROUTE', {})
        return SimpleNamespace(plan_route=plan)
    planner = DirectFirstRoutePlanner(factory, lambda: now[0], has_staging=True, attempts=[])
    with pytest.raises(RoutePlanningError):
        planner.plan_route(budget_ns=100)
    assert calls == [False]


def test_retry_cannot_extend_original_deadline():
    now = [0]
    def factory(staging, deadline):
        def plan(*args, **kwargs):
            now[0] = deadline
            raise RoutePlanningError('NO_FEASIBLE_ROUTE', {})
        return SimpleNamespace(plan_route=plan)
    attempts = []
    planner = DirectFirstRoutePlanner(factory, lambda: now[0], has_staging=True, attempts=attempts)
    with pytest.raises(RoutePlanningError):
        planner.plan_route(budget_ns=100)
    count = len(attempts)
    with pytest.raises(RoutePlanningError):
        planner.plan_route(budget_ns=10000)
    assert len(attempts) == count
    assert planner.deadline_ns == 100
