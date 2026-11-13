from luggage_planning.motion.observation_ik_search import observation_goals


def test_local_failure_reaches_other_branch_before_legacy_fallback():
    calls = []
    def refine(p, q, seed):
        calls.append(seed)
        return (None, {}) if len(calls) == 1 else (seed, {})
    results = observation_goals(refine, (), (), [0.] * 6, [1.] * 6, lambda: False)
    assert next(results)[0] is None
    goal, info = next(results)
    assert info['seed'].startswith('j6-')
    assert goal[5] < -3.
    assert len(calls) == 2  # Lazy: no extra solve after an accepted route.


def test_duplicate_solutions_and_expired_budget_do_not_spend_more_work():
    calls = []
    def refine(p, q, seed):
        calls.append(seed)
        return [0.] * 6, {}
    assert len(list(observation_goals(refine, (), (), [0.] * 6, [1.] * 6, lambda: False))) == 1
    assert len(calls) == 6
    calls.clear()
    rows = list(observation_goals(refine, (), (), [0.] * 6, [1.] * 6, lambda: True))
    assert rows[0][1]['reason'] == 'PLANNING_BUDGET_EXHAUSTED'
    assert not calls
