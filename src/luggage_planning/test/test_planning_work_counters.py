"""Observation preserves solver returns and exceptions without inventing work."""
import pytest

from luggage_planning.routing.work_counters import PlanningWorkCounters


class EdgeBoundary:
    def moveit_call_counts(self):
        return {'move_group': 2, 'cartesian': 3, 'scene_apply': 4}

    def call_durations(self):
        return {'move_group': [0.2, 0.3]}


def test_refinement_observation_preserves_operation_and_unknown_metrics():
    counters = PlanningWorkCounters()
    result = object()
    observed = []

    def solve(seed, *, tolerance):
        observed.append((seed, tolerance))
        return result

    assert counters.refine(solve, [1, 2], tolerance=0.002) is result
    assert observed == [([1, 2], 0.002)]
    failure = RuntimeError('solver failure')

    def failing():
        raise failure

    with pytest.raises(RuntimeError) as caught:
        counters.refine(failing)
    assert caught.value is failure
    snapshot = counters.snapshot(EdgeBoundary())
    assert snapshot['offline_refinement_calls'] == 2
    assert snapshot['offline_refinement_exceptions'] == 1
    assert snapshot['edge_boundary_calls']['cartesian'] == 3
    assert snapshot['edge_boundary_durations_s']['move_group'] == [0.2, 0.3]
    for metric in ('native_ik_calls', 'native_collision_checks', 'total_service_calls'):
        assert snapshot[metric] is None
    assert snapshot['complete_for_benchmark'] is False
