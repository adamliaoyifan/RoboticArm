"""Missing, duplicated or truncated T1 data must prevent certification."""
import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[3]/'scripts/g3/occ_query_runtime_campaign.py'
spec = importlib.util.spec_from_file_location('runtime_capture_health', SCRIPT)
campaign = importlib.util.module_from_spec(spec)
spec.loader.exec_module(campaign)


def fixture():
    owner = dict(snapshot_trace_count=1, snapshot_trace_dropped=0)
    events = [dict(stage='snapshot', request_id='roi-3-trace_on', started_ns=10, elapsed_ns=5)]
    observations = [dict(case='roi', index=3, trace='trace_on'),
                    dict(case='roi', index=3, trace='trace_off'),
                    dict(case='stats', index=3, trace='trace_on')]
    return owner, events, observations


def test_complete_join_passes_without_claiming_full_performance_samples():
    result = campaign.trace_capture_health(*fixture())
    assert result['outcome'] == 'pass'
    assert result['expected'] == result['actual'] == 1


@pytest.mark.parametrize('fault', ['missing', 'unexpected', 'count', 'overflow', 'time', 'duplicate'])
def test_capture_fault_fails(fault):
    owner, events, observations = fixture()
    if fault == 'missing':
        events.clear()
    elif fault == 'unexpected':
        events[0]['request_id'] = 'wrong-join'
    elif fault == 'count':
        owner['snapshot_trace_count'] = 2
    elif fault == 'overflow':
        owner['snapshot_trace_dropped'] = 1
    elif fault == 'time':
        events[0]['elapsed_ns'] = -1
    elif fault == 'duplicate':
        observations.append(observations[0])
        events.append(events[0])
        owner['snapshot_trace_count'] = 2
    assert campaign.trace_capture_health(owner, events, observations)['outcome'] == 'fail'


def test_trace_only_overhead_failure_is_known_before_owner_teardown():
    results = {'roi': {mode: dict(p95_ms=1.) for mode in ('trace_off', 'trace_on')}}
    rows = [dict(case='roi', index=index, trace=mode, cold=False, warmup=False,
                 elapsed_ns=1000 if mode == 'trace_off' else 1200)
            for index in range(30) for mode in ('trace_off', 'trace_on')]
    assert campaign.instrumentation_results(results, rows)['roi']['outcome'] == 'fail'
    rows.pop()
    assert campaign.instrumentation_results(results, rows)['roi']['outcome'] == 'not_evaluated'
