"""G4-A gen3 M3 negative controls (plan d87c5731 section 6.2): every
synthetic bad-input class must make the judge FAIL or NOT_EVALUATED --
the judge may never rescue a PASS by filtering. Pure-function tests on
occ_metrics_g3.judge; no runner required.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    'scripts', 'g3'))

from occ_metrics_g3 import (  # noqa: E402
    FAIL, NOT_EVALUATED, PASS, judge, judge_query_gate,
    judge_visibility, nearest_rank_p95)

GATES = {'query': {'p95_bound_ms': 100.0, 'max_bound_ms': 250.0},
         'visibility': {'p95_bound_ms': 1000.0, 'max_bound_ms': 2000.0}}


def query(i, ms=10.0, status='ok', state='READY', startup=False,
          reason=None):
    return {'kind': 'query', 'request_id': 'q%d' % i, 'expected':
            'CURRENT', 'returned': None, 'status': status,
            'reason': reason, 'total_ms': ms,
            'state_at_request': state, 'startup': startup}


def commit(i, revision, t_ns, ledger_id=None, accepted=True):
    return {'kind': 'commit', 'op_id': 'c%d' % i, 'ledger_id': ledger_id,
            'identity': ['e', revision, 'a', 'b', revision],
            'epoch': 'e', 'accepted': accepted, 'replayed': False,
            't_ns': t_ns, 'write_ms': 100.0}


def ledger(i, revision, status, seq, commit_ns, observed_ns):
    return {'kind': 'ledger', 'ledger_id': i,
            'identity': ['e', revision, 'a', 'b', revision],
            'commit_ns': commit_ns, 'status': status,
            'published_seq': seq, 'observed_ns': observed_ns}


def ready(seq, revision, t_ns):
    return {'event': 'ready', 'seq': seq,
            'target_identity': ['e', revision, 'a', 'b', revision],
            't_ns': t_ns}


def healthy_events(n_queries=40, n_commits=35):
    events, t = [], 0
    for i in range(n_commits):
        t += 10_000_000                     # 10 ms apart
        events.append(commit(i, i, t, ledger_id=i))
        events.append(ledger(i, i, 'exact_visible', i + 1, t, t + 5e6))
        events.append(ready(i + 1, i, t + 6_000_000))
    for i in range(n_queries):
        events.append(query(i, ms=8.0 + (i % 5)))
    return events


def test_p95_is_the_29th_of_30_sorted():
    values = list(range(1, 31))             # 1..30 ms
    assert nearest_rank_p95(values) == 29   # ceil(.95*30)-1 -> index 28
    assert nearest_rank_p95(values[:29]) is None   # under-sized


def test_healthy_stream_passes():
    verdict = judge(healthy_events(), GATES)
    assert verdict['exit'] == PASS, verdict


def test_29_samples_is_not_evaluated_never_pass():
    events = healthy_events(n_queries=29)
    code, report = judge_query_gate(
        [e for e in events if e.get('kind') == 'query'], 100.0, 250.0)
    assert code == NOT_EVALUATED
    assert report['p95_ms'] is None


def test_dropped_commit_fails_visibility():
    events = healthy_events()
    t = 10_000_000 * 100
    # A 36th commit whose generation NEVER becomes ready.
    events.append(commit(99, 99, t, ledger_id=99))
    events.append(ledger(99, 99, 'exact_visible', 100, t, t + 5e6))
    # no ready(seq=100) -- dropped on the wire
    code, report = judge_visibility(
        [e for e in events if e.get('kind') == 'commit'],
        [e for e in events if e.get('kind') == 'ledger'],
        [e for e in events if e.get('event') == 'ready'])
    assert code == FAIL
    assert report['by_status'].get('not_delivered') == 1


def test_forever_old_ready_fails_visibility():
    """100 repeated reads of ONE stale generation must not fake
    delivery for 35 newer commits (review P1: 713 samples vs 177
    writes)."""
    events, t = [], 0
    for i in range(35):
        t += 10_000_000
        events.append(commit(i, i + 1, t, ledger_id=i))
        events.append(ledger(i, i + 1, 'exact_visible', i + 2, t,
                             t + 5e6))
    events.append(ready(1, 0, 5_000_000))   # receiver stuck at rev 0
    for i in range(100):
        events.append(query(i, ms=9.0))     # all succeed on stale map
    verdict = judge(events, GATES)
    assert verdict['gates']['visibility']['verdict'] == FAIL


def test_non_ready_silent_success_fails_query_gate():
    """err is None while the replica is not READY must be a MEASUREMENT
    VIOLATION, not a silent drop (review P1 measurement hole)."""
    events = healthy_events()
    for i in range(10):
        events.append(query(1000 + i, ms=9.0, state='RESYNC_REQUIRED'))
    code, report = judge_query_gate(
        [e for e in events if e.get('kind') == 'query'], 100.0, 250.0)
    assert code == FAIL
    assert report['violation_count'] == 10
    assert report['violations'][0]['problem'] == 'non_ready_silent'


def test_named_rejects_reported_not_dropped():
    """Named rejects are LEGAL strict-contract terminals: the gate
    still passes with bounds intact, but the rate and reasons are
    REPORTED -- never silently dropped from the record."""
    events = healthy_events()
    for i in range(5):
        events.append(query(2000 + i, status='named_reject', ms=None,
                            reason='REPLICA_LAGGING'))
    code, report = judge_query_gate(
        [e for e in events if e.get('kind') == 'query'], 100.0, 250.0)
    assert code == PASS
    assert report['named_rejects'] == 5
    assert report['named_reject_reasons'] == ['REPLICA_LAGGING']
    assert report['steady_ok'] == 40


def test_exceptions_fail_query_gate():
    events = healthy_events()
    for i in range(3):
        events.append(query(3000 + i, status='exception', ms=None))
    code, report = judge_query_gate(
        [e for e in events if e.get('kind') == 'query'], 100.0, 250.0)
    assert code == FAIL
    assert report['steady_failures'] == 3


def test_rejects_alone_cannot_pass():
    """Only rejects and 5 samples: insufficient completed -> the gate
    is NOT_EVALUATED, never a PASS rescued by filtered samples."""
    events = [query(i, status='named_reject', ms=None,
                    reason='REPLICA_LAGGING') for i in range(35)]
    code, report = judge_query_gate(events, 100.0, 250.0)
    assert code == NOT_EVALUATED
    assert report['steady_ok'] == 0


def test_cold_start_full_covers_early_commits():
    """Commits whose own seqs were superseded by the receiver's first
    FULL: any LATER same-epoch READY covers them."""
    events, t = [], 0
    for i in range(35):
        t += 10_000_000
        events.append(commit(i, i, t, ledger_id=i))
        events.append(ledger(i, i, 'exact_visible', i + 1, t, t + 5e6))
    # receiver cold-starts at seq 20: only ready(20) then 21..
    events.append(ready(20, 19, 5 * 10_000_000 + 2_000_000))
    for i in range(20, 35):
        events.append(ready(i + 1, i, (i + 1) * 10_000_000 + 2_000_000))
    for i in range(40):
        events.append(query(i, ms=9.0))
    code, report = judge_visibility(
        [e for e in events if e.get('kind') == 'commit'],
        [e for e in events if e.get('kind') == 'ledger'],
        [e for e in events if e.get('event') == 'ready'])
    assert code == PASS, report['by_status']
    by = report['by_status']
    assert by.get('exact_visible', 0) + by.get('coalesced_into', 0) == 35


def test_export_failed_ledger_entry_fails_visibility():
    events = healthy_events()
    t = 10_000_000 * 100
    events.append(commit(99, 99, t, ledger_id=99))
    events.append(ledger(99, 99, 'export_failed', None, t, t + 5e6))
    code, report = judge_visibility(
        [e for e in events if e.get('kind') == 'commit'],
        [e for e in events if e.get('kind') == 'ledger'],
        [e for e in events if e.get('event') == 'ready'])
    assert code == FAIL


def test_new_world_ready_never_erases_undelivered_old_commit():
    events = healthy_events()
    t = 10_000_000 * 100
    events.append(commit(98, 98, t, ledger_id=98))       # old world
    events.append(ledger(98, 98, 'superseded_by_reset', None, t, t))
    events.append(commit(99, 99, t + 1_000_000, ledger_id=99))
    events.append(ledger(99, 99, 'exact_visible', 200, t + 1e6, t + 2e6))
    events.append(ready(200, 99, t + 3_000_000))         # new world READY
    code, report = judge_visibility(
        [e for e in events if e.get('kind') == 'commit'],
        [e for e in events if e.get('kind') == 'ledger'],
        [e for e in events if e.get('event') == 'ready'])
    assert code == FAIL
    assert report['by_status'].get('superseded_reset') == 1


def test_coalesced_delivery_counts_by_covering_generation():
    events, t = [], 0
    for i in range(35):
        t += 10_000_000
        events.append(commit(i, i, t, ledger_id=i))
    # facts 0..33 ride along in generation 34; fact 34 is exact
    for i in range(34):
        events.append(ledger(i, i, 'coalesced_into', 34,
                             t, t + 5e6))
    events.append(ledger(34, 34, 'exact_visible', 34, t, t + 5e6))
    events.append(ready(34, 34, t + 6_000_000))
    for i in range(40):
        events.append(query(i, ms=9.0))
    verdict = judge(events, GATES)
    assert verdict['exit'] == PASS
    by = verdict['gates']['visibility']['report']['by_status']
    assert by['coalesced_into'] == 34 and by['exact_visible'] == 1


def test_slow_visibility_p95_fails():
    events, t = [], 0
    for i in range(35):
        t += 10_000_000
        events.append(commit(i, i, t, ledger_id=i))
        events.append(ledger(i, i, 'exact_visible', i + 1, t, t + 5e6))
        events.append(ready(i + 1, i, t + 1_500_000_000))  # 1.5 s each
    code, report = judge_visibility(
        [e for e in events if e.get('kind') == 'commit'],
        [e for e in events if e.get('kind') == 'ledger'],
        [e for e in events if e.get('event') == 'ready'])
    assert code == FAIL
    assert report['p95_ms'] > 1000.0
