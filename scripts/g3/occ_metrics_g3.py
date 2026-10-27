#!/usr/bin/env python3
"""G4-A gen3 pure measurement judge (plan d87c5731 section 6.2).

Everything here is a PURE function over event records (JSONL lines).
The runner only COLLECTS events; every reported number must be
recomputable offline from the raw event file. The judge is deliberately
refusal-capable: synthetic bad inputs (stale READY, dropped commits,
under-sized samples, filtered failures, unknown identities) must make
the corresponding gate FAIL or not_evaluated -- never PASS.

Event kinds (see occ_concurrent_representative.py for the producer):
  commit   {op_id, identity[5], epoch, accepted, replayed, t_ns, write_ms}
  ledger   {ledger_id, identity[5]|null, commit_ns, status,
            published_seq, observed_ns}   (terminal ledger entries only)
  ready    {seq, target_identity[5], t_ns}  (receiver READY transition)
  resync   {reason, seq, t_ns}
  query    {request_id, expected, returned, status, total_ms,
            state_at_request, startup}

Gate results use the frozen exit protocol: 0 pass / 3 fail /
4 not_evaluated.
"""

import math

PASS, FAIL, NOT_EVALUATED = 0, 3, 4


def nearest_rank_p95(values, min_samples=30):
    """Nearest-rank P95: sorted[ceil(0.95*n)-1]. Returns None (=> the
    calling gate is not_evaluated) when fewer than min_samples."""
    values = sorted(v for v in values if v is not None)
    if len(values) < min_samples:
        return None
    return values[math.ceil(0.95 * len(values)) - 1]


# -- request closure ----------------------------------------------------

LEGAL_QUERY_TERMINALS = ('ok', 'named_reject', 'exception', 'timeout',
                         'incomplete')


def close_requests(queries):
    """Every query request must carry EXACTLY ONE terminal state.
    Returns (closed, violations). A query that succeeded while the
    replica was not READY is NOT ok -- it is 'non_ready_silent', a
    measurement violation (review_8a372dee P1-3 measurement hole)."""
    closed, violations = [], []
    for query in queries:
        status = query.get('status')
        if status == 'ok' and query.get('state_at_request') != 'READY':
            violations.append(
                {'request_id': query.get('request_id'),
                 'problem': 'non_ready_silent',
                 'state': query.get('state_at_request')})
            continue
        if status not in LEGAL_QUERY_TERMINALS:
            violations.append(
                {'request_id': query.get('request_id'),
                 'problem': 'missing_terminal',
                 'status': status})
            continue
        closed.append(query)
    return closed, violations


def judge_query_gate(queries, p95_bound_ms, max_bound_ms=None,
                     min_samples=30):
    """PASS requires: every request closed (no silent non-READY
    success), >=min_samples completed, P95 and max within bounds.
    Failures are REPORTED, never filtered away to rescue a PASS."""
    closed, violations = close_requests(queries)
    steady = [q for q in closed if q['status'] == 'ok'
              and not q.get('startup')]
    failures = [q for q in closed if q['status'] != 'ok'
                and not q.get('startup')]
    startup = [q for q in queries if q.get('startup')]
    report = {'total': len(queries), 'steady_ok': len(steady),
              'steady_failures': len(failures), 'startup': len(startup),
              'violations': violations[:10],
              'violation_count': len(violations)}
    if violations:
        return FAIL, report
    if failures:
        report['failure_statuses'] = sorted(
            {q['status'] for q in failures})
        return FAIL, report
    p95 = nearest_rank_p95([q['total_ms'] for q in steady],
                           min_samples=min_samples)
    report['p95_ms'] = p95
    report['max_ms'] = max((q['total_ms'] for q in steady), default=None)
    if p95 is None:
        report['reason'] = 'insufficient_samples'
        return NOT_EVALUATED, report
    if p95 > p95_bound_ms:
        return FAIL, report
    if max_bound_ms is not None and report['max_ms'] > max_bound_ms:
        return FAIL, report
    return PASS, report


# -- visibility ledger join ---------------------------------------------

def judge_visibility(commits, ledgers, readies, deadline_s=2.0,
                     p95_bound_ms=1000.0, max_bound_ms=2000.0,
                     min_samples=30):
    """Per accepted commit, exactly ONE terminal:

      exact_visible     receiver reached the SAME full identity
      coalesced_into    receiver reached the APPROVED covering
                        generation (the published_seq the exporter
                        assigned to this fact)
      not_delivered     no covering generation READY within deadline
                        (timeout / export_failed stays a failure -- a
                        later new-world READY never erases it)
      superseded_reset  world reset before delivery (NOT success)

    Latency = commit -> first READY of the covering generation.
    Undelivered commits stay in the denominator."""
    ready_by_seq = {}
    ready_by_identity = {}
    for event in readies:
        seq = int(event['seq'])
        ready_by_seq.setdefault(seq, event['t_ns'])
        identity = tuple(event['target_identity'])
        ready_by_identity.setdefault(identity, event['t_ns'])
    ledger_by_id = {int(entry['ledger_id']): entry for entry in ledgers}

    outcomes, latencies = [], []
    delivered = failed = 0
    for commit in commits:
        if not commit.get('accepted'):
            continue
        t_commit = commit['t_ns']
        identity = tuple(commit['identity'])
        outcome = {'op_id': commit.get('op_id'),
                   'identity': list(identity),
                   'commit_t_ns': t_commit, 'status': None,
                   'ready_t_ns': None, 'latency_ms': None}
        if identity in ready_by_identity:
            ready_ns = ready_by_identity[identity]
            outcome.update(status='exact_visible', ready_t_ns=ready_ns,
                           latency_ms=(ready_ns - t_commit) / 1e6)
        else:
            entry = ledger_by_id.get(commit.get('ledger_id'))
            seq = entry and entry.get('published_seq')
            if entry and entry['status'] == 'superseded_by_reset':
                outcome['status'] = 'superseded_reset'
            elif entry and entry['status'] in ('exact_visible',
                                               'coalesced_into') \
                    and seq in ready_by_seq:
                ready_ns = ready_by_seq[seq]
                outcome.update(status='coalesced_into',
                               ready_t_ns=ready_ns,
                               latency_ms=(ready_ns - t_commit) / 1e6)
            else:
                outcome['status'] = 'not_delivered'
        if outcome['status'] in ('exact_visible', 'coalesced_into'):
            delivered += 1
            latencies.append(outcome['latency_ms'])
        else:
            failed += 1
        outcomes.append(outcome)

    report = {'commits': len(outcomes), 'delivered': delivered,
              'undelivered': failed,
              'by_status': {}, 'outcomes': outcomes}
    for outcome in outcomes:
        report['by_status'][outcome['status']] = (
            report['by_status'].get(outcome['status'], 0) + 1)
    if not outcomes:
        report['reason'] = 'no_accepted_commits'
        return NOT_EVALUATED, report
    if failed:
        return FAIL, report        # undelivered stays in the denominator
    p95 = nearest_rank_p95(latencies, min_samples=min_samples)
    report['p95_ms'] = p95
    report['max_ms'] = max(latencies, default=None)
    if p95 is None:
        report['reason'] = 'insufficient_samples'
        return NOT_EVALUATED, report
    if p95 > p95_bound_ms or (max_bound_ms is not None
                              and report['max_ms'] > max_bound_ms):
        return FAIL, report
    return PASS, report


def judge(events, gates):
    """Evaluate all gates from one raw event stream.

    gates: dict like {'query': {'p95_bound_ms': 100, 'max_bound_ms':
    250}, 'visibility': {'p95_bound_ms': 1000, 'max_bound_ms': 2000}}
    Returns {'exit': 0|3|4, 'gates': {...}} -- the worst applicable
    verdict wins (fail > not_evaluated > pass)."""
    queries = [e for e in events if e.get('kind') == 'query']
    commits = [e for e in events if e.get('kind') == 'commit']
    ledgers = [e for e in events if e.get('kind') == 'ledger']
    readies = [e for e in events if e.get('event') == 'ready']
    results = {}
    if 'query' in gates:
        spec = gates['query']
        results['query'] = judge_query_gate(
            queries, spec['p95_bound_ms'],
            spec.get('max_bound_ms'),
            min_samples=spec.get('min_samples', 30))
    if 'visibility' in gates:
        spec = gates['visibility']
        results['visibility'] = judge_visibility(
            commits, ledgers, readies,
            deadline_s=spec.get('deadline_s', 2.0),
            p95_bound_ms=spec['p95_bound_ms'],
            max_bound_ms=spec.get('max_bound_ms'))
    verdict_order = {PASS: 0, NOT_EVALUATED: 1, FAIL: 2}
    worst = PASS
    for _, (code, _) in results.items():
        if verdict_order[code] > verdict_order[worst]:
            worst = code
    return {'exit': worst,
            'gates': {name: {'verdict': code, 'report': report}
                      for name, (code, report) in results.items()}}


# -- CLI: offline re-computation from a raw JSONL ------------------------

def main(argv=None):
    import argparse
    import json
    parser = argparse.ArgumentParser(
        description='Recompute G4-A gen3 gates from raw event JSONL')
    parser.add_argument('events_jsonl')
    parser.add_argument('--query-p95-ms', type=float, default=100.0)
    parser.add_argument('--query-max-ms', type=float, default=250.0)
    parser.add_argument('--vis-p95-ms', type=float, default=1000.0)
    parser.add_argument('--vis-max-ms', type=float, default=2000.0)
    parser.add_argument('--out', default=None)
    args = parser.parse_args(argv)
    with open(args.events_jsonl) as handle:
        events = [json.loads(line) for line in handle if line.strip()]
    verdict = judge(events, {
        'query': {'p95_bound_ms': args.query_p95_ms,
                  'max_bound_ms': args.query_max_ms},
        'visibility': {'p95_bound_ms': args.vis_p95_ms,
                       'max_bound_ms': args.vis_max_ms}})
    text = json.dumps(verdict, indent=1)
    if args.out:
        with open(args.out, 'w') as handle:
            handle.write(text + '\n')
    print(text)
    return verdict['exit']


if __name__ == '__main__':
    import sys
    sys.exit(main())
