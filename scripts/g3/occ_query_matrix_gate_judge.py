#!/usr/bin/env python3
"""Judge a query-matrix run on the FROZEN gate basis (Q-20261031-7 rev3).

Mirror of the PRODUCER's own gates (occ_query_runtime_benchmark.py
LIMITS_MS): per case (p95_ms, max_ms); both halves judged. Basis per the
producer: distance*/frontier_count gate on gated_ns (owner-side
pure-query latency); other interfaces on elapsed_ns (round trip);
snapshot is derived from ready-phase timings, absent from samples.jsonl
by design -> not_applicable (both eras identical).

Semantics mirrored from the producer loop:
  - ANY row exceeding max_ms -- including the cold probe and warmups --
    aborts the arm as fail(cold_or_sample_max);
  - otherwise P95 (nearest-rank) and max are judged over the 30 scored
    samples per arm;
  - an arm with no scored samples and no abort is not_evaluated (never
    silently skipped).

gen7 reconciliation (basis proof): judging gen7's recorded run
reproduces its FAIL set exactly -- 4 items, trace_off @2cm
distance_256, gated p95 54.629/52.566/56.885/57.453 ms, matching the
recorded 52.6-57.5 digit for digit.
"""
import argparse
import json
import math
from pathlib import Path

GATED_CASES_PREFIXES = ('distance', 'frontier_count')

# Verbatim from occ_query_runtime_benchmark.py LIMITS_MS.
LIMITS_MS = {'stats': (50, 200), 'summary': (50, 200),
             'frontier_count': (1, 5),
             'roi_dense': (100, 250), 'roi_empty': (100, 250),
             'frontier': (100, 250), 'instance': (100, 250),
             'instance_empty': (100, 250),
             'distance_1': (5, 20), 'distance_256': (50, 200),
             'surface': (100, 250), 'patch': (100, 250),
             'snapshot': (100, 250)}


def nearest_rank(values, q=0.95):
    ordered = sorted(values)
    return ordered[math.ceil(q*len(ordered))-1]


def arm_verdict(rows, case, trace):
    """rows: all rows of this (case, trace) arm, scored or not."""
    p95_gate, max_gate = LIMITS_MS[case]
    gated = case.startswith(GATED_CASES_PREFIXES) or case == 'frontier_count'
    field = 'gated_ns' if gated else 'elapsed_ns'
    any_over = next((r for r in rows if r[field]/1e6 > max_gate), None)
    if any_over is not None:
        return {'outcome': 'FAIL', 'reason': 'cold_or_sample_max',
                'first_failure_index': any_over.get('index'),
                'value_ms': round(any_over[field]/1e6, 3),
                'max_gate_ms': max_gate, 'basis': field}
    scored = [r[field]/1e6 for r in rows
              if not r.get('warmup') and not r.get('cold')]
    if not scored:
        return {'outcome': 'not_evaluated', 'reason': 'no_scored_samples',
                'basis': field}
    p95 = nearest_rank(scored)
    mx = max(scored)
    fail_reason = None
    if p95 > p95_gate:
        fail_reason = 'p95 %.3f > %s' % (p95, p95_gate)
    elif mx > max_gate:
        fail_reason = 'max %.3f > %s' % (mx, max_gate)
    return {'outcome': 'FAIL' if fail_reason else 'pass',
            'reason': fail_reason, 'p95_ms': round(p95, 3),
            'max_ms': round(mx, 3), 'n': len(scored),
            'p95_gate_ms': p95_gate, 'max_gate_ms': max_gate,
            'basis': field}


def judge_run(run_dir):
    out = {'run': str(run_dir), 'groups': [],
           'counts': {'pass': 0, 'FAIL': 0, 'not_evaluated': 0,
                      'not_applicable': 0}}
    for samples in sorted(Path(run_dir).glob('r*/paired/client/samples.jsonl')):
        group = samples.parts[-4]
        rows = [json.loads(line) for line in samples.open()]
        cases = sorted(set(r['case'] for r in rows))
        entry = {'group': group, 'arms': {}}
        for case in sorted(LIMITS_MS):
            if case == 'snapshot':
                # Derived from ready-phase timings by the producer; never
                # present in samples.jsonl (both eras).
                entry['arms'][case] = {'trace_on': {'outcome': 'not_applicable',
                    'reason': 'snapshot-stage T1 does not wrap this path'},
                    'trace_off': {'outcome': 'not_applicable',
                    'reason': 'snapshot-stage T1 does not wrap this path'}}
                out['counts']['not_applicable'] += 2
                continue
            for trace in ('trace_on', 'trace_off'):
                arm_rows = [r for r in rows
                            if r['case'] == case and r['trace'] == trace]
                if not arm_rows:
                    verdict = {'outcome': 'not_evaluated',
                               'reason': 'no_samples'}
                else:
                    verdict = arm_verdict(arm_rows, case, trace)
                entry['arms'][f'{case}/{trace}'] = verdict
                out['counts'][verdict['outcome']
                              if verdict['outcome'] in out['counts']
                              else 'not_evaluated'] += 1
        out['groups'].append(entry)
    total = sum(out['counts'].values())
    out['arms_total'] = total
    out['verdict'] = ('FAIL' if out['counts']['FAIL']
                      else 'PASS' if out['counts']['pass'] == total
                      else 'PASS_WITH_NOT_EVALUATED')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', required=True, action='append',
                    help='matrix run dir (repeatable)')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    report = {'kind': 'query_matrix_gate_judge_rev3', 'runs': []}
    for run in args.run:
        judged = judge_run(run)
        report['runs'].append(judged)
        c = judged['counts']
        print(f"{run}: {judged['verdict']} | pass {c['pass']} / FAIL "
              f"{c['FAIL']} / not_evaluated {c['not_evaluated']} / "
              f"not_applicable {c['not_applicable']} (arms {judged['arms_total']})")
        for g in judged['groups']:
            for arm, v in g['arms'].items():
                if v.get('outcome') == 'FAIL':
                    print(f"  FAIL {g['group']} {arm}: {v.get('reason')}"
                          f" [{v.get('basis')}] value {v.get('value_ms', v.get('p95_ms'))}ms")
    if args.out:
        Path(args.out).write_text(json.dumps(report, indent=1, sort_keys=True)+'\n')


if __name__ == '__main__':
    main()
