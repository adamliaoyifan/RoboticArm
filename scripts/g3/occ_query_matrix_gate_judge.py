#!/usr/bin/env python3
"""Judge a query-matrix run on the FROZEN gate basis (Q-20261031-7 rev2).

Gate basis (proven by gen7 reconciliation, see evidence
occ_query_opt_20261031/closure_retest/GATE_BASIS.md):
  field   = gated_ns -- for distance*/frontier_count cases this is the
             OWNER-SIDE pure-query latency (response.meta.elapsed_ns);
             the producer declares these gates "pure-query latency"
             (occ_query_runtime_benchmark.py). elapsed_ns is the full
             ROS round trip and is reported as a DIAGNOSTIC only.
  metric  = nearest-rank P95: sorted[ceil(0.95*n)-1], n=30 per arm
  slice   = warmup and cold rows excluded
  units   = per (resolution, boxes, shape) group x (trace_on, trace_off)

gen7 reconciliation proof (this basis reproduces the recorded FAIL set
52.6-57.5 ms exactly): r0.02 b0/b20 trace_off = 54.63/52.57/56.88/57.45.
"""
import argparse
import json
import math
from pathlib import Path

GATED_CASES_PREFIXES = ('distance', 'frontier_count')


def nearest_rank(values, q=0.95):
    ordered = sorted(values)
    return ordered[math.ceil(q*len(ordered))-1]


def judge_run(run_dir, gate_ms):
    out = {'run': str(run_dir), 'gate_ms': gate_ms, 'groups': []}
    fails = 0
    for samples in sorted(Path(run_dir).glob('r*/paired/client/samples.jsonl')):
        group = samples.parts[-4]
        rows = [json.loads(line) for line in samples.open()]
        entry = {'group': group, 'cases': {}}
        cases = sorted(set(r['case'] for r in rows))
        for case in cases:
            for trace in ('trace_on', 'trace_off'):
                sel = [r for r in rows if r['case'] == case
                       and r['trace'] == trace
                       and not r.get('warmup') and not r.get('cold')]
                if not sel:
                    continue
                gated = nearest_rank([r['gated_ns'] for r in sel])/1e6
                elapsed = nearest_rank([r['elapsed_ns'] for r in sel])/1e6
                # Pure-query cases gate on gated_ns; other interfaces
                # gate on the round trip (gated_ns == elapsed_ns there).
                basis = gated if any(case.startswith(p)
                                     for p in GATED_CASES_PREFIXES) else elapsed
                gate = gate_ms.get(case, gate_ms['*'])
                fail = basis > gate
                fails += fail
                entry['cases'].setdefault(case, {})[trace] = {
                    'gated_p95_ms': round(gated, 3),
                    'elapsed_p95_ms': round(elapsed, 3),
                    'basis': 'gated' if basis is gated else 'elapsed',
                    'gate_ms': gate,
                    'outcome': 'FAIL' if fail else 'pass'}
        out['groups'].append(entry)
    out['fail_items'] = fails
    out['verdict'] = 'FAIL' if fails else 'PASS'
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', required=True, action='append',
                    help='matrix run dir (repeatable)')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()
    # Frozen gates: distance P95 <= 50 ms (pure-query basis); other
    # interfaces P95 <= 100 ms / max <= 250 ms (round-trip basis; only
    # the P95 half is judged here, on the declared basis).
    gate_ms = {'*': 100., 'distance_256': 50., 'distance_1': 50.}
    report = {'kind': 'query_matrix_gate_judge', 'runs': []}
    for run in args.run:
        judged = judge_run(run, gate_ms)
        report['runs'].append(judged)
        fails = [(g['group'], case, tr, v)
                 for g in judged['groups']
                 for case, arms in g['cases'].items()
                 for tr, v in arms.items() if v['outcome'] == 'FAIL']
        print(f"{run}: {judged['verdict']} ({judged['fail_items']} items over gate)")
        for group, case, tr, v in fails:
            print(f"  FAIL {group} {case} {tr}: "
                  f"{v['basis']} p95 {v['gated_p95_ms'] if v['basis']=='gated' else v['elapsed_p95_ms']} ms")
    if args.out:
        Path(args.out).write_text(json.dumps(report, indent=1, sort_keys=True)+'\n')


if __name__ == '__main__':
    main()
