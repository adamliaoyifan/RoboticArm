#!/usr/bin/env python3
"""G4-A isolation CLI dispatch (protocol §9 entry, thin adapter).

Maps the protocol's `occ_query_eval.py isolation` shape onto the G4-A
gate scripts without modifying the G2-frozen main CLI. Exit codes are
propagated 0/3/4 from the underlying gate script.
"""

import argparse
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    parser = argparse.ArgumentParser(prog='occ_query_eval_g4')
    sub = parser.add_subparsers(dest='stage', required=True)

    codec = sub.add_parser('codec')
    codec.add_argument('--seed-start', type=int, default=0)
    codec.add_argument('--seeds', type=int, default=10)
    codec.add_argument('--transactions', type=int, default=100)
    codec.add_argument('--out', required=True)

    faults = sub.add_parser('faults')
    faults.add_argument('--repeat', type=int, default=3)
    faults.add_argument('--out', required=True)

    rep = sub.add_parser('representative')
    rep.add_argument('--warmup', type=int, default=3)
    rep.add_argument('--repeat', type=int, default=30)
    rep.add_argument('--boxes', type=int, default=0)
    rep.add_argument('--resolution', type=float, default=0.01)
    rep.add_argument('--out', required=True)

    pairing = sub.add_parser('a8-pairing')
    pairing.add_argument('--candidate-tree', required=True)
    pairing.add_argument('--boxes', type=int, default=0)
    pairing.add_argument('--resolution', type=float, default=0.01)
    pairing.add_argument('--out', required=True)

    scaling = sub.add_parser('box-scaling')
    scaling.add_argument('--boxes', default='0,1,3,5,8,12')
    scaling.add_argument('--resolution', type=float, default=0.01)
    scaling.add_argument('--samples', type=int, default=20)
    scaling.add_argument('--out', required=True)

    faults_v3 = sub.add_parser('faults-v3')
    faults_v3.add_argument('--gates', default='A4,A5,A6')
    faults_v3.add_argument('--resolution', type=float, default=0.05)
    faults_v3.add_argument('--out', required=True)

    rep_v3 = sub.add_parser('representative-v3')
    rep_v3.add_argument('--boxes', type=int, default=0)
    rep_v3.add_argument('--resolution', type=float, default=0.01)
    rep_v3.add_argument('--duration-s', type=float, default=60.0)
    rep_v3.add_argument('--query-p95-ms', type=float, default=100.0)
    rep_v3.add_argument('--query-max-ms', type=float, default=250.0)
    rep_v3.add_argument('--vis-p95-ms', type=float, default=1000.0)
    rep_v3.add_argument('--vis-max-ms', type=float, default=2000.0)
    rep_v3.add_argument('--out', required=True)

    judge = sub.add_parser('judge')
    judge.add_argument('events_jsonl')
    judge.add_argument('--query-p95-ms', type=float, default=100.0)
    judge.add_argument('--query-max-ms', type=float, default=250.0)
    judge.add_argument('--vis-p95-ms', type=float, default=1000.0)
    judge.add_argument('--vis-max-ms', type=float, default=2000.0)
    judge.add_argument('--summary', default=None)

    args = parser.parse_args()
    if args.stage == 'codec':
        # The codec gate is the C1/C2 unit suites (round-trip oracle).
        cmd = [sys.executable, '-m', 'pytest', '-q',
               'src/luggage_perception/test/'
               'test_cargo_query_snapshot_wire_a1a.py',
               'src/luggage_perception/test/test_cargo_query_export_a1.py']
        return subprocess.call(cmd, cwd=os.path.dirname(_HERE))
    if args.stage == 'faults':
        script = os.path.join(_HERE, 'occ_fault_gates.py')
        return subprocess.call(
            [sys.executable, script, '--out', args.out])
    if args.stage == 'representative':
        script = os.path.join(_HERE, 'occ_representative_gates.py')
        return subprocess.call(
            [sys.executable, script, '--boxes', str(args.boxes),
             '--resolution', str(args.resolution),
             '--samples', str(args.repeat), '--out', args.out])
    if args.stage == 'a8-pairing':
        script = os.path.join(_HERE, 'occ_a8_b33_pairing.py')
        return subprocess.call(
            [sys.executable, script,
             '--candidate-tree', args.candidate_tree,
             '--boxes', str(args.boxes),
             '--resolution', str(args.resolution),
             '--out', args.out])
    if args.stage == 'box-scaling':
        script = os.path.join(_HERE, 'occ_box_scaling.py')
        return subprocess.call(
            [sys.executable, script, '--boxes', args.boxes,
             '--resolution', str(args.resolution),
             '--samples', str(args.samples), '--out', args.out])
    if args.stage == 'faults-v3':
        script = os.path.join(_HERE, 'occ_fault_gates_v3.py')
        return subprocess.call(
            [sys.executable, script, '--gates', args.gates,
             '--resolution', str(args.resolution), '--out', args.out])
    if args.stage == 'representative-v3':
        script = os.path.join(_HERE, 'occ_concurrent_representative_v3.py')
        return subprocess.call(
            [sys.executable, script,
             '--boxes', str(args.boxes),
             '--resolution', str(args.resolution),
             '--duration-s', str(args.duration_s),
             '--query-p95-ms', str(args.query_p95_ms),
             '--query-max-ms', str(args.query_max_ms),
             '--vis-p95-ms', str(args.vis_p95_ms),
             '--vis-max-ms', str(args.vis_max_ms),
             '--out', args.out])
    if args.stage == 'judge':
        script = os.path.join(_HERE, 'occ_metrics_g3.py')
        cmd = [sys.executable, script, args.events_jsonl,
               '--query-p95-ms', str(args.query_p95_ms),
               '--query-max-ms', str(args.query_max_ms),
               '--vis-p95-ms', str(args.vis_p95_ms),
               '--vis-max-ms', str(args.vis_max_ms)]
        if args.summary:
            cmd += ['--out', args.summary]
        return subprocess.call(cmd)
    return 2


if __name__ == "__main__":
    sys.exit(main())
