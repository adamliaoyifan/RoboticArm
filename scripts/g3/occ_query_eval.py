#!/usr/bin/env python3
"""OCC G2 evidence runner. Oracle/replay stage; runtime campaigns follow.

Every run gets new immutable source archives and a new evidence directory.
A small-grid oracle pass is never reported as an overall G2 pass.
"""
import argparse
import hashlib
import itertools
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tarfile
import time


ROOT = Path(__file__).resolve().parents[2]
BASELINE = 'b33dda919521ef043fa9e30b07dcc79c87f76fce'
SOURCE_PATHS = ('src/luggage_perception', 'src/luggage_description', 'st09_contracts')
FIXTURE = Path('src/luggage_perception/test/occ_query_oracle_fixture.py')
EVIDENCE = ROOT/'docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025'
RUN_BYTES = 8*1024**3
TOTAL_BYTES = 32*1024**3


def check_evidence_budget(out):
    def size(root):
        return sum(p.stat().st_size for p in root.rglob('*') if p.is_file() and not p.is_symlink())
    if size(out) > RUN_BYTES or size(EVIDENCE) > TOTAL_BYTES:
        raise RuntimeError('RESOURCE_LIMIT: evidence budget exceeded; preserve current run')


def csv_choices(kind, choices):
    def parse(value):
        try:
            result = [kind(v) for v in value.split(',')]
        except ValueError:
            raise argparse.ArgumentTypeError('invalid comma-separated values') from None
        if not result or len(result) != len(set(result)) or any(v not in choices for v in result):
            raise argparse.ArgumentTypeError('choose unique values from '+str(choices))
        return result
    return parse


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).strip()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True)+'\n')


def source_hashes(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}


def archive(revision, destination):
    destination.mkdir()
    tar = destination.with_suffix('.tar')
    with tar.open('wb') as stream:
        subprocess.run(['git', 'archive', '--format=tar', revision, *SOURCE_PATHS],
                       cwd=ROOT, stdout=stream, check=True)
    with tarfile.open(tar) as stream:
        for member in stream.getmembers():
            target = (destination/member.name).resolve()
            if not target.is_relative_to(destination.resolve()) or member.issym() or member.islnk():
                raise RuntimeError('archive contains unsafe source path: '+member.name)
        stream.extractall(destination)
    tar.unlink()


def worker_command(fixture, source, out, args, candidate):
    command = [sys.executable, str(fixture), '--root', str(source), '--out', str(out),
        '--seed-start', str(args.seed_start), '--seeds', str(args.seeds),
        '--transactions', str(args.transactions), '--distance-points', str(args.distance_points),
        '--resolutions', *(str(v) for v in args.resolutions),
        '--boxes', *(str(v) for v in args.boxes), '--shapes', *args.shapes]
    if candidate:
        command.append('--check-indexes')
    return command


def oracle(args):
    revision = git('rev-parse', 'HEAD')
    baseline = git('rev-parse', args.baseline_revision)
    if baseline != BASELINE:
        raise RuntimeError('baseline change requires an explicit task-thread version entry')
    dirty = git('status', '--porcelain')
    if dirty:
        raise RuntimeError('commit implementation and runner before freezing a campaign')
    formal = (not args.rehearsal and args.seed_start == 0 and args.seeds == 20
        and args.transactions == 500 and args.distance_points >= 1000
        and set(args.resolutions) == {.01, .02, .05} and set(args.boxes) == {0, 5, 20}
        and set(args.shapes) == {'cuboid', 'chamfer'})
    if not formal and not args.rehearsal:
        raise RuntimeError('partial workload requires explicit --rehearsal')
    out = Path(args.out).resolve()
    if not out.is_relative_to(EVIDENCE):
        raise RuntimeError('evidence must stay under '+str(EVIDENCE))
    out.mkdir(parents=True, exist_ok=False)
    manifest = dict(kind='oracle', phase='freeze', started_ns=time.time_ns(), revision=revision,
        baseline_revision=baseline, dirty=dirty, formal_small_grid_matrix=formal,
        argv=sys.argv, python=sys.version, platform=platform.platform(),
        affinity=sorted(os.sched_getaffinity(0)), cpu_count=os.cpu_count(),
        inputs=dict(seed_start=args.seed_start, seeds=args.seeds, transactions=args.transactions,
            resolutions=args.resolutions, boxes=args.boxes, shapes=args.shapes,
            distance_points=args.distance_points),
        deployment_boundaries='not_evaluated', overall_g2='not_evaluated')
    write_json(out/'manifest.json', manifest)
    baseline_source, candidate_source = out/'baseline_source', out/'candidate_source'
    archive(baseline, baseline_source)
    archive(revision, candidate_source)
    check_evidence_budget(out)
    hashes = {name: source_hashes(root) for name, root in
              (('baseline', baseline_source), ('candidate', candidate_source))}
    write_json(out/'source_hashes.json', hashes)
    fixture = candidate_source/FIXTURE
    manifest['fixture_sha256'] = hashlib.sha256(fixture.read_bytes()).hexdigest()
    manifest['workers'] = []
    for name, source in (('baseline', baseline_source), ('candidate', candidate_source)):
        command = worker_command(fixture, source, out/name, args, name == 'candidate')
        entry = dict(name=name, argv=command, started_ns=time.time_ns())
        manifest['workers'].append(entry)
        manifest['phase'] = name
        write_json(out/'manifest.json', manifest)
        with (out/(name+'.log')).open('w') as log:
            result = subprocess.run(command, cwd='/tmp', stdout=log, stderr=subprocess.STDOUT)
        entry.update(exit_code=result.returncode, ended_ns=time.time_ns(),
                     source_hashes_unchanged=source_hashes(source) == hashes[name])
        check_evidence_budget(out)
        if result.returncode or not entry['source_hashes_unchanged']:
            manifest.update(phase='worker_failed', outcome='fail', ended_ns=time.time_ns())
            write_json(out/'manifest.json', manifest)
            return 1
    baseline_rows, candidate_rows = out/'baseline/rows.jsonl', out/'candidate/rows.jsonl'
    expected = args.seeds*args.transactions*len(args.resolutions)*len(args.boxes)*len(args.shapes)
    rows = 0
    with baseline_rows.open() as a, candidate_rows.open() as b:
        for number, (old, new) in enumerate(itertools.zip_longest(a, b)):
            rows += 1
            if old != new:
                failure = out/'first_difference';failure.mkdir()
                (failure/'baseline_row.json').write_text(old or '{}\n')
                (failure/'candidate_row.json').write_text(new or '{}\n')
                case = json.loads(new or old)
                write_json(failure/'case.json', dict(kind='oracle_difference', row=number,
                    resolution=case['resolution'], initial_boxes=case['initial_boxes'],
                    shape=case['shape'], seed=case['seed'], transaction=case['transaction'],
                    fixture=str(fixture), baseline_source=str(baseline_source),
                    candidate_source=str(candidate_source), capture_complete=False,
                    replay_possible=True))
                manifest.update(phase='oracle_difference', outcome='fail', row=number,
                                ended_ns=time.time_ns())
                write_json(out/'manifest.json', manifest)
                # Durable source/journal plus deterministic replay capture;
                # no source edits or blind rerun of a scored campaign.
                replay(argparse.Namespace(bundle=str(failure), out=str(failure/'replay')))
                return 1
    journals_equal = (out/'baseline/inputs.jsonl').read_bytes() == (out/'candidate/inputs.jsonl').read_bytes()
    matched = rows == expected and journals_equal
    manifest.update(phase='complete', rows=rows, expected_rows=expected,
        exact_match=matched,
        baseline_rows_sha256=hashlib.sha256(baseline_rows.read_bytes()).hexdigest(),
        candidate_rows_sha256=hashlib.sha256(candidate_rows.read_bytes()).hexdigest(),
        input_journals_equal=journals_equal,
        outcome=('small_grid_oracle_pass' if formal else 'rehearsal_complete')
                if matched else 'fail', ended_ns=time.time_ns())
    write_json(out/'manifest.json', manifest)
    print(json.dumps({k: manifest[k] for k in ('outcome', 'rows', 'exact_match', 'overall_g2')}))
    return 0 if matched else 1


def replay(args):
    bundle = Path(args.bundle).resolve()
    case = json.loads((bundle/'case.json').read_text())
    if case.get('kind') != 'oracle_difference':
        raise RuntimeError('this runner stage only replays oracle differences')
    out = Path(args.out).resolve() if args.out else bundle/('replay_'+str(time.time_ns()))
    out.mkdir(parents=True, exist_ok=False)
    inputs = argparse.Namespace(seed_start=case['seed'], seeds=1,
        transactions=case['transaction']+1, distance_points=0,
        resolutions=[case['resolution']], boxes=[case['initial_boxes']], shapes=[case['shape']])
    for name in ('baseline', 'candidate'):
        root = Path(case[name+'_source'])
        fixture = Path(case['fixture'])
        command = worker_command(fixture, root, out/name, inputs, False)
        command.append('--capture-final')
        with (out/(name+'.log')).open('w') as log:
            result = subprocess.run(command, cwd='/tmp', stdout=log, stderr=subprocess.STDOUT)
        if result.returncode:
            raise RuntimeError('replay failed: '+name)
    case['capture_complete'] = True
    case['durable_replay'] = str(out)
    write_json(bundle/'case.json', case)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('oracle', help='freeze and compare isolated full-matrix workers')
    p.add_argument('--seed-start', type=int, default=0)
    p.add_argument('--seeds', type=int, default=20)
    p.add_argument('--transactions', type=int, default=500)
    p.add_argument('--distance-points', type=int, default=1000)
    p.add_argument('--resolutions', type=csv_choices(float, (.01, .02, .05)), default=[.01, .02, .05])
    p.add_argument('--boxes', type=csv_choices(int, (0, 5, 20)), default=[0, 5, 20])
    p.add_argument('--shapes', type=csv_choices(str, ('cuboid', 'chamfer')), default=['cuboid', 'chamfer'])
    p.add_argument('--baseline-revision', default=BASELINE)
    p.add_argument('--rehearsal', action='store_true')
    p.add_argument('--out', required=True)
    p.set_defaults(run=oracle)
    p = commands.add_parser('replay', help='capture an oracle mismatch from frozen sources')
    p.add_argument('--bundle', required=True)
    p.add_argument('--out')
    p.set_defaults(run=replay)
    args = parser.parse_args()
    if args.command == 'oracle' and (args.seed_start < 0 or args.seeds < 1
            or args.transactions < 1 or args.distance_points < 0):
        parser.error('invalid sample counts')
    return args.run(args)


if __name__ == '__main__':
    raise SystemExit(main())
