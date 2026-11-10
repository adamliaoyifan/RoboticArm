#!/usr/bin/env python3
"""OCC G2 isolated evidence runner; no motion or shared-install mutation.

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
import shutil
import subprocess
import sys
import tarfile
import time


ROOT = Path(__file__).resolve().parents[2]
BASELINE = 'b33dda919521ef043fa9e30b07dcc79c87f76fce'
SOURCE_PATHS = ('src/luggage_perception', 'src/luggage_description', 'src/luggage_msgs',
                'src/luggage_packing', 'src/luggage_planning', 'st09_contracts', 'scripts/g3')
FIXTURE = Path('src/luggage_perception/test/occ_query_oracle_fixture.py')
EVIDENCE = ROOT/'docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025'
EVIDENCE_DEFAULT = EVIDENCE


def apply_evidence_root(root):
    """Repoint this runner at a different frozen evidence root.

    Campaigns receive this module as ``common`` and read ``common.EVIDENCE``
    for containment and budget checks, so one assignment keeps every worker,
    judge, and failure artifact on the same root. The G2 default stays
    untouched unless an explicit absolute override is given.
    """
    resolved = Path(root).expanduser()
    if not resolved.is_absolute():
        raise RuntimeError('evidence root must be an absolute path')
    global EVIDENCE
    EVIDENCE = resolved.resolve()
    return EVIDENCE


#: Matrix eras: which resolution sets a formal (non-rehearsal) run may use.
#: g6_full keeps the original 1/2/5cm contract; g7_2cm is the user-confirmed
#: 2cm representative target with the retained 2/5cm matrix (plan 9b7d465b).
FORMAL_RESOLUTION_SETS = {'g6_full': {.01, .02, .05}, 'g7_2cm': {.02, .05}}
FORMAL_REPRESENTATIVE_RESOLUTION = {'g6_full': .01, 'g7_2cm': .02}


def formal_resolution_set(era):
    try:
        return set(FORMAL_RESOLUTION_SETS[era])
    except KeyError:
        raise RuntimeError('unknown matrix era: '+str(era)
                           +' (choose from '+str(sorted(FORMAL_RESOLUTION_SETS))+')')


def formal_representative_resolution(era):
    try:
        return FORMAL_REPRESENTATIVE_RESOLUTION[era]
    except KeyError:
        raise RuntimeError('unknown matrix era: '+str(era))


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


def oracle_formal(args):
    """Formal (scored) oracle workload gate; era selects the resolution set."""
    return (not args.rehearsal and args.seed_start == 0 and args.seeds == 20
        and args.transactions == 500 and args.distance_points >= 1000
        and set(args.resolutions) == formal_resolution_set(args.matrix_era)
        and set(args.boxes) == {0, 5, 20}
        and set(args.shapes) == {'cuboid', 'chamfer'})


def oracle(args):
    revision = git('rev-parse', 'HEAD')
    baseline = git('rev-parse', args.baseline_revision)
    if baseline != BASELINE:
        raise RuntimeError('baseline change requires an explicit task-thread version entry')
    dirty = git('status', '--porcelain')
    if dirty:
        raise RuntimeError('commit implementation and runner before freezing a campaign')
    formal = oracle_formal(args)
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
        if name == 'baseline' and args.reuse_baseline_run:
            previous = Path(args.reuse_baseline_run).resolve()
            old = json.loads((previous/'manifest.json').read_text())
            old_hashes = json.loads((previous/'source_hashes.json').read_text())['baseline']
            entries = [worker for worker in old['workers'] if worker['name']=='baseline']
            if (old['baseline_revision'] != baseline or old['inputs'] != manifest['inputs']
                    or old['fixture_sha256'] != manifest['fixture_sha256'] or len(entries)!=1
                    or entries[0].get('exit_code') != 0 or not entries[0].get('source_hashes_unchanged')
                    or any(hashes['baseline'].get(path)!=digest for path,digest in old_hashes.items())
                    or source_hashes(previous/'baseline_source') != old_hashes):
                raise RuntimeError('baseline reuse provenance/input/fixture mismatch')
            shutil.copytree(previous/'baseline', out/'baseline',
                ignore=shutil.ignore_patterns('__pycache__'))
            manifest['workers'].append(dict(name='baseline', reused_from=str(previous),
                original_worker=entries[0],exit_code=0,source_hashes_unchanged=True,
                rows_sha256=hashlib.sha256((out/'baseline/rows.jsonl').read_bytes()).hexdigest(),
                inputs_sha256=hashlib.sha256((out/'baseline/inputs.jsonl').read_bytes()).hexdigest()))
            write_json(out/'manifest.json',manifest)
            continue
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
    if (bundle/'owner/ready.json').exists() and (bundle/'client/result.json').exists():
        out = Path(args.out).resolve() if args.out else bundle/('replay_'+str(time.time_ns()))
        command = [sys.executable, str(Path(__file__).with_name('occ_query_concurrency_replay.py')),
                   '--case', str(bundle), '--out', str(out)]
        return subprocess.run(command, cwd='/tmp').returncode
    if (bundle/'manifest.json').exists() and json.loads((bundle/'manifest.json').read_text()).get('kind')=='planning':
        from occ_query_placement_comparison import inspect_campaign
        return inspect_campaign(bundle,args,sys.modules[__name__])
    case = json.loads((bundle/'case.json').read_text())
    if 'response' in case and (bundle/'map.npz').exists() and (bundle/'points.npy').exists():
        return replay_distance_case(bundle,case,args)
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


def replay_distance_case(bundle, case, args):
    """Regenerate the full original transaction sequence from frozen source."""
    run = bundle.parent.parent
    original = json.loads((run/'manifest.json').read_text())
    source = run/'candidate_source'
    hashes = json.loads((run/'source_hashes.json').read_text())['candidate']
    if source_hashes(source) != hashes:
        raise RuntimeError('frozen source changed')
    out = Path(args.out).resolve() if args.out else bundle/('replay_'+str(time.time_ns()))
    out.mkdir(parents=True,exist_ok=False)
    inputs = argparse.Namespace(seed_start=case['seed'],seeds=1,
        transactions=original['inputs']['transactions'],distance_points=original['inputs']['distance_points'],
        resolutions=[case['resolution']],boxes=[case['initial_boxes']],shapes=[case['shape']])
    command = worker_command(source/FIXTURE,source,out/'candidate',inputs,True)
    with (out/'worker.log').open('w') as stream:
        result = subprocess.run(command,cwd='/tmp',stdout=stream,stderr=subprocess.STDOUT)
    failure = out/'candidate/distance_failure'
    matched = False
    if failure.exists():
        import numpy as np
        replayed = json.loads((failure/'case.json').read_text())
        with np.load(bundle/'map.npz') as before, np.load(failure/'map.npz') as after:
            matched = (set(before.files)==set(after.files)
                and all(np.array_equal(before[key],after[key],equal_nan=True) for key in before.files)
                and np.array_equal(np.load(bundle/'points.npy'),np.load(failure/'points.npy'))
                and replayed['error']==case['error']
                and replayed['response']['status']==case['response']['status'])
    write_json(out/'manifest.json',dict(kind='distance_failure_replay',original_bundle=str(bundle),
        argv=command,worker_exit_code=result.returncode,exact_map_and_points=matched,
        reproduced=bool(result.returncode and matched),source_hashes_unchanged=source_hashes(source)==hashes,
        capture_complete=failure.exists(),overall_g2='not_evaluated'))
    return 0 if result.returncode and matched else 1


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
    p.add_argument('--reuse-baseline-run')
    p.add_argument('--rehearsal', action='store_true')
    p.add_argument('--out', required=True)
    p.add_argument('--evidence-root', help='absolute override for the frozen evidence root')
    p.add_argument('--matrix-era', choices=('g6_full', 'g7_2cm'), default='g6_full',
                   help='resolution contract a formal run must match')
    p.set_defaults(run=oracle)
    p = commands.add_parser('benchmark', help='installed no-motion query timing/resource matrix')
    p.add_argument('--install', default=os.environ.get('OCC_G2_INSTALL','/tmp/occ-query-g2-install'))
    p.add_argument('--domain', type=int, default=184)
    p.add_argument('--samples', '--repeat', type=int, default=30)
    p.add_argument('--warmup', type=int, default=3)
    p.add_argument('--resolutions', type=csv_choices(float, (.01, .02, .05)), default=[.01, .02, .05])
    p.add_argument('--boxes', type=csv_choices(int, (0, 5, 20)), default=[0, 5, 20])
    p.add_argument('--shapes', type=csv_choices(str, ('cuboid', 'chamfer')), default=['cuboid', 'chamfer'])
    p.add_argument('--baseline-revision', default=BASELINE)
    p.add_argument('--rehearsal', action='store_true')
    p.add_argument('--mode', choices=('all','queries','writes'), default='all')
    p.add_argument('--screen-first', action='store_true', default=True)
    p.add_argument('--profile-distance', action='store_true', help='diagnostic rehearsal only; timing is perturbed')
    p.add_argument('--observe-gc',action='store_true',help='diagnostic rehearsal only; retain GC phase times')
    p.add_argument('--write-points', type=csv_choices(int, (30000,100000)), default=[30000,100000])
    p.add_argument('--outside-ratios', type=csv_choices(float, (.5,.9)), default=[.5,.9])
    p.add_argument('--crossing-ratios', type=csv_choices(float, (0.,.5)), default=[0.,.5])
    p.add_argument('--out', required=True)
    p.add_argument('--evidence-root', help='absolute override for the frozen evidence root')
    p.add_argument('--matrix-era', choices=('g6_full', 'g7_2cm'), default='g6_full',
                   help='resolution contract a formal run must match')
    def run_benchmark(args):
        if args.mode == 'all':
            out = Path(args.out).resolve()
            if not out.is_relative_to(EVIDENCE):
                raise RuntimeError('evidence outside task root')
            out.mkdir(parents=True, exist_ok=False)
            from occ_query_runtime_campaign import benchmark
            from occ_query_write_campaign import write_benchmark
            query_args = argparse.Namespace(**vars(args));query_args.out = str(out/'queries')
            write_args = argparse.Namespace(**vars(args));write_args.out = str(out/'writes')
            query_code = benchmark(query_args, sys.modules[__name__])
            write_code = write_benchmark(write_args, sys.modules[__name__])
            write_json(out/'manifest.json', dict(kind='benchmark',query_exit_code=query_code,
                write_exit_code=write_code,overall_g2='not_evaluated'))
            return max(query_code,write_code)
        if args.mode == 'writes':
            from occ_query_write_campaign import write_benchmark
            return write_benchmark(args, sys.modules[__name__])
        from occ_query_runtime_campaign import benchmark
        return benchmark(args, sys.modules[__name__])
    p.set_defaults(run=run_benchmark)
    p = commands.add_parser('planning', help='installed baseline/candidate structural full-request matrix')
    p.add_argument('--install',default='/tmp/occ-query-g2-install')
    p.add_argument('--baseline-install',default='/tmp/occ-query-g2-baseline-install')
    p.add_argument('--domain',type=int,default=185)
    p.add_argument('--samples','--repeat',type=int,default=30)
    p.add_argument('--warmup',type=int,default=3)
    p.add_argument('--resolutions',type=csv_choices(float,(.01,.02,.05)),default=[.01,.02,.05])
    p.add_argument('--boxes',type=csv_choices(int,(0,5,20)),default=[0,5,20])
    p.add_argument('--paths',type=csv_choices(str,('hit','fallback')),default=['hit','fallback'])
    p.add_argument('--baseline-revision',default=BASELINE)
    p.add_argument('--rehearsal',action='store_true')
    p.add_argument('--out',required=True)
    p.add_argument('--evidence-root', help='absolute override for the frozen evidence root')
    p.add_argument('--matrix-era', choices=('g6_full', 'g7_2cm'), default='g6_full',
                   help='resolution contract a formal run must match')
    def run_planning(args):
        from occ_query_placement_campaign import planning
        return planning(args,sys.modules[__name__])
    p.set_defaults(run=run_planning)
    p = commands.add_parser('concurrency',help='fixed-load original-runtime heartbeat matrix without motion')
    p.add_argument('--install',default='/tmp/occ-query-g2-install')
    p.add_argument('--baseline-install',default='/tmp/occ-query-g2-baseline-install')
    p.add_argument('--domain',type=int,default=186)
    p.add_argument('--resolution',type=float,choices=(.01,.02,.05),required=True,
                   help='explicit stress resolution; no implicit 1cm default')
    p.add_argument('--duration-wall',type=float,default=120.)
    p.add_argument('--repeat',type=int,default=3)
    p.add_argument('--boxes',type=csv_choices(int,(0,5,20)),default=[0,5,20])
    p.add_argument('--baseline-revision',default=BASELINE)
    p.add_argument('--rehearsal',action='store_true')
    p.add_argument('--out',required=True)
    p.add_argument('--evidence-root', help='absolute override for the frozen evidence root')
    p.add_argument('--matrix-era', choices=('g6_full', 'g7_2cm'), default='g6_full',
                   help='resolution contract a formal run must match')
    def run_concurrency(args):
        from occ_query_concurrency_campaign import concurrency
        return concurrency(args,sys.modules[__name__])
    p.set_defaults(run=run_concurrency)
    p = commands.add_parser('replay', help='capture an oracle mismatch from frozen sources')
    p.add_argument('--bundle', required=True)
    p.add_argument('--out')
    p.set_defaults(run=replay)
    args = parser.parse_args()
    if getattr(args, 'evidence_root', None):
        apply_evidence_root(args.evidence_root)
    if args.command == 'oracle' and (args.seed_start < 0 or args.seeds < 1
            or args.transactions < 1 or args.distance_points < 0):
        parser.error('invalid sample counts')
    if args.command in ('benchmark','planning') and (not 100 <= args.domain <= 232 or args.samples < 1 or args.warmup < 0):
        parser.error('invalid runtime domain/sample counts')
    if args.command=='concurrency' and (not 100<=args.domain<=232 or args.duration_wall<=0 or args.repeat<1):
        parser.error('invalid concurrency domain/duration/repetition')
    return args.run(args)


if __name__ == '__main__':
    raise SystemExit(main())
