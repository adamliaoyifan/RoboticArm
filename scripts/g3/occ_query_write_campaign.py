"""Alternating isolated baseline/candidate transactions, with fresh maps."""
import itertools
import json
import math
import os
import platform
from pathlib import Path
import subprocess
import sys
import time


EQUAL_FIELDS = ('input_sha256', 'identity', 'v1_sha256', 'ledger_sha256', 'facts', 'stats')


def write_benchmark(args, common):
    revision = common.git('rev-parse', 'HEAD')
    if common.git('status', '--porcelain'):
        raise RuntimeError('freeze clean source before write campaign')
    if common.git('rev-parse', args.baseline_revision) != common.BASELINE:
        raise RuntimeError('unregistered baseline')
    formal = (not args.rehearsal and args.samples == 30 and args.warmup == 3
        and set(args.resolutions) == common.formal_resolution_set(
            getattr(args, 'matrix_era', 'g6_full')) and set(args.boxes) == {0, 5, 20}
        and set(args.write_points) == {30000, 100000} and set(args.outside_ratios) == {.5, .9}
        and set(args.crossing_ratios) == {0., .5})
    if not formal and not args.rehearsal:
        raise RuntimeError('partial write matrix requires --rehearsal')
    out = Path(args.out).resolve()
    if not out.is_relative_to(common.EVIDENCE):
        raise RuntimeError('evidence outside task root')
    out.mkdir(parents=True, exist_ok=False)
    roots = {name: out/(name+'_source') for name in ('baseline', 'candidate')}
    common.archive(common.BASELINE, roots['baseline'])
    common.archive(revision, roots['candidate'])
    hashes = {name: common.source_hashes(root) for name,root in roots.items()}
    common.write_json(out/'source_hashes.json', hashes)
    manifest = dict(kind='whole_write', started_ns=time.time_ns(), revision=revision,
        baseline_revision=common.BASELINE, dirty='', formal_matrix=formal, groups=[],
        overall_g2='not_evaluated', input_profile='1000 unique interior endpoints;1000 exterior endpoints; raw multiplicity explicit',
        crossing_definition='fraction of rays actually selected by unchanged ray budget',
        serialization='both legacy surface and committed JSON prepared inside timed interval',
        independent_reset='fresh Mapper/Store and actual geometry records before every unit',
        python=sys.version,platform=platform.platform(),cpu_count=os.cpu_count(),
        cpu_affinity=sorted(os.sched_getaffinity(0)),
        ros_distro=os.environ.get('ROS_DISTRO'),load_average=list(os.getloadavg()))
    common.write_json(out/'manifest.json', manifest)
    fixture = roots['candidate']/'src/luggage_perception/test/occ_query_write_fixture.py'
    matrix = itertools.product(args.resolutions,args.boxes,args.write_points,args.outside_ratios,args.crossing_ratios)
    for resolution, boxes, points, outside, crossing in matrix:
        name = 'r%s_b%d_p%d_o%s_c%s' % (resolution,boxes,points,outside,crossing)
        group = out/name
        group.mkdir()
        entry = dict(resolution=resolution, boxes=boxes, points=points, outside=outside,
            crossing=crossing, samples=[], phase='cold')
        manifest['groups'].append(entry)
        times = {name:[] for name in roots}
        for index in range(-1, args.warmup+args.samples):
            pair = {}
            order = ('baseline','candidate') if index % 2 else ('candidate','baseline')
            for version in order:
                unit = group/(version+'_'+str(index))
                command = [sys.executable,str(fixture),'--root',str(roots[version]),'--out',str(unit),
                    '--resolution',str(resolution),'--boxes',str(boxes),'--points',str(points),
                    '--outside',str(outside),'--crossing',str(crossing),'--index',str(index),
                    '--input',str(group/'input_points.npy')]
                with (group/(version+'_'+str(index)+'.log')).open('w') as stream:
                    result = subprocess.run(command,cwd='/tmp',stdout=stream,stderr=subprocess.STDOUT,timeout=120)
                if result.returncode:
                    entry.update(phase='worker_failed',outcome='fail',version=version,index=index,
                        argv=command,exit_code=result.returncode)
                    common.write_json(out/'manifest.json',manifest)
                    return 1
                pair[version] = json.loads((unit/'result.json').read_text())
            equal = {field:pair['baseline'][field]==pair['candidate'][field] for field in EQUAL_FIELDS}
            sample = dict(index=index,cold=index==-1,warmup=0<=index<args.warmup,order=order,
                baseline_ns=pair['baseline']['elapsed_ns'],candidate_ns=pair['candidate']['elapsed_ns'],
                exact_equal=equal, selected_rays=pair['candidate']['selected_rays'],
                actual_crossing=pair['candidate']['actual_crossing'])
            entry['samples'].append(sample)
            with (group/'samples.jsonl').open('a') as stream:stream.write(json.dumps(sample)+'\n')
            if not all(equal.values()):
                entry.update(phase='oracle_difference',outcome='fail',index=index,
                    capture_complete=True,replay_possible=True,
                    replay='same frozen worker/root/config; cold maps are identical initial state, compare saved primary hashes')
                common.write_json(out/'manifest.json',manifest)
                return 1
            if index == -1 and pair['candidate']['elapsed_ns'] > pair['baseline']['elapsed_ns']*1.20+20_000_000:
                entry.update(phase='complete',outcome='fail',reason='cold_max_regression',p95='not_evaluated',
                    capture_complete=True,replay_possible=True)
                break
            if index >= args.warmup:
                for version in roots:times[version].append(pair[version]['elapsed_ns'])
            entry['phase'] = 'scoring'
            common.write_json(out/'manifest.json',manifest)
            common.check_evidence_budget(out)
        if not entry.get('outcome'):
            stats = {}
            for version, values in times.items():
                values = sorted(values)
                stats[version] = dict(n=len(values),p95_ns=values[math.ceil(.95*len(values))-1],max_ns=max(values))
            ratio = stats['candidate']['p95_ns']/stats['baseline']['p95_ns']
            maximum_ok = stats['candidate']['max_ns'] <= stats['baseline']['max_ns']*1.20+20_000_000
            outcome = ('pass' if ratio <= 1.10 and maximum_ok else 'fail') if args.samples==30 else 'not_evaluated'
            entry.update(phase='complete',outcome=outcome,stats=stats,p95_ratio=ratio,max_gate=maximum_ok)
        common.write_json(out/'manifest.json',manifest)
    manifest.update(ended_ns=time.time_ns(),phase='complete',
        sources_unchanged={name:common.source_hashes(root)==hashes[name] for name,root in roots.items()},
        outcome='measured_with_individual_gate_results')
    common.write_json(out/'manifest.json',manifest)
    return int(any(group.get('outcome')!='pass' for group in manifest['groups']))
