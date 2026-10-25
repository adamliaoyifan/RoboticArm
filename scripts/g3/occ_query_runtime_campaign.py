"""Process lifecycle and evidence for the no-motion OCC query campaign.

The CLI owns only its child mapper and client processes. Installed product
hashes must match the frozen source before a runtime sample is accepted.
"""
import itertools
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def wake_owner(script,domain,node,env=None,log=None):
    command=[sys.executable,str(script),'--domain',str(domain),'--node',node]
    try:
        result=subprocess.run(command,env=env,capture_output=True,text=True,timeout=10.)
        observation=dict(exit_code=result.returncode,stderr=result.stderr[-1000:])
    except (subprocess.TimeoutExpired,OSError) as error:
        # A broken diagnostic client must never skip owned-process cleanup.
        observation=dict(exit_code=None,error=repr(error))
    if log is not None:
        with Path(log).open('a') as stream:
            stream.write(json.dumps(dict(command=command,ended_ns=time.time_ns(),**observation))+'\n')


def stop_owner(process,*,wake_script=None,domain=None,node='cargo_volume_mapper',env=None,wake_log=None):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGUSR2)
    if wake_script is not None:
        wake_owner(wake_script,domain,node,env,wake_log)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def benchmark(args, common):
    import yaml
    revision = common.git('rev-parse', 'HEAD')
    if common.git('status', '--porcelain'):
        raise RuntimeError('commit all campaign code before freezing')
    if common.git('rev-parse', args.baseline_revision) != common.BASELINE:
        raise RuntimeError('unregistered baseline revision')
    formal = (not args.rehearsal and not args.profile_distance and not args.observe_gc and args.samples == 30 and args.warmup == 3
        and set(args.resolutions) == {.01, .02, .05} and set(args.boxes) == {0, 5, 20}
        and set(args.shapes) == {'cuboid', 'chamfer'})
    if not formal and not args.rehearsal:
        raise RuntimeError('partial matrix requires --rehearsal')
    out = Path(args.out).resolve()
    if not out.is_relative_to(common.EVIDENCE):
        raise RuntimeError('evidence outside task root')
    out.mkdir(parents=True, exist_ok=False)
    source = out/'candidate_source'
    common.archive(revision, source)
    hashes = common.source_hashes(source)
    common.write_json(out/'source_hashes.json', hashes)
    manifest = dict(kind='benchmark', revision=revision, baseline_revision=common.BASELINE,
        dirty='', formal_matrix=formal, started_ns=time.time_ns(), groups=[],
        independent_install=str(Path(args.install).resolve()),
        overall_g2='not_evaluated', baseline_reads='not_evaluated', write_regression='not_evaluated')
    manifest['instrumentation_method'] = 'same-owner adjacent pairs; balanced seeded shuffled first mode; roundtrip P95 on/off; pure-query ratios retained as diagnostics'
    manifest['trace_pair_seed'] = 20261025
    common.write_json(out/'manifest.json', manifest)
    owner_script = source/'src/luggage_perception/test/occ_query_runtime_owner.py'
    client_script = source/'src/luggage_perception/test/occ_query_runtime_benchmark.py'
    wake_script=source/'scripts/g3/occ_query_runtime_wake.py'
    original_scene = source/'src/luggage_description/config/scene_tf.yaml'
    base_scene = yaml.safe_load(original_scene.read_text())
    for r, boxes, shape in itertools.product(args.resolutions, args.boxes, args.shapes):
        name = 'r%s_b%d_%s' % (r, boxes, shape)
        group = out/name
        group.mkdir()
        scene = json.loads(json.dumps(base_scene))
        if shape == 'cuboid':
            scene['container']['inner'].pop('chamfer', None)
        scene_path = group/'scene_tf.yaml'
        scene_path.write_text(yaml.safe_dump(scene, sort_keys=False))
        entry = dict(resolution=r, boxes=boxes, shape=shape, repetitions=[])
        manifest['groups'].append(entry)
        for trace in ('paired',):
            run = group/'paired'
            run.mkdir()
            command = [sys.executable, str(owner_script), '--out', str(run/'owner'),
                '--scene', str(scene_path), '--source', str(source), '--install', args.install,
                '--domain', str(args.domain), '--resolution', str(r), '--boxes', str(boxes)]
            command.extend(('--trace','--paired-trace'))
            if args.profile_distance:
                command.append('--profile-distance')
            if args.observe_gc:
                command.append('--observe-gc')
            repetition = dict(trace=trace, owner_command=command, phase='startup', started_ns=time.time_ns())
            entry['repetitions'].append(repetition)
            common.write_json(out/'manifest.json', manifest)
            with (run/'owner.log').open('w') as log:
                process = subprocess.Popen(command, cwd='/tmp', stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic()+90.
                    while not (run/'owner/ready.json').exists():
                        if process.poll() is not None:
                            raise RuntimeError('owner startup failed: '+str(run/'owner.log'))
                        if time.monotonic() > deadline:
                            raise RuntimeError('owner startup deadline')
                        time.sleep(.05)
                    repetition['phase'] = 'scoring'
                    command = [sys.executable, str(client_script), '--out', str(run/'client'),
                        '--owner', str(run/'owner'), '--domain', str(args.domain),
                        '--samples', str(args.samples), '--warmup', str(args.warmup)]
                    command.append('--paired-trace')
                    repetition['client_command'] = command
                    common.write_json(out/'manifest.json', manifest)
                    with (run/'client.log').open('w') as client_log:
                        result = subprocess.run(command, cwd='/tmp', stdout=client_log,
                            stderr=subprocess.STDOUT, timeout=600)
                    repetition['client_exit_code'] = result.returncode
                    repetition['phase'] = 'complete'
                    if result.returncode:
                        # Preserve the failed request and a consistent full
                        # boundary map before teardown. No automatic rerun.
                        process.send_signal(signal.SIGUSR1)
                        wake_owner(wake_script,args.domain,'cargo_volume_mapper',log=run/'control-wake.jsonl')
                        deadline = time.monotonic()+45.
                        while not (run/'owner/capture.json').exists() and process.poll() is None and time.monotonic() < deadline:
                            time.sleep(.05)
                        repetition['capture_complete'] = (run/'owner/capture.json').exists()
                except Exception as error:
                    repetition.update(phase='failed', error=repr(error))
                    if process.poll() is None:
                        process.send_signal(signal.SIGUSR1)
                        wake_owner(wake_script,args.domain,'cargo_volume_mapper',log=run/'control-wake.jsonl')
                        deadline = time.monotonic()+45.
                        while not (run/'owner/capture.json').exists() and process.poll() is None and time.monotonic() < deadline:
                            time.sleep(.05)
                    repetition['capture_complete'] = (run/'owner/capture.json').exists()
                finally:
                    stop_owner(process,wake_script=wake_script,domain=args.domain,wake_log=run/'control-wake.jsonl')
                    repetition.update(ended_ns=time.time_ns(), owner_exit_code=process.returncode)
                    ready_path, final_path = run/'owner/ready.json',run/'owner/manifest.json'
                    if ready_path.exists() and final_path.exists():
                        ready,final = json.loads(ready_path.read_text()),json.loads(final_path.read_text())
                        repetition['resources'] = dict(node_init_ns=ready['node_init_ns'],reset_ns=ready['reset_ns'],
                            rss_peak_kib=final['rss_peak_kib'],retained_bytes=ready['resources']['conservative_retained_bytes'])
                        repetition['resource_gate'] = 'pass' if (ready['node_init_ns']<=30_000_000_000
                            and ready['reset_ns']<=30_000_000_000 and final['rss_peak_kib']<=2*1024**2
                            and ready['resources']['conservative_retained_bytes']<=512*1024**2) else 'fail'
                    common.write_json(out/'manifest.json', manifest)
            common.check_evidence_budget(out)
            # A startup failure supplies no query samples. Stop this group
            # and keep every prior sample; unaffected groups can continue.
            if repetition['phase'] == 'failed':
                break
        path = group/'paired/client/result.json'
        if path.exists():
            results = json.loads(path.read_text())['results']
            observations = [json.loads(line) for line in (group/'paired/client/samples.jsonl').read_text().splitlines()]
            ratios = {}
            for case,modes in results.items():
                if case in ('snapshot','stats'):
                    ratios[case] = dict(outcome='not_applicable',reason='snapshot-stage T1 does not wrap this path')
                    continue
                samples = {mode:sorted(row['elapsed_ns'] for row in observations
                    if row['case']==case and row['trace']==mode and not row['cold'] and not row['warmup'])
                    for mode in ('trace_off','trace_on')}
                if all(len(values)==30 for values in samples.values()):
                    ratio = samples['trace_on'][28]/samples['trace_off'][28]
                    a,b = modes['trace_off'],modes['trace_on']
                    ratios[case] = dict(roundtrip_p95_ratio=ratio,
                        gated_p95_ratio=b.get('p95_ms',0)/a['p95_ms'] if a.get('p95_ms',0)>0 else None,
                        outcome='pass' if ratio <= 1.10 else 'fail')
                else:
                    ratios[case] = dict(outcome='not_evaluated')
            entry['instrumentation'] = ratios
        common.write_json(out/'manifest.json', manifest)
    manifest.update(ended_ns=time.time_ns(), source_hashes_unchanged=common.source_hashes(source)==hashes,
        phase='complete', outcome='measured_with_individual_gate_results')
    common.write_json(out/'manifest.json', manifest)
    failed = any(rep.get('resource_gate')!='pass' or rep.get('client_exit_code')!=0
        for group in manifest['groups'] for rep in group['repetitions'])
    failed |= any(gate['outcome'] not in ('pass','not_applicable')
        for group in manifest['groups'] for gate in group.get('instrumentation',{}).values())
    return int(failed)
