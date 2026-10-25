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


def stop_owner(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGUSR2)
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
    formal = (not args.rehearsal and args.samples == 30 and args.warmup == 3
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
    common.write_json(out/'manifest.json', manifest)
    owner_script = source/'src/luggage_perception/test/occ_query_runtime_owner.py'
    client_script = source/'src/luggage_perception/test/occ_query_runtime_benchmark.py'
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
        for trace in (False, True):
            run = group/('trace_on' if trace else 'trace_off')
            run.mkdir()
            command = [sys.executable, str(owner_script), '--out', str(run/'owner'),
                '--scene', str(scene_path), '--source', str(source), '--install', args.install,
                '--domain', str(args.domain), '--resolution', str(r), '--boxes', str(boxes)]
            if trace:
                command.append('--trace')
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
                        deadline = time.monotonic()+20.
                        while not (run/'owner/capture.json').exists() and process.poll() is None and time.monotonic() < deadline:
                            time.sleep(.05)
                        repetition['capture_complete'] = (run/'owner/capture.json').exists()
                except Exception as error:
                    repetition.update(phase='failed', error=repr(error))
                    if process.poll() is None:
                        process.send_signal(signal.SIGUSR1)
                        deadline = time.monotonic()+20.
                        while not (run/'owner/capture.json').exists() and process.poll() is None and time.monotonic() < deadline:
                            time.sleep(.05)
                    repetition['capture_complete'] = (run/'owner/capture.json').exists()
                finally:
                    stop_owner(process)
                    repetition.update(ended_ns=time.time_ns(), owner_exit_code=process.returncode)
                    common.write_json(out/'manifest.json', manifest)
            common.check_evidence_budget(out)
            # A startup failure supplies no query samples. Stop this group
            # and keep every prior sample; unaffected groups can continue.
            if repetition['phase'] == 'failed':
                break
        if len(entry['repetitions']) == 2:
            results = []
            for mode in ('trace_off', 'trace_on'):
                path = group/mode/'client/result.json'
                results.append(json.loads(path.read_text())['results'] if path.exists() else {})
            ratios = {}
            for case in results[0].keys() & results[1].keys():
                a, b = results[0][case], results[1][case]
                if a.get('samples') == 30 and b.get('samples') == 30 and a.get('p95_ms', 0) > 0:
                    ratio = b['p95_ms']/a['p95_ms']
                    ratios[case] = dict(ratio=ratio, outcome='pass' if ratio <= 1.10 else 'fail')
                else:
                    ratios[case] = dict(outcome='not_evaluated')
            entry['instrumentation'] = ratios
        common.write_json(out/'manifest.json', manifest)
    manifest.update(ended_ns=time.time_ns(), source_hashes_unchanged=common.source_hashes(source)==hashes,
        phase='complete', outcome='measured_with_individual_gate_results')
    common.write_json(out/'manifest.json', manifest)
    return 0
