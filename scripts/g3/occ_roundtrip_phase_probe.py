#!/usr/bin/env python3
"""Round-trip phase localization probe (matrix residual FAILs, Q-20261031-7).

Frontier/roi_dense round trips at 2cm show a bimodal slow mode (~+40-60 ms
over a ~45-55 ms base, 16-46% of samples) that the offline forensics could
not localize: the recorded owner stage covers only snapshot ACQUIRE, the
operation phase is unrecorded for these interfaces, and GC is unobserved.

This probe boots the SAME owner (same scene file, install, environment)
with two diagnostic flags --observe-gc --observe-spans, replays the
FAILING arms' request CDRs byte-verified against the r3 samples, and adds
what the frozen tooling never recorded:

  client   : round-trip elapsed, client-side GC pauses in-window
  owner    : service spans (open/operation/meta-release + serving-thread
             CPU) and query-core spans (select/sort/mask/records), owner
             GC events, whole-process CPU via /proc/<pid>/schedstat
  arms     : A baseline (no writes), B same + --write-load-hz 2 writer
             thread (shared-update contention), C result-volume sweep
             (roi page sizes, frontier region scales)

FAILs are NOT re-judged here; the probe exists to attribute the extra
milliseconds, not to find a passing run.
"""
import argparse
import gc
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

_WS = Path(__file__).resolve().parents[2]
for entry in (_WS/'src'/'luggage_perception'/'test', _WS/'scripts'/'g3'):
    sys.path.insert(0, str(entry))

from occ_query_runtime_environment import installed_environment  # noqa: E402


def schedstat_ns(pid):
    # field 1: cumulative time on cpu in nanoseconds
    return int(Path('/proc/%d/schedstat' % pid).read_text().split()[0])


def load_replay_requests(replay_dir, cases, verify=True):
    """Byte-verified request reconstruction for the failing trace_off arms."""
    import rclpy.serialization
    from luggage_msgs.srv import QueryCargoCellsV2
    samples = [json.loads(line) for line in
               (replay_dir/'paired'/'client'/'samples.jsonl').open()]
    out = {}
    for case in cases:
        arm = [r for r in samples if r['case'] == case and r['trace'] == 'trace_off'
               and not r.get('warmup') and not r.get('cold')]
        if not arm:
            raise RuntimeError('no trace_off samples for %s in %s' % (case, replay_dir))
        row = arm[0]
        cdr = replay_dir/'paired'/'client'/('%s-%d-trace_off.request.cdr'
                                            % (case, row['index']))
        raw = cdr.read_bytes()
        if verify and hashlib.sha256(raw).hexdigest() != row['request_sha256']:
            raise RuntimeError('cdr hash mismatch for %s' % cdr)
        request = rclpy.serialization.deserialize_message(
            raw, QueryCargoCellsV2.Request)
        out[case] = request
    return out


def run_arm(owner_proc, node, client, template, case, count, tag, results):
    import rclpy
    import rclpy.serialization
    srv = type(template)
    gc_events = []

    def gc_cb(phase, info):
        gc_events.append((time.perf_counter_ns(), phase, info.get('generation')))

    gc.callbacks.append(gc_cb)
    try:
        for i in range(count):
            # clone through serialization: nested message fields are shared
            # references on a plain field copy, and the sweep arm mutates them
            request = rclpy.serialization.deserialize_message(
                rclpy.serialization.serialize_message(template), srv)
            request.query.request_id = 'probe-%s-%s-%03d' % (tag, case, i)
            cpu0 = schedstat_ns(owner_proc.pid)
            del gc_events[:]
            t0 = time.perf_counter_ns()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
            t1 = time.perf_counter_ns()
            cpu1 = schedstat_ns(owner_proc.pid)
            if not future.done():
                results.append(dict(tag=tag, case=case, i=i, error='timeout'))
                continue
            response = future.result()
            body = rclpy.serialization.serialize_message(response)
            fallback = False
            if (i == 0 and response.meta.status != 'OK'
                    and not template.query.use_current):
                # The replayed CDR pins r3's generation identity; if the
                # rebuilt scene digests differ, fall back to CURRENT for the
                # whole arm (recorded; query geometry is unchanged).
                template.query.use_current = True
                template.query.expected_identity = type(
                    template.query.expected_identity)()
                fallback = True
            results.append(dict(
                tag=tag, case=case, i=i,
                elapsed_ms=(t1-t0)/1e6,
                owner_cpu_delta_ms=(cpu1-cpu0)/1e6,
                response_bytes=len(body),
                returned_count=response.meta.returned_count,
                complete=response.meta.complete,
                status=response.meta.status,
                meta_elapsed_ms=response.meta.elapsed_ns/1e6,
                identity_fallback=fallback,
                client_gc=[(t-t0, phase, gen)
                           for t, phase, gen in gc_events if t0 <= t <= t1]))
            time.sleep(0.05)
    finally:
        gc.callbacks.remove(gc_cb)


def boot_owner(args, environment, work, extra=()):
    owner_script = Path(args.source)/'src/luggage_perception/test/occ_query_runtime_owner.py'
    command = [sys.executable, str(owner_script), '--out', str(work/'owner'),
               '--scene', str(args.scene), '--source', args.source,
               '--install', args.install, '--domain', str(args.domain),
               '--resolution', str(args.resolution), '--boxes', str(args.boxes),
               '--observe-gc', '--observe-spans', *extra]
    log = (work/'owner.log').open('w')
    process = subprocess.Popen(command, env=environment, cwd='/tmp',
                               stdout=log, stderr=subprocess.STDOUT)
    deadline = time.monotonic()+120.
    while not (work/'owner'/'ready.json').exists():
        if process.poll() is not None:
            raise RuntimeError('owner startup failed: see %s' % (work/'owner.log'))
        if time.monotonic() > deadline:
            raise RuntimeError('owner startup deadline')
        time.sleep(0.1)
    return process


def stop_owner(process):
    import signal
    process.send_signal(signal.SIGUSR2)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        process.terminate()


def analyze(results, owner_trace):
    import math
    import statistics
    spans = [json.loads(l) for l in open(owner_trace)]
    report = {'arms': []}
    by_arm = {}
    for r in results:
        by_arm.setdefault((r['tag'], r['case']), []).append(r)
    for (tag, case), rows in sorted(by_arm.items()):
        ok = [r for r in rows if 'error' not in r]
        if not ok:
            continue
        elapsed = sorted(r['elapsed_ms'] for r in ok)
        med = statistics.median(elapsed)
        slow = [r for r in ok if r['elapsed_ms'] > 1.5*med]
        # match service spans for this arm's probe ids
        svc = [s for s in spans if s.get('kind') == 'service'
               and str(s.get('request_id', '')).startswith('probe-%s-%s' % (tag, case))]
        ops = sorted(s['operation_ns']/1e6 for s in svc) if svc else []
        slow_ids = {r['i'] for r in slow}
        svc_slow = [s for s in svc
                    if int(str(s.get('request_id', 'x-')).rsplit('-', 1)[-1]) in slow_ids]
        arm = dict(tag=tag, case=case, n=len(ok),
                   elapsed_median_ms=round(med, 2),
                   elapsed_p95_ms=round(elapsed[math.ceil(0.95*len(elapsed))-1], 2),
                   slow_mode_count=len(slow),
                   slow_elapsed_ms=[round(r['elapsed_ms'], 1) for r in slow][:6],
                   operation_median_ms=round(statistics.median(ops), 2) if ops else None,
                   operation_p95_ms=round(sorted(ops)[math.ceil(0.95*len(ops))-1], 2) if ops else None,
                   operation_slow_median_ms=round(statistics.median(
                       [s['operation_ns']/1e6 for s in svc_slow]), 2) if svc_slow else None,
                   cpu_delta_slow_median_ms=round(statistics.median(
                       [r['owner_cpu_delta_ms'] for r in slow]), 2) if slow else None,
                   cpu_delta_fast_median_ms=round(statistics.median(
                       [r['owner_cpu_delta_ms'] for r in ok if r not in slow]), 2),
                   client_gc_in_slow=sum(len(r.get('client_gc', ())) for r in slow),
                   bytes_median=round(statistics.median(r['response_bytes'] for r in ok)))
        report['arms'].append(arm)
    report['owner_gc_events'] = sum(1 for s in spans
                                    if str(s.get('stage', '')).startswith('gc_'))
    report['write_load_events'] = sum(1 for s in spans
                                      if s.get('stage') == 'write_load')
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--install', required=True)
    ap.add_argument('--source', required=True)
    ap.add_argument('--scene', required=True)
    ap.add_argument('--domain', type=int, required=True)
    ap.add_argument('--resolution', type=float, required=True)
    ap.add_argument('--boxes', type=int, required=True)
    ap.add_argument('--replay-dir', required=True,
                    help='r3 matrix group dir with paired/client samples+cdr')
    ap.add_argument('--cases', default='frontier,roi_dense')
    ap.add_argument('--count', type=int, default=150)
    ap.add_argument('--volume-count', type=int, default=100)
    ap.add_argument('--write-hz', type=float, default=2.0)
    ap.add_argument('--gc-disable-arm', action='store_true',
                    help='add arm G: same boot with gc.disable() (GC '
                         'attribution experiment for the slow mode)')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if not 100 <= args.domain <= 232:
        ap.error('dedicated ROS domain required')
    work = Path(args.out)
    work.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    environment = installed_environment(args.install)
    cases = args.cases.split(',')
    templates = load_replay_requests(Path(args.replay_dir), cases)

    import rclpy
    from luggage_msgs.srv import QueryCargoCellsV2
    results = []

    # Arm A: baseline, no writes
    workA = work/'A'
    workA.mkdir()
    owner = boot_owner(args, environment, workA)
    rclpy.init()
    node = rclpy.create_node('phase_probe_client')
    client = node.create_client(QueryCargoCellsV2, '/cargo_map/query_cells_v2')
    if not client.wait_for_service(timeout_sec=30.):
        raise RuntimeError('query service not available')
    for case in cases:
        run_arm(owner, node, client, templates[case], case, args.count, 'A', results)

    # Arm C (volume sweep) reuses the same quiet owner: roi varies the page
    # limit, frontier scales the region box about its center.
    import rclpy.serialization
    for case in cases:
        template = templates[case]
        for sweep in ((256, None), (1024, None), (4096, None)) if case == 'roi_dense' \
                else ((None, 1.0), (None, 0.5), (None, 0.25)):
            limit, scale = sweep
            variant = rclpy.serialization.deserialize_message(
                rclpy.serialization.serialize_message(template), type(template))
            if limit is not None:
                variant.query.limit = limit
            if scale is not None:
                q = variant.query
                cx = (q.region_min.x+q.region_max.x)/2
                cy = (q.region_min.y+q.region_max.y)/2
                cz = (q.region_min.z+q.region_max.z)/2
                for pt, sign in ((q.region_min, -1), (q.region_max, 1)):
                    pt.x = cx+sign*(cx-pt.x)*scale
                    pt.y = cy+sign*(cy-pt.y)*scale
                    pt.z = cz+sign*(cz-pt.z)*scale
            run_arm(owner, node, client, variant, '%s_vol_%s'
                    % (case, limit or scale), args.volume_count, 'C', results)
    stop_owner(owner)
    rclpy.shutdown()

    # Arm B: same boot with the writer thread
    workB = work/'B'
    workB.mkdir()
    rclpy.init()
    node = rclpy.create_node('phase_probe_client_b')
    owner = boot_owner(args, environment, workB,
                       extra=('--write-load-hz', str(args.write_hz)))
    client = node.create_client(QueryCargoCellsV2, '/cargo_map/query_cells_v2')
    if not client.wait_for_service(timeout_sec=30.):
        raise RuntimeError('query service not available (B)')
    for case in cases:
        run_arm(owner, node, client, templates[case], case, args.count, 'B', results)
    stop_owner(owner)
    rclpy.shutdown()

    # Arm G: GC-attribution experiment -- same boot with gc.disable().
    # If the slow mode disappears, the +34ms on-CPU slow mode is the
    # cyclic gen2 collection (owner gen2 rate 31.3% == slow-mode rate).
    traces = [workA/'owner'/'trace.jsonl', workB/'owner'/'trace.jsonl']
    if args.gc_disable_arm:
        workG = work/'G'
        workG.mkdir()
        rclpy.init()
        node = rclpy.create_node('phase_probe_client_g')
        owner = boot_owner(args, environment, workG, extra=('--gc-disable',))
        client = node.create_client(QueryCargoCellsV2, '/cargo_map/query_cells_v2')
        if not client.wait_for_service(timeout_sec=30.):
            raise RuntimeError('query service not available (G)')
        for case in cases:
            run_arm(owner, node, client, templates[case], case, args.count, 'G', results)
        stop_owner(owner)
        rclpy.shutdown()
        traces.append(workG/'owner'/'trace.jsonl')

    merged = work/'merged_trace.jsonl'
    with merged.open('w') as out_stream:
        for trace in traces:
            out_stream.write(trace.read_text())
    report = analyze(results, merged)
    reportB_spans = [json.loads(l) for l in open(workB/'owner'/'trace.jsonl')]
    report['write_load_wall_ms'] = sorted(
        s.get('wall_ms') for s in reportB_spans if s.get('stage') == 'write_load')
    report['records'] = results
    (work/'probe_report.json').write_text(json.dumps(report, indent=1)+'\n')
    for arm in report['arms']:
        print(json.dumps(arm))
    print('owner_gc_events=%d write_load_events=%d' % (
        report['owner_gc_events'], len(report.get('write_load_wall_ms', []))))


if __name__ == '__main__':
    main()
