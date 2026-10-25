"""Bounded real-service samples; installed modules only, no motion clients."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time


LIMITS_MS = {'stats': (50, 200), 'summary': (50, 200), 'frontier_count': (1, 5),
    'roi_dense': (100, 250), 'roi_empty': (100, 250), 'frontier': (100, 250),
    'instance': (100, 250), 'instance_empty': (100, 250),
    'distance_1': (5, 20), 'distance_256': (50, 200),
    'surface': (100, 250), 'patch': (100, 250), 'snapshot': (100, 250)}


def summarize(samples, limits, expected=30):
    if len(samples) != expected:
        return dict(outcome='not_evaluated', samples=len(samples), required=expected)
    ordered = sorted(samples)
    p95, maximum = ordered[math.ceil(.95*len(ordered))-1], ordered[-1]
    return dict(outcome='pass' if p95 <= limits[0] and maximum <= limits[1] else 'fail',
        samples=len(samples), p95_ms=p95, max_ms=maximum, limits_ms=limits)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', required=True)
    p.add_argument('--owner', required=True)
    p.add_argument('--domain', type=int, required=True)
    p.add_argument('--samples', type=int, default=30)
    p.add_argument('--warmup', type=int, default=3)
    p.add_argument('--paired-trace', action='store_true')
    a = p.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(a.domain), ROS_LOCALHOST_ONLY='1')
    import numpy as np
    import rclpy
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import Point
    from luggage_msgs import srv
    from rosidl_runtime_py.set_message import set_message_fields
    out = Path(a.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    ready = json.loads((Path(a.owner)/'ready.json').read_text())
    rclpy.init()
    node = rclpy.create_node('occ_g2_benchmark_client')
    clients = {}
    samples, results = [], {}
    try:
        for kind, endpoint in ((srv.GetCargoMapStats, 'get_stats'),
                (srv.GetCargoMapSummaryV2, 'get_summary_v2'),
                (srv.QueryCargoCellsV2, 'query_cells_v2'),
                (srv.QueryCargoDistanceV2, 'query_distance_v2'),
                (srv.GetCargoSurfacePatchV2, 'get_surface_patch_v2')):
            client = node.create_client(kind, '/cargo_map/'+endpoint)
            if not client.wait_for_service(timeout_sec=10.):
                raise RuntimeError('missing endpoint: '+endpoint)
            clients[kind] = client
        rng = np.random.default_rng(20261025)
        points = rng.uniform([-.7, -.8, ready['floor_z']+.4],
                             [.7, .4, ready['ceiling_z']-.01], (256, 3))
        for case in LIMITS_MS:
            if case == 'snapshot':
                values = [n/1e6 for n in ready['snapshot_ns'][4:]]
                result = summarize(values, LIMITS_MS[case])
                results[case] = {mode:result for mode in ('trace_off','trace_on')} if a.paired_trace else result
                continue
            kind = (srv.GetCargoMapStats if case == 'stats' else
                srv.GetCargoMapSummaryV2 if case == 'summary' else
                srv.QueryCargoDistanceV2 if case.startswith('distance') else
                srv.GetCargoSurfacePatchV2 if case in ('surface', 'patch') else srv.QueryCargoCellsV2)
            measured = {mode:[] for mode in ('trace_off','trace_on')}
            failures = {}
            sequence = [(i,mode) for i in range(-1,a.warmup+a.samples)
                for mode in (('trace_off','trace_on') if i%2 else ('trace_on','trace_off'))] if a.paired_trace else [
                    (i,'trace_off') for i in range(-1,a.warmup+a.samples)]
            for i,mode in sequence:
                request = kind.Request()
                request_id = case+'-'+str(i)+('-'+mode if a.paired_trace else '')
                if hasattr(request, 'query'):
                    q = request.query
                    q.request_id, q.frame_id = request_id, 'container_link'
                    q.use_current = q.release_after_query = True
                    q.limit = 4096
                if case.startswith('roi'):
                    q.has_region = True
                    lower, upper = ((10., 10., 10.), (11., 11., 11.)) if case == 'roi_empty' else (
                        (-.15, -.15, ready['floor_z']), (.15, .15, ready['floor_z']+.3))
                    q.region_min, q.region_max = Point(x=lower[0], y=lower[1], z=lower[2]), Point(x=upper[0], y=upper[1], z=upper[2])
                elif case in ('frontier', 'frontier_count'):
                    request.mode = 1 if case == 'frontier' else 3
                elif case.startswith('instance'):
                    request.mode = 2
                    if case == 'instance' and ready['actual_boxes']:
                        request.instance.kind = 2
                        request.instance.box_id = 'runtime-box-0'
                    else:
                        # A valid absent sensor key needs the current epoch.
                        request.instance.kind = 1
                        request.instance.sensor_instance_id = 987654321
                        request.instance.producer_namespace = 'legacy:'+ready['since_identity']['epoch']
                elif case.startswith('distance'):
                    request.points = [Point(x=float(x), y=float(y), z=float(z))
                        for x, y, z in points[:1 if case == 'distance_1' else 256]]
                    request.max_radius_m = .30
                elif case == 'patch':
                    request.mode = 1
                    set_message_fields(request.since_identity, ready['since_identity'])
                # CDR capture is outside the measured service interval.
                payload = serialize_message(request)
                (out/(request_id+'.request.cdr')).write_bytes(payload)
                begin = time.perf_counter_ns()
                future = clients[kind].call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
                elapsed_ns = time.perf_counter_ns()-begin
                if not future.done() or future.exception():
                    raise RuntimeError('deadline/error: '+request_id)
                response = future.result()
                body = serialize_message(response)
                (out/(request_id+'.response.cdr')).write_bytes(body)
                status = response.meta.status if hasattr(response, 'meta') else ('OK' if response.success else response.message)
                # Distance/frontier-count gates are pure-query latency; also
                # retain the complete ROS round trip for every request.
                latency_ns = response.meta.elapsed_ns if case.startswith('distance') or case == 'frontier_count' else elapsed_ns
                record = dict(case=case, index=i, cold=i==-1, warmup=0<=i<a.warmup, status=status,
                    trace=mode,
                    monotonic_ns=begin, elapsed_ns=elapsed_ns, gated_ns=latency_ns,
                    request_sha256=hashlib.sha256(payload).hexdigest(),
                    response_sha256=hashlib.sha256(body).hexdigest(), response_bytes=len(body))
                if hasattr(response, 'meta'):
                    record.update(returned_count=response.meta.returned_count,
                        complete=response.meta.complete, examined_cells=response.meta.examined_cells)
                samples.append(record)
                with (out/'samples.jsonl').open('a') as stream:
                    stream.write(json.dumps(record)+'\n')
                if status != 'OK':
                    failures[mode] = dict(outcome='fail', reason=status, first_failure=i)
                    break
                if latency_ns/1e6 > LIMITS_MS[case][1]:
                    failures[mode] = dict(outcome='fail', reason='cold_or_sample_max',
                        first_failure=i, max_ms=latency_ns/1e6)
                    break
                if i >= a.warmup:
                    measured[mode].append(latency_ns/1e6)
            result = {mode:failures.get(mode,summarize(values,LIMITS_MS[case]))
                      for mode,values in measured.items()}
            results[case] = result if a.paired_trace else result['trace_off']
    finally:
        (out/'result.json').write_text(json.dumps(dict(results=results,
            overall_g2='not_evaluated', samples_requested=a.samples, warmup=a.warmup,
            complete=len(results)==len(LIMITS_MS)), indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    verdicts = [result for modes in results.values() for result in modes.values()] if a.paired_trace else results.values()
    return int(any(r['outcome'] != 'pass' for r in verdicts))


if __name__ == '__main__':
    raise SystemExit(main())
