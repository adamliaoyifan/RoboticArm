#!/usr/bin/env python3
"""Production-shape probe: idle -> ONE query (the placement pattern).

The placement flow updates OCC only when conditions are met and queries
OCC once before computing a position. The matrix/C2 contracts instead
drive back-to-back query streams -- under which allocation pressure
triggers cyclic gen2 collections inside request paths (the localized
slow mode). This probe measures the production shape directly: an idle
gap, then a single query, repeated across gap lengths, for the two
query types that matter:

  distance_256 : what placement actually asks (small response)
  roi_dense    : the worst large-response interface (upper bound)

Question answered: single-query worst-case round trip vs the 5 s
placement budget, and whether the slow mode even occurs when queries
are sparse.
"""
import argparse
import json
import statistics
import sys
import time
from pathlib import Path

_WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_WS/'scripts'/'g3'))
sys.path.insert(0, str(_WS/'src'/'luggage_perception'/'test'))

from occ_query_runtime_environment import installed_environment  # noqa: E402
from occ_roundtrip_phase_probe import (boot_owner, load_replay_requests,  # noqa: E402
                                        rss_kib, stop_owner)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--install', required=True)
    ap.add_argument('--source', required=True)
    ap.add_argument('--scene', required=True)
    ap.add_argument('--domain', type=int, required=True)
    ap.add_argument('--resolution', type=float, required=True)
    ap.add_argument('--boxes', type=int, required=True)
    ap.add_argument('--replay-dir', required=True)
    ap.add_argument('--cycles', type=int, default=4,
                    help='repetitions of the idle ladder')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if not 100 <= args.domain <= 232:
        ap.error('dedicated ROS domain required')

    import rclpy
    import rclpy.serialization
    from luggage_msgs.srv import (QueryCargoCellsV2, QueryCargoDistanceV2)

    templates = load_replay_requests(Path(args.replay_dir), ('roi_dense',))
    roi = templates['roi_dense']
    dist_rows = [json.loads(l) for l in
                 (Path(args.replay_dir)/'paired'/'client'/'samples.jsonl').open()]
    dist_row = next(r for r in dist_rows if r['case'] == 'distance_256'
                    and r['trace'] == 'trace_off' and not r.get('warmup')
                    and not r.get('cold'))
    dist_cdr = Path(args.replay_dir)/'paired'/'client'/(
        'distance_256-%d-trace_off.request.cdr' % dist_row['index'])
    dist = rclpy.serialization.deserialize_message(
        dist_cdr.read_bytes(), QueryCargoDistanceV2.Request)

    work = Path(args.out)
    work.mkdir(parents=True, exist_ok=True)
    import os
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    environment = installed_environment(args.install)
    owner = boot_owner(args, environment, work)
    rclpy.init()
    node = rclpy.create_node('production_shape_probe')
    cells_client = node.create_client(QueryCargoCellsV2, '/cargo_map/query_cells_v2')
    dist_client = node.create_client(QueryCargoDistanceV2, '/cargo_map/query_distance_v2')
    if not (cells_client.wait_for_service(timeout_sec=30.)
            and dist_client.wait_for_service(timeout_sec=30.)):
        raise RuntimeError('services not available')

    def ask(client, template, tag, idle_s, seq):
        request = rclpy.serialization.deserialize_message(
            rclpy.serialization.serialize_message(template), type(template))
        request.query.request_id = 'prod-%s-%d-%d' % (tag, int(idle_s), seq)
        time.sleep(idle_s)
        t0 = time.perf_counter_ns()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.)
        t1 = time.perf_counter_ns()
        return dict(kind=tag, idle_s=idle_s, seq=seq,
                    elapsed_ms=(t1-t0)/1e6,
                    status=future.result().meta.status
                    if future.done() else 'TIMEOUT')

    ladder = (30., 10., 5., 2., 1.)
    records = []
    try:
        for cycle in range(args.cycles):
            for idle in ladder:
                records.append(ask(cells_client, roi, 'roi_dense', idle, cycle))
                records.append(ask(dist_client, dist, 'distance_256', idle, cycle))
    finally:
        stop_owner(owner, work=work)
        rclpy.shutdown()

    summary = {}
    for kind in ('roi_dense', 'distance_256'):
        vals = sorted(r['elapsed_ms'] for r in records
                      if r['kind'] == kind and r['status'] == 'OK')
        if vals:
            summary[kind] = dict(
                n=len(vals),
                median_ms=round(statistics.median(vals), 2),
                p95_ms=round(vals[min(len(vals)-1, int(.95*len(vals)))], 2),
                max_ms=round(vals[-1], 2),
                over_100ms=sum(1 for v in vals if v > 100),
                budget_ms=5000.,
                verdict=('PASS' if vals[-1] < 5000 else 'FAIL')
                + ' (placement budget 5s)')
    report = dict(kind='production_shape_probe', records=records,
                  summary=summary, ladder_s=ladder, cycles=args.cycles)
    (work/'production_shape.json').write_text(json.dumps(report, indent=1)+'\n')
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
