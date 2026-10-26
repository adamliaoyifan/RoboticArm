#!/usr/bin/env python3
"""ROI_CAUSE diagnostic: where the 445-578 ms went, old vs pruned.

Same failing input as G2 concurrency_rehearsal_06 (candidate side):
QueryCargoCellsV2 region (-0.15,-0.15,floor)..(0.15,0.15,floor+0.3),
limit 4096, deployment container, 1 cm, 0/5/20 boxes (same box layout
as occ_query_concurrency_owner). Alternates DISABLE_PRUNE on/off IN THE
SAME BINARY so instrumentation overhead is identical, 3 warmup + 30
measured per mode. Optional writer overlap pass: a background thread
applying ADD_BOX_V1 updates at ~2 Hz (offline stand-in for the cloud
integrate stream; labeled approximation -- the original 30k-cloud
write path replays in the formal D1).

Outputs a mutually-exclusive stage table (select/sort/mask/prefix/
records + residual) plus registry lock wait/hold spans. No ROS.
"""

import argparse
import json
import os
import statistics
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(_WS_SRC, "src"))

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper  # noqa: E402
from luggage_perception.occupancy_map_store import (  # noqa: E402
    MapUpdate, OccupancyMapStore, UpdateKind)
import luggage_perception.cargo_map_query as query_module  # noqa: E402


def build_store(boxes):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.01)
    store = OccupancyMapStore(mapper, epoch='roi-cause')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        result = store.apply_update(MapUpdate(
            operation_id='setup-%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.),
            box_id='load-box-%d' % i))
        assert result.accepted, result.reason
    return store, mapper


def region_for(mapper):
    floor = mapper._hull.floor_z
    return ((-0.15, -0.15, floor), (0.15, 0.15, floor + 0.3))


def run_mode(store, mapper, spans, lock_spans, warmup=3, samples=30):
    from luggage_perception.cargo_map_query import QueryPage
    region = region_for(mapper)
    rows = []
    for _ in range(warmup + samples):
        spans.clear()
        lock_spans.clear()
        with store.acquire_query_snapshot() as snapshot:
            page = snapshot.query_cells(region, (0, 1, 2),
                                        page=QueryPage(4096, ''))
        rows.append(dict(spans[0], lock_spans=list(lock_spans)))
    return rows[warmup:]


def summarize(rows):
    def stage(name):
        values = [r[name] for r in rows]
        return values

    out = {}
    total = stage('total_ns')
    out['total_ms'] = _stats(total)
    for key in ('select_ns', 'sort_ns', 'mask_ns', 'prefix_sort_ns',
                'records_ns'):
        out[key.replace('_ns', '_ms')] = _stats(stage(key))
    residual = [r['total_ns'] - sum(r[k] for k in
               ('select_ns', 'sort_ns', 'mask_ns', 'prefix_sort_ns',
                'records_ns')) for r in rows]
    out['residual_ms'] = _stats(residual)
    out['candidate_blocks'] = _stats([r['candidate_blocks'] for r in rows])
    out['examined_blocks'] = _stats([r['examined_blocks'] for r in rows])
    out['returned'] = rows[0]['returned']
    out['complete'] = rows[0]['complete']
    locks = [lock for r in rows for lock in r['lock_spans']]
    by_op = {}
    for lock in locks:
        by_op.setdefault(lock['operation'], []).append(lock)
    out['locks'] = {op: {
        'n': len(v),
        'wait_ms': _stats([x['wait_ns'] for x in v]),
        'hold_ms': _stats([x['hold_ns'] for x in v]),
    } for op, v in by_op.items()}
    return out


def _stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        'n': len(values),
        'median_ms': round(statistics.median(values) / 1e6, 3),
        'p95_ms': round(ordered[int(round(0.95 * len(values))) - 1] / 1e6, 3),
        'max_ms': round(max(values) / 1e6, 3),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--writer-overlap', action='store_true',
                        help='second pass with a 2 Hz ADD_BOX_V1 writer '
                             'thread (offline overlap approximation)')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    store, mapper = build_store(args.boxes)
    query_spans = []
    lock_spans = []
    collector = lambda record: (  # noqa: E731
        query_spans.append(record) if 'total_ns' in record
        else lock_spans.append(record))
    query_module.set_span_collector(query_spans)
    store._query_registry.span_collector = lock_spans

    result = {'boxes': args.boxes, 'resolution': 0.01,
              'region': region_for(mapper), 'limit': 4096}

    def one_query():
        from luggage_perception.cargo_map_query import QueryPage
        query_spans.clear()
        with store.acquire_query_snapshot() as snapshot:
            snapshot.query_cells(region_for(mapper), (0, 1, 2),
                                 page=QueryPage(4096, ''))
        return dict(query_spans[0], lock_spans=list(lock_spans))

    def measure(label):
        rows = run_mode(store, mapper, query_spans,
                        samples=args.samples)
        result[label] = summarize(rows)

    # Interleave A/B per sample (mode order alternates) so warm/cold
    # asymmetry cannot masquerade as a pruning effect.
    pruned_rows = []
    no_prune_rows = []
    for i in range(3 + args.samples):
        query_module.DISABLE_PRUNE = False
        row = one_query()
        if i >= 3:
            pruned_rows.append(row)
        query_module.DISABLE_PRUNE = True
        row = one_query()
        if i >= 3:
            no_prune_rows.append(row)
    result['pruned'] = summarize(pruned_rows)
    result['no_prune'] = summarize(no_prune_rows)

    if args.writer_overlap:
        stop = threading.Event()
        counter = {'n': 0, 'accepted': 0}
        lock = threading.Lock()

        def writer():
            i = 0
            while not stop.is_set():
                with lock:
                    i += 1
                    counter['n'] += 1
                    result_accept = store.apply_update(MapUpdate(
                        operation_id='w-%d' % i,
                        kind=UpdateKind.ADD_BOX_V1,
                        box=((0.4, 0.5, mapper._hull.floor_z + 0.11),
                             (0.2, 0.2, 0.15), 0.),
                        box_id='wbox-%d' % i))
                    if result_accept.accepted:
                        counter['accepted'] += 1
                stop.wait(0.5)

        thread = threading.Thread(target=writer, daemon=True)
        thread.start()
        time.sleep(1.0)
        query_module.DISABLE_PRUNE = False
        rows = run_mode(store, mapper, query_spans, lock_spans,
                        samples=args.samples)
        result['pruned_with_writer'] = summarize(rows)
        query_module.DISABLE_PRUNE = True
        rows = run_mode(store, mapper, query_spans, lock_spans,
                        samples=args.samples)
        result['no_prune_with_writer'] = summarize(rows)
        stop.set()
        thread.join(timeout=2.0)
        result['writer'] = dict(counter)

    with open(args.out, 'w') as handle:
        json.dump(result, handle, indent=1)
    print(json.dumps({k: v for k, v in result.items()
                      if k in ('pruned', 'no_prune')}, indent=1)[:1200])
    print('saved', args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
