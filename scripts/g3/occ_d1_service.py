#!/usr/bin/env python3
"""D1: same-request service account under the REAL write stream.

Single process, real ROS service (MapQueryService), real 30k-point
INTEGRATE_VIEW writes (same construction as occ_query_write_fixture).
Per ROI request, aligned on one timeline:

  client_send -> [residual_1: transport + executor scheduling, NOT
  further split here] -> callback_enter -> open (lock wait/hold from
  the registry spans) -> pure query (query-core spans) -> typed
  construction -> meta/close -> callback_exit (wall + serving-thread
  CPU) -> [residual_2: response transport + CDR, labeled, never
  estimated] -> client_received.

Writer spans record each apply_update (wall + writer-thread CPU +
registry publish wait/hold) so request/write window overlap is exact.
GC intervals recorded via gc.callbacks; overlap flagged per request.
Answers the four-way question per slowed request: more compute /
lock wait / thread-not-scheduled (callback wall - CPU) / transport +
callback scheduling (residuals).
"""

import argparse
import gc
import json
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
for _p in (("src", "luggage_perception"), ("src", "luggage_description"),
           ("src",)):
    sys.path.insert(0, os.path.join(_WS_SRC, *_p))

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Point
from luggage_msgs.srv import QueryCargoCellsV2

from luggage_perception.cargo_map_query_service import MapQueryService
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
import luggage_perception.cargo_map_query as query_module


def build_store(boxes):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.01,
                               max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='d1')
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


def make_write_update(mapper, store, points, index):
    minimum_z = mapper._hull.floor_z + .05
    origin = (0.0, 0.0, mapper._hull.ceiling_z - 0.1)
    return MapUpdate(
        operation_id='d1-%d' % index, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity, geometry_hash=mapper.geometry_hash,
        points=points, origin=origin, minimum_z=minimum_z,
        stamp_key=(index + 1, 0),
        acquisition_stamp_ns=1_000_000_000 + index, settled=True,
        schema_version=1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--points', default='')
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--warmup', type=int, default=3)
    parser.add_argument('--no-writer', action='store_true')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    store, mapper = build_store(args.boxes)
    if args.points and os.path.exists(args.points):
        points = np.load(args.points)
    else:
        rng = np.random.default_rng(7)
        points = np.column_stack([
            rng.uniform(-0.6, 0.6, 30000),
            rng.uniform(-0.8, 0.8, 30000),
            rng.uniform(mapper._hull.floor_z + .1,
                        mapper._hull.floor_z + 1.2, 30000)])

    spans = []
    query_module.set_span_collector(spans)
    store._query_registry.span_collector = spans

    gc_events = []

    _gc_open = {}

    def on_gc(phase, info):
        now = time.perf_counter_ns()
        gen = info.get('generation')
        if phase == 'start':
            _gc_open['t'] = now
            _gc_open['cpu'] = time.thread_time_ns()
        else:
            begin = _gc_open.pop('t', None)
            if begin is not None:
                gc_events.append({'begin_ns': begin, 'end_ns': now,
                                  'gen': gen,
                                  'wall_ns': now - begin,
                                  'cpu_ns': (time.thread_time_ns()
                                             - _gc_open.pop('cpu', now))})

    gc.callbacks.append(on_gc)

    rclpy.init()
    node = Node('occ_d1_service')
    MapQueryService(node, store, ReentrantCallbackGroup())
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    client = Node('occ_d1_client')
    executor.add_node(client)
    query_client = client.create_client(
        QueryCargoCellsV2, '/cargo_map/query_cells_v2')
    while not query_client.wait_for_service(timeout_sec=2.0):
        pass

    floor = mapper._hull.floor_z

    def make_request(seq):
        request = QueryCargoCellsV2.Request()
        request.query.request_id = 'd1-%d' % seq
        request.query.frame_id = 'container_link'
        request.query.use_current = True
        request.query.release_after_query = True
        request.query.limit = 4096
        request.query.has_region = True
        request.query.region_min = Point(x=-.15, y=-.15, z=floor)
        request.query.region_max = Point(x=.15, y=.15, z=floor + .3)
        return request

    writer_rows = []
    stop = threading.Event()
    writer_cpu_base = {'cpu': 0.0}
    # In-flight write window: a request overlapping a write that has
    # not COMPLETED yet must still count (completed-only rows would
    # systematically miss the exact contention we measure).
    in_flight = {'begin_ns': None}

    def writer():
        index = 0
        while not stop.is_set():
            index += 1
            update = make_write_update(mapper, store, points, index)
            t0 = time.perf_counter_ns()
            in_flight['begin_ns'] = t0
            cpu0 = time.thread_time_ns()
            result = store.apply_update(update)
            wall = time.perf_counter_ns() - t0
            cpu = time.thread_time_ns() - cpu0
            in_flight['begin_ns'] = None
            writer_cpu_base['cpu'] = cpu
            writer_rows.append({'index': index,
                                'accepted': bool(result.accepted),
                                'begin_ns': t0, 'end_ns': t0 + wall,
                                'wall_ns': wall, 'cpu_ns': cpu})
            stop.wait(0.5)

    if not args.no_writer:
        thread = threading.Thread(target=writer, daemon=True)
        thread.start()
        time.sleep(1.0)

    rows = []
    seq = 0
    for i in range(args.warmup + args.samples):
        seq += 1
        spans.clear()
        request = make_request(seq)
        t_send = time.perf_counter_ns()
        future = query_client.call_async(request)
        while not future.done():
            time.sleep(0.0005)
        t_recv = time.perf_counter_ns()
        response = future.result()
        if i < args.warmup:
            continue
        flight_begin = in_flight['begin_ns']
        records = list(spans)
        service = next((r for r in records if r.get('kind') == 'service'),
                       None)
        splits = [r for r in records if r.get('kind') == 'cells_split']
        locks = [r for r in records if 'operation' in r]
        qcore = [r for r in records if 'total_ns' in r]
        rows.append({
            'seq': seq,
            'ok': response.meta.status == 'OK',
            'client_wall_ns': t_recv - t_send,
            't_send': t_send, 't_recv': t_recv,
            'service': service,
            'pure_query_ns': next((r['pure_query_ns'] for r in splits
                                   if 'pure_query_ns' in r), None),
            'typed_ns': next((r['typed_ns'] for r in splits
                              if 'typed_ns' in r), None),
            'locks': locks,
            'qcore': qcore[0] if qcore else None,
            'gc_in_window': [e for e in gc_events
                             if e['begin_ns'] < t_recv and e['end_ns'] > t_send],
            'writers_in_window': [w for w in writer_rows
                                  if w['end_ns'] >= t_send
                                  and w['begin_ns'] <= t_recv],
            'write_in_flight': (flight_begin is not None
                                and flight_begin <= t_recv),
        })

    stop.set()
    if not args.no_writer:
        thread.join(timeout=2.0)
    gc.callbacks.remove(on_gc)
    executor.shutdown()
    rclpy.shutdown()

    # Per-request account with explicit labeled residuals.
    account = []
    for row in rows:
        svc = row['service'] or {}
        open_wait = sum(l['wait_ns'] for l in row['locks']
                        if l['operation'] in ('open', 'resolve'))
        writer_overlap = sum(min(w['end_ns'], row['t_recv'])
                             - max(w['begin_ns'], row['t_send'])
                             for w in row['writers_in_window'])
        account.append({
            'seq': row['seq'], 'ok': row['ok'],
            'client_wall_ms': round(row['client_wall_ns'] / 1e6, 3),
            'residual1_transport_executor_ms': None if not svc else round(
                (svc['callback_wall_ns'] - svc['callback_wall_ns']) / 1e6, 3),
            'callback_wall_ms': None if not svc else round(
                svc['callback_wall_ns'] / 1e6, 3),
            'callback_cpu_ms': None if not svc else round(
                svc['callback_cpu_ns'] / 1e6, 3),
            'thread_not_scheduled_ms': None if not svc else round(
                (svc['callback_wall_ns'] - svc['callback_cpu_ns']) / 1e6, 3),
            'open_ms': None if not svc else round(svc['open_ns'] / 1e6, 3),
            'lock_wait_ms': round(open_wait / 1e6, 3),
            'pure_query_ms': None if row['pure_query_ns'] is None else round(
                row['pure_query_ns'] / 1e6, 3),
            'typed_ms': None if row['typed_ns'] is None else round(
                row['typed_ns'] / 1e6, 3),
            'close_meta_ms': None if not svc else round(
                svc['close_meta_ns'] / 1e6, 3),
            'gc_overlaps': len(row['gc_in_window']),
            'gc_wall_ms': round(sum(e['wall_ns'] for e in
                                    row['gc_in_window']) / 1e6, 3),
            't_send_ms': round(row['t_send'] / 1e6, 1),
            't_recv_ms': round(row['t_recv'] / 1e6, 1),
            'writer_windows': [[round(w['begin_ns'] / 1e6, 1),
                                round(w['end_ns'] / 1e6, 1)]
                               for w in row['writers_in_window']],
            'write_in_flight': row['write_in_flight'],
            'writer_overlaps': len(row['writers_in_window']),
            'writer_overlap_ms': round(writer_overlap / 1e6, 3),
            'writer_cpu_ms': round(sum(w['cpu_ns'] for w in
                                       row['writers_in_window']) / 1e6, 3),
            'residual2_transport_cdr_ms': None if not svc else round(
                (row['client_wall_ns'] - svc['callback_wall_ns']) / 1e6, 3),
        })
    out = {
        'boxes': args.boxes, 'points': len(points),
        'writer': not args.no_writer,
        'account': account,
        'writers': [{'begin_ms': round(w['begin_ns'] / 1e6, 1),
                     'end_ms': round(w['end_ns'] / 1e6, 1),
                     'wall_ms': round(w['wall_ns'] / 1e6, 3),
                     'cpu_ms': round(w['cpu_ns'] / 1e6, 3),
                     'accepted': w['accepted']} for w in writer_rows],
        'gc_events': [{'begin_ms': round(e['begin_ns'] / 1e6, 1),
                       'wall_ms': round(e['wall_ns'] / 1e6, 3),
                       'gen': e['gen']} for e in gc_events
                      if e['wall_ns'] > 1_000_000],
    }
    with open(args.out, 'w') as handle:
        json.dump(out, handle, indent=1)
    slow = sorted(account, key=lambda r: -r['client_wall_ms'])[:5]
    print(json.dumps(slow, indent=1))
    print('saved', args.out)


if __name__ == '__main__':
    sys.exit(main())
