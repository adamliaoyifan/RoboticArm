#!/usr/bin/env python3
"""G4-A gen3 A9 capture on/off pairing v4 (review_64f0bdf0 Finding 1).

Both arms run the SAME isolated backend and topology: a parent writer
process and an INDEPENDENT query subprocess (QueryReplica + facade).
The ONLY variable is the diagnostic capture recorder:
- OFF: replica record_events=False, no span collector, no per-request
  identity-join records
- ON : replica event log + per-operation span collector + one JSON
  identity-join record per request

v4 PRE-REGISTERED design (fixed before running; no best-of, no
post-hoc threshold changes -- the gate stays P95 ratio <= 1.10):
Per kind (roi / distance / surface) the sample schedule is
  2 warmup pairs with orders AB, BA (both arms warm, each position
  once), then 30 measurement pairs ALTERNATING AB, BA, AB, BA...
Each pair takes two consecutive requests (one per arm); the arm in
position 1 alternates, so recorder-ON and recorder-OFF each land in
position 1 exactly 15 times. The parity-position bias that produced
the spurious surface ~1.17 in v3 cancels by construction, and the
remaining ratio measures capture cost. The IDENTICAL control runs
the SAME schedule with the recorder off in BOTH buckets -- it must
PASS; the SLOW control (ON recorder sleeps 10ms) must FAIL.

Every measurement sample persists latency + pair + order + position
+ arm + the request identity the handle returned, so the order
decomposition AND the capture join are auditable from the output
alone.

v4.1 (reviews follow-up on ed44accb -- two fixes ONLY; schedule,
gate and control definitions unchanged): (a) the identical control
now computes the effective recorder state AT THE SOURCE, so the ON
bucket appends NO per-request join record either (the v4 probe
still appended one); the identical report must show records == 0.
(b) capture completeness is a REAL identity join implemented once
in capture_join_complete() and called by BOTH the CLI and the
behavior tests: every ON measurement sample must be matched by
exactly one record whose request_id encodes its (kind, pair) and
whose identity EQUALS the identity that request actually returned
-- unique ids + non-empty identities + count alone is not a join.

Exit 0 pass / 3 fail / 4 not_evaluated.
"""

import argparse
import fcntl
import json
import math
import os
import subprocess
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_description"))
sys.path.insert(0, _WS_SRC)

import numpy as np

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'
REGION = ((-0.2, -0.2, 0.5), (0.2, 0.2, 0.9))
PASS, FAIL, NOT_EVALUATED = 0, 3, 4
KINDS = ('roi', 'distance', 'surface')


def nearest_rank_p95(values, min_samples=30):
    values = sorted(v for v in values if v is not None)
    if len(values) < min_samples:
        return None
    return values[math.ceil(0.95 * len(values)) - 1]


def pair_schedule(samples, warmup_pairs=2):
    """Pre-fixed v4 schedule: `warmup_pairs` warmup pairs opening
    AB, BA (both arms warm, each position once), then `samples`
    measurement pairs alternating AB, BA. Deterministic, no RNG,
    fixed before any measurement -- the balance property (each arm
    in each position exactly samples/2 times) is pinned by test."""
    return ['AB', 'BA'] + ['AB' if (p % 2 == 0) else 'BA'
                           for p in range(samples)]


def position_order_effect(samples, arm):
    """Order-effect decomposition for one arm from per-sample records
    (v4): mean full_ms by pair position (1/2) and by order type, plus
    the position effect mean(pos2)-mean(pos1). Warmup excluded. A
    balanced schedule leaves each arm 15 samples per position."""
    by_position = {1: [], 2: []}
    by_order = {'AB': [], 'BA': []}
    for sample in samples:
        if sample.get('arm') != arm or sample.get('warmup'):
            continue
        by_position[sample['position']].append(sample['full_ms'])
        by_order[sample['order']].append(sample['full_ms'])

    def mean(values):
        return round(sum(values) / len(values), 4) if values else None

    mean1, mean2 = mean(by_position[1]), mean(by_position[2])
    return {
        'mean_by_position_ms': {'1': mean1, '2': mean2},
        'mean_by_order_ms': {o: mean(v) for o, v in by_order.items()},
        'position_effect_ms': (round(mean2 - mean1, 4)
                               if mean1 is not None and mean2 is not None
                               else None),
        'n_by_position': {'1': len(by_position[1]),
                          '2': len(by_position[2])},
    }


def capture_join_complete(samples_by_kind, records, expected_pairs,
                          kinds=KINDS):
    """REAL per-request identity join (v4.1; the single judge shared
    by the CLI and the behavior tests). For every kind, each ON-arm
    measurement sample (pair 0..expected_pairs-1) must be matched by
    EXACTLY ONE record whose request_id is '<kind>-p<pair>' and whose
    identity EQUALS the identity that request returned (stamped on
    the sample by the worker). Records with unknown/duplicate ids,
    empty identities, identity mismatches, missing pairs or
    duplicated ON samples all FAIL -- unique ids + non-empty
    identities + count alone is NOT a join."""
    expected = {'%s-p%d' % (kind, pair)
                for kind in kinds for pair in range(expected_pairs)}
    on_identity = {}
    duplicate_on_samples = []
    for kind, samples in samples_by_kind.items():
        for sample in samples:
            if sample.get('arm') != 'on' or sample.get('warmup'):
                continue
            key = '%s-p%d' % (kind, sample['pair'])
            if key in on_identity:
                duplicate_on_samples.append(key)
            else:
                on_identity[key] = sample.get('identity')
    seen = set()
    joined = set()
    duplicate_ids, unmatched_ids = [], []
    missing_identity, identity_mismatch = [], []
    for record in records:
        rid = record.get('request_id')
        if rid in seen:
            duplicate_ids.append(str(rid))
            continue
        seen.add(rid)
        if rid not in expected:
            unmatched_ids.append(str(rid))
            continue
        identity = record.get('identity')
        if not identity:
            missing_identity.append(str(rid))
            continue
        if identity != on_identity.get(rid):
            identity_mismatch.append(str(rid))
            continue
        joined.add(rid)
    missing = sorted(expected - joined)
    complete = (not missing and not duplicate_ids and not unmatched_ids
                and not missing_identity and not identity_mismatch
                and not duplicate_on_samples
                and len(records) == len(expected))
    return {
        'records': len(records),
        'expected': len(expected),
        'joined': len(joined),
        'on_measurement_samples': len(on_identity),
        'missing': missing[:10],
        'duplicate_ids': duplicate_ids[:10],
        'unmatched_ids': unmatched_ids[:10],
        'missing_identity': missing_identity[:10],
        'identity_mismatch': identity_mismatch[:10],
        'duplicate_on_samples': duplicate_on_samples[:10],
        'rule': ('record identity == sample identity per (kind,pair), '
                 'exactly one record per ON measurement request'),
        'complete': complete,
    }


def identical_recorder_off(records):
    """Identical-control contract (v4.1): the recorder is fully off in
    BOTH buckets -- no replica events, no spans, and no per-request
    join records either. Anything beyond zero records FAILs."""
    return {'records': len(records),
            'recorder_fully_off': not records,
            'rule': 'identical control must produce zero join records'}


WORKER = r'''
import json, os, sys, time
root, out_path, mode_flag = sys.argv[1:4]
slow = mode_flag == 'slow'
for entry in (%r + "/src/luggage_description",
              %r + "/src/luggage_perception", %r):
    sys.path.insert(0, entry)
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception import cargo_map_query as core

REGION = %r
records = []
spans = []


def record(entry):
    if slow:
        time.sleep(0.010)          # injected slow recorder (control)
    records.append(entry)


def set_capture(on):
    """The ONLY variable between arms, toggled PER SAMPLE inside a
    pre-fixed AB/BA pair schedule so machine drift and position bias
    hit both arms equally. The caller passes the EFFECTIVE state:
    in the identical control the recorder is disabled at the call
    site for BOTH buckets, so no arm records events, spans, or
    per-request join records (v4.1)."""
    replica._record_events = on
    core.set_span_collector(spans if on else None)


replica = QueryReplica(root, record_events=False)
facade = QueryMapReadFacade(replica)
deadline = time.monotonic() + 120
while time.monotonic() < deadline:
    if replica.poll() == ReplicaState.READY:
        break
    time.sleep(0.05)
assert replica.state == ReplicaState.READY, str(replica.state)

rng_points = __import__('numpy').random.default_rng(11)
POINTS = [(float(x), float(y), float(z)) for x, y, z in
          rng_points.uniform((-0.6, -0.6, 0.55), (0.6, 0.6, 1.35),
                             (60, 3))]


def op(handle, kind):
    if kind == 'roi':
        handle.query_cells(REGION, (0, 1, 2),
                           page=core.QueryPage(4096, ''))
    elif kind == 'distance':
        handle.nearest_obstacles(POINTS, 0.5, 'separate')
    else:
        handle.surface()


WARMUP_PAIRS = 2
SCHEDULE = %r                     # pre-fixed by the parent (v4)
out = {'kinds': {}, 'records': [], 'spans': 0}
for kind in ('roi', 'distance', 'surface'):
    buckets = {'on': {'full': [], 'direct': []},
               'off': {'full': [], 'direct': []}}
    samples = []
    schedule = SCHEDULE
    for pair_index, order in enumerate(schedule):
        arms = ('on', 'off') if order == 'AB' else ('off', 'on')
        warmup = pair_index < WARMUP_PAIRS
        for position, arm in enumerate(arms, start=1):
            # capture_enabled is the EFFECTIVE recorder state,
            # computed at the source (v4.1): the identical control
            # (A9V2_IDENTICAL=1) disables the recorder for BOTH
            # buckets entirely -- no events, no spans, and no
            # per-request join record either -- while the buckets
            # themselves stay REAL and separate (labels unchanged).
            capture_enabled = (arm == 'on'
                               and os.environ.get('A9V2_IDENTICAL')
                               != '1')
            set_capture(capture_enabled)
            t0 = time.perf_counter_ns()
            handle = facade.acquire_query_snapshot('CURRENT')
            t1 = time.perf_counter_ns()
            op(handle, kind)
            t2 = time.perf_counter_ns()
            ident = list(handle.identity)
            handle.close()
            # capture cost INSIDE the request window; warmup pairs
            # excluded from buckets and from the join record set
            if capture_enabled and not warmup:
                record({'kind': kind,
                        'request_id': '%%s-p%%d' %% (
                            kind, pair_index - WARMUP_PAIRS),
                        'identity': ident})
            t3 = time.perf_counter_ns()
            sample = {'pair': pair_index - WARMUP_PAIRS,
                      'order': order, 'arm': arm, 'position': position,
                      'warmup': warmup, 'identity': ident,
                      'full_ms': round((t3 - t0) / 1e6, 4),
                      'direct_ms': round((t2 - t1) / 1e6, 4)}
            samples.append(sample)
            if not warmup:
                buckets[arm]['full'].append(sample['full_ms'])
                buckets[arm]['direct'].append(sample['direct_ms'])
    out['kinds'][kind] = {'buckets': buckets, 'samples': samples}
out['records'] = records
out['spans'] = len(spans)
with open(out_path, 'w') as handle:
    json.dump(out, handle)
'''


def build_world(resolution, boxes):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='a9v2')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        assert store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i)).accepted
    rng = np.random.default_rng(20261023)
    points = np.column_stack([
        rng.uniform(-0.5, 0.5, 20000),
        rng.uniform(-0.6, 0.6, 20000),
        rng.uniform(mapper._hull.floor_z + .1,
                    mapper._hull.floor_z + 1.2, 20000)])
    assert store.apply_update(MapUpdate(
        operation_id='seed', kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=points,
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(1, 0), acquisition_stamp_ns=1_000_000_001,
        settled=True, schema_version=1)).accepted
    return store


def run_worker(root, mode, samples):
    out_path = tempfile.mktemp(prefix='a9v2-%s-' % mode,
                               suffix='.json')
    subprocess.run(
        [sys.executable, '-c',
         WORKER % (_WS_SRC, _WS_SRC, _WS_SRC, REGION,
                   tuple(pair_schedule(samples))),
         root, out_path, mode, str(samples)],
        check=True, timeout=600)
    with open(out_path) as handle:
        data = json.load(handle)
    os.unlink(out_path)
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--boxes', type=int, default=20)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--ratio-bound', type=float, default=1.10)
    parser.add_argument('--mode', choices=('normal', 'identical',
                                           'slow'),
                        default='normal')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    if os.path.exists(args.out):
        raise SystemExit('refusing to overwrite %s' % args.out)

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)
    store = build_world(args.resolution, args.boxes)
    root = tempfile.mkdtemp(prefix='a9v2-root-')
    hook = wire_store_export(store, root)
    store._snapshot = store._build_snapshot()
    hook.wait_pending(120.0)

    if args.mode == 'identical':
        # both buckets recorder-OFF AT THE SOURCE (v4.1): the worker
        # computes the effective state per sample, so ON samples run
        # the identical recorder-off path AND append no join records
        os.environ['A9V2_IDENTICAL'] = '1'
    mode_flag = ('slow' if args.mode == 'slow'
                 else 'identical' if args.mode == 'identical'
                 else 'normal')
    data = run_worker(root, mode_flag, args.samples)
    # BOTH buckets are real measured arrays in every mode (identical:
    # the recorder was disabled in both -- the two arrays still come
    # from genuinely different samples and must agree on their own).
    off = {'kinds': {k: {'full_ms': v['buckets']['off']['full'],
                         'direct_ms': v['buckets']['off']['direct']}
                     for k, v in data['kinds'].items()}}
    on = {'kinds': {k: {'full_ms': v['buckets']['on']['full'],
                        'direct_ms': v['buckets']['on']['direct']}
                    for k, v in data['kinds'].items()}}
    hook.close()
    os.close(lock_fd)

    # Capture completeness (ON bucket): REAL per-request identity join
    # via the shared judge -- exactly one record per counted ON
    # request, and each record's identity must EQUAL the identity that
    # request returned (the worker stamps it on the sample). Missing,
    # duplicated, unknown-id or identity-mismatched records FAIL.
    # identical mode must show ZERO records (recorder fully off in
    # both buckets, v4.1).
    records = data.get('records', [])
    if args.mode == 'identical':
        join_detail = identical_recorder_off(records)
        complete = join_detail['recorder_fully_off']
    else:
        join_detail = capture_join_complete(
            {kind: payload['samples']
             for kind, payload in data['kinds'].items()},
            records, args.samples)
        complete = join_detail['complete']

    report = {
        'revision': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC,
            text=True).strip(),
        'config': {'resolution': args.resolution,
                   'boxes': args.boxes, 'samples': args.samples,
                   'ratio_bound': args.ratio_bound},
        'mode': args.mode,
        'scope': ('capture recorder on/off, independent query process '
                  'both arms, same isolated backend; pre-fixed balanced '
                  'AB/BA pair schedule; full-request P95 ratio per kind; '
                  'completeness = exact per-request identity join '
                  '(record identity == sample identity per (kind,pair))'),
        'capture_join': join_detail,
        'records_on': records[:5],
        'recorder_boundary': ('in-memory ring/list append only (the '
                              'production recorders are the replica '
                              'event deque and the span list; no per-'
                              'request persistence is claimed)'),
        'spans_on': data.get('spans'),
        'capture_records_expected': args.samples * len(KINDS),
        'capture_complete': complete,
        'kinds': {},
        'exit': None,
    }
    verdict = PASS
    for kind in KINDS:
        off_p95 = nearest_rank_p95(off['kinds'][kind]['full_ms'])
        on_p95 = nearest_rank_p95(on['kinds'][kind]['full_ms'])
        ratio = (round(on_p95 / off_p95, 4)
                 if off_p95 and on_p95 else None)
        entry = {
            'off_full_p95_ms': round(off_p95, 3) if off_p95 else None,
            'on_full_p95_ms': round(on_p95, 3) if on_p95 else None,
            'off_direct_p95_ms': round(
                nearest_rank_p95(off['kinds'][kind]['direct_ms'])
                or -1, 3),
            'on_direct_p95_ms': round(
                nearest_rank_p95(on['kinds'][kind]['direct_ms'])
                or -1, 3),
            'ratio': ratio,
            'schedule': ('2 warmup pairs (AB,BA) + %d measurement pairs '
                         'alternating AB/BA (pre-fixed)' % args.samples),
            'position_balance': {
                arm: position_order_effect(
                    data['kinds'][kind]['samples'], arm)
                for arm in ('on', 'off')},
            'raw': {'off_full': [round(v, 3) for v in
                                 off['kinds'][kind]['full_ms']],
                    'on_full': [round(v, 3) for v in
                                on['kinds'][kind]['full_ms']],
                    'samples': data['kinds'][kind]['samples']}}
        if ratio is None:
            entry['verdict'] = NOT_EVALUATED
            if verdict != FAIL:
                verdict = NOT_EVALUATED
        elif ratio > args.ratio_bound:
            entry['verdict'] = FAIL
            verdict = FAIL
        else:
            entry['verdict'] = PASS
        report['kinds'][kind] = entry
    if not complete:
        verdict = FAIL
        report['capture_completeness_verdict'] = 'FAIL'
    else:
        report['capture_completeness_verdict'] = 'PASS'
    # mode expectations (the controls must behave as designed)
    if args.mode == 'identical' and verdict != PASS:
        verdict = FAIL
    if args.mode == 'slow' and verdict == PASS:
        verdict = FAIL
        report['control_note'] = ('slow-recorder control unexpectedly '
                                  'passed -- judge lacks discriminating '
                                  'power')
    report['exit'] = verdict
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps({k: {kk: vv for kk, vv in v.items()
                          if kk != 'raw'}
                      for k, v in report['kinds'].items()}, indent=1))
    print('mode', args.mode, 'capture_complete', complete,
          'exit', verdict)
    return verdict


if __name__ == '__main__':
    sys.exit(main())
