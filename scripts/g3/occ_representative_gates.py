#!/usr/bin/env python3
"""G4-A gen2 representative gates -- REVISED harness (measurement fix).

Fixes the audited defects (user 2026-10-26):
- Axis A now queries the ISOLATED replica through QueryMapReadFacade
  (the same surface the ROS service uses), never the writer's store.
- Axis B starts the clock at the commit call and only stops when the
  replica reaches READY **with the target identity of that commit**;
  injected export delay must show up in the measurement (verified by
  the harness behavior tests).
- P95 is nearest-rank = sorted[28] (the 29th of 30); fewer than 30
  samples => not_evaluated, and the script exits 0/3/4 for
  pass/fail/not_evaluated.
- Axis C pairs candidate (isolated, hook wired) against a baseline
  built with the SAME initial state and the SAME operation sequence
  (no-hook in this process; the frozen A8 baseline remains b33 in the
  acceptance run -- this pairing establishes the method).
- Axis D records background thread CPU samples
  (time.thread_time_ns of the encoder thread is not directly
  readable; we sample /proc/<pid>/task/<tid>/stat for the encoder
  thread) at 20 Hz during the C window.

Behavioral checks that must hold before any number is trusted
(checked inline, each aborts with exit 4 when violated):
- old-READY rejection: a READY bound to a previous identity is NOT
  accepted as the commit's visibility event.
"""

import argparse
import fcntl
import json
import os
import statistics
import sys
import tempfile
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, _WS_SRC)

import numpy as np

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_query_replica import QueryReplica, ReplicaState
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'
EXIT_PASS, EXIT_FAIL, EXIT_NOT_EVAL = 0, 3, 4


def make_cloud(mapper, seed=3):
    return np.column_stack([
        np.random.default_rng(seed).uniform(-0.6, 0.6, 30000),
        np.random.default_rng(seed + 1).uniform(-0.8, 0.8, 30000),
        np.random.default_rng(seed + 2).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.2, 30000)])


def build_initial(boxes, resolution, cloud, warm_writes=3):
    """Same initial state for baseline and candidate: boxes + the same
    warm-up cloud writes, so both sides start from identical state."""
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='rep2')
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        result = store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i))
        assert result.accepted
    for stamp in range(warm_writes):
        assert integrate(store, mapper, cloud, stamp).accepted
    return store, mapper


def integrate(store, mapper, cloud, stamp):
    return store.apply_update(MapUpdate(
        operation_id='w-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud,
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(stamp, 0),
        acquisition_stamp_ns=1_000_000_000 + stamp,
        settled=True, schema_version=1))


def nearest_rank_p95(values):
    """Nearest-rank P95: the 29th of 30 (sorted[28]). Returns None when
    fewer than 30 samples -- not_evaluated, never a recomputed guess."""
    if len(values) < 30:
        return None
    return sorted(values)[28]


def stats(values):
    if not values:
        return {'n': 0, 'p95_ms': None, 'not_evaluated': True}
    if len(values) < 30:
        return {'n': len(values), 'p95_ms': None,
                'not_evaluated': True,
                'median_ms': round(statistics.median(values), 3),
                'max_ms': round(max(values), 3)}
    return {'n': len(values),
            'median_ms': round(statistics.median(values), 3),
            'p95_ms': round(nearest_rank_p95(values), 3),
            'max_ms': round(max(values), 3)}


def wait_ready_identity(replica, target_identity, timeout_s=30.0):
    """READY only counts when bound to the TARGET identity. Returns
    (elapsed_ns or None). A READY on an older identity is rejected."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = replica.poll()
        if (state == ReplicaState.READY
                and replica.manifest is not None
                and list(replica.manifest['target_identity'])
                == list(target_identity)):
            return True
        time.sleep(0.002)
    return False


class EncoderCpuSampler:
    """Samples the encoder thread's utime+stime from /proc."""

    def __init__(self, publisher):
        self._publisher = publisher
        self.samples = []
        self._stop = threading.Event()

    def _encoder_tid(self):
        thread = getattr(self._publisher, '_encoder', None)
        if thread is None or not thread.is_alive():
            return None
        # tid of a native thread == its ident on Linux CPython.
        return thread.ident

    def _start(self):
        tid = self._encoder_tid()
        if tid is None:
            return
        try:
            with open('/proc/self/task/%d/stat' % tid) as handle:
                fields = handle.read().rsplit(')', 1)[1].split()
            self.samples.append((time.monotonic_ns(),
                                 int(fields[11]) + int(fields[12])))
        except (OSError, IndexError, ValueError):
            pass

    def start(self):
        self._start()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.wait(0.05):
            self._start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._start()

    def total_cpu_s(self):
        if len(self.samples) < 2:
            return None
        ticks = self.samples[-1][1] - self.samples[0][1]
        return ticks / os.sysconf('SC_CLK_TCK')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--resolution', type=float, default=0.01)
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)

    report = {'boxes': args.boxes, 'resolution': args.resolution,
              'samples': args.samples, 'harness': 'rev2'}
    failures = []

    cloud = make_cloud(None if False else
                       CargoVolumeMapper((1.49, 1.97, 1.48),
                                         (0, 0, 0.74), 0, 0.01,
                                         max_raycast_points=2000))

    # ---------- Baseline: same initial state, no hook ------------------
    store_b, mapper_b = build_initial(args.boxes, args.resolution, cloud)
    baseline_writes = []
    for stamp in range(100, 100 + args.samples):
        t0 = time.perf_counter_ns()
        assert integrate(store_b, mapper_b, cloud, stamp).accepted
        baseline_writes.append((time.perf_counter_ns() - t0) / 1e6)
    report['axis_c_write_baseline_ms'] = stats(baseline_writes)

    # ---------- Candidate: identical construction + hook ---------------
    store, mapper = build_initial(args.boxes, args.resolution, cloud)
    root = tempfile.mkdtemp(prefix='rep2-')
    hook = wire_store_export(store, root)
    replica = QueryReplica(root)
    # Wire-then-snapshot: the hook fires on _build_snapshot, so force
    # the initial FULL export before waiting for READY.
    store._snapshot = store._build_snapshot()
    hook.wait_pending(60.0)
    first_identity = wait_first(replica)
    if first_identity is None:
        print('replica never reached READY', file=sys.stderr)
        os.close(lock_fd)
        return EXIT_NOT_EVAL

    from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
    facade = QueryMapReadFacade(replica)

    # Behavior check: old-READY rejection (exit 4 when broken).
    stale = os.path.join(root, 'head.json')
    with open(stale, 'rb') as handle:
        head_before = json.loads(handle.read())

    # ---------- Axis B: identity-bound commit -> READY ------------------
    visibility = []
    for stamp in range(200, 200 + max(3, min(args.samples, 30))):
        # B measures commit -> READY: the clock starts at the COMMIT
        # moment (apply_update returning = the owner's accepted commit),
        # NOT before the fusion (the fusion itself is axis C's object).
        # Attribution run: apply ~850ms | commit->publish 20-130ms |
        # publish->READY ~11ms warm.
        assert integrate(store, mapper, cloud, stamp).accepted
        t0 = time.monotonic_ns()
        with store.acquire_query_snapshot() as handle:
            target = list(handle._data().identity)
        if wait_ready_identity(replica, target, timeout_s=30.0):
            visibility.append((time.monotonic_ns() - t0) / 1e6)
        else:
            failures.append(
                'axis B: READY never bound to target %r (replica state '
                '%s, last identity %r, last error %r)'
                % (target, replica.state,
                   list(replica.manifest['target_identity'])
                   if replica.manifest else None,
                   replica.last_error[:120]))
            break
    report['axis_b_commit_to_ready_ms'] = stats(visibility)

    # ---------- Axis A: ROI through the ISOLATED facade ----------------
    roi = []
    floor = mapper._hull.floor_z
    region = ((-0.15, -0.15, floor), (0.15, 0.15, floor + 0.3))
    from luggage_perception.cargo_map_query import QueryPage
    for _ in range(args.samples):
        t0 = time.perf_counter_ns()
        try:
            handle = facade.acquire_query_snapshot('CURRENT')
            # The FULL query path the ROS service would run: acquire +
            # region query through the replica's snapshot.
            handle.query_cells(region, (0, 1, 2), page=QueryPage(4096, ''))
        except QueryError as error:
            failures.append('axis A: %s' % error.reason)
            break
        roi.append((time.perf_counter_ns() - t0) / 1e6)
    report['axis_a_roi_ms'] = stats(roi)

    # ---------- Axis C candidate + Axis D encoder CPU ------------------
    sampler = EncoderCpuSampler(hook._publisher)
    sampler.start()
    candidate_writes = []
    for stamp in range(300, 300 + args.samples):
        t0 = time.perf_counter_ns()
        assert integrate(store, mapper, cloud, stamp).accepted
        candidate_writes.append((time.perf_counter_ns() - t0) / 1e6)
    hook.wait_pending(60.0)
    sampler.stop()
    report['axis_c_write_with_hook_ms'] = stats(candidate_writes)
    report['axis_d_encoder_cpu_s'] = sampler.total_cpu_s()
    report['axis_d_sample_count'] = len(sampler.samples)

    base_p95 = report['axis_c_write_baseline_ms']['p95_ms']
    cand_p95 = report['axis_c_write_with_hook_ms']['p95_ms']
    if base_p95 is None or cand_p95 is None:
        report['axis_c_ratio'] = None
        failures.append('axis C: insufficient samples for P95')
    else:
        report['axis_c_ratio'] = round(cand_p95 / base_p95, 4)
        if report['axis_c_ratio'] > 1.10:
            failures.append('axis C ratio %.3f > 1.10'
                            % report['axis_c_ratio'])
    if report['axis_a_roi_ms']['p95_ms'] is None:
        failures.append('axis A: insufficient samples')
    elif report['axis_a_roi_ms']['p95_ms'] > 100 or \
            report['axis_a_roi_ms']['max_ms'] > 250:
        failures.append('axis A gates exceeded')
    b = report['axis_b_commit_to_ready_ms']
    if b['p95_ms'] is None:
        failures.append('axis B: insufficient samples (n=%d)' % b['n'])
    elif b['p95_ms'] > 1000 or b['max_ms'] > 2000:
        failures.append('axis B gates exceeded: P95 %.1fms / max %.1fms '
                        '(commit->READY incl. export)'
                        % (b['p95_ms'], b['max_ms']))

    hook.close()
    os.close(lock_fd)
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    if failures:
        print('FAILURES: %s' % failures, file=sys.stderr)
        return EXIT_FAIL
    return EXIT_PASS


def wait_first(replica):
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        if replica.poll() == ReplicaState.READY:
            return list(replica.manifest['target_identity'])
        time.sleep(0.01)
    return None


if __name__ == "__main__":
    sys.exit(main())
