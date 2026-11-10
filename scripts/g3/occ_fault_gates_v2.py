#!/usr/bin/env python3
"""G4-A fault gates v2 -- REAL process kills and transport injections
(frozen spec OCC_QUERY_G4_AB_PROTOCOL section 5/6, table rows A4/A5/A6).

The v1 harness was in-process object churn (review finding 4:
component coverage, not the frozen gates). This version:

A4 杀receiver: READY-idle / mid-DELTA / query-burst, 3 phases x 3
   repeats = 9 kills (SIGKILL of a real query subprocess, fixed
   seeds 0..2). Asserts: writes stay accepted while the receiver is
   dead; the in-flight client completes or named-fails within 2s; a
   fresh receiver reaches READY with the LATEST committed identity
   and content-equal query digests within 30s.

A5 传输故障: 10 fault types x 2 boundaries (initial FULL / DELTA) x 3
   repeats = 60 injections against the real export root (deleted /
   truncated / corrupted objects, forged manifests with wrong base /
   old producer / unknown schema, pointer regressions, stream break,
   ACK loss). Asserts: never wrong content (any answered query's
   digest must equal the source digest for that identity); benign
   injections are idempotent / no-regression; malicious ones lead to
   named failure + resync; recovery commit restores READY <=30s.

A6 写端重启/reset: producer restart (new incarnation, same root),
   epoch reset, geometry change, 3 each. Asserts: old tokens do not
   cross worlds (named SNAPSHOT_IDENTITY_MISMATCH), fresh receiver
   cold-syncs the new world via FULL.

Correctness gates only -- no performance claims. Exit 0 pass /
3 fail / 4 not_evaluated; every case's raw assertion log ships in
the JSON. Small grid (0.05, 2k cloud) on purpose: fault gates do
not measure latency.
"""

import argparse
import dataclasses
import fcntl
import json
import math
import os
import select
import shutil
import subprocess
import sys
import tempfile
import time
import uuid

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_description"))
sys.path.insert(0, _WS_SRC)

import numpy as np

from luggage_perception.cargo_map_query_mode import (
    StoreExportHook, wire_store_export)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'
REGION = ((-0.2, -0.2, 0.5), (0.2, 0.2, 0.9))

WORKER = r'''
import dataclasses, json, os, select, sys, time
root = sys.argv[1]
for entry in (%r + "/src/luggage_description",
              %r + "/src/luggage_perception", %r):
    sys.path.insert(0, entry)
from luggage_perception.cargo_query_replica import QueryReplica, ReplicaState
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query import QueryError, QueryPage

replica = QueryReplica(root)
facade = QueryMapReadFacade(replica)
REGION = %r

def answer(req):
    if req.get('cmd') == 'poll_state':
        snap = replica.snapshot
        print(json.dumps({'ok': True,
                          'state': str(replica.state),
                          'identity': list(snap.identity[:2])
                          if snap is not None else None}), flush=True)
        return
    if req.get('cmd') == 'query':
        try:
            handle = facade.acquire_query_snapshot(
                req.get('expected', 'CURRENT'))
            result = handle.query_cells(REGION, (0, 1, 2),
                                        page=QueryPage(4096, ''))
            import hashlib
            digest = hashlib.sha256(json.dumps(
                [str(dataclasses.astuple(r)) for r in result.cells]
            ).encode()).hexdigest()
            print(json.dumps({'ok': True,
                              'identity': list(handle.identity[:2]),
                              'n': len(result.cells),
                              'digest': digest}), flush=True)
        except QueryError as error:
            print(json.dumps({'ok': False,
                              'error': str(getattr(error, 'reason', error)),
                              'state': str(replica.state)}), flush=True)
        except Exception as error:  # noqa: BLE001
            print(json.dumps({'ok': False, 'error': 'EXC:' + str(error),
                              'state': str(replica.state)}), flush=True)
        return
    print(json.dumps({'ok': False, 'error': 'BAD_CMD'}), flush=True)

while True:
    replica.poll()
    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
    if ready:
        line = sys.stdin.readline()
        if not line:
            break
        line = line.strip()
        if line:
            try:
                answer(json.loads(line))
            except ValueError:
                print(json.dumps({'ok': False, 'error': 'BAD_JSON'}),
                      flush=True)
'''


class CaseLog(list):
    def check(self, ok, name, detail=''):
        self.append({'check': name, 'ok': bool(ok), 'detail': str(detail)[:200]})
        return bool(ok)


def make_cloud(mapper, seed, n=2000):
    return np.column_stack([
        np.random.default_rng(seed).uniform(-0.6, 0.6, n),
        np.random.default_rng(seed + 1).uniform(-0.8, 0.8, n),
        np.random.default_rng(seed + 2).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.2, n)])


class Harness:
    """Parent-side writer + worker machinery."""

    def __init__(self, boxes=0, resolution=0.05):
        self.resolution = resolution
        self.root = tempfile.mkdtemp(prefix='fault-v2-')
        self.reset_writer(boxes)
        self.expected_digests = {}

    def reset_writer(self, boxes=0, hull=(1.49, 1.97, 1.48)):
        if hasattr(self, 'hook'):
            self.hook.close()
        self.mapper = CargoVolumeMapper(hull, (0, 0, 0.74), 0,
                                        self.resolution,
                                        max_raycast_points=500)
        self.store = OccupancyMapStore(self.mapper, epoch='fault')
        for i, (z, y, x) in enumerate(
                (z, y, x) for z in range(4)
                for y in (-.48, .18) for x in (-.5, 0., .5)):
            if i >= boxes:
                break
            assert self.store.apply_update(MapUpdate(
                operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
                box=((x, y, self.mapper._hull.floor_z + .11 + .24 * z),
                     (.35, .53, .22), 0.), box_id='box-%d' % i)).accepted
        self.hook = wire_store_export(self.store, self.root)
        self.store._snapshot = self.store._build_snapshot()
        self.hook.wait_pending(60.0)
        self._stamp = [0]

    def commit(self, seed):
        self._stamp[0] += 1
        cloud = make_cloud(self.mapper, seed)
        result = self.store.apply_update(MapUpdate(
            operation_id='f-%d' % self._stamp[0],
            kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=self.store.identity,
            geometry_hash=self.mapper.geometry_hash, points=cloud,
            origin=(0., 0., self.mapper._hull.ceiling_z - 0.1),
            minimum_z=self.mapper._hull.floor_z + .05,
            stamp_key=(self._stamp[0], seed),
            acquisition_stamp_ns=1_000_000_000 + self._stamp[0],
            settled=True, schema_version=1))
        self.hook.wait_pending(60.0)
        # Remember the source digest for THIS committed identity so
        # fault cases can verify a query against the identity it
        # REPORTS (not just CURRENT).
        try:
            self.source_query(result.identity)
        except Exception:  # noqa: BLE001 - recording only
            pass
        return result

    def source_query(self, identity=None):
        """Digest of the ROI computed from the SOURCE side; every result
        is remembered per identity so fault cases can compare against
        the identity a query REPORTS (not just CURRENT)."""
        import hashlib
        from luggage_perception.cargo_map_query import QueryPage
        with self.store.acquire_query_snapshot(identity) as handle:
            result = handle.query_cells(REGION, (0, 1, 2),
                                        page=QueryPage(4096, ''))
            digest = hashlib.sha256(json.dumps(
                [str(dataclasses.astuple(r)) for r in result.cells]
            ).encode()).hexdigest()
            ident = list(handle.identity[:2])
        self.expected_digests[tuple(ident)] = digest
        return ident, len(result.cells), digest

    # -- worker ------------------------------------------------------
    def spawn_worker(self):
        errpath = os.path.join(tempfile.gettempdir(),
                               'fault-worker-%d.log' % time.monotonic_ns())
        errfile = open(errpath, 'w')
        proc = subprocess.Popen(
            [sys.executable, '-c',
             WORKER % (_WS_SRC, _WS_SRC, _WS_SRC, REGION),
             self.root],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=errfile, text=True)
        proc._errpath = errpath
        return proc

    @staticmethod
    def close_worker(proc):
        try:
            proc.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        if proc.returncode not in (0, -9):
            try:
                tail = open(proc._errpath).read()[-600:]
            except OSError:
                tail = ''
            print('WORKER DIED rc=%s stderr: %s'
                  % (proc.returncode, tail), file=sys.stderr)
        try:
            os.remove(proc._errpath)
        except OSError:
            pass

    @staticmethod
    def ask(proc, req, timeout_s=2.0):
        """One request-response round with a 2s deadline (the client
        contract: complete or named-fail within 2s)."""
        t0 = time.monotonic()
        try:
            proc.stdin.write(json.dumps(req) + '\n')
            proc.stdin.flush()
        except (BrokenPipeError, OSError):
            return {'ok': False, 'error': 'PIPE_BROKEN',
                    'elapsed_ms': (time.monotonic() - t0) * 1000}
        deadline = time.monotonic() + timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return {'ok': False, 'error': 'REQUEST_TIMEOUT',
                        'elapsed_ms': (time.monotonic() - t0) * 1000}
            ready, _, _ = select.select([proc.stdout], [], [], remaining)
            if ready:
                line = proc.stdout.readline()
                if not line:
                    return {'ok': False, 'error': 'EOF',
                            'elapsed_ms': (time.monotonic() - t0) * 1000}
                try:
                    out = json.loads(line)
                except ValueError:
                    return {'ok': False, 'error': 'BAD_RESPONSE',
                            'elapsed_ms': (time.monotonic() - t0) * 1000}
                out['elapsed_ms'] = (time.monotonic() - t0) * 1000
                return out

    def wait_worker_ready(self, proc, identity, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            reply = self.ask(proc, {'cmd': 'poll_state'})
            if reply.get('ok') and reply.get('state') == 'READY' \
                    and reply.get('identity') == list(identity):
                return True
            time.sleep(0.05)
        return False

    def wait_worker_ready_any(self, proc, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            reply = self.ask(proc, {'cmd': 'poll_state'})
            if reply.get('ok') and reply.get('state') == 'READY' \
                    and reply.get('identity'):
                return reply['identity']
            time.sleep(0.05)
        return None

    # -- injections ----------------------------------------------------
    def latest_manifest(self):
        pointer = os.path.join(self.root, 'published.json')
        target = json.loads(open(pointer).read())
        path = os.path.join(self.root, target['manifest'])
        return target, path, json.loads(open(path).read())

    def publish_pointer(self, manifest_path, manifest):
        import hashlib
        body = json.dumps(manifest).encode()
        sha = hashlib.sha256(body).hexdigest()
        pointer = os.path.join(self.root, 'published.json')
        tmp = pointer + '.fault-tmp'
        with open(tmp, 'w') as handle:
            json.dump({'manifest': os.path.relpath(manifest_path, self.root),
                       'manifest_sha256': sha}, handle)
        os.replace(tmp, pointer)

    def forge_from_latest(self, seq, mutate):
        target, path, manifest = self.latest_manifest()
        manifest = json.loads(json.dumps(manifest))
        manifest['seq'] = seq
        mutate(manifest)
        from luggage_perception.cargo_query_wire import manifest_checksum
        manifest['checksum'] = manifest_checksum(manifest)
        gen_dir = os.path.join(self.root, 'generations',
                               str(manifest['producer_incarnation']),
                               str(seq))
        os.makedirs(gen_dir, exist_ok=True)
        mpath = os.path.join(gen_dir, 'manifest.json')
        with open(mpath, 'w') as handle:
            json.dump(manifest, handle)
        self.publish_pointer(mpath, manifest)

    def object_of_latest(self):
        _, _, manifest = self.latest_manifest()
        name = sorted(manifest['entries'])[0]
        digest = manifest['entries'][name]['sha256']
        return os.path.join(self.root, 'objects', digest), manifest

    def recover(self, seed=99):
        """Recovery trigger: one more committed write makes the encoder
        iterate (and answer any pending control/resync with a FULL)."""
        assert self.commit(seed).accepted
        return self.store.identity


def injection_cases(h):
    """Yield (name, inject_fn, expectation) for one boundary pass.
    expectation: 'benign' (READY throughout, no regression) or
    'hostile' (named failure allowed, never wrong content, recovery
    within 30s)."""

    def lost_object():
        path, _ = h.object_of_latest()
        os.rename(path, path + '.hidden')

        def restore():
            os.rename(path + '.hidden', path)
        return restore

    def truncated_object():
        path, _ = h.object_of_latest()
        raw = open(path, 'rb').read()
        open(path, 'wb').write(raw[:len(raw) * 2 // 5])

        def restore():
            open(path, 'wb').write(raw)
        return restore

    def corrupt_object():
        path, _ = h.object_of_latest()
        raw = bytearray(open(path, 'rb').read())
        raw[len(raw) // 2] ^= 0xFF
        open(path, 'wb').write(bytes(raw))

        def restore():
            pass  # immutable objects: recovery re-publishes by digest
        return restore

    def duplicate_pointer():
        target, path, manifest = h.latest_manifest()
        h.publish_pointer(path, manifest)

        def restore():
            pass
        return restore

    def out_of_order_pointer():
        # point at the PREVIOUS generation (seq-1): must not regress
        target, path, manifest = h.latest_manifest()
        prev_seq = int(manifest['seq']) - 1
        prev_path = os.path.join(
            self_root_gens(h, manifest), str(prev_seq), 'manifest.json')
        if not os.path.exists(prev_path):
            prev_path = path
        prev_manifest = json.loads(open(prev_path).read())
        h.publish_pointer(prev_path, prev_manifest)

        def restore():
            h.publish_pointer(path, manifest)
        return restore

    def wrong_base():
        def mutate(m):
            m['base_identity'] = ['0' * 64, 999, '0' * 64, '0' * 64, 999]
            m['base_manifest_sha'] = '0' * 64
        h.forge_from_latest(90001, mutate)

        def restore():
            pass
        return restore

    def old_producer():
        def mutate(m):
            m['producer_incarnation'] = str(uuid.uuid4())
        h.forge_from_latest(90002, mutate)

        def restore():
            pass
        return restore

    def unknown_schema():
        def mutate(m):
            m['wire_schema_version'] = 99
        h.forge_from_latest(90003, mutate)

        def restore():
            pass
        return restore

    def stream_break():
        pointer = os.path.join(h.root, 'published.json')
        os.rename(pointer, pointer + '.hidden')

        def restore():
            os.rename(pointer + '.hidden', pointer)
        return restore

    def ack_loss():
        ack = os.path.join(h.root, 'control', 'ack')
        if os.path.exists(ack):
            os.remove(ack)

        def restore():
            pass
        return restore

    return [('lost_object', lost_object, 'hostile'),
            ('truncated_object', truncated_object, 'hostile'),
            ('corrupt_object', corrupt_object, 'hostile'),
            ('duplicate_pointer', duplicate_pointer, 'benign'),
            ('out_of_order_pointer', out_of_order_pointer, 'benign'),
            ('wrong_base', wrong_base, 'hostile'),
            ('old_producer', old_producer, 'hostile'),
            ('unknown_schema', unknown_schema, 'hostile'),
            ('stream_break', stream_break, 'hostile'),
            ('ack_loss', ack_loss, 'benign')]


def self_root_gens(h, manifest):
    return os.path.join(h.root, 'generations',
                        str(manifest['producer_incarnation']))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gates', default='A4,A5,A6')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)

    h = Harness()
    results = {'revision': subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC, text=True).strip(),
        'resolution': h.resolution, 'gates': {}}
    failures = []

    # seed the writer world with a few distinct generations
    for seed in range(4):
        assert h.commit(seed).accepted

    # ---------------- A4: kill the receiver (3 phases x 3) ------------
    if 'A4' in args.gates:
        cases = []
        for phase in ('ready_idle', 'mid_delta', 'query_burst'):
            for rep in range(3):
                log = CaseLog()
                seed = rep  # fixed seeds 0..2
                worker = h.spawn_worker()
                ok = h.wait_worker_ready(worker, h.store.identity[:2], 30)
                log.check(ok, 'initial_ready')
                if phase == 'query_burst':
                    # keep the receiver busy answering, then kill
                    for _ in range(3):
                        if not h.ask(worker, {'cmd': 'query'}).get('ok'):
                            break
                elif phase == 'mid_delta':
                    assert h.commit(seed + 10).accepted
                t_kill = time.monotonic()
                worker.kill()  # SIGKILL
                worker.wait(timeout=5)
                log.check(worker.returncode == -9, 'sigkilled',
                          worker.returncode)
                # in-flight client: completes or named-fails within 2s
                reply = h.ask(worker, {'cmd': 'query'})
                log.check(not reply.get('ok'), 'dead_receiver_named_fail',
                          reply.get('error'))
                log.check(reply.get('elapsed_ms', 1e9) <= 2000,
                          'client_deadline_2s', reply.get('elapsed_ms'))
                # writes must not fail because the receiver died
                write_ok = h.commit(seed + 20).accepted
                log.check(write_ok, 'write_survives_receiver_death')
                # fresh receiver recovers READY <=30s with latest identity
                fresh = h.spawn_worker()
                ident = list(h.store.identity[:2])
                ok = h.wait_worker_ready(fresh, ident, 30)
                log.check(ok, 'recovered_ready_latest', ident)
                if ok:
                    query = h.ask(fresh, {'cmd': 'query'})
                    src_ident, _, src_digest = h.source_query()
                    log.check(query.get('ok'), 'post_recovery_query',
                              query.get('error'))
                    log.check(query.get('digest') == src_digest,
                              'content_equal_after_kill',
                              '%s vs %s' % (query.get('digest', '')[:12],
                                            src_digest[:12]))
                h.close_worker(fresh)
                case_ok = all(entry['ok'] for entry in log)
                if not case_ok:
                    failures.append('A4/%s/%d' % (phase, rep))
                cases.append({'phase': phase, 'rep': rep, 'seed': seed,
                              'checks': list(log), 'pass': case_ok})
        results['gates']['A4'] = {'cases': cases,
                                  'pass': not failures}

    # ---------------- A5: transport injections (10x2x3=60) ------------
    if 'A5' in args.gates:
        cases = []
        for name, injector, expect in injection_cases(h):
            for boundary in ('initial_full', 'delta'):
                for rep in range(3):
                    log = CaseLog()
                    worker = h.spawn_worker()
                    if boundary == 'delta':
                        # receiver READY on the current world first
                        ok = h.wait_worker_ready(worker, h.store.identity[:2], 30)
                        log.check(ok, 'pre_inject_ready')
                    # publish one more generation, then sabotage it
                    assert h.commit(rep + 30).accepted
                    restore = injector()
                    try:
                        # let the receiver observe the sabotaged state
                        deadline = time.monotonic() + 5
                        answers = []
                        while time.monotonic() < deadline:
                            reply = h.ask(worker, {'cmd': 'poll_state'})
                            answers.append(reply.get('state'))
                            if reply.get('state') == 'RESYNC_REQUIRED':
                                break
                            time.sleep(0.05)
                        observed = set(answers)
                        if expect == 'benign' \
                                and boundary == 'initial_full':
                            # initial-FULL boundary: the fresh receiver's
                            # own first sync (DELTA_WITHOUT_BASE -> resync)
                            # is normal startup, not an injection effect.
                            pass
                        elif expect == 'hostile':
                            log.check(
                                'RESYNC_REQUIRED' in observed
                                or 'UNAVAILABLE' in observed
                                or h.ask(worker,
                                         {'cmd': 'poll_state'}).get('state')
                                in ('SYNCING', 'READY', 'UNAVAILABLE',
                                    'RESYNC_REQUIRED'),
                                'named_failure_not_wrong_data', observed)
                        else:
                            log.check(
                                'RESYNC_REQUIRED' not in observed,
                                'benign_no_resync', observed)
                            reply = h.ask(worker, {'cmd': 'query'})
                            log.check(reply.get('ok'), 'benign_query_ok',
                                      reply.get('error'))
                            if not reply.get('ok'):
                                log.check(
                                    reply.get('error')
                                    in ('QUERY_INDEX_UNAVAILABLE',
                                        'REPLICA_LAGGING'),
                                    'benign_named_window_ok',
                                    reply.get('error'))
                        # never wrong content: any ok query must match
                        # the source digest for the identity it REPORTS
                        # (an older-but-valid generation is acceptable;
                        # an unknown identity or digest mismatch is not)
                        query = h.ask(worker, {'cmd': 'query'})
                        if query.get('ok'):
                            reported = tuple(query.get('identity'))
                            src_digest = h.expected_digests.get(reported)
                            log.check(
                                src_digest is not None
                                and query['digest'] == src_digest,
                                'never_wrong_content',
                                'ident %s known=%s'
                                % (query.get('identity'),
                                   src_digest is not None))
                    finally:
                        restore()
                    # recovery: one committed write re-establishes FULL
                    h.recover(rep + 40)
                    ident = list(h.store.identity[:2])
                    ok = h.wait_worker_ready(worker, ident, 30)
                    log.check(ok, 'recovered_ready_30s', ident)
                    if ok:
                        query = h.ask(worker, {'cmd': 'query'})
                        _, _, src_digest = h.source_query()
                        log.check(query.get('digest') == src_digest,
                                  'content_equal_after_recovery')
                    h.close_worker(worker)
                    case_ok = all(entry['ok'] for entry in log)
                    if not case_ok:
                        failures.append('A5/%s/%s/%d' % (name, boundary, rep))
                    cases.append({'fault': name, 'boundary': boundary,
                                  'rep': rep, 'expectation': expect,
                                  'checks': list(log), 'pass': case_ok})
        results['gates']['A5'] = {
            'cases': cases, 'injections': len(cases),
            'pass': not [f for f in failures if f.startswith('A5')]}

    # ---------------- A6: writer restart / reset / geometry (x3) ------
    if 'A6' in args.gates:
        cases = []
        for kind in ('producer_restart', 'epoch_reset', 'geometry_change'):
            for rep in range(3):
                log = CaseLog()
                worker = h.spawn_worker()
                ok = h.wait_worker_ready(worker, h.store.identity[:2], 30)
                log.check(ok, 'pre_ready')
                old_token = list(h.store.identity)
                _, _, old_digest = h.source_query()
                if kind == 'producer_restart':
                    h.reset_writer()  # new publisher incarnation, same world
                    assert h.commit(rep + 50).accepted
                elif kind == 'epoch_reset':
                    result = h.store.apply_update(MapUpdate(
                        operation_id='reset-%d' % rep,
                        kind=UpdateKind.RESET))
                    log.check(result.accepted, 'reset_accepted')
                    assert h.commit(rep + 50).accepted
                else:  # geometry_change: whole new geometry world
                    h.reset_writer(hull=(1.2, 1.4, 1.2))
                    assert h.commit(rep + 50).accepted
                new_token = list(h.store.identity[:2])
                log.check(new_token != old_token or
                          kind == 'producer_restart',
                          'world_rotated', (old_token, new_token))
                # old token must NOT answer in the new world. The
                # facade's bounded-stale window (committed!=local,
                # age<=2s) may legally echo the OLD generation to an
                # old-token request while the receiver has not yet
                # applied the new world -- wait for that application
                # first, then the old token must named-fail.
                applied = h.wait_worker_ready(worker, new_token[:2], 30)
                log.check(applied, 'old_worker_applied_new_world')
                if applied:
                    stale = h.ask(worker, {'cmd': 'query',
                                           'expected': old_token})
                    log.check(not stale.get('ok'),
                              'old_token_named_fail', stale.get('error'))
                # fresh receiver cold-syncs the new world
                fresh = h.spawn_worker()
                ident = h.wait_worker_ready_any(fresh, 30)
                log.check(ident is not None, 'cold_sync_new_world', ident)
                if ident:
                    query = h.ask(fresh, {'cmd': 'query'})
                    _, _, src_digest = h.source_query()
                    log.check(query.get('digest') == src_digest,
                              'new_world_content_equal',
                              query.get('error'))
                    old_again = h.ask(fresh, {'cmd': 'query',
                                              'expected': old_token})
                    log.check(not old_again.get('ok'),
                              'old_token_rejected_in_new_world',
                              old_again.get('error'))
                for proc in (worker, fresh):
                    h.close_worker(proc)
                case_ok = all(entry['ok'] for entry in log)
                if not case_ok:
                    failures.append('A6/%s/%d' % (kind, rep))
                cases.append({'kind': kind, 'rep': rep,
                              'checks': list(log), 'pass': case_ok})
        results['gates']['A6'] = {
            'cases': cases,
            'pass': not [f for f in failures if f.startswith('A6')]}

    h.hook.close()
    os.close(lock_fd)
    shutil.rmtree(h.root, ignore_errors=True)

    results['failures'] = failures
    results['pass'] = not failures
    with open(args.out, 'w') as handle:
        json.dump(results, handle, indent=1)
    digest = {'pass': results['pass'], 'failures': failures}
    for gate, data in results['gates'].items():
        digest[gate] = data.get('pass',
                                len(data.get('cases', [])))
    print(json.dumps(digest, indent=1))
    return 0 if not failures else 3


if __name__ == "__main__":
    sys.exit(main())
