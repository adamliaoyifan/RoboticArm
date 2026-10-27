#!/usr/bin/env python3
"""G4-A gen3 fault gates v3 (plan d87c5731 section 6.3, review_8a372dee
fault-gate findings).

What v2 could not prove, v3 must:

A4 (3 phases x seeds 0-2, REAL SIGKILL): every phase is confirmed by
an explicit stage marker BEFORE the kill --
  ready_idle    receiver READY with no pending work (poll + query ok)
  mid_apply     receiver is INSIDE generation application (a marker
                file proves _apply/_load was entered and had not
                returned) -- no longer a guess from commit timing
  query_in_flight  a real query has STARTED (marker written at entry)
                and had NOT returned when SIGKILL landed
Recovery: fresh receiver READY<=30s with FULL-SNAPSHOT fingerprint
equality (wire-encoding digest over every block/summary/surface/
identity -- not one ROI digest).

A5 (10 fault types x initial-FULL/DELTA x 3 = 60 + the same-seq
forgery required by the plan = 66 injections): each case records the
fault id, target seq and object digest; DETECTION must be proven by
receiver events (validate_failed / resync / unavailable) that occurred
AFTER the injection -- a later READY alone never counts as detection;
benign injections must show NO new detection events (beyond the
pre-injection startup window) and stay queryable; recovery is a
detection->ready event chain with fingerprint equality.

A6: producer_restart is a REAL writer subprocess crash + a NEW writer
process on the same root (new incarnation); old-token rejection
compares FULL 5-leg identities; cold FULL is fingerprint-checked.

Exit 0 pass / 3 fail / 4 not_evaluated; every case ships its raw
check log; untriggered barrier attempts are recorded and retried (max
5) -- a case that never triggers its barrier is not_evaluated, not
pass.
"""

import argparse
import dataclasses
import fcntl
import hashlib
import json
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

from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'
REGION = ((-0.2, -0.2, 0.5), (0.2, 0.2, 0.9))

FINGERPRINT = r'''
def fingerprint(snapshot):
    """Digest over the COMPLETE wire encoding of a snapshot: every
    block object (evidence/frontier/members), summary, surface,
    identity -- the same bytes the transport itself moves."""
    from luggage_perception.cargo_query_snapshot_wire import encode_snapshot
    from luggage_perception.cargo_query_wire import sha256_hex
    payloads = encode_snapshot(snapshot, 'fingerprint', 0,
                               dirty_blocks=None,
                               provenance_changed=True)
    table = {name: sha256_hex(payload) for name, payload
             in sorted(payloads.items())}
    return hashlib.sha256(json.dumps(table, sort_keys=True)
                          .encode()).hexdigest()
'''

WORKER = r'''
import hashlib, json, os, select, sys, time, threading
root, marker_dir = sys.argv[1], sys.argv[2]
for entry in (%r + "/src/luggage_description",
              %r + "/src/luggage_perception", %r):
    sys.path.insert(0, entry)
from luggage_perception.cargo_query_replica import QueryReplica, ReplicaState
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query import QueryError, QueryPage
import dataclasses

''' + FINGERPRINT + r'''

replica = QueryReplica(root)
facade = QueryMapReadFacade(replica)
REGION = %r
events_cursor = 0


def marker(name):
    with open(os.path.join(marker_dir, name), 'w') as handle:
        handle.write(str(time.monotonic_ns()))


def query_once(expected='CURRENT'):
    handle = facade.acquire_query_snapshot(expected)
    result = handle.query_cells(REGION, (0, 1, 2),
                                page=QueryPage(4096, ''))
    digest = hashlib.sha256(json.dumps(
        [str(dataclasses.astuple(r)) for r in result.cells]
    ).encode()).hexdigest()
    identity = list(handle.identity)
    handle.close()
    return {'ok': True, 'identity': identity, 'n': len(result.cells),
            'digest': digest}


def answer(req):
    if req.get('cmd') == 'hold':
        global holding
        holding = True
        print(json.dumps({'ok': True}), flush=True)
        return
    if req.get('cmd') == 'resume':
        holding = False
        print(json.dumps({'ok': True}), flush=True)
        return
    if req.get('cmd') == 'poll_state':
        snap = replica.snapshot
        print(json.dumps({'ok': True, 'state': str(replica.state),
                          'identity': list(snap.identity)
                          if snap is not None else None}), flush=True)
        return
    if req.get('cmd') == 'events':
        global events_cursor
        events = replica.events()
        fresh = [dict(e) for e in list(events)[events_cursor:]]
        events_cursor = len(events)
        print(json.dumps({'ok': True, 'events': fresh}), flush=True)
        return
    if req.get('cmd') == 'query':
        try:
            out = query_once(req.get('expected', 'CURRENT'))
        except QueryError as error:
            out = {'ok': False,
                   'error': str(getattr(error, 'reason', error)),
                   'state': str(replica.state)}
        except Exception as error:  # noqa: BLE001
            out = {'ok': False, 'error': 'EXC:' + str(error),
                   'state': str(replica.state)}
        print(json.dumps(out), flush=True)
        return
    if req.get('cmd') == 'fingerprint':
        snap = replica.snapshot
        if snap is None:
            print(json.dumps({'ok': False, 'error': 'NO_SNAPSHOT'}),
                  flush=True)
        else:
            from luggage_perception.cargo_query_snapshot_wire import (
                encode_snapshot)
            from luggage_perception.cargo_query_wire import sha256_hex
            payloads = encode_snapshot(snap, 'fingerprint', 0,
                                       dirty_blocks=None,
                                       provenance_changed=True)
            table = {name: sha256_hex(payload)[:12]
                     for name, payload in sorted(payloads.items())}
            print(json.dumps({'ok': True,
                              'fingerprint': fingerprint(snap),
                              'identity': list(snap.identity),
                              'table': table}),
                  flush=True)
        return
    if req.get('cmd') == 'query_in_flight':
        # Start a REAL query, publish the entry marker, and hold the
        # answer until 'collect' -- SIGKILL while the marker exists
        # proves the request was in flight at death.
        def run():
            try:
                out = query_once()
            except Exception as error:  # noqa: BLE001
                out = {'ok': False, 'error': str(error)}
            answer_box.append(out)
            marker('query_answered')
        answer_box = []
        marker('query_started')
        threading.Thread(target=run, daemon=True).start()
        print(json.dumps({'ok': True}), flush=True)
        return
    if req.get('cmd') == 'arm_mid_apply':
        # Patch the NEXT generation application to publish a marker at
        # entry and block: SIGKILL with the marker present and no
        # 'applied' marker proves mid-apply death.
        original_delta = replica._apply_delta
        original_load = replica._load_objects_all

        def marked_delta(manifest):
            marker('apply_entered')
            import threading as _t
            block = _t.Event()
            replica._apply_release = block
            marker('apply_waiting')
            block.wait(10)
            try:
                return original_delta(manifest)
            finally:
                marker('applied')

        def marked_load(manifest):
            marker('apply_entered')
            import threading as _t
            block = _t.Event()
            replica._apply_release = block
            marker('apply_waiting')
            block.wait(10)
            try:
                return original_load(manifest)
            finally:
                marker('applied')

        replica._apply_delta = marked_delta
        replica._load_objects_all = marked_load
        print(json.dumps({'ok': True}), flush=True)
        return
    if req.get('cmd') == 'release_apply':
        block = getattr(replica, '_apply_release', None)
        if block is not None:
            block.set()
        print(json.dumps({'ok': True}), flush=True)
        return
    print(json.dumps({'ok': False, 'error': 'BAD_CMD'}), flush=True)


holding = False
while True:
    if not holding:
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

WRITER_PROC = r'''
import hashlib, json, os, select, sys, time
root, mode, seed_arg = sys.argv[1], sys.argv[2], int(sys.argv[3])
for entry in (%r + "/src/luggage_description",
              %r + "/src/luggage_perception", %r):
    sys.path.insert(0, entry)
import numpy as np
from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)

''' + FINGERPRINT + r'''

hull = (1.49, 1.97, 1.48) if mode != 'geometry' else (1.2, 1.4, 1.2)
mapper = CargoVolumeMapper(hull, (0, 0, 0.74), 0, 0.05,
                           max_raycast_points=500)
epoch = ('fault-w' + str(seed_arg)) if mode == 'restart' else 'fault'
store = OccupancyMapStore(mapper, epoch=epoch)
hook = wire_store_export(store, root)
store._snapshot = store._build_snapshot()
hook.wait_pending(60.0)
stamp = [0]


def cloud(seed, n=2000):
    return np.column_stack([
        np.random.default_rng(seed).uniform(-0.6, 0.6, n),
        np.random.default_rng(seed + 1).uniform(-0.8, 0.8, n),
        np.random.default_rng(seed + 2).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.2, n)])


def commit(seed):
    stamp[0] += 1
    return store.apply_update(MapUpdate(
        operation_id='w-' + str(stamp[0]),
        kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=cloud(seed),
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(stamp[0], seed),
        acquisition_stamp_ns=1_000_000_000 + stamp[0],
        settled=True, schema_version=1))


for seed in range(3):
    assert commit(seed).accepted
hook.wait_pending(60.0)

while True:
    ready, _, _ = select.select([sys.stdin], [], [], 0.05)
    if not ready:
        continue
    line = sys.stdin.readline()
    if not line:
        break
    req = json.loads(line.strip())
    if req.get('cmd') == 'commit':
        result = commit(req.get('seed', stamp[0] + 1))
        hook.wait_pending(60.0)
        print(json.dumps({'ok': result.accepted,
                          'identity': list(result.identity)}), flush=True)
    elif req.get('cmd') == 'identity':
        with store.acquire_query_snapshot() as handle:
            print(json.dumps({'ok': True,
                              'identity': list(handle.identity)}),
                  flush=True)
    elif req.get('cmd') == 'fingerprint':
        with store.acquire_query_snapshot() as handle:
            print(json.dumps({'ok': True,
                              'fingerprint': fingerprint(handle._data()),
                              'identity': list(handle.identity)}),
                  flush=True)
    elif req.get('cmd') == 'shutdown':
        hook.close()
        print(json.dumps({'ok': True}), flush=True)
        break
    else:
        print(json.dumps({'ok': False, 'error': 'BAD_CMD'}), flush=True)
'''


class CaseLog(list):
    def check(self, ok, name, detail=''):
        self.append({'check': name, 'ok': bool(ok),
                     'detail': str(detail)[:300]})
        return bool(ok)


def make_cloud(mapper, seed, n=2000):
    return np.column_stack([
        np.random.default_rng(seed).uniform(-0.6, 0.6, n),
        np.random.default_rng(seed + 1).uniform(-0.8, 0.8, n),
        np.random.default_rng(seed + 2).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.2, n)])


def fingerprint_source(snapshot):
    from luggage_perception.cargo_query_snapshot_wire import encode_snapshot
    payloads = encode_snapshot(snapshot, 'fingerprint', 0,
                               dirty_blocks=None, provenance_changed=True)
    table = {name: hashlib.sha256(payload).hexdigest()
             for name, payload in sorted(payloads.items())}
    return hashlib.sha256(json.dumps(table, sort_keys=True)
                          .encode()).hexdigest()


def fingerprint_table(h):
    from luggage_perception.cargo_query_snapshot_wire import encode_snapshot
    from luggage_perception.cargo_query_wire import sha256_hex
    with h.store.acquire_query_snapshot() as handle:
        payloads = encode_snapshot(handle._data(), 'fingerprint', 0,
                                   dirty_blocks=None,
                                   provenance_changed=True)
    return {name: sha256_hex(payload)[:12]
            for name, payload in sorted(payloads.items())}


def ask(proc, req, timeout_s=2.0):
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


class Harness:
    def __init__(self, resolution=0.05):
        self.resolution = resolution
        self.root = tempfile.mkdtemp(prefix='fault-v3-')
        self.marker_dir = tempfile.mkdtemp(prefix='fault-v3-markers-')
        self.expected_digests = {}
        self.mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74),
                                        0, resolution, max_raycast_points=500)
        self.store = OccupancyMapStore(self.mapper, epoch='fault')
        self.hook = wire_store_export(self.store, self.root)
        self.store._snapshot = self.store._build_snapshot()
        self.hook.wait_pending(60.0)
        self._stamp = 0

    def commit(self, seed):
        self._stamp += 1
        result = self.store.apply_update(MapUpdate(
            operation_id='f-%d' % self._stamp,
            kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=self.store.identity,
            geometry_hash=self.mapper.geometry_hash,
            points=make_cloud(self.mapper, seed),
            origin=(0., 0., self.mapper._hull.ceiling_z - 0.1),
            minimum_z=self.mapper._hull.floor_z + .05,
            stamp_key=(self._stamp, seed),
            acquisition_stamp_ns=1_000_000_000 + self._stamp,
            settled=True, schema_version=1))
        self.hook.wait_pending(60.0)
        try:
            self.source_query(result.identity)
        except Exception:  # noqa: BLE001 - recording only
            pass
        return result

    def source_query(self, identity=None):
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

    def source_fingerprint(self):
        with self.store.acquire_query_snapshot() as handle:
            return (fingerprint_source(handle._data()),
                    list(handle.identity))

    def spawn_worker(self, root=None):
        errpath = os.path.join(tempfile.gettempdir(),
                               'fault-v3-%d.log' % time.monotonic_ns())
        errfile = open(errpath, 'w')
        proc = subprocess.Popen(
            [sys.executable, '-c',
             WORKER % (_WS_SRC, _WS_SRC, _WS_SRC, REGION),
             root or self.root, self.marker_dir],
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
        try:
            os.remove(proc._errpath)
        except OSError:
            pass

    def wait_ready(self, proc, identity2, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            reply = ask(proc, {'cmd': 'poll_state'})
            if (reply.get('ok') and reply.get('state') == 'READY'
                    and (reply.get('identity') or [])[:2]
                    == list(identity2)):
                return True
            time.sleep(0.05)
        return False

    def wait_ready_any(self, proc, timeout_s=30.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            reply = ask(proc, {'cmd': 'poll_state'})
            if reply.get('ok') and reply.get('state') == 'READY' \
                    and reply.get('identity'):
                return reply['identity']
            time.sleep(0.05)
        return None

    # -- marker barriers -----------------------------------------------

    def marker_path(self, name):
        return os.path.join(self.marker_dir, name)

    def wait_marker(self, name, timeout_s=5.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if os.path.exists(self.marker_path(name)):
                return True
            time.sleep(0.005)
        return False

    def clear_markers(self):
        for name in os.listdir(self.marker_dir):
            os.remove(self.marker_path(name))

    # -- injections ----------------------------------------------------

    def latest_manifest(self):
        pointer = os.path.join(self.root, 'published.json')
        target = json.loads(open(pointer).read())
        path = os.path.join(self.root, target['manifest'])
        return target, path, json.loads(open(path).read())

    def publish_pointer(self, manifest_path, manifest):
        # The REAL publisher writes canonical bytes; the pointer hash
        # must match them or every re-published pointer reads as
        # POINTER_MISMATCH (a harness bug v3's stricter checks exposed).
        from luggage_perception.cargo_query_wire import (
            canonical_json_bytes, sha256_hex)
        body = canonical_json_bytes(manifest)
        sha = sha256_hex(body)
        pointer = os.path.join(self.root, 'published.json')
        tmp = pointer + '.fault-tmp'
        with open(tmp, 'wb') as handle:
            handle.write(canonical_json_bytes(
                {'manifest': os.path.relpath(manifest_path, self.root),
                 'manifest_sha256': sha}))
        os.replace(tmp, pointer)

    def forge_from_latest(self, seq, mutate):
        _, path, manifest = self.latest_manifest()
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
        from luggage_perception.cargo_query_wire import canonical_json_bytes
        with open(mpath, 'wb') as handle:
            handle.write(canonical_json_bytes(manifest))
        self.publish_pointer(mpath, manifest)

    def object_of_latest(self):
        """Target an object the receiver MUST read: an entry EMITTED by
        the latest generation whose digest is NEW (a digest already
        present in an earlier manifest is either cache-hit for DELTA
        receivers or rewritten by any FULL re-publish -- a no-op
        injection v2 could not distinguish from detection)."""
        _, _, manifest = self.latest_manifest()
        previous_digests = set()
        gens = os.path.join(self.root, 'generations',
                            str(manifest['producer_incarnation']))
        if os.path.isdir(gens):
            for seq_dir in sorted(os.listdir(gens)):
                if seq_dir.isdigit() and int(seq_dir) >= int(
                        manifest['seq']):
                    continue
                try:
                    with open(os.path.join(gens, seq_dir,
                                           'manifest.json')) as fh:
                        prev = json.load(fh)
                except (OSError, ValueError):
                    continue
                previous_digests |= {entry['sha256']
                                     for entry in prev['entries'].values()}
        candidates = [
            name for name in sorted(manifest['entries'])
            if manifest['entries'][name].get('type') == 'object'
            and manifest['entries'][name]['sha256'] not in previous_digests]
        if not candidates:
            candidates = sorted(
                name for name, entry in manifest['entries'].items()
                if entry.get('type') == 'object') \
                or sorted(manifest['entries'])
        name = candidates[0]
        digest = manifest['entries'][name]['sha256']
        return (os.path.join(self.root, 'objects', digest), digest,
                manifest['seq'], name)

    def recover(self, seed):
        assert self.commit(seed).accepted
        return self.store.identity


def drain_events(proc):
    reply = ask(proc, {'cmd': 'events'}, timeout_s=5.0)
    return reply.get('events', []) if reply.get('ok') else []


DETECTION_EVENTS = ('resync', 'unavailable', 'validate_failed',
                    'no_published')


def _startup_resync(event, boundary, expect='benign'):
    """Fresh-receiver startup resync semantics (initial-FULL boundary):

    - benign: a DELTA_WITHOUT_BASE FULL request is NORMAL startup (the
      current published generation may legitimately be a DELTA) -- not
      an injection detection;
    - hostile: refusing the published stream IS the detection -- the
      fresh receiver has no base to compare against, so its named
      rejection (resync/unavailable of ANY reason) after the injection
      is the strongest detection available at this boundary."""
    return (boundary == 'initial_full'
            and expect == 'benign'
            and event.get('event') == 'resync'
            and event.get('reason') == 'DELTA_WITHOUT_BASE'
            and event.get('seq') is None)


def injection_cases(h):
    """(name, inject_fn, expectation, forensics_fn) per case; the
    forensics fn returns a dict recording exactly WHAT was attacked
    (target seq / object digest) for the case log."""

    def lost_object():
        path, digest, seq, name = h.object_of_latest()

        def restore():
            os.rename(path + '.hidden', path)
        os.rename(path, path + '.hidden')
        return restore, {'target_seq': seq, 'object': digest,
                         'entry': name}

    def truncated_object():
        path, digest, seq, name = h.object_of_latest()
        raw = open(path, 'rb').read()
        open(path, 'wb').write(raw[:len(raw) * 2 // 5])

        def restore():
            open(path, 'wb').write(raw)
        return restore, {'target_seq': seq, 'object': digest}

    def corrupt_object():
        path, digest, seq, name = h.object_of_latest()
        raw = bytearray(open(path, 'rb').read())
        raw[len(raw) // 2] ^= 0xFF
        open(path, 'wb').write(bytes(raw))

        def restore():
            pass
        return restore, {'target_seq': seq, 'object': digest}

    def duplicate_pointer():
        target, path, manifest = h.latest_manifest()
        h.publish_pointer(path, manifest)

        def restore():
            pass
        return restore, {'target_seq': manifest['seq']}

    def out_of_order_pointer():
        target, path, manifest = h.latest_manifest()
        prev_seq = int(manifest['seq']) - 1
        prev_path = os.path.join(
            h.root, 'generations', str(manifest['producer_incarnation']),
            str(prev_seq), 'manifest.json')
        if not os.path.exists(prev_path):
            prev_path = path
        prev_manifest = json.loads(open(prev_path).read())
        h.publish_pointer(prev_path, prev_manifest)

        def restore():
            h.publish_pointer(path, manifest)
        return restore, {'target_seq': manifest['seq'],
                         'regressed_to': prev_seq}

    def wrong_base():
        def mutate(m):
            m['base_identity'] = ['0' * 64, 999, '0' * 64, '0' * 64, 999]
            m['base_manifest_sha'] = '0' * 64
        h.forge_from_latest(90001, mutate)

        def restore():
            pass
        return restore, {'forged_seq': 90001}

    def old_producer():
        def mutate(m):
            m['producer_incarnation'] = str(uuid.uuid4())
        h.forge_from_latest(90002, mutate)

        def restore():
            pass
        return restore, {'forged_seq': 90002}

    def unknown_schema():
        def mutate(m):
            m['wire_schema_version'] = 99
        h.forge_from_latest(90003, mutate)

        def restore():
            pass
        return restore, {'forged_seq': 90003}

    def stream_break():
        pointer = os.path.join(h.root, 'published.json')
        os.rename(pointer, pointer + '.hidden')

        def restore():
            os.rename(pointer + '.hidden', pointer)
        return restore, {}

    def ack_loss():
        ack = os.path.join(h.root, 'control', 'ack')
        if os.path.exists(ack):
            os.remove(ack)

        def restore():
            pass
        return restore, {}

    def same_seq_forged_content():
        """Malicious SAME-seq manifest with different content (plan
        6.3: must be rejected, never adopted)."""
        def mutate(m):
            name = sorted(m['entries'])[0]
            m['entries'][name]['sha256'] = 'f' * 64
        _, _, manifest = h.latest_manifest()
        h.forge_from_latest(int(manifest['seq']), mutate)

        def restore():
            pass
        return restore, {'forged_seq': manifest['seq'], 'same_seq': True}

    return [('lost_object', lost_object, 'hostile'),
            ('truncated_object', truncated_object, 'hostile'),
            ('corrupt_object', corrupt_object, 'hostile'),
            ('duplicate_pointer', duplicate_pointer, 'benign'),
            ('out_of_order_pointer', out_of_order_pointer, 'benign'),
            ('wrong_base', wrong_base, 'hostile'),
            ('old_producer', old_producer, 'hostile'),
            ('unknown_schema', unknown_schema, 'hostile'),
            ('stream_break', stream_break, 'hostile'),
            ('ack_loss', ack_loss, 'benign'),
            ('same_seq_forged_content', same_seq_forged_content,
             'hostile')]


def run_a4(h, results):
    cases = []
    for phase in ('ready_idle', 'mid_apply', 'query_in_flight'):
        for rep in range(3):
            log = CaseLog()
            seed = rep
            triggered = True
            h.clear_markers()
            worker = h.spawn_worker()
            ok = h.wait_ready(worker, h.store.identity[:2], 30)
            log.check(ok, 'initial_ready')
            # drain pre-existing events as the baseline
            drain_events(worker)
            if phase == 'ready_idle':
                reply = ask(worker, {'cmd': 'query'})
                log.check(reply.get('ok'), 'idle_query_ok',
                          reply.get('error'))
            elif phase == 'mid_apply':
                ask(worker, {'cmd': 'arm_mid_apply'})
                assert h.commit(seed + 10).accepted
                triggered = (h.wait_marker('apply_entered', 5)
                             and h.wait_marker('apply_waiting', 2))
                log.check(triggered, 'mid_apply_barrier_confirmed',
                          os.listdir(h.marker_dir))
            else:  # query_in_flight
                ask(worker, {'cmd': 'query_in_flight'})
                triggered = (h.wait_marker('query_started', 5)
                             and not os.path.exists(
                                 h.marker_path('query_answered')))
                log.check(triggered,
                          'in_flight_query_confirmed',
                          os.listdir(h.marker_dir))
            if not triggered:
                h.close_worker(worker)
                cases.append({'phase': phase, 'rep': rep, 'seed': seed,
                              'checks': list(log), 'pass': False,
                              'result': 'not_evaluated'})
                continue
            worker.kill()                      # the REAL SIGKILL
            worker.wait(timeout=5)
            log.check(worker.returncode == -9, 'sigkilled',
                      worker.returncode)
            if phase == 'query_in_flight':
                log.check(not os.path.exists(
                    h.marker_path('query_answered')),
                    'query_unanswered_at_death')
            # dead receiver: the client must complete or named-fail <=2s
            reply = ask(worker, {'cmd': 'query'})
            log.check(not reply.get('ok'), 'dead_receiver_named_fail',
                      reply.get('error'))
            log.check(reply.get('elapsed_ms', 1e9) <= 2000,
                      'client_deadline_2s', reply.get('elapsed_ms'))
            log.check(h.commit(seed + 20).accepted,
                      'write_survives_receiver_death')
            fresh = h.spawn_worker()
            ident2 = list(h.store.identity[:2])
            ok = h.wait_ready(fresh, ident2, 30)
            log.check(ok, 'recovered_ready_latest', ident2)
            if ok:
                fp = ask(fresh, {'cmd': 'fingerprint'}, timeout_s=10.0)
                src_fp, _ = h.source_fingerprint()
                log.check(fp.get('ok') and
                          fp.get('fingerprint') == src_fp,
                          'full_snapshot_equal_after_kill',
                          '%s vs %s' % (str(fp.get('fingerprint'))[:12],
                                        src_fp[:12]))
            h.close_worker(fresh)
            h.close_worker(worker)
            case_ok = all(entry['ok'] for entry in log)
            cases.append({'phase': phase, 'rep': rep, 'seed': seed,
                          'checks': list(log), 'pass': case_ok})
    passed = [c for c in cases if c.get('result') != 'not_evaluated']
    results['A4'] = {
        'cases': cases,
        'not_evaluated': len(cases) - len(passed),
        'pass': bool(passed) and all(c['pass'] for c in passed)}


def run_a5(h, results):
    cases = []
    for name, injector, expect in injection_cases(h):
        for boundary in ('initial_full', 'delta'):
            for rep in range(3):
                log = CaseLog()
                fault_id = '%s/%s/%d' % (name, boundary, rep)
                worker = h.spawn_worker()
                if boundary == 'delta':
                    ok = h.wait_ready(worker, h.store.identity[:2], 30)
                    log.check(ok, 'pre_inject_ready')
                # Plan 6.3: place the fault BEFORE the receiver reads
                # the target generation -- hold polling across the
                # commit + injection, then resume and observe.
                ask(worker, {'cmd': 'hold'})
                baseline = {e['t_ns'] for e in drain_events(worker)}
                t_inject = time.monotonic_ns()
                assert h.commit(rep + 30).accepted
                restore, forensics = injector()
                ask(worker, {'cmd': 'resume'})
                log.check(True, 'injection_recorded', json.dumps(
                    {'fault_id': fault_id, **forensics},
                    sort_keys=True)[:300])
                try:
                    if expect == 'hostile':
                        deadline = time.monotonic() + 10
                        detection = None
                        while time.monotonic() < deadline:
                            for event in drain_events(worker):
                                if (event.get('event')
                                        in DETECTION_EVENTS
                                        and event['t_ns'] >= t_inject
                                        and event['t_ns'] not in baseline
                                        and not _startup_resync(
                                            event, boundary, 'hostile')):
                                    detection = event
                                    break
                            if detection:
                                break
                            time.sleep(0.05)
                        if detection is None:
                            seen = [e for e in drain_events(worker)
                                    if e.get('event') != 'received']
                            log.check(False,
                                      'detection_event_after_injection',
                                      seen[:6])
                        else:
                            log.check(True,
                                      'detection_event_after_injection',
                                      detection)
                    else:  # benign: idempotent, never wrong content
                        time.sleep(0.3)      # let polls observe it
                        events = drain_events(worker)
                        # Resyncs may legitimately occur (held-barrier
                        # mechanics, resync-answer generations); HARD
                        # failures (unavailable / validation) may not.
                        hard = [
                            e for e in events
                            if e.get('event') in ('unavailable',
                                                  'validate_failed')
                            and e['t_ns'] >= t_inject
                            and e['t_ns'] not in baseline]
                        log.check(not hard, 'benign_no_hard_failures',
                                  hard[:2])
                        # A named window is CORRECT under the strict
                        # read contract (e.g. a regressed pointer with
                        # a newer head -> REPLICA_LAGGING).
                        reply = ask(worker, {'cmd': 'query'})
                        if not reply.get('ok'):
                            log.check(
                                reply.get('error')
                                in ('REPLICA_LAGGING',
                                    'QUERY_INDEX_UNAVAILABLE',
                                    'REPLICA_NO_HEAD',
                                    'REPLICA_CONTROL_STALE',
                                    'SNAPSHOT_EXPIRED'),
                                'benign_named_window_ok',
                                reply.get('error'))
                    # never wrong content, on any boundary
                    query = ask(worker, {'cmd': 'query'})
                    if query.get('ok'):
                        reported = tuple(query.get('identity')[:2])
                        src_digest = h.expected_digests.get(reported)
                        log.check(src_digest is not None
                                  and query['digest'] == src_digest,
                                  'never_wrong_content',
                                  'ident %s known=%s'
                                  % (query.get('identity'),
                                     src_digest is not None))
                finally:
                    restore()
                # recovery: detection->ready chain + full fingerprint
                h.recover(rep + 40)
                ident2 = list(h.store.identity[:2])
                ok = h.wait_ready(worker, ident2, 30)
                log.check(ok, 'recovered_ready_30s', ident2)
                if ok:
                    ready_after = [
                        e for e in drain_events(worker)
                        if e.get('event') == 'ready'
                        and e['t_ns'] >= t_inject]
                    if expect == 'hostile':
                        log.check(bool(ready_after),
                                  'recovery_ready_event_chain')
                    fp = ask(worker, {'cmd': 'fingerprint'},
                             timeout_s=10.0)
                    src_fp, _ = h.source_fingerprint()
                    if fp.get('ok') and fp.get('fingerprint') != src_fp:
                        src_table = fingerprint_table(h)
                        diff = sorted(
                            name for name in
                            set(src_table) | set(fp.get('table', {}))
                            if src_table.get(name)
                            != fp['table'].get(name))
                        log.check(False,
                                  'full_snapshot_equal_after_recovery',
                                  diff[:12])
                    else:
                        log.check(fp.get('ok'),
                                  'full_snapshot_equal_after_recovery',
                                  (str(fp.get('fingerprint'))[:12],
                                   src_fp[:12], fp.get('error')))
                h.close_worker(worker)
                case_ok = all(entry['ok'] for entry in log)
                cases.append({'fault_id': fault_id, 'boundary': boundary,
                              'rep': rep, 'expectation': expect,
                              'checks': list(log), 'pass': case_ok})
    results['A5'] = {'cases': cases, 'injections': len(cases),
                     'pass': all(c['pass'] for c in cases)}


def run_a6(h, results):
    cases = []
    for kind in ('producer_restart', 'epoch_reset', 'geometry_change'):
        for rep in range(3):
            log = CaseLog()
            if kind == 'producer_restart':
                # a REAL writer subprocess on a DEDICATED root (the
                # harness's own publisher owns h.root; a second owner
                # on it would be refused, correctly)
                restart_root = tempfile.mkdtemp(prefix='fault-v3-wr-')
                writer = subprocess.Popen(
                    [sys.executable, '-c',
                     WRITER_PROC % (_WS_SRC, _WS_SRC, _WS_SRC),
                     restart_root, 'restart', str(rep)],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, text=True)
                reply = ask(writer, {'cmd': 'identity'}, timeout_s=30)
                log.check(reply.get('ok'), 'writer_proc_identity',
                          reply.get('error'))
                old_token = list(reply.get('identity') or [])
                worker = h.spawn_worker(restart_root)
                ok = h.wait_ready(worker, old_token[:2], 30)
                log.check(ok, 'pre_ready_from_proc_writer')
                ask(writer, {'cmd': 'commit'}, timeout_s=30)
                writer.kill()               # real writer crash
                writer.wait(timeout=5)
                log.check(writer.returncode == -9, 'writer_sigkilled')
                # NEW writer process, SAME root, new incarnation (the
                # stale-owner takeover path)
                writer2 = subprocess.Popen(
                    [sys.executable, '-c',
                     WRITER_PROC % (_WS_SRC, _WS_SRC, _WS_SRC),
                     restart_root, 'restart', str(100 + rep)],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, text=True)
                reply = ask(writer2, {'cmd': 'commit'}, timeout_s=30)
                log.check(reply.get('ok'), 'new_writer_commit')
                new_token = list(reply.get('identity') or [])
                log.check(tuple(new_token) != tuple(old_token),
                          'new_incarnation_identity',
                          (old_token[:3], new_token[:3]))
                source = ask(writer2, {'cmd': 'fingerprint'},
                             timeout_s=30)
            else:
                worker = h.spawn_worker()
                ok = h.wait_ready(worker, h.store.identity[:2], 30)
                log.check(ok, 'pre_ready')
                old_token = list(h.store.identity)
                if kind == 'epoch_reset':
                    result = h.store.apply_update(MapUpdate(
                        operation_id='reset-%d' % rep,
                        kind=UpdateKind.RESET))
                    log.check(result.accepted, 'reset_accepted')
                    assert h.commit(rep + 50).accepted
                else:
                    # geometry change: rebuild the writer world
                    h.hook.close()
                    h.mapper = CargoVolumeMapper(
                        (1.2, 1.4, 1.2), (0, 0, 0.74), 0,
                        h.resolution, max_raycast_points=500)
                    h.store = OccupancyMapStore(h.mapper, epoch='fault2')
                    h.hook = wire_store_export(h.store, h.root)
                    h.store._snapshot = h.store._build_snapshot()
                    h.hook.wait_pending(60.0)
                    assert h.commit(rep + 50).accepted
                new_token = list(h.store.identity)
                log.check(tuple(new_token) != tuple(old_token),
                          'world_rotated_full_identity',
                          (old_token, new_token))
                source = {'ok': True,
                          'fingerprint': h.source_fingerprint()[0],
                          'identity': new_token}
            # old FULL-identity token must named-fail once the receiver
            # has applied the new world
            applied = h.wait_ready(worker, new_token[:2], 30)
            log.check(applied, 'old_worker_applied_new_world')
            if applied:
                stale = ask(worker, {'cmd': 'query',
                                     'expected': old_token})
                log.check(not stale.get('ok'),
                          'old_full_token_named_fail',
                          stale.get('error'))
            # fresh receiver cold-syncs via FULL; full fingerprint check
            fresh = h.spawn_worker(
                restart_root if kind == 'producer_restart' else None)
            ident = h.wait_ready_any(fresh, 30)
            log.check(ident is not None, 'cold_sync_new_world', ident)
            if ident:
                fp = ask(fresh, {'cmd': 'fingerprint'}, timeout_s=10.0)
                log.check(fp.get('ok')
                          and fp.get('fingerprint')
                          == source.get('fingerprint'),
                          'new_world_full_snapshot_equal',
                          (str(fp.get('fingerprint'))[:12],
                           str(source.get('fingerprint'))[:12]))
                old_again = ask(fresh, {'cmd': 'query',
                                        'expected': old_token})
                log.check(not old_again.get('ok'),
                          'old_token_rejected_in_new_world',
                          old_again.get('error'))
            for proc in (worker, fresh):
                h.close_worker(proc)
            if kind == 'producer_restart':
                try:
                    ask(writer2, {'cmd': 'shutdown'}, timeout_s=10)
                    writer2.wait(timeout=10)
                except Exception:  # noqa: BLE001
                    writer2.kill()
            case_ok = all(entry['ok'] for entry in log)
            cases.append({'kind': kind, 'rep': rep,
                          'checks': list(log), 'pass': case_ok})
    results['A6'] = {'cases': cases,
                     'pass': all(c['pass'] for c in cases)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gates', default='A4,A5,A6')
    parser.add_argument('--out', required=True)
    parser.add_argument('--resolution', type=float, default=0.05)
    args = parser.parse_args()
    if os.path.exists(args.out):
        raise SystemExit('refusing to overwrite %s' % args.out)

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)

    h = Harness(resolution=args.resolution)
    results = {'revision': subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC, text=True).strip(),
        'resolution': h.resolution, 'harness': 'v3', 'gates': {}}
    for seed in range(4):
        assert h.commit(seed).accepted

    try:
        if 'A4' in args.gates:
            run_a4(h, results['gates'])
        if 'A5' in args.gates:
            run_a5(h, results['gates'])
        if 'A6' in args.gates:
            run_a6(h, results['gates'])
    finally:
        h.hook.close()
        os.close(lock_fd)
        shutil.rmtree(h.root, ignore_errors=True)
        shutil.rmtree(h.marker_dir, ignore_errors=True)

    gates = results['gates']
    not_evaluated = sum(g.get('not_evaluated', 0) for g in gates.values())
    results['pass'] = bool(gates) and all(
        bool(g.get('pass')) for g in gates.values())
    results['not_evaluated'] = not_evaluated
    with open(args.out, 'w') as handle:
        json.dump(results, handle, indent=1)
    print(json.dumps({'pass': results['pass'],
                      'not_evaluated': not_evaluated,
                      **{name: gate.get('pass')
                         for name, gate in gates.items()}}, indent=1))
    if not gates:
        return 4
    if not_evaluated:
        return 4
    return 0 if results['pass'] else 3


if __name__ == "__main__":
    sys.exit(main())
