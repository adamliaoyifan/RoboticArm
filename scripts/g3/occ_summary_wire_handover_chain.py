#!/usr/bin/env python3
"""Q-20261028-1 handover close-out chain (review round 4, 2026-10-29).

ONE run of the REAL chain on a FIXED install version:

    writer process -> FULL -> DELTA -> independent reader process
    -> typed summary -> client

with, in the SAME run: source/identity/field verification against
independent records, and one v1-rejection -> forced-FULL recovery.
Raw evidence (this script never deletes the export root) lands in
the output directory.

Subprocess contract (both import the product ONLY from the install
tree; PYTHONPATH excludes the repo source, cwd is the install dir,
and each reports a per-module sha256 lock the parent verifies
against the install tree AND the committed source):

  ctl/PHASE1_DONE   writer: gen1 FULL published, gen2 DELTA published,
                    writer_state.json carries owner identity/summary
                    per generation
  ctl/cmd_more      writer: publish one further metadata generation
  ctl/cmd_finish    writer: stop
  reader_status.jsonl  reader: one line per poll cycle state change
  reader_version_lock.json / writer_version_lock.json

Fault injection (by THIS parent, on disk, between phases):
  - the current head manifest is rewritten as a v1 stream with a
    matching published.json sha (the reader must traverse it);
  - after two new generations, the retained intermediate manifest is
    deleted (retention-pruning fault) so bounded catch-up cannot
    chain; the writer's encoder loop answers the reader's resync
    request with a forced FULL, which the reader APPLIES.

Usage:
  python3 scripts/g3/occ_summary_wire_handover_chain.py \
      --install <dir>/install --out <evidence-dir> [--source <repo>]
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

MODULES = ['cargo_query_replica.py', 'cargo_query_snapshot_wire.py',
           'cargo_map_read_facade.py', 'cargo_map_query_service.py',
           'cargo_query_wire.py', 'cargo_map_query_mode.py',
           'cargo_query_export.py', 'occupancy_map_store.py',
           'cargo_volume_mapper.py']

TYPED_VALUE_FIELDS = ('total_voxels', 'inactive_count', 'unknown_count',
                      'free_count', 'occupied_count', 'frontier_count',
                      'unknown_ratio', 'occupancy_ratio',
                      'occupied_volume', 'free_volume')

TYPED_ATTR_FIELDS = ('occupancy_confirmed', 'change_event', 'map_source',
                     'cargo_observed', 'acquisition_stamp_ns',
                     'index_status')


def sha256_file(path):
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def read_json(path, timeout_s=30):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if os.path.exists(path):
            try:
                with open(path) as handle:
                    return json.load(handle)
            except ValueError:
                pass
        time.sleep(0.05)
    raise TimeoutError('never appeared: %s' % path)


def wait_true(predicate, timeout_s=30, interval_s=0.05, what=''):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval_s)
    raise TimeoutError('condition never met: %s' % (what or predicate))


WRITER_PROC = r'''
import hashlib, json, os, sys, time
root, ctl, state_path, lock_path = sys.argv[1:5]
import numpy as np
import luggage_perception
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
from luggage_perception.cargo_map_query_mode import wire_store_export

package_dir = os.path.dirname(os.path.abspath(luggage_perception.__file__))
with open(lock_path, 'w') as handle:
    json.dump({'package_dir': package_dir,
               'schema_version': 2,
               'files': {name: hashlib.sha256(open(
                   os.path.join(package_dir, name), 'rb').read()
               ).hexdigest() for name in __MODULES__}}, handle,
              sort_keys=True)

mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.1)
store = OccupancyMapStore(mapper, epoch='handover')
hook = wire_store_export(store, root)
state = {'generations': []}

def record(tag):
    handle = store.acquire_query_snapshot()
    summary = handle.summary()
    entry = {'tag': tag,
             'identity': [str(part) for part in handle.identity],
             'occupancy_confirmed': summary.occupancy_confirmed,
             'change_event': summary.change_event,
             'pending_changes': [list(pair) for pair in
                                 summary.pending_changes],
             'map_source': summary.map_source,
             'cargo_observed': summary.cargo_observed,
             'acquisition_stamp_ns': summary.acquisition_stamp_ns,
             'index_status': summary.index_status,
             'values': {key: summary.values[key]
                        for key in __VALUE_FIELDS__}}
    handle.close()
    state['generations'].append(entry)
    with open(state_path, 'w') as out:
        json.dump(state, out, sort_keys=True, indent=1)

store.apply_update(MapUpdate(
    operation_id='box0', kind=UpdateKind.ADD_BOX_V1,
    box=((0., 0., mapper._hull.floor_z + .11), (.3, .4, .2), 0.),
    box_id='chain-box'))
record('gen1')
hook.wait_pending(30.0)
open(os.path.join(ctl, 'GEN1_PUBLISHED'), 'w').close()
# wait until the reader has APPLIED gen1 before publishing the DELTA
while not os.path.exists(os.path.join(ctl, 'reader_applied_gen1')):
    time.sleep(0.05)
pts = np.random.default_rng(31).uniform(-0.3, 0.3, (3000, 3))
store.apply_update(MapUpdate(
    operation_id='view1', kind=UpdateKind.INTEGRATE_VIEW,
    expected_snapshot=store.identity,
    geometry_hash=mapper.geometry_hash, points=pts,
    origin=(0., 0., 1.4), minimum_z=mapper._hull.floor_z + .05,
    stamp_key=(1, 0), acquisition_stamp_ns=11, settled=True,
    schema_version=1))
record('gen2')
hook.wait_pending(30.0)
open(os.path.join(ctl, 'PHASE1_DONE'), 'w').close()

more = 0
while True:
    if os.path.exists(os.path.join(ctl, 'cmd_finish')):
        break
    if os.path.exists(os.path.join(ctl, 'cmd_more')):
        os.unlink(os.path.join(ctl, 'cmd_more'))
        more += 1
        # a REAL world change: an empty-point metadata apply does not
        # change the snapshot identity and the export ledger dedups
        # it (no new generation) -- each cmd_more adds a real box so
        # the identity advances and a genuine DELTA ships
        store.apply_update(MapUpdate(
            operation_id='more%d' % more,
            kind=UpdateKind.ADD_BOX_V1,
            box=((0.05 * more, 0.05 * more,
                  mapper._hull.floor_z + .11 + .06 * more),
                 (.1, .1, .05), 0.),
            box_id='chain-more-%d' % more))
        record('more%d' % more)
        hook.wait_pending(30.0)
        open(os.path.join(ctl, 'more%d_done' % more), 'w').close()
    time.sleep(0.05)
hook.close()
'''

READER_PROC = r'''
import hashlib, json, os, sys, threading, time
root, ctl, status_path, lock_path = sys.argv[1:5]
import rclpy
import luggage_perception
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query_service import MapQueryService

package_dir = os.path.dirname(os.path.abspath(luggage_perception.__file__))
with open(lock_path, 'w') as handle:
    json.dump({'package_dir': package_dir,
               'schema_version': 2,
               'files': {name: hashlib.sha256(open(
                   os.path.join(package_dir, name), 'rb').read()
               ).hexdigest() for name in __MODULES__}}, handle,
              sort_keys=True)

replica = QueryReplica(root)
facade = QueryMapReadFacade(replica)
status = {'lines': 0}

def dump(reason):
    events = replica.events()
    applied = sorted({int(e['seq']) for e in events
                      if e.get('event') in ('validated', 'ready')})
    kinds = {int(e['seq']): e.get('kind') for e in events
             if e.get('event') == 'received'}
    line = {'t': time.time(), 'reason': reason,
            'state': str(replica.state),
            'last_error': str(replica.last_error),
            'applied_seqs': applied,
            'applied_full_seqs': [s for s in applied
                                  if kinds.get(s) == 'FULL'],
            'head_seq': (int(replica.manifest['seq'])
                         if replica.manifest else None),
            'head_target_identity': (list(
                replica.manifest['target_identity'])
                if replica.manifest else None)}
    with open(status_path, 'a') as out:
        out.write(json.dumps(line, sort_keys=True) + '\n')
        out.flush()
    status['lines'] += 1

def poll_loop():
    last = None
    while not os.path.exists(os.path.join(ctl, 'cmd_finish')):
        if os.path.exists(os.path.join(ctl, 'pause_polling')):
            # verifier-ordered pause (a real reader may throttle):
            # deterministic window for the retention-pruning fault
            time.sleep(0.02)
            continue
        state = replica.poll()
        key = (str(state), str(replica.last_error),
               int(replica.manifest['seq']) if replica.manifest else -1)
        if key != last:
            dump('transition')
            last = key
            if (replica.manifest is not None
                    and int(replica.manifest['seq']) == 1
                    and state == ReplicaState.READY):
                open(os.path.join(ctl, 'reader_applied_gen1'),
                     'w').close()
        time.sleep(0.02)

thread = threading.Thread(target=poll_loop, daemon=True)
thread.start()
rclpy.init()
node = rclpy.create_node('swf_handover_reader')
MapQueryService(node, facade, None)
dump('serving')
rclpy.spin(node)
'''

WRITER_PROC = WRITER_PROC.replace(
    '__MODULES__', repr(MODULES)).replace(
    '__VALUE_FIELDS__', repr(list(TYPED_VALUE_FIELDS)))
READER_PROC = READER_PROC.replace('__MODULES__', repr(MODULES))


def child_env(install, extra_ld):
    env = dict(os.environ)
    repo = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    dist = [os.path.join(install, pkg, 'local', 'lib', 'python3.10',
                         'dist-packages')
            for pkg in ('luggage_msgs', 'luggage_perception',
                        'luggage_description', 'st09_contracts')]
    keep = [p for p in env.get('PYTHONPATH', '').split(os.pathsep)
            if p and os.path.normpath(p) != repo
            and not os.path.normpath(p).startswith(repo + os.sep)]
    env['PYTHONPATH'] = os.pathsep.join(dist + keep)
    env['LD_LIBRARY_PATH'] = os.pathsep.join(
        [os.path.join(install, 'luggage_msgs', 'lib')]
        + ([extra_ld] if extra_ld else [])
        + env.get('LD_LIBRARY_PATH', '').split(os.pathsep))
    return env


def check_version_lock(name, lock, install_pkg, source_pkg, evidence):
    ok_path = os.path.normpath(lock['package_dir']) == os.path.normpath(
        install_pkg)
    files = {}
    for module, sha in sorted(lock['files'].items()):
        installed = sha256_file(os.path.join(install_pkg, module))
        source = sha256_file(os.path.join(source_pkg, module))
        files[module] = {'reported': sha, 'install': installed,
                         'source': source,
                         'match': sha == installed == source}
    verdict = {'process': name, 'package_dir_is_install': ok_path,
               'schema_version': lock['schema_version'],
               'files': files,
               'all_locked': ok_path and lock['schema_version'] == 2
               and all(f['match'] for f in files.values())}
    evidence['version_locks'][name] = verdict
    return verdict['all_locked']


def typed_call_and_compare(client, node, expected, label, evidence):
    import rclpy
    from luggage_msgs.srv import GetCargoMapSummaryV2
    request = GetCargoMapSummaryV2.Request()
    request.query.request_id = 'handover-%s' % label
    request.query.frame_id = 'container_link'
    request.query.use_current = True
    request.query.release_after_query = True
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    assert future.done(), 'typed call timeout (%s)' % label
    response = future.result()
    checks = {'meta_status': response.meta.status == 'OK',
              'identity_epoch':
                  response.meta.identity.epoch == expected['identity'][0],
              'identity_revision': int(response.meta.identity.revision)
              == int(expected['identity'][1]),
              'identity_digest':
                  response.meta.identity.content_digest
                  == expected['identity'][2]}
    for field in TYPED_ATTR_FIELDS:
        checks['field_' + field] = (getattr(response, field)
                                    == expected[field])
    for field in TYPED_VALUE_FIELDS:
        checks['value_' + field] = (getattr(response, field)
                                    == expected['values'][field])
    evidence['typed_checks'][label] = {
        'status': response.meta.status,
        'reason': response.meta.reason,
        'checks': checks,
        'ok': all(checks.values())}
    return evidence['typed_checks'][label]['ok']


def tamper_head_to_v1(root):
    import glob
    pointer = json.load(open(os.path.join(root, 'published.json')))
    manifest_path = os.path.normpath(
        os.path.join(root, pointer['manifest']))
    manifest = json.load(open(manifest_path))
    assert manifest['wire_schema_version'] == 2
    v1 = dict(manifest, wire_schema_version=1)
    v1.pop('checksum', None)
    with open(manifest_path, 'w') as handle:
        handle.write(json.dumps(v1, sort_keys=True))
    body = open(manifest_path, 'rb').read()
    with open(os.path.join(root, 'published.json'), 'w') as handle:
        handle.write(json.dumps(
            {'manifest': os.path.relpath(manifest_path, root),
             'manifest_sha256': hashlib.sha256(body).hexdigest()},
            sort_keys=True))
    return manifest_path


def run_chain(install, out_dir, source_repo=None, domain_id=None):
    install = os.path.abspath(install)
    # parent-side import resolution: the fixed install's message
    # package must win over any shared-workspace copy predating the
    # V2 srvs (the caller must still provide its native lib dir on
    # LD_LIBRARY_PATH -- a running process cannot retro-fix that)
    _dist = os.path.join(install, 'luggage_msgs', 'local', 'lib',
                         'python3.10', 'dist-packages')
    if _dist not in sys.path:
        sys.path.insert(0, _dist)
    import rclpy
    from luggage_msgs.srv import GetCargoMapSummaryV2
    repo = source_repo or os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', '..'))
    install_pkg = os.path.join(
        install, 'luggage_perception', 'local', 'lib', 'python3.10',
        'dist-packages', 'luggage_perception')
    source_pkg = os.path.join(repo, 'src', 'luggage_perception',
                              'luggage_perception')
    for module in MODULES:
        if sha256_file(os.path.join(install_pkg, module)) != sha256_file(
                os.path.join(source_pkg, module)):
            raise RuntimeError(
                'install tree is STALE vs source for %s -- re-freeze '
                'before running the chain' % module)

    out_dir = os.path.abspath(out_dir)   # children run with cwd=install
    os.makedirs(out_dir, exist_ok=True)
    ctl = os.path.join(out_dir, 'ctl')
    os.makedirs(ctl, exist_ok=True)
    root = os.path.join(out_dir, 'export_root')
    if domain_id is not None:
        # one domain for parent AND children (rclpy reads it at init)
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)

    evidence = {'started': time.time(), 'install': install,
                'source_repo': repo, 'version_locks': {},
                'phases': {}, 'typed_checks': {}}

    env = child_env(install, None)
    if domain_id is not None:
        env['ROS_DOMAIN_ID'] = str(domain_id)
    writer = subprocess.Popen(
        [sys.executable, '-c', WRITER_PROC, root, ctl,
         os.path.join(out_dir, 'writer_state.json'),
         os.path.join(out_dir, 'writer_version_lock.json')],
        env=env, cwd=install,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    reader = subprocess.Popen(
        [sys.executable, '-c', READER_PROC, root, ctl,
         os.path.join(out_dir, 'reader_status.jsonl'),
         os.path.join(out_dir, 'reader_version_lock.json')],
        env=env, cwd=install,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    verdict = {'ok': False}
    try:
        ok_w = check_version_lock(
            'writer', read_json(
                os.path.join(out_dir, 'writer_version_lock.json')),
            install_pkg, source_pkg, evidence)
        ok_r = check_version_lock(
            'reader', read_json(
                os.path.join(out_dir, 'reader_version_lock.json')),
            install_pkg, source_pkg, evidence)
        assert ok_w and ok_r, 'version lock failed -- see evidence'

        # phase 1: writer publishes FULL then DELTA; reader applies BOTH
        wait_true(lambda: os.path.exists(
            os.path.join(ctl, 'PHASE1_DONE')), timeout_s=60,
            what='phase1 published')

        def last_status():
            path = os.path.join(out_dir, 'reader_status.jsonl')
            if not os.path.exists(path):
                return None
            lines = [json.loads(line) for line in
                     open(path).read().splitlines() if line.strip()]
            return lines[-1] if lines else None

        def _phase1_applied():
            status = last_status() or {}
            if (status.get('head_seq') == 2
                    and 1 in status.get('applied_full_seqs', [])
                    and 2 in status.get('applied_seqs', [])):
                return status
            return None

        applied = wait_true(
            _phase1_applied, timeout_s=60,
            what='reader applied FULL gen1 + DELTA gen2')
        evidence['phases']['phase1'] = {
            'reader': applied,
            'writer_state': read_json(
                os.path.join(out_dir, 'writer_state.json'))}

        # typed over the DELTA head, compared to the writer's own
        # record for gen2
        rclpy.init()
        node = rclpy.create_node('swf_handover_client')
        client = node.create_client(
            GetCargoMapSummaryV2, '/cargo_map/get_summary_v2')
        assert client.wait_for_service(timeout_sec=30.0), \
            'typed service never appeared'
        generations = evidence['phases']['phase1']['writer_state'][
            'generations']
        ok_t1 = typed_call_and_compare(
            client, node, generations[-1], 'delta_head', evidence)
        assert ok_t1, 'typed comparison over the DELTA head failed'

        # recovery: v1 head -> named rejection -> broken intermediate
        # -> forced FULL anchor
        tamper_head_to_v1(root)

        def _rejected():
            status = last_status()
            if status and 'UNKNOWN_SCHEMA' in status['last_error']:
                return status
            return None

        rejected = wait_true(
            _rejected, timeout_s=30, what='named v1 rejection')
        evidence['phases']['v1_rejection'] = rejected

        base_seq = int((last_status() or {}).get(
            'head_seq') or applied['head_seq'])
        # pause the reader so the pruning fault is deterministic (a
        # live reader would otherwise catch-up through the soon-to-be
        # pruned intermediate before it disappears)
        pause = os.path.join(ctl, 'pause_polling')
        open(pause, 'w').close()
        try:
            for more in (1, 2):
                open(os.path.join(ctl, 'cmd_more'), 'w').close()
                wait_true(lambda m=more: os.path.exists(
                    os.path.join(ctl, 'more%d_done' % m)),
                    timeout_s=60, what='cmd_more %d' % more)
            import glob
            victims = glob.glob(os.path.join(
                root, 'generations', '*', str(base_seq + 1),
                'manifest.json'))
            assert len(victims) == 1, victims
            shutil.rmtree(os.path.dirname(victims[0]))
            evidence['phases']['pruned_manifest'] = victims[0]
        finally:
            os.unlink(pause)

        def _recovered():
            status = last_status()
            if (status and status['state'].endswith('READY')
                    and status['applied_full_seqs']
                    and max(status['applied_full_seqs']) > base_seq):
                return status
            return None

        recovered = wait_true(
            _recovered, timeout_s=60, what='forced-FULL anchored recovery')
        evidence['phases']['recovery'] = recovered

        generations = read_json(
            os.path.join(out_dir, 'writer_state.json'))['generations']
        ok_t2 = typed_call_and_compare(
            client, node, generations[-1], 'recovered_head', evidence)
        assert ok_t2, 'typed comparison after recovery failed'

        # the recovered head IS the writer's latest on-disk record
        pointer = json.load(open(os.path.join(root, 'published.json')))
        head_manifest = json.load(open(os.path.normpath(os.path.join(
            root, pointer['manifest']))))
        writer_last = generations[-1]
        evidence['phases']['head_manifest'] = {
            'path': pointer['manifest'], 'kind': head_manifest['kind'],
            'target_identity': head_manifest['target_identity'],
            'writer_identity': writer_last['identity']}
        assert [str(part) for part in
                head_manifest['target_identity'][:3]] == \
            writer_last['identity'][:3], 'head identity != writer latest'
        node.destroy_node()
        rclpy.shutdown()

        open(os.path.join(ctl, 'cmd_finish'), 'w').close()
        verdict = {'ok': True,
                   'anchor_full_seqs':
                       recovered['applied_full_seqs'],
                   'head_seq': recovered['head_seq']}
        evidence['verdict'] = verdict
    finally:
        open(os.path.join(ctl, 'cmd_finish'), 'w').close()
        try:
            writer.wait(15)
        except subprocess.TimeoutExpired:
            writer.kill()
        try:
            reader.wait(15)
        except subprocess.TimeoutExpired:
            reader.kill()
        for name, proc in (('writer', writer), ('reader', reader)):
            if proc.poll() is not None and proc.returncode != 0:
                evidence['%s_output' % name] = (
                    proc.stdout.read()[-4000:]
                    if proc.stdout else '')
        evidence['finished'] = time.time()
        with open(os.path.join(out_dir, 'chain_evidence.json'),
                  'w') as handle:
            json.dump(evidence, handle, sort_keys=True, indent=1)
    return verdict


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0])
    parser.add_argument('--install', required=True,
                        help='fixed install dir (e.g. <repo>/install)')
    parser.add_argument('--out', required=True,
                        help='evidence output dir (kept, never deleted)')
    parser.add_argument('--source', default=None,
                        help='repo root for the source-side hash check')
    parser.add_argument('--domain-id', type=int, default=None,
                        help='ROS_DOMAIN_ID for the typed service')
    args = parser.parse_args()
    verdict = run_chain(args.install, args.out, args.source,
                        args.domain_id)
    print(json.dumps(verdict, sort_keys=True))
    return 0 if verdict.get('ok') else 1


if __name__ == '__main__':
    sys.exit(main())
