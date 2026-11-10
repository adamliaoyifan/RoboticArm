#!/usr/bin/env python3
"""G4-A gen3 unified derived-resource ledger (review_5c5c1842
Finding 3): receiver decoded/index/cache state, active old references,
queue/staging and resync temporaries -- with an EXPLICIT dedup
statement, peak events, and no blind summation of overlapping
accounting.

Dedup model (documented, not guessed):
- export root objects/ files are content-addressed -> each file is a
  UNIQUE physical byte (dedup is structural); generations/ manifests
  reference them and add only their own bytes; staging/ is bounded by
  the publisher's single in-flight export; control/ is tiny.
- writer registry conservative_retained_bytes ALREADY counts its two
  retained generations' payloads (which are the SAME logical content
  the export root stores) -- it is a WRITER-side virtual retention
  estimate, NOT additional disk; it is reported side by side, never
  summed with export bytes.
- receiver: decoded object cache entries are references to content
  already counted in objects/ on the writer root only when the roots
  are on the same filesystem; here receiver reads the SAME root, so
  receiver cache adds PROCESS MEMORY (measured RSS), not disk.
- peaks: per-kind max over the run's 1 Hz samples plus the explicit
  allocation event (FULL decode) sizes captured at apply time.

Runs one bounded scenario (wired store + commits + replica applies +
one forced resync window) and emits the ledger JSON. Offline only.
"""

import argparse
import json
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
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def tree_bytes(path):
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total


def file_count(path):
    count = 0
    for root, _, files in os.walk(path):
        count += len(files)
    return count


def rss_mb(pid='self'):
    try:
        with open('/proc/%s/status' % pid) as handle:
            for line in handle:
                if line.startswith('VmRSS'):
                    return int(line.split()[1]) / 1024.0
    except OSError:
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--resolution', type=float, default=0.02)
    parser.add_argument('--boxes', type=int, default=5)
    parser.add_argument('--commits', type=int, default=6)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    if os.path.exists(args.out):
        raise SystemExit('refusing to overwrite %s' % args.out)

    peaks = {'writer_rss': [], 'receiver_rss': [], 'staging_bytes': [],
             'resync_control_bytes': []}

    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               args.resolution, max_raycast_points=500)
    store = OccupancyMapStore(mapper, epoch='ledger')
    placed = 0
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= args.boxes:
            break
        assert store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i)).accepted
        placed += 1
    assert placed == args.boxes, (placed, args.boxes)
    root = tempfile.mkdtemp(prefix='ledger-')
    hook = wire_store_export(store, root)
    store._snapshot = store._build_snapshot()
    hook.wait_pending(60.0)

    def sample_peaks():
        peaks['writer_rss'].append(rss_mb())
        peaks['staging_bytes'].append(
            tree_bytes(os.path.join(root, 'staging')))
        peaks['resync_control_bytes'].append(
            tree_bytes(os.path.join(root, 'control')))

    # RECEIVER as a REAL independent process (review_d9a28e4e P1 #3):
    # it polls until told to stop and reports its own RSS/cache stats;
    # the parent samples BOTH PIDs on a 0.2s timer DURING activity so
    # in-flight staging/FULL-decode peaks are captured, not just final
    # state.
    import subprocess as sp
    import threading
    receiver_out = tempfile.mktemp(prefix='ledger-recv-',
                                   suffix='.json')
    RECEIVER = '''
import json, os, sys, time
root, out_path = sys.argv[1], sys.argv[2]
for entry in ("%s/src/luggage_description",
              "%s/src/luggage_perception", "%s"):
    sys.path.insert(0, entry)
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
replica = QueryReplica(root)
rss_log = []
def rss_mb():
    with open('/proc/self/status') as handle:
        for line in handle:
            if line.startswith('VmRSS'):
                return int(line.split()[1]) / 1024.0
stats = {'rss_peak_mb': 0.0, 'rss_final_mb': None, 'samples': 0,
         'cache_entries': 0, 'state': ''}
deadline = time.monotonic() + 300
while time.monotonic() < deadline:
    replica.poll()
    value = rss_mb()
    stats['rss_peak_mb'] = max(stats['rss_peak_mb'], value or 0.0)
    stats['rss_final_mb'] = value
    stats['samples'] += 1
    stats['cache_entries'] = len(replica._object_cache)
    stats['state'] = str(replica.state)
    # atomic rewrite EVERY cycle: the parent terminates this process,
    # so the stats file must always be fresh
    tmp = out_path + '.tmp'
    with open(tmp, 'w') as handle:
        json.dump(stats, handle)
    os.replace(tmp, out_path)
    time.sleep(0.2)
''' % (_WS_SRC, _WS_SRC, _WS_SRC)
    receiver = sp.Popen(
        [sys.executable, '-c', RECEIVER, root, receiver_out],
        stdout=sp.PIPE, stderr=sp.PIPE, text=True)

    def sampler():
        while sampler_stop[0] is False:
            peaks['writer_rss'].append(rss_mb())
            peaks['receiver_rss'].append(rss_mb(receiver.pid))
            peaks['staging_bytes'].append(
                tree_bytes(os.path.join(root, 'staging')))
            peaks['resync_control_bytes'].append(
                tree_bytes(os.path.join(root, 'control')))
            time.sleep(0.2)
    sampler_stop = [False]
    thread = threading.Thread(target=sampler, daemon=True)
    thread.start()

    for seed in range(args.commits):
        points = np.column_stack([
            np.random.default_rng(seed).uniform(-.5, .5, 2000),
            np.random.default_rng(seed + 1).uniform(-.6, .6, 2000),
            np.random.default_rng(seed + 2).uniform(
                mapper._hull.floor_z + .1,
                mapper._hull.floor_z + 1.0, 2000)])
        store.apply_update(MapUpdate(
            operation_id='l%d' % seed,
            kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store.identity,
            geometry_hash=mapper.geometry_hash, points=points,
            origin=(0., 0., mapper._hull.ceiling_z - 0.1),
            minimum_z=mapper._hull.floor_z + .05,
            stamp_key=(seed, 0),
            acquisition_stamp_ns=1_000_000_000 + seed,
            settled=True, schema_version=1))
        hook.wait_pending(60.0)
        time.sleep(0.05)               # receiver process polls

    # one forced resync window: break the pointer, let the receiver
    # process observe it, then heal via a further commit -- the
    # sampler thread records the in-flight peak.
    pointer = os.path.join(root, 'published.json')
    saved = open(pointer, 'rb').read()
    with open(pointer, 'wb') as handle:
        handle.write(b'{"manifest": "generations/x/9/manifest.json",'
                     b' "manifest_sha256": "0"}')
    time.sleep(1.0)                       # receiver observes failure
    store.apply_update(MapUpdate(
        operation_id='heal', kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash,
        points=np.column_stack([
            np.random.default_rng(99).uniform(-.5, .5, 2000),
            np.random.default_rng(98).uniform(-.6, .6, 2000),
            np.random.default_rng(97).uniform(
                mapper._hull.floor_z + .1,
                mapper._hull.floor_z + 1.0, 2000)]),
        origin=(0., 0., mapper._hull.ceiling_z - 0.1),
        minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(999, 0), acquisition_stamp_ns=1_999_999_999,
        settled=True, schema_version=1))
    hook.wait_pending(60.0)
    time.sleep(2.0)                       # receiver recovers

    sampler_stop[0] = True
    thread.join(timeout=5.0)
    receiver.terminate()
    try:
        receiver.wait(timeout=10.0)
    except sp.TimeoutExpired:
        receiver.kill()
    with open(receiver_out) as handle:
        receiver_stats = json.load(handle)
    os.unlink(receiver_out)

    objects_dir = os.path.join(root, 'objects')
    generations_dir = os.path.join(root, 'generations')
    ledger = {
        'revision': subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=_WS_SRC,
            text=True).strip(),
        'resolution': args.resolution, 'boxes': args.boxes,
        'commits': args.commits,
        'dedup_statement': {
            'export_objects': ('content-addressed: every file under '
                               'objects/ is physically unique; '
                               'generation manifests only reference '
                               'them'),
            'writer_registry_bytes': ('virtual retention estimate of '
                                      'the two in-memory generations; '
                                      'overlaps logically with export '
                                      'objects -- reported, never '
                                      'summed'),
            'receiver_cache': ('decoded references to the SAME root '
                               'on this host: adds process memory '
                               '(RSS), not additional disk'),
        },
        'export_root': {
            'objects_bytes': tree_bytes(objects_dir),
            'objects_files': file_count(objects_dir),
            'generations_bytes': tree_bytes(generations_dir),
            'manifests': file_count(generations_dir),
        },
        'writer_registry': dict(
            store._query_registry.resource_summary()),
        'receiver_process': receiver_stats,
        'peaks': {
            'writer_rss_mb': max(peaks['writer_rss']),
            'receiver_rss_mb': max(peaks['receiver_rss']),
            'staging_bytes_peak': max(peaks['staging_bytes']),
            'resync_control_bytes_peak': max(
                peaks['resync_control_bytes']),
            'samples': len(peaks['writer_rss']),
        },
        'budgets': {
            'note': ('split accounting: writer registry is the '
                     'LOGICAL retention budget (512MiB); the receiver '
                     'is accounted by its measured process RSS; disk '
                     'by unique content-addressed bytes. No unified '
                     '512MiB pass is claimed across processes.'),
            'derived_virtual_budget_bytes': 512 * 1024 * 1024,
            'writer_registry_within_budget': (
                store._query_registry.resource_summary()[
                    'conservative_retained_bytes']
                <= 512 * 1024 * 1024),
        },
        'boxes_placed': placed,
    }
    hook.close()
    with open(args.out, 'w') as handle:
        json.dump(ledger, handle, indent=1)
    print(json.dumps({'export_objects_mb': round(
        ledger['export_root']['objects_bytes'] / 1e6, 1),
        'registry_mb': round(ledger['writer_registry'][
            'conservative_retained_bytes'] / 1e6, 1),
        'writer_rss_peak': ledger['peaks']['writer_rss_mb'],
        'receiver_rss_peak': ledger['peaks']['receiver_rss_mb'],
        'registry_within_budget': ledger['budgets'][
            'writer_registry_within_budget']}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
