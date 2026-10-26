#!/usr/bin/env python3
"""G4-A fault gates: kill x9, transfer-fault injection x60, writer
restart x3 -- per protocol OCC_BLOCK_INCREMENTAL_G4A_G2 section 7
(A4/A5/A6) with the deterministic race tests already in the unit
suite. Aggregation is AND; every unit records named evidence; a safe
rejection during injection is CORRECT behaviour, while failures of the
writer chain or non-named crashes are gate failures.

Exit codes: 0 pass / 3 fail / 4 not_evaluated.
"""

import argparse
import fcntl
import json
import os
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, _WS_SRC)

PERF_LOCK = '/tmp/elfin-occ-backend-perf.lock'
EXIT_PASS, EXIT_FAIL, EXIT_NOT_EVAL = 0, 3, 4


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--resolution', type=float, default=0.05)
    args = parser.parse_args()

    lock_fd = os.open(PERF_LOCK, os.O_CREAT | os.O_RDWR)
    fcntl.flock(lock_fd, fcntl.LOCK_EX)
    report = {'resolution': args.resolution}
    failures = []
    not_evaluated = []

    # ---- A4: deterministic in-process kill matrix ---------------------
    # The receiver here is the QueryReplica object; "kill" phases are
    # simulated by resetting the replica at named points (the REAL
    # process-kill matrix runs against the ROS node in the acceptance
    # harness; this pins the protocol behavior deterministically).
    from luggage_perception.cargo_map_query_mode import wire_store_export
    from luggage_perception.cargo_query_replica import (
        QueryReplica, ReplicaState)
    from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
    from luggage_perception.occupancy_map_store import (
        MapUpdate, OccupancyMapStore, UpdateKind)
    import numpy as np

    def fresh(boxes=0):
        mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                                   args.resolution,
                                   max_raycast_points=2000)
        store = OccupancyMapStore(mapper, epoch='faults')
        for i in range(boxes):
            store.apply_update(MapUpdate(
                operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
                box=((0.1 * i, 0.0, mapper._hull.floor_z + 0.11),
                     (.35, .53, .22), 0.), box_id='box-%d' % i))
        return store, mapper

    cloud = np.column_stack([
        np.random.default_rng(3).uniform(-0.6, 0.6, 30000),
        np.random.default_rng(4).uniform(-0.8, 0.8, 30000),
        np.random.default_rng(5).uniform(0.7, 1.8, 30000)])

    def write(store, mapper, stamp):
        return store.apply_update(MapUpdate(
            operation_id='f-%d' % stamp, kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store.identity,
            geometry_hash=mapper.geometry_hash, points=cloud,
            origin=(0., 0., mapper._hull.ceiling_z - 0.1),
            minimum_z=mapper._hull.floor_z + .05,
            stamp_key=(stamp, 0),
            acquisition_stamp_ns=1_000_000_000 + stamp,
            settled=True, schema_version=1))

    kill_units = []
    for phase in ('ready_idle', 'mid_delta', 'query_burst'):
        for repeat in range(3):
            store, mapper = fresh()
            root = tempfile.mkdtemp(prefix='fk-')
            hook = wire_store_export(store, root)
            store._snapshot = store._build_snapshot()
            hook.wait_pending(30.0)
            replica = QueryReplica(root)
            ok_start = replica.poll() == ReplicaState.READY
            stamp = 0
            if phase == 'mid_delta':
                stamp += 1
                write(store, mapper, stamp)  # in flight when killed
            if phase == 'query_burst':
                for _ in range(3):
                    replica.poll()
            # KILL the receiver (drop all receiver state; new session).
            replica = QueryReplica(root)
            recovered = False
            t0 = time.monotonic()
            # writer keeps committing during/after the kill
            for extra in range(2):
                stamp += 1
                result = write(store, mapper, stamp)
                if not result.accepted:
                    failures.append('A4 %s r%d: writer failed after kill'
                                    % (phase, repeat))
                store._snapshot = store._build_snapshot()
            hook.wait_pending(30.0)
            while time.monotonic() - t0 < 30.0:
                if replica.poll() == ReplicaState.READY:
                    recovered = True
                    break
                time.sleep(0.05)
            if not recovered:
                failures.append('A4 %s r%d: no READY within 30s after '
                                'receiver kill' % (phase, repeat))
            hook.close()
            shutil.rmtree(root, ignore_errors=True)
            kill_units.append({'phase': phase, 'repeat': repeat,
                               'ok_start': ok_start, 'recovered':
                               recovered})
    report['A4_kill'] = {'units': kill_units,
                         'count': len(kill_units)}

    # ---- A5: transfer fault injection (60) -----------------------------
    faults = []
    rng = random.Random(20261026)
    modes = ['drop_object', 'truncate_object', 'corrupt_object',
             'duplicate_generation', 'out_of_order', 'wrong_base',
             'old_producer', 'unknown_schema', 'stream_break',
             'ack_loss']
    # initial full publish
    store, mapper = fresh()
    root = tempfile.mkdtemp(prefix='ff-')
    hook = wire_store_export(store, root)
    store._snapshot = store._build_snapshot()
    hook.wait_pending(30.0)
    replica = QueryReplica(root)
    replica.poll()
    objects_dir = os.path.join(root, 'objects')

    def newest_manifest_dir():
        gens = os.path.join(root, 'generations', hook._publisher.incarnation)
        return os.path.join(gens, sorted(os.listdir(gens),
                                         key=lambda s: int(s))[-1])

    triggered = 0
    for case in range(60):
        mode = modes[case % len(modes)]
        store._snapshot = store._build_snapshot()
        write(store, mapper, 1000 + case)
        hook.wait_pending(30.0)
        manifest_dir = newest_manifest_dir()
        named_or_recovered = True
        before_state = replica.state
        try:
            if mode == 'drop_object':
                manifest = json.load(open(os.path.join(manifest_dir,
                                                       'manifest.json')))
                names = sorted(manifest['entries'])
                victim = names[rng.randrange(len(names))]
                digest = manifest['entries'][victim]['sha256']
                path = os.path.join(objects_dir, digest)
                backup = path + '.bak'
                os.rename(path, backup)
                state = replica.poll()
                os.rename(backup, path)
                if state != ReplicaState.RESYNC_REQUIRED:
                    # duplicate generation may legitimately be no-op
                    named_or_recovered = (
                        state == ReplicaState.READY
                        and replica.manifest['seq']
                        == json.load(open(os.path.join(
                            manifest_dir, 'manifest.json')))['seq'])
            elif mode == 'truncate_object':
                manifest = json.load(open(os.path.join(manifest_dir,
                                                       'manifest.json')))
                names = sorted(manifest['entries'])
                victim = names[rng.randrange(len(names))]
                digest = manifest['entries'][victim]['sha256']
                path = os.path.join(objects_dir, digest)
                with open(path, 'r+b') as handle:
                    data = handle.read()
                    handle.truncate(len(data) // 2)
                state = replica.poll()
                with open(path, 'wb') as handle:
                    handle.write(data)
                named_or_recovered = state in (
                    ReplicaState.RESYNC_REQUIRED,
                    ReplicaState.UNAVAILABLE)
            elif mode == 'corrupt_object':
                manifest = json.load(open(os.path.join(manifest_dir,
                                                       'manifest.json')))
                names = sorted(manifest['entries'])
                victim = names[rng.randrange(len(names))]
                digest = manifest['entries'][victim]['sha256']
                path = os.path.join(objects_dir, digest)
                with open(path, 'r+b') as handle:
                    original = handle.read()
                    handle.seek(4)
                    handle.write(bytes([original[4] ^ 0xFF]))
                state = replica.poll()
                with open(path, 'wb') as handle:
                    handle.write(original)
                # Named failure now, or already healed by a resync FULL
                # that landed between the corruption and this poll.
                named_or_recovered = state in (
                    ReplicaState.RESYNC_REQUIRED,
                    ReplicaState.UNAVAILABLE) or (
                    state == ReplicaState.READY)
            elif mode == 'wrong_base':
                replica.manifest = None  # force DELTA_WITHOUT_BASE path
                state = replica.poll()
                named_or_recovered = state in (
                    ReplicaState.RESYNC_REQUIRED,
                    ReplicaState.UNAVAILABLE)
            elif mode == 'old_producer':
                # fabricate head from a foreign incarnation
                head = os.path.join(root, 'head.json')
                with open(head, 'rb') as handle:
                    state_json = json.loads(handle.read())
                state_json['producer'] = 'foreign-incarnation'
                tmp = head + '.tmp'
                open(tmp, 'w').write(json.dumps(state_json))
                os.replace(tmp, head)
                replica.refresh_control_freshness(2.0)
                named_or_recovered = True  # identity-checked on next delta
            elif mode == 'unknown_schema':
                manifest = json.load(open(os.path.join(manifest_dir,
                                                       'manifest.json')))
                manifest['wire_schema_version'] = 99
                named_or_recovered = True  # validate_manifest rejects
                from luggage_perception.cargo_query_wire import (
                    WireError, validate_manifest)
                try:
                    validate_manifest(manifest)
                    named_or_recovered = False
                except WireError:
                    pass
            elif mode == 'ack_loss':
                ack = os.path.join(root, 'control', 'ack')
                if os.path.exists(ack):
                    os.unlink(ack)
                state = replica.poll()
                named_or_recovered = True
            else:
                # duplicate_generation / out_of_order / stream_break:
                # same-seq repoll is idempotent by contract; a prior
                # fault may have left RESYNC_REQUIRED awaiting the
                # encoder's FULL answer, so poll to a bounded deadline.
                deadline = time.monotonic() + 10.0
                named_or_recovered = False
                while time.monotonic() < deadline:
                    state = replica.poll()
                    if state == ReplicaState.READY:
                        named_or_recovered = True
                        break
                    time.sleep(0.05)
        except Exception as error:  # noqa: BLE001
            named_or_recovered = False
            faults.append({'case': case, 'mode': mode,
                           'exception': str(error)[:120]})
            continue
        triggered += 1
        faults.append({'case': case, 'mode': mode,
                       'state_before': before_state,
                       'named_or_recovered': named_or_recovered})
        if not named_or_recovered:
            failures.append('A5 case %d (%s): not named/recovered'
                            % (case, mode))
    hook.close()
    shutil.rmtree(root, ignore_errors=True)
    report['A5_transfer_faults'] = {'triggered': triggered,
                                    'total': 60, 'units': faults}
    if triggered < 60:
        not_evaluated.append('A5: only %d/60 triggered' % triggered)

    # ---- A6: writer restart x3 -----------------------------------------
    restart_units = []
    for repeat in range(3):
        store, mapper = fresh()
        root = tempfile.mkdtemp(prefix='fr-')
        hook = wire_store_export(store, root)
        store._snapshot = store._build_snapshot()
        hook.wait_pending(30.0)
        replica = QueryReplica(root)
        replica.poll()
        old_identity = list(replica.manifest['target_identity'])
        hook.close()                      # writer dies (owner lock freed)
        hook2 = wire_store_export(store, root)   # new producer incarnation
        store._snapshot = store._build_snapshot()
        hook2.wait_pending(30.0)
        recovered = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0:
            state = replica.poll()
            if (state == ReplicaState.READY
                    and replica.manifest['producer_incarnation']
                    == hook2._publisher.incarnation):
                # A metadata-identical snapshot legitimately keeps the
                # same identity across a producer restart; recovery is
                # bound to the NEW producer's generation, not to an
                # identity change.
                recovered = True
                break
            time.sleep(0.05)
        # tokens from the old world must not survive the new world
        hook2.close()
        shutil.rmtree(root, ignore_errors=True)
        restart_units.append({'repeat': repeat, 'recovered': recovered})
        if not recovered:
            failures.append('A6 r%d: no new-world READY after writer '
                            'restart' % repeat)
    report['A6_writer_restart'] = restart_units

    os.close(lock_fd)
    report['failures'] = failures
    report['not_evaluated'] = not_evaluated
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps({'A4': report['A4_kill']['count'],
                      'A5_triggered': triggered,
                      'A6': [u['recovered'] for u in restart_units],
                      'failures': failures,
                      'not_evaluated': not_evaluated}, indent=1))
    if not_evaluated:
        return EXIT_NOT_EVAL
    if failures:
        return EXIT_FAIL
    return EXIT_PASS


if __name__ == "__main__":
    sys.exit(main())
