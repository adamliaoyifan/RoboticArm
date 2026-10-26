#!/usr/bin/env python3
"""C0 probe: I1 three-case reachability on the CURRENT code (baseline).

Case 1a replay, 1b metadata-only (NOTE_EVENT: confirmation transition,
no voxel mutation), 1c new-stamp same cloud. Counts, per case:

- derived side: blocks whose VoxelBlock object identity changed
  (rebuild count), delta candidates size, surface columns updated;
- encode side: JSON payload constructions inside encode_snapshot
  (patched counter -- measures pre-gating waste), emitted objects,
  publish result;
- receive side: receiver rebuild behaviour (full rebuild vs delta path
  is structural in gen-1: recorded as the known gap, counted via
  rebuild_snapshot block iterations).

Honest baseline per plan section 6 C0: no claims, measurements only.
"""

import argparse
import json
import os
import sys
import tempfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS_SRC = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_WS_SRC, "src", "luggage_perception"))
sys.path.insert(0, _WS_SRC)

import numpy as np

from luggage_perception.cargo_query_export import QueryExportPublisher
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception import cargo_query_snapshot_wire as snapshot_wire
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


def build_store(boxes, resolution):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               resolution, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='c0',
                              require_confirmation=True)
    for i, (z, y, x) in enumerate(
            (z, y, x) for z in range(4)
            for y in (-.48, .18) for x in (-.5, 0., .5)):
        if i >= boxes:
            break
        result = store.apply_update(MapUpdate(
            operation_id='b%d' % i, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24 * z),
                 (.35, .53, .22), 0.), box_id='box-%d' % i))
        assert result.accepted, result.reason
    return store, mapper


def snapshot_data(store):
    store._snapshot = store._build_snapshot()
    with store.acquire_query_snapshot() as handle:
        return handle._data()


def block_rebuilds(before, after):
    return sum(1 for a, b in zip(before.blocks, after.blocks)
               if a is not b), \
        sum(1 for a, b in zip(before.blocks, after.blocks)
            if a.evidence != b.evidence or a.frontier != b.frontier)


class JsonCounter:
    def __init__(self):
        self.count = 0

    def __enter__(self):
        self._original = snapshot_wire.encode_json_object
        outer = self

        def counting(value):
            outer.count += 1
            return self._original(value)
        snapshot_wire.encode_json_object = counting
        return self

    def __exit__(self, *_):
        snapshot_wire.encode_json_object = self._original
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--boxes', type=int, default=0)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    # require_confirmation=True: a CONTAINER_POSE fact then records a
    # pending confirmation (token non-None) and advances the revision
    # with zero voxel mutation -- the real metadata-only transaction.
    store, mapper = build_store(args.boxes, args.resolution)
    root = tempfile.mkdtemp(prefix='c0-')
    publisher = QueryExportPublisher(root)
    report = {'resolution': args.resolution, 'boxes': args.boxes}

    # Baseline snapshot + FULL publish.
    data0 = snapshot_data(store)
    result = publisher.offer_committed(data0, dirty_blocks=None)
    report['initial_full'] = result
    identity0 = tuple(data0.identity)
    n_blocks = len(data0.blocks)

    # --- Case 1a: replay the SAME operation id -----------------------
    # The replayed operation must be one that was genuinely accepted
    # before (with boxes=0 the setup loop added nothing, so add one now
    # and rebuild the baseline identity around it).
    if not args.boxes:
        first_add = store.apply_update(MapUpdate(
            operation_id='seed-0', kind=UpdateKind.ADD_BOX_V1,
            box=((-0.5, -0.48, mapper._hull.floor_z + 0.11),
                 (.35, .53, .22), 0.), box_id='box-0'))
        assert first_add.accepted, first_add.reason
        data0 = snapshot_data(store)
        result = publisher.offer_committed(data0, dirty_blocks=None)
        report['initial_full'] = result
        identity0 = tuple(data0.identity)
    replay = store.apply_update(MapUpdate(
        operation_id='b0' if args.boxes else 'seed-0',
        kind=UpdateKind.ADD_BOX_V1,
        box=((-0.5, -0.48, mapper._hull.floor_z + 0.11),
             (.35, .53, .22), 0.), box_id='box-0'))
    data_after = snapshot_data(store)
    report['case_1a_replay'] = {
        'status': replay.status, 'reason': replay.reason,
        'identity_changed': tuple(data_after.identity) != identity0,
        'blocks_rebuilt': block_rebuilds(data0, data_after),
    }

    # --- Case 1b: metadata-only (NOTE_EVENT confirmation) ------------
    data_before = data_after
    # CONTAINER_POSE is the real metadata-only path: "a confirmation
    # transition with no voxel mutation" (occupancy_map_store). NOTE_EVENT
    # with an arbitrary name is rejected by the pending vocabulary.
    event = store.apply_update(MapUpdate(
        operation_id='note-1', kind=UpdateKind.CONTAINER_POSE,
        event_stamp_ns=1_234_567))
    data1b = snapshot_data(store)
    rebuilt, changed = block_rebuilds(data_before, data1b)
    with JsonCounter() as counter:
        objects = snapshot_wire.encode_snapshot(
            data1b, 'inc', publisher._seq + 1, dirty_blocks=set())
    publish = publisher.offer_committed(
        data1b, dirty_blocks=set(),
        provenance_changed=(tuple(data1b.identity) != identity0))
    report['case_1b_metadata_only'] = {
        'status': event.status, 'reason': event.reason,
        'identity_changed': tuple(data1b.identity) != tuple(
            data_before.identity),
        'blocks_rebuilt_identity': rebuilt,
        'blocks_changed_bytes': changed,
        'json_payloads_constructed': counter.count,
        'json_payloads_emitted': len(objects),
        'publish': publish,
        'note': 'receiver gen-1 does a FULL rebuild_snapshot per '
                'generation regardless of dirty set (known structural '
                'gap; counted here as %d block rebuild iterations on '
                'receive)' % n_blocks,
    }

    # --- Case 1c: new stamp, same cloud ------------------------------
    points = np.column_stack([
        np.random.default_rng(3).uniform(-0.6, 0.6, 30000),
        np.random.default_rng(4).uniform(-0.8, 0.8, 30000),
        np.random.default_rng(5).uniform(
            mapper._hull.floor_z + .1,
            mapper._hull.floor_z + 1.2, 30000)])

    def integrate(stamp):
        return store.apply_update(MapUpdate(
            operation_id='cloud-%d' % stamp,
            kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store.identity,
            geometry_hash=mapper.geometry_hash,
            points=points, origin=(0.0, 0.0, mapper._hull.ceiling_z - 0.1),
            minimum_z=mapper._hull.floor_z + .05,
            stamp_key=(stamp, 0), acquisition_stamp_ns=1_000_000_000 + stamp,
            settled=True, schema_version=1))

    first = integrate(1)
    data_c1 = snapshot_data(store)
    second = integrate(2)
    data_c2 = snapshot_data(store)
    rebuilt, changed = block_rebuilds(data_c1, data_c2)
    report['case_1c_same_cloud_new_stamp'] = {
        'first': {'status': first.status, 'reason': first.reason},
        'second': {'status': second.status, 'reason': second.reason},
        'identity_changed': tuple(data_c2.identity)
        != tuple(data_c1.identity),
        'blocks_rebuilt_identity': rebuilt,
        'blocks_changed_bytes': changed,
    }

    publisher.close()
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
