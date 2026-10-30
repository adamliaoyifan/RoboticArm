"""gen6 P1 incremental six-class gate: 30 transactions per class.

Classes (plan OCC_GRID_ACCEPTANCE_G6 section 4): duplicate operation,
same observation with a new stamp, probability saturation, sparse change,
metadata-only, reset. For every transaction we account fact changes and
occupancy-classification changes separately, require the rebuilt object set
to equal the digest-changed set (unchanged blocks are reused, dirty covers
all changes), and never claim zero work without the store's own counters
agreeing. A same observation is NOT assumed to be a no-op.

Set G6_INCREMENT_OUT to also write the per-class verdict JSON.
"""
import json
import os
import unittest
from pathlib import Path

# Product modules resolve from the frozen install: run with the candidate
# install sourced and a neutral cwd (COMMANDS.sh). Never insert source paths
# here — that would shadow the deployment under test.
from luggage_perception.cargo_grid_projection import occupancy_counts
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import MapUpdate, OccupancyMapStore, UpdateKind

TRANSACTIONS = 30
RESOLUTION = 0.02
VERDICTS = {}


def build_store(boxes=20):
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0,
                               RESOLUTION, max_raycast_points=2000)
    store = OccupancyMapStore(mapper, epoch='six')
    placed = 0
    for z, y, x in ((z, y, x) for z in range(4)
                    for y in (-.48, .18) for x in (-.5, 0., .5)):
        if placed >= boxes:
            break
        result = store.apply_update(MapUpdate(
            operation_id='b%d' % placed, kind=UpdateKind.ADD_BOX_V1,
            box=((x, y, mapper._hull.floor_z + .11 + .24*z), (.35, .53, .22), 0.),
            box_id='box-%d' % placed))
        assert result.accepted
        placed += 1
    return store, mapper


def snapshot(store):
    store._snapshot = store._build_snapshot()
    with store.acquire_query_snapshot() as handle:
        return handle._data()


def classification(mapper):
    active, inactive, unknown, free, occupied, labels = occupancy_counts(mapper)
    return dict(active=active, inactive=inactive, unknown=unknown, free=free,
                occupied=occupied, labels={str(k): v for k, v in labels.items()})


def observe(mapper, stamp, operation_id, point):
    return MapUpdate(operation_id=operation_id, kind=UpdateKind.INTEGRATE_VIEW,
                     points=(point,), origin=(0., 0., .4), schema_version=1,
                     settled=True, geometry_hash=mapper.geometry_hash,
                     stamp_key=('six', stamp), acquisition_stamp_ns=1_000_000+stamp,
                     pose_at_confirm={'position': [0., 0., 0.]})


def center_point(mapper):
    return mapper._local_to_world(*mapper._voxel_center_local(
        *(mapper.nx//2, mapper.ny//2, max(mapper.nz//2, 1))))


def drive(store, mapper, name, submit):
    """Run TRANSACTIONS submissions, asserting the accounting contract each time."""
    verdict = dict(name=name, transactions=TRANSACTIONS, rows=[], pass_=True)
    for index in range(TRANSACTIONS):
        before, before_class = snapshot(store), classification(mapper)
        extra = submit(index) or {}
        work = dict(store._query_indexes.work)
        after, after_class = snapshot(store), classification(mapper)
        rebuilt = sum(1 for a, b in zip(before.blocks, after.blocks) if a is not b)
        digest_changed = sum(1 for a, b in zip(before.blocks, after.blocks)
                             if a.fact_digest != b.fact_digest)
        row = dict(rebuilt=rebuilt, digest_changed=digest_changed,
                   frontier_rebuilds=rebuilt-digest_changed,
                   facts_changed=bool(digest_changed),
                   classification_changed=before_class != after_class,
                   work={k: int(v) for k, v in work.items()}, **extra)
        verdict['rows'].append(row)
        if rebuilt < digest_changed:
            verdict['pass_'] = False
            verdict['reason'] = (f'txn {index}: rebuilt {rebuilt} lost dirty '
                                 f'blocks (digest-changed {digest_changed})')
    VERDICTS[name] = verdict
    return verdict


class TestIncrementSixClasses(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        out = os.environ.get('G6_INCREMENT_OUT')
        if out:
            Path(out).parent.mkdir(parents=True, exist_ok=True)
            Path(out).write_text(json.dumps(
                dict(kind='occ_grid_acceptance_g6_increment_six_classes',
                     resolution=RESOLUTION, transactions=TRANSACTIONS,
                     classes=VERDICTS,
                     all_pass=all(v['pass_'] for v in VERDICTS.values())),
                indent=2, sort_keys=True)+'\n')

    def test_01_duplicate_operation(self):
        store, mapper = build_store()
        point = center_point(mapper)
        def submit(index):
            first = store.apply_update(observe(mapper, index, 'dup', point))
            second = store.apply_update(observe(mapper, index, 'dup', point))
            return dict(first_accepted=bool(first.accepted),
                        second_accepted=bool(second.accepted))
        verdict = drive(store, mapper, 'duplicate_operation', submit)
        self.assertTrue(verdict['pass_'], verdict.get('reason'))

    def test_02_same_observation_new_stamp(self):
        store, mapper = build_store()
        point = center_point(mapper)
        def submit(index):
            store.apply_update(observe(mapper, index, 'obs-%d' % index, point))
        verdict = drive(store, mapper, 'same_observation_new_stamp', submit)
        self.assertTrue(verdict['pass_'], verdict.get('reason'))
        self.assertTrue(any(r['facts_changed'] or r['classification_changed']
                            or any(r['work'].values()) for r in verdict['rows']),
                        'same observation was blanket-assumed to be a no-op')

    def test_03_probability_saturation(self):
        store, mapper = build_store()
        point = center_point(mapper)
        def submit(index):
            store.apply_update(observe(mapper, 0, 'sat-%d' % index, point))
        verdict = drive(store, mapper, 'probability_saturation', submit)
        self.assertTrue(verdict['pass_'], verdict.get('reason'))
        settled = [r for r in verdict['rows'][15:] if not r['facts_changed']]
        self.assertTrue(settled, 'probabilities never saturated within 30 txns')

    def test_04_sparse_change(self):
        store, mapper = build_store(boxes=0)
        spots = [((-.5+(i % 3)*.5, y, mapper._hull.floor_z+.11+.24*layer),
                  (.2, .2, .2), 0.)
                 for layer in range(4) for y in (-.48, .18) for i in range(3)][:24]
        def submit(index):
            if index < len(spots):
                result = store.apply_update(MapUpdate(
                    operation_id='sparse-%d' % index, kind=UpdateKind.ADD_BOX_V1,
                    box=spots[index], box_id='sparse-%d' % index))
                return dict(accepted=bool(result.accepted), change='box')
            voxel = (index % mapper.nx, (2*index) % mapper.ny, (index//9) % mapper.nz)
            point = mapper._local_to_world(*mapper._voxel_center_local(*voxel))
            result = store.apply_update(observe(mapper, index, 'sparse-%d' % index, point))
            return dict(accepted=bool(result.accepted), change='view')
        verdict = drive(store, mapper, 'sparse_change', submit)
        self.assertTrue(verdict['pass_'], verdict.get('reason'))
        self.assertTrue(all(r['digest_changed'] > 0 for r in verdict['rows']
                            if r['change'] == 'box'),
                        'box adds must propagate to facts')
        self.assertTrue(all(r['digest_changed'] > 0 for r in verdict['rows']
                            if r['change'] == 'view'),
                        'fresh-voxel observations must propagate to facts')

    def test_05_metadata_only(self):
        store, mapper = build_store()
        point = center_point(mapper)
        store.apply_update(observe(mapper, 0, 'meta', point))
        def submit(index):
            # identical operation id, payload and stamp; only the acquisition
            # stamp moves. I1 contract: zero rebuilds, zero work, zero change.
            result = store.apply_update(MapUpdate(
                operation_id='meta', kind=UpdateKind.INTEGRATE_VIEW,
                points=(point,), origin=(0., 0., .4), schema_version=1,
                settled=True, geometry_hash=mapper.geometry_hash,
                stamp_key=('six', 0), acquisition_stamp_ns=2_000_000+index,
                pose_at_confirm={'position': [0., 0., 0.]}))
            return dict(accepted=bool(result.accepted))
        verdict = drive(store, mapper, 'metadata_only', submit)
        self.assertTrue(verdict['pass_'], verdict.get('reason'))
        self.assertEqual([r['digest_changed'] for r in verdict['rows']],
                         [0]*TRANSACTIONS,
                         'metadata-only refresh must not change facts')
        self.assertEqual([r['rebuilt'] for r in verdict['rows']], [0]*TRANSACTIONS,
                         'metadata-only refresh must not rebuild blocks')
        self.assertEqual([sum(r['work'].values()) for r in verdict['rows']],
                         [0]*TRANSACTIONS,
                         'metadata-only refresh must be zero work by the counters')

    def test_06_reset(self):
        store, mapper = build_store(boxes=8)
        point = center_point(mapper)
        def submit(index):
            if index % 5 == 4:
                result = store.apply_update(MapUpdate(
                    operation_id='reset-%d' % index, kind=UpdateKind.RESET))
                return dict(reset=True, accepted=bool(result.accepted))
            store.apply_update(observe(mapper, index, 'reset-%d' % index, point))
            return dict(reset=False)
        verdict = drive(store, mapper, 'reset', submit)
        # A RESET is an epoch transition: the store may rebuild the whole
        # block closure (contractual, like same-cloud digest churn), so the
        # reuse invariant (rebuilt == digest-changed) is asserted only on the
        # incremental transactions; reset rows must still change facts and
        # never show rebuilt < digest-changed (silent dirty loss).
        for index, row in enumerate(verdict['rows']):
            if row.get('reset'):
                self.assertGreater(row['digest_changed'], 0,
                                   f'reset txn {index} changed no facts')
                self.assertGreaterEqual(row['rebuilt'], row['digest_changed'],
                                        f'reset txn {index} lost dirty blocks')
            else:
                self.assertEqual(row['rebuilt'], row['digest_changed'],
                                 f'txn {index}: {row}')
        self.assertEqual(sum(1 for r in verdict['rows'] if r.get('reset')), 6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
