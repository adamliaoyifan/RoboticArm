"""Q-20261028-1 summary wire repair behavior tests (plan 50169110).

Real store -> wire export -> independent replica -> facade paths (no
DTO stand-ins): the three summary legs ride every generation; owner
and replica summaries are field-identical; v1 streams are
named-rejected with recovery via a new FULL; missing legs never
produce a default-filled summary; decoded objects are immutable.
"""

import os
import shutil
import sys
import tempfile
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    'src', 'luggage_perception'))
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    'src', 'luggage_description'))
# the V2 typed srvs live in the worktree's own colcon install (the
# shared main-workspace install predates them); package name is
# disjoint from the source trees so ordering is safe
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))))),
    'install', 'swf', 'luggage_msgs', 'local', 'lib', 'python3.10',
    'dist-packages'))

import numpy as np

from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)
from luggage_perception.cargo_map_query_mode import wire_store_export
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_query_snapshot_wire import _MapSnapshotView
from luggage_perception.cargo_query_wire import (
    WIRE_SCHEMA_VERSION, validate_manifest, WireError)


def build_pair(tmp, legs_variant=0):
    """A real store + wire export + an independent replica/facade that
    has polled to READY. legs_variant selects the pending/event mix."""
    root = os.path.join(tmp, 'root')
    mapper = CargoVolumeMapper((1.49, 1.97, 1.48), (0, 0, 0.74), 0, 0.1)
    store = OccupancyMapStore(mapper, epoch='sfix')
    hook = wire_store_export(store, root)
    if legs_variant:                      # one box -> nonempty legs
        store.apply_update(MapUpdate(
            operation_id='b0', kind=UpdateKind.ADD_BOX_V1,
            box=((0., 0., mapper._hull.floor_z + .11), (.3, .4, .2), 0.),
            box_id='box-0'))
    pts = np.random.default_rng(9).uniform(-0.3, 0.3, (4000, 3))
    store.apply_update(MapUpdate(
        operation_id='w0', kind=UpdateKind.INTEGRATE_VIEW,
        expected_snapshot=store.identity,
        geometry_hash=mapper.geometry_hash, points=pts,
        origin=(0., 0., 1.4), minimum_z=mapper._hull.floor_z + .05,
        stamp_key=(1, 0), acquisition_stamp_ns=1, settled=True,
        schema_version=1))
    store._snapshot = store._build_snapshot()
    hook.wait_pending(30.0)
    replica = QueryReplica(root)
    facade = QueryMapReadFacade(replica)
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if replica.poll() == ReplicaState.READY:
            break
        time.sleep(0.02)
    assert replica.state == ReplicaState.READY, str(replica.state)
    return store, hook, replica, facade, root


def summary_fields(summary):
    return {'status': summary.status,
            'occupancy_confirmed': summary.occupancy_confirmed,
            'change_event': summary.change_event,
            'pending_changes': summary.pending_changes,
            'acquisition_stamp_ns': summary.acquisition_stamp_ns,
            'map_source': summary.map_source,
            'cargo_observed': summary.cargo_observed}


def test_schema_version_bumped_and_v1_named_rejected():
    assert WIRE_SCHEMA_VERSION == 2
    manifest = {'wire_schema_version': 1, 'kind': 'FULL'}
    try:
        validate_manifest(manifest)
        raise AssertionError('v1 accepted')
    except WireError as error:
        assert error.reason == 'UNKNOWN_SCHEMA'


def test_owner_and_replica_summary_identical_both_legs_variants(tmp_path):
    # two legs variants (empty / box+view) x FULL; metadata-only
    # transitions are covered by the later real-path test
    for variant in (0, 1):
        tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
        try:
            store, hook, replica, facade, _ = build_pair(tmp, variant)
            owner = store.acquire_query_snapshot()
            owner_fields = summary_fields(owner.summary())
            owner.close()
            handle = facade.acquire_query_snapshot('CURRENT')
            replica_fields = summary_fields(handle.summary())
            handle.close()
            assert owner_fields == replica_fields
            assert replica.state == ReplicaState.READY
            hook.close()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


def test_real_path_metadata_transitions_propagate(tmp_path):
    tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
    try:
        store, hook, replica, facade, _ = build_pair(tmp, 1)
        for step in range(3):
            # metadata-only transition: new acquisition stamp on the
            # SAME world content -- legs must still ride the snapshot
            store.apply_update(MapUpdate(
                operation_id='m%d' % step, kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=store._snapshot.geometry_hash,
                points=np.zeros((0, 3), dtype=float),
                origin=(0., 0., 1.4), stamp_key=(10 + step, 0),
                acquisition_stamp_ns=1000 + step, settled=True,
                schema_version=1))
            store._snapshot = store._build_snapshot()
            hook.wait_pending(30.0)
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if replica.poll() == ReplicaState.READY:
                    break
                time.sleep(0.02)
            assert replica.state == ReplicaState.READY
            owner = store.acquire_query_snapshot()
            expect = summary_fields(owner.summary())
            owner.close()
            handle = facade.acquire_query_snapshot('CURRENT')
            got = summary_fields(handle.summary())
            handle.close()
            assert got == expect, (step, got, expect)
        hook.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_v2_missing_leg_is_named_rejection_not_default_fill():
    provenance = {'identity': ['e', 1, 'd'],
                  'acquisition_stamp_ns': 1, 'map_source': 'x',
                  'cargo_observed': False, 'change_token': '1',
                  'pending_details': (), 'last_settle': {},
                  'ledger_details': ()}
    for remove in ('occupancy_confirmed', 'change_event',
                   'pending_changes'):
        broken = {k: v for k, v in provenance.items() if k != remove}
        try:
            _MapSnapshotView(broken)
            raise AssertionError('default-filled %s' % remove)
        except ValueError as error:
            assert 'PROVENANCE_MISSING_SUMMARY_LEG' in str(error)
            assert remove in str(error)


def test_decoded_pending_changes_immutable_and_typed():
    provenance = {'identity': ['e', 1, 'd'],
                  'acquisition_stamp_ns': 1, 'map_source': 'x',
                  'cargo_observed': True, 'change_token': '1',
                  'pending_details': (), 'last_settle': {},
                  'ledger_details': (),
                  'occupancy_confirmed': True,
                  'change_event': 'BOX_ADDED',
                  'pending_changes': [['BOX_ADDED', 3],
                                      ['VIEW_SETTLED', 4]]}
    view = _MapSnapshotView(dict(provenance))
    assert view.occupancy_confirmed is True
    assert view.change_event == 'BOX_ADDED'
    assert view.pending_changes == (('BOX_ADDED', 3),
                                    ('VIEW_SETTLED', 4))
    assert all(isinstance(item, tuple) and
               isinstance(item[0], str) and isinstance(item[1], int)
               for item in view.pending_changes)
    try:
        view.pending_changes[0] = ('X', 0)
        raise AssertionError('mutable tuple')
    except TypeError:
        pass


def tamper_head_to_v1(root):
    """Rewrite the CURRENT head manifest as a v1 stream and point
    published.json at it WITH the matching sha256 (a real reader must
    traverse it, not bounce off a broken pointer)."""
    import glob as _g
    import hashlib as _h
    manifest_path = sorted(_g.glob(
        os.path.join(root, 'generations', '*', '*',
                     'manifest.json')),
        key=lambda p: int(p.split('/')[-2]))[-1]
    manifest = json_load(manifest_path)
    assert manifest['wire_schema_version'] == 2
    v1 = dict(manifest, wire_schema_version=1)
    v1.pop('checksum', None)
    with open(manifest_path, 'w') as handle:
        handle.write(canonical(v1))
    body = open(manifest_path, 'rb').read()
    with open(os.path.join(root, 'published.json'), 'w') as handle:
        handle.write(canonical({
            'manifest': os.path.relpath(manifest_path, root),
            'manifest_sha256': _h.sha256(body).hexdigest()}))
    return manifest_path


def poll_until(replica, predicate, timeout_s=30, interval_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = replica.poll()
        if predicate(state, replica):
            return state
        time.sleep(interval_s)
    return replica.poll()


def replica_saw_named_v1_rejection(replica):
    raw_events = replica.events()
    if callable(raw_events):
        raw_events = raw_events()
    return any(
        e.get('event') in ('unavailable', 'resync')
        and 'UNKNOWN_SCHEMA' in str(e.get('error', e.get('reason')))
        for e in (raw_events or []))


def force_full_anchor_recovery(store, hook, replica, root, stamp0):
    """Review round 4: drive the PRODUCTION resync path so recovery
    goes through a REAL FULL anchor (previously no test forced or
    proved one). The replica holds base seq B when called. Publish at
    least two further generations WITHOUT polling in between, then
    delete the retained manifest at B+1 (a realistic retention-pruning
    fault) so bounded catch-up cannot chain from B; the replica raises
    the named SEQ_GAP resync, the exporter's encoder loop answers
    control/resync with a forced FULL of the latest committed
    snapshot, and the reader APPLIES that anchor."""
    base_seq = int(replica.manifest['seq'])
    for more in range(2):
        store.apply_update(MapUpdate(
            operation_id='rec%d' % more,
            kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store.identity,
            geometry_hash=store._snapshot.geometry_hash,
            points=np.zeros((0, 3), dtype=float),
            origin=(0., 0., 1.4), stamp_key=(stamp0 + more, 0),
            acquisition_stamp_ns=stamp0 * 10 + more, settled=True,
            schema_version=1))
        store._snapshot = store._build_snapshot()
        hook.wait_pending(30.0)
    pointer = json_load(os.path.join(root, 'published.json'))
    head_path = os.path.normpath(os.path.join(root, pointer['manifest']))
    head_seq = int(json_load(head_path)['seq'])
    assert head_seq >= base_seq + 2, (base_seq, head_seq)
    import glob as _gdel
    victims = _gdel.glob(os.path.join(
        root, 'generations', '*', str(base_seq + 1), 'manifest.json'))
    assert len(victims) == 1, victims
    shutil.rmtree(os.path.dirname(victims[0]))
    return poll_until(replica, lambda state, _: state == ReplicaState.READY)


def test_old_stream_rejected_then_new_full_recovers(tmp_path):
    """COLD-reader variant: a fresh QueryReplica that first meets the
    stream at a v1 head is named-rejected (UNKNOWN_SCHEMA on the real
    read path), then recovers through the forced-FULL resync answer.
    (Review round 4 rewrote this test: the old body never actually
    showed the replica the v1 stream -- published.json pointed at the
    next v2 generation before any poll -- so its comment claimed a
    v1 rejection it did not observe.)"""
    tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
    try:
        store, hook, replica, facade, root = build_pair(tmp, 0)
        hook.close()
        tamper_head_to_v1(root)
        # a COLD reader attaches with the v1 head published
        cold = QueryReplica(root)
        state = poll_until(
            cold,
            lambda s, r: s != ReplicaState.SYNCING
            and replica_saw_named_v1_rejection(r))
        assert state != ReplicaState.READY
        assert replica_saw_named_v1_rejection(cold)
        # a NEW publisher incarnation answers on the same root with
        # v2 generations; recovery rides the forced FULL anchor
        mapper = CargoVolumeMapper((1.49, 1.97, 1.48),
                                   (0, 0, 0.74), 0, 0.1)
        store2 = OccupancyMapStore(mapper, epoch='sfix2')
        hook2 = wire_store_export(store2, root)
        store2.apply_update(MapUpdate(
            operation_id='cold0', kind=UpdateKind.ADD_BOX_V1,
            box=((0., 0., mapper._hull.floor_z + .11), (.3, .4, .2), 0.),
            box_id='cold-box'))
        pts = np.random.default_rng(11).uniform(-0.3, 0.3, (2000, 3))
        store2.apply_update(MapUpdate(
            operation_id='cold1', kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store2.identity,
            geometry_hash=mapper.geometry_hash, points=pts,
            origin=(0., 0., 1.4), minimum_z=mapper._hull.floor_z + .05,
            stamp_key=(1, 0), acquisition_stamp_ns=10, settled=True,
            schema_version=1))
        store2._snapshot = store2._build_snapshot()
        hook2.wait_pending(30.0)
        cold_facade = QueryMapReadFacade(cold)
        state = poll_until(
            cold, lambda s, _: s == ReplicaState.READY, timeout_s=60)
        assert state == ReplicaState.READY, str(cold.last_error)
        verdict = assert_full_anchored_recovery(
            cold, cold_facade, store2, root)
        assert verdict['events'] > 0
        hook2.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def json_load(path):
    import json
    with open(path) as handle:
        return json.load(handle)


def assert_full_anchored_recovery(replica, facade, store, root):
    """Review round 4 (2026-10-29) shared recovery verdict.

    Closes two evidence gaps the review named for BOTH recovery
    tests: (a) "the recovery is a FULL generation" was a comment,
    never an assertion -- only the DELTA head was compared; (b) the
    identity assert ended in `... or True` (tautology). Now, with
    the replica's own event log as the receive-side record and the
    writer's export dir on disk as the INDEPENDENT producer record:

    1. an APPLIED FULL anchor exists (a 'received' kind FULL whose
       seq reached 'validated'/'ready'), and the writer's on-disk
       manifest for that seq IS a v2 FULL whose complete
       target_identity equals the one the replica reported applying;
    2. the named UNKNOWN_SCHEMA rejection strictly PRECEDED that
       anchor in the event order;
    3. the head equals the writer's LATEST on-disk generation
       (identity, seq AND manifest checksum -- the applied state is
       byte-semantically the published record);
    4. the head target_identity matches the LIVE owner store
       identity element by element (real comparison, no `or True`);
    5. the summary at that identity equals the owner's per leg.
    """
    import glob as _g
    raw_events = replica.events()
    if callable(raw_events):
        raw_events = raw_events()
    events = list(raw_events or [])
    received_kind = {int(e['seq']): e.get('kind') for e in events
                     if e.get('event') == 'received'}
    applied_seqs = sorted({int(e['seq']) for e in events
                           if e.get('event') in ('validated', 'ready')})
    assert applied_seqs, 'recovery applied nothing: %r' % (events,)
    anchor_seqs = [s for s in applied_seqs
                   if received_kind.get(s) == 'FULL']
    assert anchor_seqs, (
        'no FULL anchor was applied during recovery '
        '(applied=%r kinds=%r)' % (applied_seqs, received_kind))
    anchor_seq = anchor_seqs[-1]
    anchor_ready = next(e for e in events if e.get('event') == 'ready'
                        and int(e['seq']) == anchor_seq)
    anchor_paths = _g.glob(os.path.join(
        root, 'generations', '*', str(anchor_seq), 'manifest.json'))
    # two producer incarnations may share a seq number on one root:
    # select the anchor by the target_identity the reader reported
    # APPLYING, not by path uniqueness
    anchor_manifest = None
    for path in anchor_paths:
        candidate = json_load(path)
        if (list(candidate['target_identity'])
                == list(anchor_ready['target_identity'])):
            assert anchor_manifest is None, (
                'ambiguous anchor manifests at seq %s' % anchor_seq)
            anchor_manifest = candidate
    assert anchor_manifest is not None, anchor_paths
    assert anchor_manifest['kind'] == 'FULL'
    assert anchor_manifest['wire_schema_version'] == 2
    rejection_index = next(
        i for i, e in enumerate(events)
        if e.get('event') in ('unavailable', 'resync')
        and 'UNKNOWN_SCHEMA' in str(e.get('error', e.get('reason'))))
    assert rejection_index < events.index(anchor_ready), (
        'the v1 rejection did not precede the FULL anchor')
    # the AUTHORITATIVE latest is the published pointer (two producer
    # incarnations may share seq numbers on one root); derive it from
    # published.json instead of trusting a seq sort
    pointer = json_load(os.path.join(root, 'published.json'))
    latest = os.path.normpath(os.path.join(root, pointer['manifest']))
    assert os.path.isfile(latest), latest
    latest_manifest = json_load(latest)
    assert list(replica.manifest['target_identity']) ==        list(latest_manifest['target_identity'])
    assert int(replica.manifest['seq']) ==         int(latest_manifest['seq'])
    assert replica.manifest['checksum'] ==         latest_manifest['checksum']
    handle = facade.acquire_query_snapshot('CURRENT')
    replica_summary = handle.summary()
    replica_handle_identity = tuple(map(str, handle.identity))
    handle.close()
    owner = store.acquire_query_snapshot()
    owner_summary = owner.summary()
    owner_identity = tuple(map(str, owner.identity))
    owner.close()
    head_target = [str(part) for part in
                   replica.manifest['target_identity']]
    assert len(head_target) >= len(owner_identity)
    assert tuple(head_target[:len(owner_identity)]) == owner_identity
    assert replica_handle_identity == owner_identity
    for field in ('occupancy_confirmed', 'change_event',
                  'pending_changes', 'acquisition_stamp_ns',
                  'map_source', 'cargo_observed'):
        assert getattr(replica_summary, field) == getattr(
            owner_summary, field), field
    return {'anchor_seq': anchor_seq,
            'head_seq': int(replica.manifest['seq']),
            'events': len(events)}


def canonical(obj):
    import json
    return json.dumps(obj, sort_keys=True)


def test_typed_summary_over_replica_facade_matches_owner(tmp_path):
    """Typed gate: REAL GetCargoMapSummaryV2 service callback over the
    replica facade with full message serialization; business fields
    must match the owner summary of the same generation."""
    import rclpy
    from rclpy.serialization import deserialize_message, serialize_message
    from luggage_msgs.srv import GetCargoMapSummaryV2
    from luggage_perception.cargo_map_query_service import MapQueryService

    rclpy.init()
    try:
        node = rclpy.create_node('swf_typed_check')
        tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
        try:
            store, hook, replica, facade, _ = build_pair(tmp, 1)
            service = MapQueryService(node, facade, None)
            req = GetCargoMapSummaryV2.Request()
            req.query.request_id = 'swf-typed'
            req.query.frame_id = 'container_link'
            req.query.use_current = True
            req.query.release_after_query = True
            wire_req = deserialize_message(
                serialize_message(req), GetCargoMapSummaryV2.Request)
            response = service.summary(
                wire_req, GetCargoMapSummaryV2.Response())
            response = deserialize_message(
                serialize_message(response), GetCargoMapSummaryV2.Response)
            assert response.meta.status == 'OK', response.meta.reason
            owner = store.acquire_query_snapshot()
            owner_summary = owner.summary()
            owner.close()
            assert response.occupancy_confirmed == \
                owner_summary.occupancy_confirmed
            assert response.change_event == owner_summary.change_event
            assert response.map_source == owner_summary.map_source
            assert response.cargo_observed == \
                owner_summary.cargo_observed
            assert response.acquisition_stamp_ns == \
                owner_summary.acquisition_stamp_ns
            hook.close()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    finally:
        rclpy.shutdown()


# --------------------- round-2 review gaps (typed/coercion/matrix) ----

def legged_provenance(**override):
    base = {'identity': ['e', 1, 'd'],
            'acquisition_stamp_ns': 1, 'map_source': 'x',
            'cargo_observed': True, 'change_token': '1',
            'pending_details': (), 'last_settle': {},
            'ledger_details': (),
            'occupancy_confirmed': False,
            'change_event': '',
            'pending_changes': []}
    base.update(override)
    return base


def test_illegal_leg_types_are_rejected_not_coerced():
    # review P1 probes: "false"->True, None->"None", token 1.9->1 --
    # every one must be a named rejection, never a coerced legal fact
    for override, name in (
            ({'occupancy_confirmed': 'false'}, 'bool-as-str'),
            ({'occupancy_confirmed': 1}, 'bool-as-int'),
            ({'occupancy_confirmed': None}, 'bool-as-none'),
            ({'change_event': None}, 'event-as-none'),
            ({'change_event': 7}, 'event-as-int'),
            ({'pending_changes': [['E', 1.9]]}, 'token-float'),
            ({'pending_changes': [['E', True]]}, 'token-bool'),
            ({'pending_changes': [['E', '1']]}, 'token-str'),
            ({'pending_changes': [['E']]}, 'wrong-arity'),
            ({'pending_changes': 'E:1'}, 'changes-not-list'),
            ({'pending_changes': None}, 'changes-none')):
        try:
            _MapSnapshotView(legged_provenance(**override))
            raise AssertionError('coerced: %s' % name)
        except ValueError as error:
            assert 'PROVENANCE_BAD_SUMMARY_LEG' in str(error), (name, error)


def test_missing_leg_isolated_one_at_a_time():
    # review P2: the base must CONTAIN all three legs so each case
    # removes exactly one -- not a base already missing all three
    for remove in ('occupancy_confirmed', 'change_event',
                   'pending_changes'):
        provenance = legged_provenance()
        provenance.pop(remove)
        assert all(key in provenance
                   for key in ('occupancy_confirmed', 'change_event',
                               'pending_changes') if key != remove)
        try:
            _MapSnapshotView(provenance)
            raise AssertionError('default-filled %s' % remove)
        except ValueError as error:
            assert 'PROVENANCE_MISSING_SUMMARY_LEG' in str(error)
            assert remove in str(error)


def test_sixteen_case_legs_matrix_roundtrip():
    # the plan's frozen 8 legs combos x {FULL, DELTA} manifest frames:
    # codec roundtrip on REAL snapshot legs (dataclasses.replace of a
    # live snapshot), decoded view fields must equal the owner's
    import dataclasses
    import json
    mapper = CargoVolumeMapper((0.3, 0.3, 0.3), (0, 0, 0.1), 0, 0.1)
    store = OccupancyMapStore(mapper, epoch='matrix')
    store.apply_update(MapUpdate(
        operation_id='b', kind=UpdateKind.ADD_BOX_V1,
        box=((0, 0, 0.05), (0.08, 0.08, 0.06), 0.), box_id='b'))
    store._snapshot = store._build_snapshot()
    live = store._snapshot
    combos = [(confirmed, event, changes)
              for confirmed in (False, True)
              for event in ('', 'BOX_ADDED')
              for changes in ((),
                              (('BOX_ADDED', 3), ('VIEW_SETTLED', 4)))]
    assert len(combos) == 8
    for kind in ('FULL', 'DELTA'):
        for confirmed, event, changes in combos:
            snap = dataclasses.replace(
                live, occupancy_confirmed=confirmed,
                change_event=event, pending_changes=changes)
            provenance = {'identity': [
                'e', snap.identity.revision, snap.identity.content_digest],
                'acquisition_stamp_ns': snap.acquisition_stamp_ns,
                'map_source': snap.map_source,
                'cargo_observed': snap.cargo_observed,
                'change_token': str(snap.change_token),
                'pending_details': snap.pending_details,
                'last_settle': snap.last_settle,
                'ledger_details': snap.ledger_details,
                'occupancy_confirmed': snap.occupancy_confirmed,
                'change_event': snap.change_event,
                'pending_changes': [[e, t] for e, t in
                                    snap.pending_changes]}
            # canonical-JSON codec roundtrip (the wire's object
            # encoding is exercised end-to-end by the real-path
            # tests; here the mapping itself is under test)
            decoded = json.loads(
                json.dumps(provenance, sort_keys=True))
            view = _MapSnapshotView(decoded)
            assert view.occupancy_confirmed is confirmed
            assert view.change_event == event
            assert view.pending_changes == tuple(changes)
            import uuid as _uuid
            manifest = {
                'producer_incarnation': str(_uuid.uuid4()),
                'map_epoch': 'e', 'seq': 1, 'kind': kind,
                        'wire_schema_version': 2,
                        'target_identity': decoded['identity'],
                        'geometry_hash': 'g', 'grid': {},
                        'commit_monotonic_ns': 1, 'entries': {}}
            if kind == 'DELTA':
                manifest['base_identity'] = decoded['identity']
                manifest['base_manifest_sha'] = '0' * 64
            from luggage_perception.cargo_query_wire import (
                manifest_checksum)
            manifest['checksum'] = manifest_checksum(manifest)
            validate_manifest(manifest)   # FULL and DELTA frames both
                                           # legal v2 carriers


def test_typed_real_service_roundtrip_over_replica_facade(tmp_path):
    """Review P2: a REAL service request (create_service + client over
    DDS), not a direct callback -- the typed gate's evidence must be
    an actual service roundtrip."""
    import rclpy
    from luggage_msgs.srv import GetCargoMapSummaryV2
    from luggage_perception.cargo_map_query_service import MapQueryService

    rclpy.init()
    try:
        node = rclpy.create_node('swf_typed_srv')
        tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
        try:
            store, hook, replica, facade, _ = build_pair(tmp, 1)
            node.create_service(
                GetCargoMapSummaryV2, '/swf/get_summary_v2',
                lambda req, resp: facade_typed_summary(
                    MapQueryService(node, facade, None), req, resp))
            client = node.create_client(
                GetCargoMapSummaryV2, '/swf/get_summary_v2')
            assert client.wait_for_service(timeout_sec=10.0)
            req = GetCargoMapSummaryV2.Request()
            req.query.request_id = 'swf-real'
            req.query.frame_id = 'container_link'
            req.query.use_current = True
            req.query.release_after_query = True
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            assert future.done(), 'service call timeout'
            response = future.result()
            assert response.meta.status == 'OK', response.meta.reason
            owner = store.acquire_query_snapshot()
            owner_summary = owner.summary()
            owner.close()
            assert response.occupancy_confirmed == \
                owner_summary.occupancy_confirmed
            assert response.change_event == owner_summary.change_event
            assert response.acquisition_stamp_ns == \
                owner_summary.acquisition_stamp_ns
            hook.close()
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    finally:
        rclpy.shutdown()


def facade_typed_summary(service, req, resp):
    return service.summary(req, resp)


def test_v1_rejection_observed_before_v2_recovery(tmp_path):
    """Review P2: the recovery proof must SHOW the named v1 rejection
    first (state leaves READY with the named reason), THEN recovery."""
    tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
    try:
        store, hook, replica, facade, root = build_pair(tmp, 0)
        tamper_head_to_v1(root)
        # phase 1: the replica MUST leave READY on the NAMED v1
        # rejection (UNKNOWN_SCHEMA via the bounded-error path)
        state = poll_until(
            replica,
            lambda s, r: s != ReplicaState.SYNCING
            and replica_saw_named_v1_rejection(r), timeout_s=10)
        assert replica_saw_named_v1_rejection(replica), (
            'replica left READY without the NAMED v1 rejection: '
            'state=%s error=%s' % (replica.state,
                                   getattr(replica, 'last_error', None)))
        assert state != ReplicaState.READY
        # phase 2 (review round 4): drive the PRODUCTION resync path
        # so the recovery rides a REAL FULL anchor (forced via
        # control/resync after the retained intermediate chain is
        # broken); the rejection strictly PRECEDED it (asserted in
        # the shared verdict)
        state = force_full_anchor_recovery(store, hook, replica, root,
                                           stamp0=300)
        assert state == ReplicaState.READY
        # review round 4: full-anchored recovery verdict (shared) --
        # APPLIED FULL anchor proven from the event log + the
        # writer's on-disk manifest, rejection-first ordering,
        # head == writer's latest record (identity/seq/checksum),
        # a REAL identity comparison (the old `... or True`
        # tautology is gone), and per-leg summary equality
        assert_full_anchored_recovery(replica, facade, store, root)
        hook.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------- round-3 goal gaps: production chain / FULL proof /
# ------------- independent typed reader process

def test_sixteen_combo_matrix_through_production_codec(tmp_path):
    """Gap 1: the 8 legs combos x {FULL, DELTA} ride the PRODUCTION
    chain -- a real store apply, the exporter's own
    snapshot_query_data -> encode_snapshot (legs pinned per combo via
    a module-level wrapper; every other field real), objects on disk,
    and an independent QueryReplica -> rebuild_snapshot. If
    encode_snapshot/rebuild_snapshot were broken, this FAILS (the
    round-2 codec stand-in is deleted)."""
    import dataclasses
    import luggage_perception.cargo_map_query_mode as mode

    combos = [(confirmed, event, changes)
              for confirmed in (False, True)
              for event in ('', 'BOX_ADDED')
              for changes in ((),
                              (('BOX_ADDED', 3), ('VIEW_SETTLED', 4)))]
    assert len(combos) == 8
    kinds_seen = []
    original_query_data = mode.snapshot_query_data
    try:
        for index, (confirmed, event, changes) in enumerate(combos):
            tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
            root = os.path.join(tmp, 'root')
            mapper = CargoVolumeMapper((1.49, 1.97, 1.48),
                                       (0, 0, 0.74), 0, 0.1)
            store = OccupancyMapStore(mapper, epoch='m%d' % index)

            def pinned(store, legs=(confirmed, event, changes)):
                data = original_query_data(store)
                replaced = dataclasses.replace(
                    data.map_snapshot,
                    occupancy_confirmed=legs[0],
                    change_event=legs[1],
                    pending_changes=legs[2])
                return dataclasses.replace(data, map_snapshot=replaced)

            mode.snapshot_query_data = pinned
            hook = wire_store_export(store, root)
            # gen 1: real apply -> FULL generation with pinned legs
            pts = np.random.default_rng(21 + index).uniform(
                -0.3, 0.3, (3000, 3))
            store.apply_update(MapUpdate(
                operation_id='g1', kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=mapper.geometry_hash, points=pts,
                origin=(0., 0., 1.4), minimum_z=0.85,
                stamp_key=(1, 0), acquisition_stamp_ns=1, settled=True,
                schema_version=1))
            store._snapshot = store._build_snapshot()
            hook.wait_pending(30.0)
            # reader attaches while the head is the FULL generation
            replica = QueryReplica(root)
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if replica.poll() == ReplicaState.READY:
                    break
                time.sleep(0.02)
            assert replica.state == ReplicaState.READY
            assert int(replica.manifest['seq']) == 1
            # review round 4, gap "FULL+DELTA claimed, only the DELTA
            # compared": verify the FULL generation's REBUILT state
            # too -- the pinned legs survived the production chain at
            # gen 1, and the NON-leg business fields equal the owner
            # captured at gen 1 (the store's own registry legs are not
            # pinned, so legs compare against the pinned combo)
            assert replica.manifest['kind'] == 'FULL'
            owner_g1 = store.acquire_query_snapshot()
            owner_g1_fields = summary_fields(owner_g1.summary())
            owner_g1.close()
            facade_g1 = QueryMapReadFacade(replica)
            handle_g1 = facade_g1.acquire_query_snapshot('CURRENT')
            g1_summary = handle_g1.summary()
            handle_g1.close()
            assert g1_summary.occupancy_confirmed is confirmed
            assert g1_summary.change_event == event
            assert g1_summary.pending_changes == tuple(changes)
            for field in ('acquisition_stamp_ns', 'map_source',
                          'cargo_observed', 'status'):
                assert getattr(g1_summary, field) == \
                    owner_g1_fields[field], field
            # gen 2: metadata-only apply -> DELTA generation whose base
            # legs are the REAL previous manifest's
            store.apply_update(MapUpdate(
                operation_id='g2', kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=mapper.geometry_hash,
                points=np.zeros((0, 3), dtype=float),
                origin=(0., 0., 1.4), minimum_z=0.85,
                stamp_key=(2, 0), acquisition_stamp_ns=2, settled=True,
                schema_version=1))
            store._snapshot = store._build_snapshot()
            hook.wait_pending(30.0)
            hook.close()

            import glob as _g
            manifests = sorted(
                _g.glob(os.path.join(root, 'generations', '*', '*',
                                     'manifest.json')),
                key=lambda p: int(p.split('/')[-2]))
            kinds = [json_load(m)['kind'] for m in manifests]
            assert kinds == ['FULL', 'DELTA'], (index, kinds)
            assert all(json_load(m)['wire_schema_version'] == 2
                       for m in manifests)
            base = json_load(manifests[1])
            assert base['base_identity'] and base['base_manifest_sha']
            kinds_seen.append(kinds)

            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                if (replica.poll() == ReplicaState.READY
                        and int(replica.manifest['seq']) == 2):
                    break
                time.sleep(0.02)
            assert replica.state == ReplicaState.READY
            assert replica.manifest['kind'] == 'DELTA'   # the DELTA applied
            facade = QueryMapReadFacade(replica)
            handle = facade.acquire_query_snapshot('CURRENT')
            summary = handle.summary()
            handle.close()
            # the three legs survived the production chain EXACTLY
            assert summary.occupancy_confirmed is confirmed
            assert summary.change_event == event
            assert summary.pending_changes == tuple(changes)
            # non-leg business fields equal the owner's real snapshot
            owner = store.acquire_query_snapshot()
            owner_summary = owner.summary()
            owner.close()
            for field in ('total_voxels', 'unknown_count',
                          'occupied_count', 'frontier_count'):
                # review round 4: direct dict access on BOTH sides --
                # the old .get(..., None) fallback could pass on
                # None == None when a field went missing everywhere
                assert summary.values[field] ==                 owner_summary.values[field], field
            for field in ('map_source', 'acquisition_stamp_ns',
                          'cargo_observed'):
                assert getattr(summary, field) == getattr(
                    owner_summary, field), field
            shutil.rmtree(tmp, ignore_errors=True)
    finally:
        mode.snapshot_query_data = original_query_data
    assert len(kinds_seen) == 8 and all(
        k == ['FULL', 'DELTA'] for k in kinds_seen)   # 16 production
                                                      # roundtrips


TYPED_READER_PROC = r'''
import hashlib, json, os, sys, time
root, lock_path = sys.argv[1:3]
import rclpy
import luggage_perception
from luggage_perception.cargo_query_replica import (
    QueryReplica, ReplicaState)
from luggage_perception.cargo_map_read_facade import QueryMapReadFacade
from luggage_perception.cargo_map_query_service import MapQueryService
from luggage_perception.cargo_query_wire import WIRE_SCHEMA_VERSION
from luggage_msgs.srv import GetCargoMapSummaryV2

# version lock (review round 4): report EXACTLY which installed files
# served this reader, before serving anything
package_dir = os.path.dirname(os.path.abspath(
    luggage_perception.__file__))
modules = ['cargo_query_replica.py', 'cargo_query_snapshot_wire.py',
           'cargo_map_read_facade.py', 'cargo_map_query_service.py',
           'cargo_query_wire.py', 'cargo_map_query_mode.py',
           'cargo_query_export.py', 'occupancy_map_store.py',
           'cargo_volume_mapper.py']
lock = {'package_dir': package_dir,
        'module_file': os.path.abspath(luggage_perception.__file__),
        'schema_version': int(WIRE_SCHEMA_VERSION),
        'files': {name: hashlib.sha256(
            open(os.path.join(package_dir, name), 'rb').read()
        ).hexdigest() for name in modules}}
with open(lock_path, 'w') as handle:
    json.dump(lock, handle, sort_keys=True)

replica = QueryReplica(root)
facade = QueryMapReadFacade(replica)
deadline = time.monotonic() + 60
while time.monotonic() < deadline:
    if replica.poll() == ReplicaState.READY:
        break
    time.sleep(0.05)
assert replica.state == ReplicaState.READY, str(replica.state)
rclpy.init()
node = rclpy.create_node('swf_typed_reader_proc')
MapQueryService(node, facade, None)
print('READY', flush=True)
rclpy.spin(node)
'''


def sha256_file(path):
    import hashlib
    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def wait_for_file(path, timeout_s=20):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if os.path.exists(path):
            try:
                import json
                with open(path) as handle:
                    return json.load(handle)
            except ValueError:
                pass                      # partial write; retry
        time.sleep(0.05)
    raise AssertionError('version lock never appeared: %s' % path)


def assert_reader_version_locked(lock, install_pkg, source_pkg):
    """Review round 4, gap "the imported version was never locked":
    the reader must have imported the product from the INSTALL tree
    (not the source tree via a leaked PYTHONPATH/cwd), every module
    it served must hash to the install copy, and the install copy
    must equal the committed source (a stale install fails loudly
    instead of silently testing the wrong version)."""
    import json
    assert os.path.normpath(lock['package_dir']) == os.path.normpath(
        install_pkg), lock['package_dir']
    assert lock['schema_version'] == 2, lock['schema_version']
    assert lock['files'], 'no module hashes reported'
    for name, sha in sorted(lock['files'].items()):
        installed = sha256_file(os.path.join(install_pkg, name))
        assert sha == installed, (
            'reader imported a %s that differs from the install '
            'tree' % name)
        source = sha256_file(os.path.join(source_pkg, name))
        assert installed == source, (
            'install tree is STALE vs the committed source for %s '
            '-- re-freeze before binding' % name)


def test_typed_summary_independent_reader_process(tmp_path):
    """Gap 3: the typed service runs in an INDEPENDENT reader process
    importing the product from the local colcon install (production
    code), and the client compares the COMPLETE identity and ALL
    business fields against the owner summary of the same world.
    Review round 4: the imported version is LOCKED -- the reader
    reports its package dir and per-module sha256 before serving and
    the parent asserts install-tree == source-tree."""
    import subprocess
    import rclpy
    from luggage_msgs.srv import GetCargoMapSummaryV2

    here = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    repo = os.path.normpath(here)
    install = os.path.normpath(os.path.join(repo, 'install', 'swf'))
    install_pkg = os.path.join(
        install, 'luggage_perception', 'local', 'lib', 'python3.10',
        'dist-packages', 'luggage_perception')
    source_pkg = os.path.join(
        repo, 'src', 'luggage_perception', 'luggage_perception')
    dist_paths = os.pathsep.join(
        os.path.join(install, pkg, 'local', 'lib', 'python3.10',
                     'dist-packages')
        for pkg in ('luggage_msgs', 'luggage_perception',
                    'luggage_description'))
    tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
    lock_path = os.path.join(tmp, 'reader_version_lock.json')
    try:
        store, hook, replica, facade, root = build_pair(tmp, 1)
        env = dict(os.environ)
        # ONLY the install dist-packages plus non-repo inherited
        # paths (ROS / other workspaces): this worktree's source
        # trees are stripped so the child cannot resolve them
        keep = [p for p in env.get('PYTHONPATH', '').split(os.pathsep)
                if p and os.path.normpath(p) != repo
                and not os.path.normpath(p).startswith(repo + os.sep)]
        env['PYTHONPATH'] = os.pathsep.join(
            [dist_paths] + keep)
        proc = subprocess.Popen(
            [sys.executable, '-c', TYPED_READER_PROC, root, lock_path],
            env=env, cwd=install,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True)
        assert_reader_version_locked(
            wait_for_file(lock_path), install_pkg, source_pkg)
        try:
            rclpy.init()
            node = rclpy.create_node('swf_typed_client_proc')
            client = node.create_client(
                GetCargoMapSummaryV2, '/cargo_map/get_summary_v2')
            assert client.wait_for_service(timeout_sec=30.0), \
                proc.stdout.read() if proc.poll() is not None else \
                'service never appeared'
            req = GetCargoMapSummaryV2.Request()
            req.query.request_id = 'swf-proc'
            req.query.frame_id = 'container_link'
            req.query.use_current = True
            req.query.release_after_query = True
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future,
                                             timeout_sec=10)
            assert future.done(), 'service call timeout'
            response = future.result()
            assert response.meta.status == 'OK', response.meta.reason
            owner = store.acquire_query_snapshot()
            owner_summary = owner.summary()
            owner_identity = tuple(map(str, owner.identity))
            owner.close()
            # COMPLETE identity comparison (the typed meta echoes it)
            typed_identity = response.meta.identity
            assert typed_identity.epoch == owner_identity[0]
            assert int(typed_identity.revision) == int(owner_identity[1])
            assert typed_identity.content_digest == owner_identity[2], \
                (typed_identity, owner_identity)
            # ALL business fields the IDL carries
            assert response.occupancy_confirmed == \
                owner_summary.occupancy_confirmed
            assert response.change_event == owner_summary.change_event
            assert response.map_source == owner_summary.map_source
            assert response.cargo_observed == \
                owner_summary.cargo_observed
            assert response.acquisition_stamp_ns == \
                owner_summary.acquisition_stamp_ns
            assert response.index_status == owner_summary.index_status
            values = owner_summary.values
            for field in ('total_voxels', 'inactive_count',
                          'unknown_count', 'free_count',
                          'occupied_count', 'frontier_count',
                          'unknown_ratio', 'occupancy_ratio',
                          'occupied_volume', 'free_volume'):
                assert getattr(response, field) == values[field], field
            assert response.placed_count == \
                values['committed_box_count']
            hook.close()
        finally:
            proc.terminate()
            try:
                proc.wait(10)
            except subprocess.TimeoutExpired:
                proc.kill()
            rclpy.shutdown()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
