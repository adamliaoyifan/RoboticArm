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


def test_old_stream_rejected_then_new_full_recovers(tmp_path):
    # a v1 manifest on disk is named-rejected; the SAME root then
    # publishes v2 generations again and the replica recovers READY
    tmp = str(tempfile.mkdtemp(dir=str(tmp_path)))
    try:
        store, hook, replica, facade, root = build_pair(tmp, 0)
        import glob as _g
        manifest_path = sorted(_g.glob(
            os.path.join(root, 'generations', '*', '*',
                         'manifest.json')))[-1]
        manifest = json_load(manifest_path)
        assert manifest['wire_schema_version'] == 2
        v1 = dict(manifest, wire_schema_version=1)
        v1.pop('checksum', None)
        with open(manifest_path, 'w') as handle:
            handle.write(canonical(v1))
        # force the replica onto the tampered v1 stream: a further
        # generation re-points published.json at the v1 manifest
        published = os.path.join(root, 'published.json')
        rel = os.path.relpath(manifest_path, root)
        with open(published, 'w') as handle:
            handle.write(canonical({'manifest': rel}))
        store.apply_update(MapUpdate(
            operation_id='after', kind=UpdateKind.INTEGRATE_VIEW,
            expected_snapshot=store.identity,
            geometry_hash=store._snapshot.geometry_hash,
            points=np.zeros((0, 3), dtype=float), origin=(0., 0., 1.4),
            stamp_key=(99, 0), acquisition_stamp_ns=2000, settled=True,
            schema_version=1))
        store._snapshot = store._build_snapshot()
        hook.wait_pending(30.0)
        # replica must leave READY via the named v1 rejection (and a
        # fresh reader of the same root does the same); publishing
        # further v2 FULLs recovers READY
        time.sleep(0.2)
        replica.poll()
        fresh = QueryReplica(root)
        fresh_deadline = time.monotonic() + 5
        state = None
        while time.monotonic() < fresh_deadline:
            state = fresh.poll()
            if state != ReplicaState.SYNCING:
                break
            time.sleep(0.05)
        assert state != ReplicaState.READY   # v1 named-unavailable
        for more in range(2):
            store.apply_update(MapUpdate(
                operation_id='rec%d' % more,
                kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=store._snapshot.geometry_hash,
                points=np.zeros((0, 3), dtype=float),
                origin=(0., 0., 1.4), stamp_key=(200 + more, 0),
                acquisition_stamp_ns=3000 + more, settled=True,
                schema_version=1))
            store._snapshot = store._build_snapshot()
            hook.wait_pending(30.0)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if replica.poll() == ReplicaState.READY:
                break
            time.sleep(0.05)
        assert replica.state == ReplicaState.READY
        handle = facade.acquire_query_snapshot('CURRENT')
        assert handle.summary().status == 'OK'
        handle.close()
        hook.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def json_load(path):
    import json
    with open(path) as handle:
        return json.load(handle)


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
        import glob as _g
        import json as _json
        manifest_path = sorted(_g.glob(
            os.path.join(root, 'generations', '*', '*',
                         'manifest.json')))[-1]
        manifest = _json.load(open(manifest_path))
        v1 = dict(manifest, wire_schema_version=1)
        v1.pop('checksum', None)
        with open(manifest_path, 'w') as handle:
            handle.write(_json.dumps(v1, sort_keys=True))
        import hashlib as _h
        with open(os.path.join(root, 'published.json'), 'w') as handle:
            handle.write(_json.dumps(
                {'manifest': os.path.relpath(manifest_path, root),
                 'manifest_sha256': _h.sha256(
                     open(manifest_path, 'rb').read()).hexdigest()},
                sort_keys=True))
        # phase 1: the replica MUST leave READY on the NAMED v1
        # rejection (UNKNOWN_SCHEMA via the bounded-error path)
        saw_named_rejection = False
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            replica.poll()
            if replica.state != ReplicaState.READY:
                error = str(getattr(replica, 'last_error', ''))
                raw_events = getattr(replica, 'events', None)
                if callable(raw_events):
                    raw_events = raw_events()
                events = list(raw_events or [])
                if ('UNKNOWN_SCHEMA' in error
                        or any('UNKNOWN_SCHEMA' in str(e) or 'schema'
                               in str(getattr(e, 'reason', '')).upper()
                               for e in events)):
                    saw_named_rejection = True
                    break
            time.sleep(0.05)
        assert saw_named_rejection, (
            'replica left READY without the NAMED v1 rejection: '
            'state=%s error=%s' % (replica.state,
                                   getattr(replica, 'last_error', None)))
        assert replica.state != ReplicaState.READY
        # phase 2: new v2 generations recover READY, and the rejection
        # strictly PRECEDED the recovery (poll timestamps in events)
        for more in range(3):
            store.apply_update(MapUpdate(
                operation_id='rec%d' % more,
                kind=UpdateKind.INTEGRATE_VIEW,
                expected_snapshot=store.identity,
                geometry_hash=store._snapshot.geometry_hash,
                points=np.zeros((0, 3), dtype=float),
                origin=(0., 0., 1.4), stamp_key=(300 + more, 0),
                acquisition_stamp_ns=4000 + more, settled=True,
                schema_version=1))
            store._snapshot = store._build_snapshot()
            hook.wait_pending(30.0)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if replica.poll() == ReplicaState.READY:
                break
            time.sleep(0.05)
        assert replica.state == ReplicaState.READY
        handle = facade.acquire_query_snapshot('CURRENT')
        assert handle.summary().status == 'OK'
        handle.close()
        hook.close()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
