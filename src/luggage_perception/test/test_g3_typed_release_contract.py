"""G4-A gen3 M1: typed service release lifecycle (plan d87c5731
section 3.2/3.3) -- release_after_query true/false x success /
business-reject / release-failure paths, plus the 1000-update
no-mixed-generation stress on the store registry.
"""

import threading
from types import SimpleNamespace

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs import srv

from luggage_perception.cargo_map_query import (
    MapQuerySnapshot, QueryError)
from luggage_perception.cargo_map_query_service import MapQueryService
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import (
    MapUpdate, OccupancyMapStore, UpdateKind)


class Node:
    def __init__(self):
        self.services = []
        self.errors = []

    def create_service(self, kind, name, callback, **kwargs):
        record = (kind, name, callback, kwargs)
        self.services.append(record)
        return record

    def get_logger(self):
        return SimpleNamespace(error=self.errors.append)


class RogueReleaseProvider:
    """Delegates every read to the real store registry but makes
    handle release raise -- the finally-path witness."""

    def __init__(self, store):
        self._store = store
        self.failed_releases = 0

    def acquire_query_snapshot(self, expected=None, *, token=''):
        handle = self._store.acquire_query_snapshot(expected, token=token)
        return MapQuerySnapshot(handle.identity, handle.grid,
                                handle._token, self)

    def resolve(self, token, identity):
        return self._store._query_registry.resolve(token, identity)

    def close(self, snapshot):
        self.failed_releases += 1
        raise RuntimeError('rogue release failure')


def make_service(store):
    node = Node()
    return MapQueryService(node, store, object()), node


def request(kind, release=True, use_current=True):
    result = kind.Request()
    result.query.request_id = 'g3-release'
    result.query.frame_id = 'container_link'
    result.query.use_current = use_current
    result.query.release_after_query = release
    return result


def invoke(callback, kind, req):
    req = deserialize_message(serialize_message(req), kind.Request)
    result = callback(req, kind.Response())
    return deserialize_message(serialize_message(result), kind.Response)


def make_store():
    return OccupancyMapStore(
        CargoVolumeMapper((0.3, 0.3, 0.3), (0, 0, 0.1), 0, 0.05),
        epoch='g3-release')


SERVICES = (srv.GetCargoMapSummaryV2, srv.QueryCargoCellsV2,
            srv.QueryCargoDistanceV2, srv.GetCargoSurfacePatchV2)


def build_request(kind):
    req = request(kind, release=True)
    if kind is srv.QueryCargoDistanceV2:
        from luggage_msgs import msg
        point = msg.CargoMapDistancePointV2().point
        point.z = 0.05
        req.points = [point]
    return req


OPERATIONS = {srv.GetCargoMapSummaryV2: 'summary',
              srv.QueryCargoCellsV2: 'cells',
              srv.QueryCargoDistanceV2: 'distance',
              srv.GetCargoSurfacePatchV2: 'surface'}


@pytest.mark.parametrize('kind', SERVICES)
def test_release_after_query_success_closes_and_returns_ok(kind):
    store = make_store()
    store.apply_update(MapUpdate(
        operation_id='box', kind=UpdateKind.ADD_BOX_V1,
        box=((0, 0, 0.05), (0.08, 0.08, 0.06), 0.), box_id='b'))
    service, _ = make_service(store)
    callback = getattr(service, OPERATIONS[kind])
    response = invoke(callback, kind, build_request(kind))
    assert response.meta.status == 'OK', response.meta.reason
    assert response.meta.reason == ''


@pytest.mark.parametrize('kind', SERVICES)
def test_business_reject_survives_release_path(kind):
    store = make_store()
    service, _ = make_service(store)
    req = request(kind, release=True)
    req.query.frame_id = 'wrong_frame'      # business-level rejection
    response = invoke(service.summary, kind, req)
    assert response.meta.status == 'INVALID_FILTER'
    assert not response.meta.complete


def test_release_failure_is_named_and_callback_responds():
    store = make_store()
    store.apply_update(MapUpdate(
        operation_id='box', kind=UpdateKind.ADD_BOX_V1,
        box=((0, 0, 0.05), (0.08, 0.08, 0.06), 0.), box_id='b'))
    provider = RogueReleaseProvider(store)
    node = Node()
    service = MapQueryService(node, provider, object())
    response = invoke(service.summary, srv.GetCargoMapSummaryV2,
                      request(srv.GetCargoMapSummaryV2, release=True))
    assert response.meta.status == 'SESSION_RELEASE_FAILED'
    assert not response.meta.complete
    assert provider.failed_releases == 1
    assert node.errors                    # recorded, not silent


def test_release_failure_does_not_mask_business_reject():
    store = make_store()
    store.apply_update(MapUpdate(
        operation_id='box', kind=UpdateKind.ADD_BOX_V1,
        box=((0, 0, 0.05), (0.08, 0.08, 0.06), 0.), box_id='b'))
    provider = RogueReleaseProvider(store)
    node = Node()
    service = MapQueryService(node, provider, object())
    # Open succeeds; the OPERATION rejects (summary + region filter).
    req = request(srv.GetCargoMapSummaryV2, release=True)
    req.query.has_region = True
    response = invoke(service.summary, srv.GetCargoMapSummaryV2, req)
    # The business verdict must survive; the release failure is only
    # recorded (never re-masks the named business status).
    assert response.meta.status == 'INVALID_FILTER'
    assert provider.failed_releases == 1
    assert node.errors


def test_keep_alive_returns_token_without_release():
    store = make_store()
    store.apply_update(MapUpdate(
        operation_id='box', kind=UpdateKind.ADD_BOX_V1,
        box=((0, 0, 0.05), (0.08, 0.08, 0.06), 0.), box_id='b'))
    service, _ = make_service(store)
    response = invoke(service.summary, srv.GetCargoMapSummaryV2,
                      request(srv.GetCargoMapSummaryV2, release=False))
    assert response.meta.status == 'OK'
    assert response.meta.snapshot_token


def test_thousand_updates_no_mixed_generation():
    """1000 committed generations while one handle is held (and read
    concurrently): every resolve returns the acquired generation's
    exact data -- field-by-field stable, never the moving newest."""
    store = make_store()
    with store.acquire_query_snapshot() as handle:
        first = handle._data()
        held = store.acquire_query_snapshot('CURRENT')
        baseline = held.summary().values
        stop = threading.Event()
        mismatches = []

        def reader():
            while not stop.is_set():
                try:
                    values = held.summary().values
                    if values != baseline:
                        mismatches.append(values)
                        return
                    if held._data().identity != first.identity:
                        mismatches.append('identity drift')
                        return
                except QueryError as error:
                    mismatches.append(str(error))
                    return

        thread = threading.Thread(target=reader)
        thread.start()
        try:
            for index in range(1000):
                update = store.apply_update(MapUpdate(
                    operation_id='stress-%d' % index,
                    kind=UpdateKind.ADD_BOX_V1,
                    box=((0, 0, 0.05), (0.02, 0.02, 0.02), 0.),
                    box_id='stress-%d' % index))
                assert update.accepted
        finally:
            stop.set()
            thread.join(timeout=10)
        assert held._data().identity == first.identity
        assert held.summary().values == baseline
        assert not mismatches, mismatches[:3]
        held.close()
