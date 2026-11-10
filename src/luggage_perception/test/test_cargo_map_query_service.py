"""Real generated typed messages through the read boundary (no ROS graph)."""
from types import SimpleNamespace

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs import msg, srv

from luggage_perception.cargo_map_query import QueryError
from luggage_perception.cargo_map_query_adapter import QuerySnapshotRegistry
from luggage_perception.cargo_map_query_service import MapQueryService, set_identity
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper
from luggage_perception.occupancy_map_store import OccupancyMapStore


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


def boundary():
    s = OccupancyMapStore(CargoVolumeMapper((.34, .34, .34), (0, 0, .17), 0, .02), epoch='typed')
    n = Node()
    return s, n, MapQueryService(n, s, object())


def request(kind, release=True):
    result = kind.Request()
    result.query.request_id = 'typed-test'
    result.query.frame_id = 'container_link'
    result.query.use_current = True
    result.query.release_after_query = release
    return result


def invoke(callback, kind, req):
    # Exercise actual ROS serialization in both directions, not a spelling test.
    req = deserialize_message(serialize_message(req), kind.Request)
    result = callback(req, kind.Response())
    return deserialize_message(serialize_message(result), kind.Response)


def publish(s):
    s._snapshot = s._build_snapshot()


def test_typed_summary_and_only_read_services_are_registered():
    s, node, service = boundary()
    assert {r[1] for r in node.services} == {
        '/cargo_map/get_summary_v2', '/cargo_map/query_cells_v2',
        '/cargo_map/query_distance_v2', '/cargo_map/get_surface_patch_v2'}
    before = s.identity
    response = invoke(service.summary, srv.GetCargoMapSummaryV2, request(srv.GetCargoMapSummaryV2))
    assert response.meta.status == 'OK' and response.meta.complete
    assert response.unknown_count == 17**3 and response.free_count == 0
    assert response.meta.identity.epoch == 'typed'
    assert response.meta.identity.voxel_digest
    assert response.meta.grid.frame_id == 'container_link'
    assert response.meta.snapshot_token == ''
    assert s._query_registry.resource_summary()['sessions'] == 0
    assert s.identity == before and not node.errors


def test_pagination_resumes_old_generation_and_rejects_mixed_identity():
    s, node, service = boundary()
    req = request(srv.QueryCargoCellsV2, release=False)
    req.query.limit = 2
    response = invoke(service.cells, srv.QueryCargoCellsV2, req)
    assert response.meta.status == 'OK' and not response.meta.complete
    assert [c.flat_index for c in response.cells] == [0, 1]
    old_identity = response.meta.identity
    s.mapper.mark_occupied_voxel(4, 4, 4)
    publish(s)
    req.query.use_current = False
    req.query.expected_identity = old_identity
    req.query.snapshot_token = response.meta.snapshot_token
    req.query.cursor = response.meta.next_cursor
    good_cursor = req.query.cursor
    req.query.cursor = 'tampered'
    rejected = invoke(service.cells, srv.QueryCargoCellsV2, req)
    assert rejected.meta.reason == 'INVALID_CURSOR'
    req.query.cursor = good_cursor
    req.query.release_after_query = True
    response = invoke(service.cells, srv.QueryCargoCellsV2, req)
    assert response.meta.status == 'OK' and [c.flat_index for c in response.cells] == [2, 3]
    # Explicit release makes token unusable; no lifetime renewal.
    response = invoke(service.cells, srv.QueryCargoCellsV2, req)
    assert response.meta.status == 'SNAPSHOT_EXPIRED' and not response.meta.complete
    assert response.cells == [] and not node.errors


def test_resume_identity_and_clock_expiry_are_strict():
    s, _, _ = boundary()
    data = s.acquire_query_snapshot()._data()
    now = [1]
    registry = QuerySnapshotRegistry(clock_ns=lambda: now[0])
    registry.publish(data)
    handle = registry.open()
    now[0] = 4_000_000_001
    resumed = registry.resume(handle._token, handle.identity)
    assert resumed.summary().status == 'OK'
    with pytest.raises(QueryError, match='SNAPSHOT_IDENTITY_MISMATCH'):
        registry.resume(handle._token, handle.identity._replace(revision=999))
    now[0] = 5_000_000_001
    with pytest.raises(QueryError, match='SNAPSHOT_EXPIRED'):
        registry.resume(handle._token, handle.identity)


def test_schema_two_pin_is_accepted_and_schema_one_pin_is_rejected():
    _, node, service = boundary()
    result = invoke(service.summary, srv.GetCargoMapSummaryV2,
                    request(srv.GetCargoMapSummaryV2))
    assert result.meta.identity.index_schema_version == 2
    req = request(srv.GetCargoMapSummaryV2)
    req.query.use_current = False
    req.query.expected_identity = result.meta.identity
    pinned = invoke(service.summary, srv.GetCargoMapSummaryV2, req)
    assert pinned.meta.status == 'OK'
    assert pinned.meta.identity == result.meta.identity
    req.query.expected_identity.index_schema_version = 1
    rejected = invoke(service.summary, srv.GetCargoMapSummaryV2, req)
    assert rejected.meta.reason == 'SNAPSHOT_IDENTITY_MISMATCH'
    assert not rejected.meta.complete and not node.errors


def test_named_validation_errors_survive_typed_serialization():
    s, node, service = boundary()
    req = request(srv.QueryCargoCellsV2)
    req.query.has_region = True
    req.query.region_max.x = float('nan')
    result = invoke(service.cells, srv.QueryCargoCellsV2, req)
    assert result.meta.reason == 'INVALID_REGION'
    req = request(srv.QueryCargoCellsV2)
    req.mode, req.instance.kind, req.instance.box_id = 2, 2, 'missing'
    result = invoke(service.cells, srv.QueryCargoCellsV2, req)
    assert result.meta.reason == 'INSTANCE_BINDING_MISSING'
    req.instance.kind = 99
    assert invoke(service.cells, srv.QueryCargoCellsV2, req).meta.reason == 'INVALID_FILTER'
    req = request(srv.GetCargoMapSummaryV2)
    req.query.frame_id = 'world'
    assert invoke(service.summary, srv.GetCargoMapSummaryV2, req).meta.reason == 'INVALID_FILTER'
    req.query.frame_id = 'container_link'
    req.query.expected_identity.index_schema_version = 1
    assert invoke(service.summary, srv.GetCargoMapSummaryV2, req).meta.reason == 'INVALID_FILTER'
    assert not node.errors


def test_distance_unknown_policy_and_outside_point_are_typed():
    _, node, service = boundary()
    req = request(srv.QueryCargoDistanceV2)
    assert req.max_radius_m == .3
    req.points = [msg.CargoMapDistancePointV2().point]
    req.points[0].z = .1
    result = invoke(service.distance, srv.QueryCargoDistanceV2, req)
    assert result.meta.status == 'OK'
    assert result.results[0].occupied.status == 'NO_MATCH'
    assert not result.results[0].occupied.has_distance
    assert result.results[0].unknown.exact and result.results[0].unknown.distance_m == 0.
    req.points[0].x = 100.
    result = invoke(service.distance, srv.QueryCargoDistanceV2, req)
    assert result.results[0].status == 'OUTSIDE_MAP'
    assert not result.results[0].occupied.exact
    req.unknown_policy = 9
    assert invoke(service.distance, srv.QueryCargoDistanceV2, req).meta.reason == 'INVALID_FILTER'
    assert not node.errors


def test_positive_distance_leaf_counts_serialize_as_builtin_integers():
    s, node, service = boundary()
    s.mapper.mark_occupied_voxel(1, 1, 1)
    publish(s)
    req = request(srv.QueryCargoDistanceV2)
    req.points = [msg.CargoMapDistancePointV2().point for _ in range(2)]
    for point in req.points:
        point.z = .1
    result = invoke(service.distance, srv.QueryCargoDistanceV2, req)
    assert result.meta.status == 'OK' and result.meta.complete
    assert result.meta.examined_cells > 0 and result.meta.examined_blocks > 0
    assert all(r.occupied.exact and r.occupied.distance_m > 0 for r in result.results)
    assert not node.errors


def test_distance_nested_messages_belong_to_each_point_and_response():
    s, node, service = boundary()
    s.mapper.mark_occupied_voxel(1, 1, 1)
    publish(s)
    req = request(srv.QueryCargoDistanceV2)
    req.points = [msg.CargoMapDistancePointV2().point for _ in range(2)]
    for point in req.points:
        point.z = .1
    first = service.distance(req, srv.QueryCargoDistanceV2.Response())
    second = service.distance(req, srv.QueryCargoDistanceV2.Response())
    before = serialize_message(second)
    first.results[0].occupied.cell.semantic_label = 99
    first.results[0].unknown.cell.geometry_membership = ['changed-locally']
    assert first.results[1].occupied.cell.semantic_label != 99
    assert not first.results[1].unknown.cell.geometry_membership
    assert serialize_message(second) == before
    req.unknown_policy = 1
    occupied_only = invoke(service.distance, srv.QueryCargoDistanceV2, req)
    assert all(not p.has_unknown and not p.unknown.has_cell and not p.unknown.has_distance
               for p in occupied_only.results)
    assert not node.errors


def test_full_surface_flattening_and_bounded_patch_are_typed():
    s, node, service = boundary()
    old = s.acquire_query_snapshot().identity
    s.mapper.mark_occupied_voxel(1, 2, 3, label=7)
    publish(s)
    req = request(srv.GetCargoSurfacePatchV2)
    result = invoke(service.surface, srv.GetCargoSurfacePatchV2, req)
    assert result.meta.status == 'OK' and result.is_full
    assert list(result.shape_xy) == [17, 17]
    index = 1+17*2
    assert result.height[index] == .08 and result.state[index] == 2
    assert result.semantic_label[index] == 7
    assert result.has_surface_z[index] and not result.has_surface_z[0]
    assert result.surface_z_container[0] == 0.
    req.mode = 1
    set_identity(req.since_identity, old)
    result = invoke(service.surface, srv.GetCargoSurfacePatchV2, req)
    assert result.meta.status == 'OK' and not result.is_full
    assert len(result.patches) == 1
    assert (result.patches[0].ix, result.patches[0].iy) == (1, 2)
    assert result.patches[0].semantic_label == 7
    assert not node.errors
