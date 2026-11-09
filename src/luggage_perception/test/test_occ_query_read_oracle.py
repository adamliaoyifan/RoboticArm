"""Independent replay must detect corrupted typed answers and unsafe refusals."""

from copy import deepcopy

import pytest
from geometry_msgs.msg import Point
from luggage_msgs import srv
from luggage_description.container_geometry import contains_point

from occ_query_read_oracle import QueryReadOracle
from test_cargo_map_query_service import boundary, invoke, request


@pytest.fixture
def sample():
    store, _, service = boundary()
    store.mapper.mark_occupied_voxel(5, 5, 5, label=7, instance_id=31)
    store._snapshot = store._build_snapshot()
    oracle = QueryReadOracle(store.mapper, store, {}, contains_point)
    return store, service, oracle


def answer(sample, kind):
    _, service, _ = sample
    message, callback = {
        "summary": (srv.GetCargoMapSummaryV2, service.summary),
        "roi": (srv.QueryCargoCellsV2, service.cells),
        "distance": (srv.QueryCargoDistanceV2, service.distance),
        "surface": (srv.GetCargoSurfacePatchV2, service.surface),
    }[kind]
    req = request(message)
    req.query.limit = 17
    if kind == "distance":
        req.points = [Point(x=-0.06, y=-0.06, z=0.11), Point(x=0.0, y=0.0, z=0.18)]
        req.max_radius_m = 0.3
    return req, invoke(callback, message, req)


@pytest.mark.parametrize("kind", ["summary", "roi", "distance", "surface"])
def test_actual_serialized_answers_and_corrupted_identity(sample, kind):
    req, response = answer(sample, kind)
    before = sample[0].identity
    sample[2].verify(kind, req, response)
    assert sample[0].identity == before
    response.meta.identity.revision += 1
    with pytest.raises(AssertionError, match="RESPONSE_IDENTITY"):
        sample[2].verify(kind, req, response)


@pytest.mark.parametrize(
    "kind,reason",
    [
        ("summary", "unknown_count"),
        ("roi", "CELL_FACTS"),
        ("distance", "NEAREST_DISTANCE"),
        ("surface", "height"),
    ],
)
def test_changed_payload_is_rejected(sample, kind, reason):
    req, response = answer(sample, kind)
    if kind == "summary":
        response.unknown_count += 1
    elif kind == "roi":
        response.cells[0].state = 2
    elif kind == "distance":
        response.results[0].occupied.distance_m += 0.01
    else:
        response.height[0] += 0.01
    with pytest.raises(AssertionError, match=reason):
        sample[2].verify(kind, req, response)


def test_budget_refusal_requires_no_false_clearance_certificate(sample):
    req, response = answer(sample, "distance")
    response.meta.status = "QUERY_BUDGET_EXHAUSTED"
    response.meta.complete = False
    point = response.results[0]
    point.status = point.occupied.status = "QUERY_BUDGET_EXHAUSTED"
    point.occupied.exact = point.occupied.has_cell = point.occupied.has_distance = False
    point.occupied.lower_bound_m = 0.0
    sample[2].verify("distance", req, response)
    unsafe = deepcopy(response)
    unsafe.results[0].occupied.exact = True
    with pytest.raises(AssertionError, match="BUDGET_CERTIFICATION"):
        sample[2].verify("distance", req, unsafe)
    point.occupied.lower_bound_m = 0.2
    with pytest.raises(AssertionError, match="BUDGET_LOWER_BOUND_TRUTH"):
        sample[2].verify("distance", req, response)


def test_legacy_admission_refusal_is_not_a_successful_volume(sample):
    store, _, oracle = sample
    req = srv.GetCargoVolumeV2.Request()
    req.scene_revision = "scene"
    req.geometry_hash = store.mapper.geometry_hash
    req.expected_epoch = store.identity.epoch
    req.expected_content_digest = store.identity.content_digest
    req.expected_map_revision = store.identity.revision
    response = srv.GetCargoVolumeV2.Response()
    response.reason = "NO_CONFIRMED_MEASURED_VOLUME"
    oracle.verify_legacy("roi", req, response, {"scene_revision": "scene"})
    response.states = [2]
    with pytest.raises(AssertionError, match="LEGACY_REFUSAL_DATA"):
        oracle.verify_legacy("roi", req, response, {"scene_revision": "scene"})
    response.states = []
    req.expected_map_revision += 1
    with pytest.raises(AssertionError, match="LEGACY_REFUSAL"):
        oracle.verify_legacy("roi", req, response, {"scene_revision": "scene"})
    response.reason = "SNAPSHOT_IDENTITY_MISMATCH"
    oracle.verify_legacy("roi", req, response, {"scene_revision": "scene"})


def test_voxel_hash_cannot_be_replaced_by_an_unrelated_nonempty_digest(sample):
    req, response = answer(sample, "summary")
    response.meta.identity.voxel_digest = "a" * 64
    with pytest.raises(AssertionError, match="VOXEL_DIGEST"):
        sample[2].verify("summary", req, response)


@pytest.mark.parametrize("field", ["logodds", "locked", "shadow", "shadow_label"])
def test_private_primary_fact_is_bound_by_independent_hash(sample, field):
    req, response = answer(sample, "summary")
    mapper = sample[0].mapper
    if field == "logodds":
        mapper._occupancy.values[0] += 0.01
    elif field == "locked":
        mapper._occupancy.geometry_locked[0] = True
    elif field == "shadow":
        mapper._sensor_values[0] += 0.01
    else:
        mapper._sensor_labels[0] = 19
    oracle = QueryReadOracle(mapper, sample[0], {}, contains_point)
    with pytest.raises(AssertionError, match="VOXEL_DIGEST"):
        oracle.verify("summary", req, response)


def test_named_overlap_geometry_and_binding_are_in_the_hash(sample):
    from luggage_perception.occupancy_map_store import MapUpdate, UpdateKind

    store, service, _ = sample
    geometry = {}
    for name, x in [("first", -0.04), ("second", 0.04)]:
        box = ((x, 0.0, 0.08), (0.12, 0.12, 0.12), 0.0)
        assert store.apply_update(
            MapUpdate(operation_id=name, kind=UpdateKind.ADD_BOX_V1, box=box, box_id=name)
        ).accepted
        geometry[name] = frozenset(store.mapper._iter_placed_box_voxels(*box))
    oracle = QueryReadOracle(store.mapper, store, geometry, contains_point)
    req = request(srv.GetCargoMapSummaryV2)
    response = invoke(service.summary, srv.GetCargoMapSummaryV2, req)
    oracle.verify("summary", req, response)
    renamed = {"wrong-name": geometry["first"], "second": geometry["second"]}
    with pytest.raises(AssertionError, match="VOXEL_DIGEST"):
        QueryReadOracle(store.mapper, store, renamed, contains_point).verify(
            "summary", req, response
        )
