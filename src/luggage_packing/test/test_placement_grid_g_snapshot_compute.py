"""Unit checks for the grid_g single-snapshot compute (G v2 wiring).

The real ``_grid_g_snapshot_compute`` runs against SCRIPTED service
clients (no ROS graph): one get_surface_patch_v2 FULL acquire plus
query_cells_v2 token pages. These checks pin the wire-format assembly
the live test cannot easily inspect — the x-fastest-to-rows transpose,
the UNKNOWN-by-default dense scatter, the epoch/geometry/identity
fail-closed branches, and the verifier built from the SAME snapshot.
"""
import os
import runpy
import sys
import types

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..'))

RESOLUTION = 0.1
NX, NY, NZ = 3, 2, 2
# grid_g epochs are full UUIDs in canonical 32-hex form (the validity
# intake normalizes; the direct ValidityObservation constructor does not)
EPOCH, REVISION = "bef3707b9b274859b9fb3eee84f823bf", 7
HULL = "hull-cb41"


def load_node_module():
    try:
        from luggage_msgs.srv import (  # noqa: F401
            GetCargoSurfacePatchV2, QueryCargoCellsV2)
    except ImportError as exc:
        import unittest
        raise unittest.SkipTest("luggage_msgs unavailable: %s" % exc)
    path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts', 'placement_planner_node.py')
    return runpy.run_path(path)


class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ScriptedClient:
    """Replays canned responses in order and records every request."""

    def __init__(self, name, responses, available=True):
        self.srv_name = name
        self._available = available
        self._responses = list(responses)
        self.requests = []

    def wait_for_service(self, timeout_sec=None):
        return self._available

    def call_async(self, request):
        self.requests.append(request)
        assert self._responses, "unexpected extra call to %s" % self.srv_name
        return _FakeFuture(self._responses.pop(0))


def _identity(epoch=EPOCH, revision=REVISION):
    return types.SimpleNamespace(
        epoch=epoch, revision=revision, content_digest="cd-1",
        voxel_digest="vd-1", index_schema_version=2)


def _meta(**overrides):
    meta = types.SimpleNamespace(
        request_id="r", status="OK", reason="", identity=_identity(),
        grid=types.SimpleNamespace(
            shape=[NX, NY, NZ],
            origin=types.SimpleNamespace(x=-0.15, y=-0.10, z=0.0),
            resolution=RESOLUTION, frame_id="container_link",
            geometry_hash=HULL),
        snapshot_token="tok-1", complete=True, next_cursor="",
        returned_count=0, examined_cells=0, examined_blocks=0,
        elapsed_ns=1)
    for key, value in overrides.items():
        setattr(meta, key, value)
    return meta


def _patch():
    """FULL surface patch: every column observed free at z=0.2."""
    count = NX * NY
    return types.SimpleNamespace(
        meta=_meta(complete=True),
        is_full=True, since_identity=_identity(),
        floor_z_container=0.0, ceiling_z_container=0.2,
        origin_index_xy=[0, 0], shape_xy=[NX, NY],
        height=[0.2] * count, clearance=[0.0] * count,
        state=[1] * count, known_ratio=[1.0] * count,
        confidence=[1] * count, semantic_label=[0] * count,
        has_surface_z=[True] * count, surface_z_container=[0.2] * count,
        has_height_above_floor=[True] * count,
        height_above_floor=[0.2] * count,
        contract_confidence=[1] * count, patches=[])


def _dense_page(states, *, complete=True, next_cursor="",
                identity=None, status="OK"):
    """One dense-plane page: raw state bytes over the 3x2x2 grid."""
    return types.SimpleNamespace(
        meta=_meta(
            status=status, complete=complete, next_cursor=next_cursor,
            identity=identity or _identity(), snapshot_token="tok-1"),
        states=bytes(states))


def _plane_bytes(free_indices=()):
    plane = bytearray(NX * NY * NZ)
    for index in free_indices:
        plane[index] = 1        # FREE
    return bytes(plane)


def make_stub(module, patch, cells_pages, cells_mode=True):
    planner = module['PlacementPlannerNode']
    admission = module['AdmissionGate']()
    stub = types.SimpleNamespace(
        _validity_backend="grid_g",
        _admission=admission,
        _geometry_hash=HULL,
        _inner_size=(0.3, 0.2, 0.2),
        _hull=types.SimpleNamespace(
            descriptor=lambda: {"geometry_hash": HULL, "floor_z": 0.0}),
        _surface=None,
        _rejected_maps=0,
        _grid_g_clients={
            "get_surface_patch_v2": _ScriptedClient("/cargo_map/get_surface_patch_v2",
                                             [patch]),
            "query_cells_dense_v2": _ScriptedClient(
                "/cargo_map/query_cells_dense_v2",
                list(cells_pages) + [types.SimpleNamespace(
                    meta=_meta(), states=b"\x00")])},
        get_parameter=lambda name: types.SimpleNamespace(
            value={"volume_evidence_source": "grid_g",
                   "volume_service_timeout_s": 2.0,
                   "volume_grid_g_cells": cells_mode}.get(name, "")),
        get_logger=lambda: types.SimpleNamespace(
            info=lambda *a, **k: None, error=lambda *a, **k: None),
    )
    for name in ('_grid_g_snapshot_compute', '_compute_inputs',
                 '_surface_view_from_record', '_admit_surface_map',
                 '_evidence_unavailable_code'):
        setattr(stub, name,
                types.MethodType(getattr(planner, name), stub))
    # bind the planner's VALID epoch so the snapshot binding passes
    admission.observe_validity(module['ValidityObservation'](
        "VALID", EPOCH, 3, transition_mono_ns=1))
    return stub


def _candidate():
    """Box volume = cells (1, 0..1, 0): x in cell 1, z in layer 0."""
    return {
        "center_container": [0.0, 0.0, 0.05],
        "size": [0.08, 0.08, 0.1],
        # the verifier consumes the solver's footprint form: [l, w]
        "footprint": [0.08, 0.08],
    }


def test_healthy_snapshot_builds_same_revision_surface_and_verifier():
    module = load_node_module()
    plane = bytearray(_plane_bytes())
    for index in (1, 4, 6, 11):
        plane[index] = 1
    pages = [
        _dense_page(bytes(plane[:7]), complete=False, next_cursor="c2"),
        _dense_page(bytes(plane[7:]), complete=True),
    ]
    stub = make_stub(module, _patch(), pages)
    surface, verifier = stub._grid_g_snapshot_compute()
    assert (surface["nx"], surface["ny"]) == (NX, NY), surface["nx"]
    assert surface["map_revision"] == REVISION
    assert surface["height"] == [[0.2, 0.2], [0.2, 0.2], [0.2, 0.2]]
    # the queried box volume is observed FREE on the same snapshot
    assert verifier(_candidate()) is None
    # every resume page carried the full pinned identity; the FIRST
    # page is a use_current acquire (no token by contract)
    dense = stub._grid_g_clients["query_cells_dense_v2"]
    assert dense.requests[0].query.use_current is True
    assert dense.requests[0].query.snapshot_token == ""
    for request in dense.requests[1:]:
        assert request.query.snapshot_token == "tok-1"
        assert request.query.use_current is False
        assert request.query.expected_identity.epoch == EPOCH
        assert request.query.expected_identity.revision == REVISION
    # the last scripted call is the explicit one-byte release
    assert dense.requests[-1].query.release_after_query is True
    assert dense.requests[-1].query.limit == 1


def test_absent_cells_default_unknown_and_fail_the_volume_gate():
    module = load_node_module()
    # no cell pages at all: the snapshot exists but the volume is
    # unobserved -> the verifier must fail closed, never pass.
    stub = make_stub(module, _patch(),
                     [_dense_page(_plane_bytes(), complete=True)])
    _surface, verifier = stub._grid_g_snapshot_compute()
    reason = verifier(_candidate())
    assert reason and "volume_unknown" in reason, reason


def test_epoch_mismatch_fails_closed():
    module = load_node_module()
    stub = make_stub(module, _patch(),
                     [_dense_page(_plane_bytes(), complete=True)])
    patch = stub._grid_g_clients["get_surface_patch_v2"]._responses[0]
    patch.meta.identity = _identity(epoch="ep-other")
    try:
        stub._grid_g_snapshot_compute()
    except RuntimeError as error:
        assert "epoch" in str(error), error
    else:
        raise AssertionError("epoch mismatch admitted")


def test_foreign_hull_is_rejected_with_geometry_attribution():
    module = load_node_module()
    stub = make_stub(module, _patch(),
                     [_dense_page(_plane_bytes(), complete=True)])
    patch = stub._grid_g_clients["get_surface_patch_v2"]._responses[0]
    patch.meta.grid.geometry_hash = "hull-of-another-container"
    try:
        stub._grid_g_snapshot_compute()
    except RuntimeError as error:
        assert "CARGO_MAP_GEOMETRY_MISMATCH" in str(error), error
    else:
        raise AssertionError("foreign hull admitted")


def test_page_identity_drift_inside_one_snapshot_fails_closed():
    module = load_node_module()
    plane = _plane_bytes((1,))
    pages = [
        _dense_page(plane[:6], complete=False, next_cursor="c2"),
        _dense_page(plane[6:], complete=True,
                    identity=_identity(revision=REVISION + 1)),
    ]
    stub = make_stub(module, _patch(), pages)
    try:
        stub._grid_g_snapshot_compute()
    except RuntimeError as error:
        assert "identity" in str(error), error
    else:
        raise AssertionError("drifted page admitted")


def test_short_dense_plane_fails_closed():
    module = load_node_module()
    # a complete plane that is SHORT of nx*ny*nz must never be padded
    # silently into evidence
    stub = make_stub(module, _patch(),
                     [_dense_page(b"\x00" * 5, complete=True)])
    try:
        stub._grid_g_snapshot_compute()
    except RuntimeError as error:
        assert "dense plane short" in str(error), error
    else:
        raise AssertionError("short plane admitted")


def test_default_mode_is_snapshot_surface_without_cells():
    """volume_grid_g_cells=false (default): ONE surface call, released
    with the query (no token session), verifier None — the
    chain-accepted G semantics; the cells dump cannot fit the service's
    five-second token at container scale."""
    module = load_node_module()
    stub = make_stub(module, _patch(), [], cells_mode=False)
    surface, verifier = stub._grid_g_snapshot_compute()
    assert surface["map_revision"] == REVISION
    assert verifier is None
    patch_client = stub._grid_g_clients["get_surface_patch_v2"]
    assert patch_client.requests[0].query.release_after_query is True
    # no cells calls at all were made
    assert stub._grid_g_clients["query_cells_dense_v2"].requests == []


def test_compute_inputs_dispatches_grid_g_and_names_failures():
    module = load_node_module()
    stub = make_stub(module, _patch(),
                     [_dense_page(_plane_bytes(), complete=True)])
    # dispatch: _compute_inputs routes grid_g to the snapshot compute
    surface, verifier = stub._compute_inputs()
    assert surface["map_revision"] == REVISION
    assert stub._evidence_unavailable_code() == "GRID_G_SNAPSHOT_UNAVAILABLE"
    # and a missing service surface is NAMED, never a silent fallback
    stub.create_client = lambda srv, name: _ScriptedClient(
        name, [], available=False)
    stub._grid_g_clients = None
    try:
        stub._compute_inputs()
    except RuntimeError as error:
        assert "service missing" in str(error), error
    else:
        raise AssertionError("missing service admitted")


if __name__ == "__main__":
    test_healthy_snapshot_builds_same_revision_surface_and_verifier()
    test_absent_cells_default_unknown_and_fail_the_volume_gate()
    test_epoch_mismatch_fails_closed()
    test_foreign_hull_is_rejected_with_geometry_attribution()
    test_page_identity_drift_inside_one_snapshot_fails_closed()
    test_short_dense_plane_fails_closed()
    test_compute_inputs_dispatches_grid_g_and_names_failures()
    print("grid_g snapshot compute checks passed")
