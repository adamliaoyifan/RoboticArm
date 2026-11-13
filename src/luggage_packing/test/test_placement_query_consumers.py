"""Real service callbacks consume one geometric policy; no ROS node starts."""
import math
from types import SimpleNamespace as NS
import types

import pytest

from luggage_packing.placement_query import DIRECTED_PLACEMENT_YAWS
from luggage_packing.placement_volume_query import KnownFreeVolume, evidence_from_volume
from test_placement_planner_validity_wiring import load_node_module, make_stub, open_gate, v2_request
from test_placement_prior_requests import confirmed_grid, confirmed_volume


def compute_fixture():
    from luggage_planning.place.occupancy_place_paths import _legacy_view_of_surface_contract
    import json
    confirmed, volume = confirmed_grid(), confirmed_volume()
    metadata = json.loads(confirmed.confirmation_json)
    surface = _legacy_view_of_surface_contract(confirmed.record(), metadata['inner_size'],
                                                metadata['geometry_descriptor'])
    module = load_node_module()
    planner = module['PlacementPlannerNode']
    stub, admission, _ = make_stub(module)
    stub._geometry_hash = metadata['geometry_descriptor']['geometry_hash']
    stub._frame_scene_revision = 'cal'
    stub._allowed_yaws = DIRECTED_PLACEMENT_YAWS
    stub._params = dict(top_n=2000)
    stub._compute_inputs = lambda: (surface, KnownFreeVolume(evidence_from_volume(volume)))
    stub._slot_message = types.MethodType(planner._slot_message, stub)
    stub._slot_record = types.MethodType(planner._slot_record, stub)
    open_gate(module, admission, epoch=1, generation=2,
              transition_ns=8_000_000_000, robot_stamp_ns=9_000_000_000)
    request, response = v2_request()
    request.geometry_hash = stub._geometry_hash
    request.expected_map_revision = 3
    request.box_wdh = [.2, .1, .2]
    request.max_candidates = 8
    return stub, request, response, confirmed, volume


def heading(quat):
    return round(2 * math.atan2(quat.z, quat.w) / (math.pi / 2)) % 4


def test_compute_and_complete_prepare_preserve_the_same_directed_candidates():
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    stub, request, response, confirmed, volume = compute_fixture()
    result = stub._handle_v2(request, response)
    assert result.success, result.message
    assert len(result.candidates) == 8
    assert {heading(c.pose.orientation) for c in result.candidates} == {0, 1, 2, 3}
    from luggage_planning.planning_contract.payload_envelope import EnvelopeLimits
    prepare = NS(slot_limit=8, margin=0., vertical_margin=0., revision='cal',
                 limits=EnvelopeLimits(0., (0.,0.,0.), (0.,0.), 0.))
    prepare_request = NS(detection=NS(width=.2, depth=.1, height=.2), slots=[])
    pool = JointOperationPrepareNode._placements(prepare, prepare_request, confirmed, volume)
    assert len(pool) == len(result.candidates)
    for slot, candidate in zip(result.candidates, pool):
        actual = (slot.pose.position.x, slot.pose.position.y, slot.pose.position.z)
        assert actual == pytest.approx(candidate.container_from_box.translation)
        assert heading(slot.pose.orientation) == round(
            2 * math.atan2(candidate.container_from_box.quaternion_xyzw[2],
                           candidate.container_from_box.quaternion_xyzw[3]) / (math.pi / 2)) % 4
    # A directed requested target returns unchanged, with no generated ':pi'.
    prepare_request.slots = list(result.candidates)
    requested = JointOperationPrepareNode._placements(prepare, prepare_request, confirmed, volume)
    assert len(requested) == 8
    assert all(':pi' not in candidate.slot_id for candidate in requested)


def test_compute_refuses_unknown_volume_and_unverified_planned_box_copy():
    stub, request, response, _, volume = compute_fixture()
    volume.states = bytes([0]) * len(volume.states)
    surface, _ = stub._compute_inputs()
    stub._compute_inputs = lambda: (surface, KnownFreeVolume(evidence_from_volume(volume)))
    result = stub._handle_v2(request, response)
    assert not result.success and not result.candidates
    assert result.reason_code != 'BIN_FULL'
    stub, request, response, _, _ = compute_fixture()
    # Contents need not be decoded: without a measured ledger no planned copy
    # can be promoted to occupancy, even if the volume would admit a target.
    request.placed = [object()]
    request.placed_wdh_flat = [.2, .1, .2]
    result = stub._handle_v2(request, response)
    assert not result.success and 'PLACED_LEDGER_REQUIRED' in result.message
