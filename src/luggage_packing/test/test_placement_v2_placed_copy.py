#!/usr/bin/env python3
"""V2 request-carried placed copies — the frame-contract input form
validated once, for real (user ruling 2026-11-10: option b).

ComputePlacementV2 requests CAN carry a placed copy (PoseContractV2
slot records + placed_wdh_flat); the race matrix deliberately sends
none, so the wire form itself was never exercised. These checks drive
the REAL machinery end to end:

- ``_placed_aabbs_v2``: slot record -> floor-relative container AABB
  (contract decode, yaw pi/2 footprint swap, wdh rejection, contract
  violations refused by name)
- v1/v2 wire PARITY: the same physical box through the v1
  elfin_base_link SlotSpec path and the v2 container slot path lands
  on IDENTICAL floor-relative AABBs — the solver must not care which
  wire form the caller used
- ``_handle_v2`` copy placement: before the first commit the request
  copy IS the source (startup fail-open); once the committed ledger
  is latched a consistent copy defers to the LEDGER AABBs and a
  phantom copy (the witness_02 class) is refused by name with the
  solver never called.
"""
import math
import os
import runpy
import sys
import types
import unittest

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..'))

SCENE_REV = "rev-placed-copy-test"
HULL_HASH = "geom-test"
INNER = (1.49, 1.97, 1.48)
FLOOR_Z = 0.53
#: a mid-air box center in container_link (absolute z; 0.10 above the
#: floor once floor-relative) — exercising the frame shift, not the
#: degenerate z=0 case
CENTER = [0.3, 0.4, 0.63]
WDH = [0.4, 0.3, 0.2]

_MODULE = {}


def load_node_module():
    if "node" in _MODULE:
        return _MODULE["node"]
    try:
        from luggage_msgs.msg import PoseContractV2  # noqa: F401
        from luggage_msgs.srv import ComputePlacementV2  # noqa: F401
    except ImportError as exc:
        raise unittest.SkipTest("luggage_msgs unavailable: %s" % exc)
    path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts', 'placement_planner_node.py')
    _MODULE["node"] = runpy.run_path(path)
    return _MODULE["node"]


def slot_message(module, request_id, center, yaw=0.0):
    """A VALID PoseContractV2 slot record through the real encoder."""
    from luggage_description.frame_diagnostics import BoundaryContext
    from luggage_description.ros_frame_bindings.poses import (
        pose_to_message)
    half = yaw / 2.0
    record = {
        "contract_version": 2, "frame_id": "container_link",
        "length_unit": "m", "time_basis": "scene",
        "scene_revision": SCENE_REV, "request_id": request_id,
        "stamp_ns": 1, "kind": "pose",
        "reference": "box_center", "orientation_semantics": "box_axis_pi",
        "position": [float(v) for v in center],
        "quaternion_xyzw": [0.0, 0.0, math.sin(half), math.cos(half)],
    }
    context = BoundaryContext(
        "F07", "v2_client", "placement_planner", request_id, SCENE_REV,
        "container_link", "box_center", "scene",
        geometry_hash=HULL_HASH, map_revision=None)
    return pose_to_message(record, "slot", context)


def make_placed_stub(module):
    """A stub carrying ONLY the fields the real placed methods read."""
    planner = module['PlacementPlannerNode']
    stub = types.SimpleNamespace(
        _inner_size=INNER, _floor_z=FLOOR_Z,
        _frame_scene_revision=SCENE_REV, _geometry_hash=HULL_HASH)
    for name in ('_placed_aabbs_v2', '_placed_aabbs',
                 '_placed_matches_ledger', '_ledger_aabbs'):
        setattr(stub, name,
                types.MethodType(getattr(planner, name), stub))
    return stub


def _close(actual, expected, tol=1e-9):
    """AABB-list comparison with float tolerance (the scene-conversion
    round trips carry representation tails by construction). Both
    sides are lists of 6-component AABB sequences."""
    assert len(actual) == len(expected), (actual, expected)
    for a, e in zip(actual, expected):
        assert len(a) == len(e) == 6, (a, e)
        for x, y in zip(a, e):
            assert abs(x - y) <= tol, (actual, expected)


def test_slot_copy_converts_to_floor_relative_aabb():
    module = load_node_module()
    stub = make_placed_stub(module)
    aabbs = stub._placed_aabbs_v2(
        [slot_message(module, "plc-1", CENTER)], list(WDH))
    # z_rel = 0.63 - 0.53 = 0.10; unrotated footprint w x d on x x y
    _close(aabbs, [(0.1, 0.25, 0.0, 0.5, 0.55, 0.2)])


def test_yaw_pi_2_swaps_footprint():
    module = load_node_module()
    stub = make_placed_stub(module)
    aabbs = stub._placed_aabbs_v2(
        [slot_message(module, "plc-1", CENTER, yaw=math.pi / 2.0)],
        list(WDH))
    _close(aabbs, [(0.15, 0.2, 0.0, 0.45, 0.6, 0.2)])


def test_nonpositive_wdh_refused():
    module = load_node_module()
    stub = make_placed_stub(module)
    try:
        stub._placed_aabbs_v2(
            [slot_message(module, "plc-1", CENTER)], [0.4, 0.3, 0.0])
    except ValueError as error:
        assert "needs positive wdh" in str(error), error
    else:
        raise AssertionError("zero-height placed box admitted")


def test_contract_violation_refused_by_name():
    module = load_node_module()
    stub = make_placed_stub(module)
    message = slot_message(module, "plc-1", CENTER)
    message.reference = "surface_contact"   # not a slot-profile value
    try:
        stub._placed_aabbs_v2([message], list(WDH))
    except ValueError as error:
        assert "rejected" in str(error), error
    else:
        raise AssertionError("contract violation admitted")


def test_v1_v2_wire_parity_same_box_same_aabb():
    """The strongest statement: one physical box, both wire forms.

    v2 says it in container_link (slot semantics); v1 says the same
    box as an elfin_base_link SlotSpec — the base pose comes from
    inverting the SAME frozen-scene transform the planner uses. The
    scene's robot base is yawed +90 deg in world and the container
    -90 deg in base, so the container sits world-aligned: a container
    yaw of 0/90 deg maps to base-frame yaw -90/+0 deg. Both yaw
    polarities (footprint swap and no swap) must land on identical
    floor-relative AABBs."""
    module = load_node_module()
    from luggage_description.scene_tf_config_utils import (
        _local_point_to_base_link, load_scene_tf_config)
    here = os.path.dirname(os.path.abspath(__file__))
    src_root = os.path.dirname(os.path.dirname(here))
    scene_path = os.path.join(
        src_root, 'luggage_description', 'config', 'scene_tf.yaml')
    scene = load_scene_tf_config(scene_path)
    stub = make_placed_stub(module)
    stub._scene = scene
    base_center = _local_point_to_base_link(CENTER, scene)

    # (container yaw, base-frame yaw): base +90 in world cancels the
    # container's -90-in-base, so base_yaw = container_yaw - 90 deg
    for container_yaw, base_yaw in ((0.0, -math.pi / 2.0),
                                    (math.pi / 2.0, 0.0)):
        v2_aabbs = stub._placed_aabbs_v2(
            [slot_message(module, "plc-1", CENTER, yaw=container_yaw)],
            list(WDH))
        pose = types.SimpleNamespace(
            position=types.SimpleNamespace(
                x=base_center[0], y=base_center[1], z=base_center[2]),
            orientation=types.SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(base_yaw / 2.0),
                w=math.cos(base_yaw / 2.0)))
        slot = types.SimpleNamespace(
            place_pose=pose, width=WDH[0], depth=WDH[1], height=WDH[2])
        v1_aabbs = stub._placed_aabbs([slot])
        _close(v1_aabbs, v2_aabbs)
        # non-degenerate: the box really converted, not clamped
        z0, z1 = v1_aabbs[0][2], v1_aabbs[0][5]
        assert abs((z1 - z0) - WDH[2]) < 1e-9, (z0, z1)


# ---------------------------------------------------------------------------
# _handle_v2 placement of the copy (the wiring-test harness drives the
# real handler end to end; only the placed machinery is real here)


def _latch(boxes):
    return {"boxes": boxes, "map_revision": 1,
            "geometry_hash": HULL_HASH, "epoch": "e1",
            "content_digest": "d1"}


def run_v2(module, placed_messages, wdh_flat, committed):
    from test_placement_planner_validity_wiring import (
        make_stub, open_gate)
    stub, admission, _published = make_stub(module)
    # the REAL placed machinery (the wiring stub fakes it with [])
    stub._inner_size = INNER
    stub._floor_z = FLOOR_Z
    stub._frame_scene_revision = SCENE_REV
    stub._committed = committed
    planner = module['PlacementPlannerNode']
    for name in ('_placed_aabbs_v2', '_placed_matches_ledger',
                 '_ledger_aabbs'):
        setattr(stub, name,
                types.MethodType(getattr(planner, name), stub))
    seen_placed = []
    solve_calls = []

    def spy_verdict(candidate, placed, size):
        seen_placed.append([tuple(a) for a in placed])
        return None, True

    stub._constraint_verdict = spy_verdict

    def fake_solve(surface, size, **kwargs):
        solve_calls.append(1)
        # exercise the handler-built candidate_validator ONCE: the
        # lambda routes into the spy above carrying placed_aabbs
        validator = kwargs.get("candidate_validator")
        if validator is not None:
            validator({"center_container": [0.0, 0.0, 0.1],
                       "size": list(size)})
        candidate = {"feasible": True, "score": 1.0, "reason": "",
                     "center_base_link": [0, 0, 0],
                     "center_world": [0, 0, 0]}
        return {"success": True, "candidates": [candidate],
                "selected": candidate, "preferred_candidate": candidate,
                "reason_code": "", "message": "fake-feasible",
                "reject_histogram": {}, "capacity_feasible_count": 1}
    module['solve_placement'] = fake_solve
    module['PlacementPlannerNode'].__dict__[
        '_handle_v2'].__globals__['solve_placement'] = fake_solve

    open_gate(module, admission, epoch=1, generation=2,
              transition_ns=8_000_000_000, robot_stamp_ns=9_000_000_000)
    request = types.SimpleNamespace(
        height_measured=True, geometry_hash="",
        expected_map_revision=1, box_wdh=list(WDH),
        placed=list(placed_messages), placed_wdh_flat=list(wdh_flat),
        max_candidates=4)
    response = types.SimpleNamespace(
        geometry_hash="", success=None, reason_code="", message="",
        slot=None, candidates=[], candidate_scores=[],
        candidate_reasons=[], resulting_map_revision=0)
    stub._handle_v2(request, response)
    return response, seen_placed, solve_calls


def test_request_copy_is_the_source_before_first_commit():
    """No latch: the converted REQUEST copy feeds the solver gates."""
    module = load_node_module()
    response, seen_placed, solve_calls = run_v2(
        module, [slot_message(module, "plc-1", CENTER)], WDH, None)
    assert response.success, (response.reason_code, response.message)
    assert solve_calls, "solver never ran"
    assert len(seen_placed) == 1, seen_placed
    _close(seen_placed[0], [(0.1, 0.25, 0.0, 0.5, 0.55, 0.2)])


def test_consistent_copy_defers_to_the_ledger_aabbs():
    """Latch present + copy within tolerance: the solver gates run on
    the LEDGER's boxes (single source), not the caller's copy — the
    copy's 3 cm offset must NOT reach the solver."""
    module = load_node_module()
    ledger_box = {"center": list(CENTER), "size": list(WDH), "yaw": 0.0}
    offset_copy = [0.33, 0.4, 0.63]        # 3 cm off, inside 0.10 tol
    response, seen_placed, solve_calls = run_v2(
        module,
        [slot_message(module, "plc-1", offset_copy)], WDH,
        _latch([ledger_box]))
    assert response.success, (response.reason_code, response.message)
    # TWO identical captures: the compute pass AND the A6 commit-time
    # recheck — both must gate on the LEDGER box, never the 3 cm-offset
    # request copy
    assert len(seen_placed) == 2, seen_placed
    for capture in seen_placed:
        _close(capture, [(0.1, 0.25, 0.0, 0.5, 0.55, 0.2)])


def test_phantom_copy_refused_by_name_solver_never_called():
    """Latch present + a box the ledger does not hold (witness_02
    phantom class): named rejection, no solver call."""
    module = load_node_module()
    ledger_box = {"center": list(CENTER), "size": list(WDH), "yaw": 0.0}
    phantom = [1.2, 0.4, 0.63]             # 0.9 m from any ledger box
    response, seen_placed, solve_calls = run_v2(
        module, [slot_message(module, "plc-1", phantom)], WDH,
        _latch([ledger_box]))
    assert not response.success
    assert response.reason_code == \
        module['CODE_PLACED_LEDGER_MISMATCH'], response.reason_code
    assert "from any ledger box" in response.message, response.message
    assert not solve_calls and not seen_placed


if __name__ == "__main__":
    test_slot_copy_converts_to_floor_relative_aabb()
    test_yaw_pi_2_swaps_footprint()
    test_nonpositive_wdh_refused()
    test_contract_violation_refused_by_name()
    test_v1_v2_wire_parity_same_box_same_aabb()
    test_request_copy_is_the_source_before_first_commit()
    test_consistent_copy_defers_to_the_ledger_aabbs()
    test_phantom_copy_refused_by_name_solver_never_called()
    print("v2 placed-copy validation passed")
