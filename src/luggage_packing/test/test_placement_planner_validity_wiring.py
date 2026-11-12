"""Callback-boundary regressions for the reset-contract commit wiring
(plan OCTOMAP_RESET_VALIDITY_NEXT_20261103 §§2-3; review 2026-11-03
finding 1).

The gen1 defect lived in the CALLER WIRING, not the gate class: both real
planner callbacks committed with ``self._admission.validity.generation`` —
the board's CURRENT generation re-read at commit — so a candidate computed
before a reset was admitted after reset AND recovery once the gate had
re-opened. These checks drive the REAL ``_handle`` (ComputePlacement) and
``_handle_v2`` (ComputePlacementV2) methods on a stub node, with the reset
and recovery observations injected from inside the computation (the same
interleaving a second executor thread produces via the validity
subscription callback), for three scenarios each:

- healthy: no injection -> the plan commits;
- reset only: INVALID during computation -> VALIDITY_COMMIT_REJECTED with
  a closed-gate reason;
- reset AND recovery: INVALID then fresh VALID(map, robot) during
  computation -> the gate is OPEN again, yet the pre-reset candidate is
  rejected by the admission-token generation binding
  (commit_generation_mismatch). Under the gen1 wiring this case returned
  success=True (reviewer reproduction).
"""
import os
import runpy
import sys
import types

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..'))

NOW = [10_000_000_000]          # fake monotonic clock, ns


def load_node_module():
    path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts', 'placement_planner_node.py')
    return runpy.run_path(path)


def make_stub(module):
    """Minimal node surface the two handlers touch, with the REAL
    reset-contract methods (_validity_admit/_validity_commit and the
    handlers themselves) bound from PlacementPlannerNode."""
    planner = module['PlacementPlannerNode']
    admission = module['AdmissionGate']()
    last_published = []

    class _FakeTime:
        # rclpy shape: ``Time.nanoseconds`` is an int PROPERTY. The old
        # callable stub masked the real-node TypeError (the four
        # `.nanoseconds()` call-site defects fixed in G1) — this stub now
        # exercises the property-read form the production code uses.
        @property
        def nanoseconds(self):
            return NOW[0]

    stub = types.SimpleNamespace(
        # reset-contract state (real gate, enforced stream)
        _admission=admission,
        _validity_seen=True,
        _validity_gate_enforce=True,
        _validity_topic="/uld_occ_observer/validity",
        _validity_backend="observer_o",
        _robot_max_age_ns=1_000_000_000,
        get_clock=lambda: types.SimpleNamespace(now=lambda: _FakeTime()),
        # planner surfaces the handlers read
        _geometry_hash="geom-test",
        _runtime=None,
        _surface={"map_revision": 1},
        _rejected_maps=0,
        _geometry_rejected_maps=0,
        _committed=None,
        _active_surface=lambda: {"map_revision": 1,
                                 "geometry_hash": "geom-test"},
        _placed_aabbs=lambda placed: [],
        _placed_aabbs_v2=lambda placed, wdh: [],
        _allowed_yaws=[0],
        _params={},
        _constraint_verdict=lambda candidate, placed, size: (True, ""),
        _atlas_hook=lambda size: None,
        _atlas_cfg={"enabled": False},
        _atlas_for=lambda size: ({}, {}),
        _slot_message=lambda best, request_id: (
            types.SimpleNamespace(slot_id=1), {}),
        _slot_from_candidate=lambda best, size: (
            types.SimpleNamespace(),
            {"center_base_link": [0, 0, 0], "center_world": [0, 0, 0]}),
        _annotate_frames=lambda candidate: candidate,
        _publish_last=last_published.append,
        behavior_hook=lambda name, detail="": None,
        get_parameter=lambda name: types.SimpleNamespace(
            value=("" if name == "volume_evidence_source"
                   else False if name == "structural_search" else 8)),
    )
    for name in ('_validity_admit', '_validity_commit', '_handle',
                 '_handle_v2', '_compute_inputs', '_placed_ledger_recheck'):
        setattr(stub, name,
                types.MethodType(getattr(planner, name), stub))
    return stub, admission, last_published


def open_gate(module, admission, *, epoch, generation,
              transition_ns, robot_stamp_ns):
    admission.observe_validity(module['ValidityObservation'](
        "VALID", epoch, generation, transition_mono_ns=transition_ns))
    admission.observe_map(module['MapObservation'](
        epoch=epoch, generation=generation, map_revision=1))
    admission.observe_robot_state(module['RobotStateSample'](
        stamp_mono_ns=robot_stamp_ns))


def install_fake_solver(module, during_compute):
    """Patch placement_query in the node's module namespace: the fake
    runs the mid-computation injection (the validity callback firing on
    another executor thread) and returns a feasible candidate.

    runpy.run_path returns a COPY of the module dict, so the patch must
    go into the real globals the bound methods resolve against.
    """
    def fake_solve(surface, size, **kwargs):
        for inject in during_compute:
            inject()
        candidate = {"feasible": True, "score": 1.0, "reason": "",
                     "center_base_link": [0, 0, 0],
                     "center_world": [0, 0, 0]}
        return {"success": True, "candidates": [candidate],
                "selected": candidate, "preferred_candidate": candidate,
                "reason_code": "", "message": "fake-feasible",
                "reject_histogram": {}, "capacity_feasible_count": 1}
    module['placement_query'] = fake_solve
    real_globals = module['PlacementPlannerNode'].__dict__[
        '_handle_v2'].__globals__
    real_globals['placement_query'] = fake_solve


def v2_request():
    request = types.SimpleNamespace(
        height_measured=True, geometry_hash="geom-test",
        expected_map_revision=1, box_wdh=[0.3, 0.2, 0.2],
        placed=[], placed_wdh_flat=[], max_candidates=4)
    response = types.SimpleNamespace(
        geometry_hash="", success=None, reason_code="", message="",
        slot=None, candidates=[], candidate_scores=[], candidate_reasons=[],
        resulting_map_revision=0)
    return request, response


def v1_request():
    box = types.SimpleNamespace(width=0.3, depth=0.2, height=0.2,
                                height_valid=True, height_source=1)
    request = types.SimpleNamespace(
        box=box, geometry_hash="", placed=[], max_candidates=4)
    response = types.SimpleNamespace(
        geometry_hash="", success=None, reason_code="", message="",
        slot=None, candidates=[], candidate_scores=[], candidate_reasons=[])
    return request, response


def scenario_injections(module, admission_holder, scenario):
    """Mid-computation world changes for the three scenarios."""
    if scenario == "healthy":
        return []
    if scenario == "reset_only":
        def inject_reset():
            admission_holder[0].observe_validity(
                module['ValidityObservation'](
                    "INVALID", 2, 3, transition_mono_ns=NOW[0]))
        return [inject_reset]
    if scenario == "reset_and_recover":
        def inject_reset_and_recover():
            gate = admission_holder[0]
            gate.observe_validity(module['ValidityObservation'](
                "INVALID", 2, 3, transition_mono_ns=NOW[0]))
            NOW[0] += 500_000_000
            gate.observe_validity(module['ValidityObservation'](
                "VALID", 2, 4, transition_mono_ns=NOW[0]))
            gate.observe_map(module['MapObservation'](
                epoch=2, generation=4, map_revision=2))
            gate.observe_robot_state(module['RobotStateSample'](
                stamp_mono_ns=NOW[0] + 1))
        return [inject_reset_and_recover]
    raise ValueError(scenario)


def check_scenario(module, handler_name, make_request, scenario):
    """Drive one handler through one scenario; returns (response,
    admission). The injection closures need the gate, which is created
    here -- thread them through a holder resolved at call time."""
    holder = [None]
    NOW[0] = 10_000_000_000
    stub, admission, last_published = make_stub(module)
    holder[0] = admission
    injs = scenario_injections(module, holder, scenario)
    install_fake_solver(module, injs)
    open_gate(module, admission, epoch=1, generation=2,
              transition_ns=NOW[0] - 2_000_000_000,
              robot_stamp_ns=NOW[0] - 1_000_000_000)
    request, response = make_request()
    getattr(stub, handler_name)(request, response)
    return response, admission


def test_v2_callback_wiring():
    module = load_node_module()
    # healthy: commits.
    response, admission = check_scenario(
        module, '_handle_v2', v2_request, "healthy")
    assert response.success is True, response.message
    assert admission.outstanding_admissions == []   # token consumed
    # reset only: rejected with a closed gate.
    response, _ = check_scenario(
        module, '_handle_v2', v2_request, "reset_only")
    assert response.success is False
    assert response.reason_code == "VALIDITY_COMMIT_REJECTED"
    assert "validity_invalid" in response.message
    # reset AND recovery (reviewer reproduction): the gate is open again
    # but the pre-reset candidate must NOT commit.
    response, admission = check_scenario(
        module, '_handle_v2', v2_request, "reset_and_recover")
    assert response.success is False, (
        "old candidate committed across reset+recovery: %s"
        % response.message)
    assert response.reason_code == "VALIDITY_COMMIT_REJECTED"
    assert "commit_generation_mismatch" in response.message
    assert admission.gate_state(now_mono_ns=NOW[0],
                                robot_max_age_ns=1_000_000_000).admitted


def test_v1_callback_wiring():
    module = load_node_module()
    response, admission = check_scenario(
        module, '_handle', v1_request, "healthy")
    assert response.success is True, response.message
    assert admission.outstanding_admissions == []
    response, _ = check_scenario(
        module, '_handle', v1_request, "reset_only")
    assert response.success is False
    assert response.reason_code == "VALIDITY_COMMIT_REJECTED"
    assert "validity_invalid" in response.message
    response, admission = check_scenario(
        module, '_handle', v1_request, "reset_and_recover")
    assert response.success is False, (
        "old candidate committed across reset+recovery: %s"
        % response.message)
    assert response.reason_code == "VALIDITY_COMMIT_REJECTED"
    assert "commit_generation_mismatch" in response.message
    assert admission.gate_state(now_mono_ns=NOW[0],
                                robot_max_age_ns=1_000_000_000).admitted


def test_gate_absent_fails_closed():
    """[G1/I1] No validity stream: fail closed, never the legacy floor-
    prior success path. The gate is mandatory in a common deployment;
    before the first validity publication every request is refused with
    VALIDITY_GATE_UNBOUND and produces zero candidates/zero downstream
    admission (the historical "unseen stream keeps pre-contract behavior"
    branch is the exact defect this replaces)."""
    module = load_node_module()
    NOW[0] = 10_000_000_000
    stub, admission, _last = make_stub(module)
    stub._validity_seen = False
    install_fake_solver(module, [])
    request, response = v2_request()
    stub._handle_v2(request, response)
    assert response.success is False
    assert response.reason_code == "VALIDITY_GATE_CLOSED"
    assert "VALIDITY_GATE_UNBOUND" in response.message
    assert not response.candidates
    assert admission.validity.generation == 0


if __name__ == "__main__":
    test_v2_callback_wiring()
    test_v1_callback_wiring()
    test_gate_absent_fails_closed()
    print("planner validity wiring checks passed")
