"""Baseline-first watchdog and diagnostic traces; no controller/robot process."""
import json
from types import SimpleNamespace
import threading

import pytest

from luggage_msgs.srv import PrepareJointOperationV2
from luggage_planning.preparation_profile import PreparationBudget, StageTimings
from luggage_planning.joint_operation_prepare_node import JointOperationPrepareNode
from luggage_planning.joint_operation_preparation import JointOperationPreparation, PreparationInputs
from luggage_planning.placement_prior_contracts import Transform
from st09_contracts.payload_envelope import EnvelopeLimits
from test_directed_suction_service import request_and_context
from test_joint_operation_candidates import snapshot
from test_joint_operation_planner import ports, prepare


@pytest.mark.parametrize('cap', [0., -1., float('nan'), float('inf'), True])
def test_invalid_watchdog_rejected(cap):
    with pytest.raises(ValueError, match='WATCHDOG_REQUIRED'):PreparationBudget(cap)


@pytest.mark.parametrize('requested,expected', [(30., True), (60., True), (60.01, False),
    (0., False), (-1., False), (float('nan'), False), (float('inf'), False), (True, False)])
def test_budget_is_configured_not_hidden_ten_seconds(requested, expected):
    assert PreparationBudget(60.).admits(requested) is expected


def query_and_inputs(budget):
    measured, context = request_and_context()
    snap = snapshot()
    request = PrepareJointOperationV2.Request(request_id='profile', observation_id='observation',
        detection=measured.detection, generation=context.domain.generation,
        geometry_hash=snap.geometry_hash, expected_map_revision=snap.map_revision,
        expected_model_sha256=snap.model_hash, robot_state_stamp_valid=True,
        robot_state_stamp_ns=context.domain.stamp_ns, candidate_limit=8, budget_s=budget)
    inputs = PreparationInputs(snap, context.domain, (), Transform((0., 0., 0.), (0., 0., 0., 1.)), 'pickup_box')
    return request, context, inputs


@pytest.mark.parametrize('budget,cap,accepted', [(30., 60., True), (30., 20., False),
    (0., 60., False), (float('nan'), 60., False), (float('inf'), 60., False)])
def test_real_root_callback_budget_gate_timing_and_unlock(budget, cap, accepted):
    request, _, _ = query_and_inputs(budget)
    looked_up, records = [], []
    def domain(query):
        looked_up.append(query.request_id)
        raise ValueError('DOMAIN_ADMISSION_PROBE')
    root = SimpleNamespace(_active=threading.Lock(), budget=PreparationBudget(cap),
        model=SimpleNamespace(model_sha256=request.expected_model_sha256), _current=domain,
        _record=lambda kind, value:records.append(kind),
        get_logger=lambda:SimpleNamespace(error=lambda text:None))
    response = JointOperationPrepareNode._prepare(root, request, PrepareJointOperationV2.Response())
    assert not response.success and not response.plan.plan_digest
    assert bool(looked_up) is accepted
    assert ('DOMAIN_ADMISSION_PROBE' in response.reason) is accepted
    trace = json.loads(response.trace_json)
    stages = {r[2]:r for r in trace if r[0] == 'timing'}
    assert stages['input_admission'][4] == ('completed' if accepted else 'error')
    if accepted:assert stages['domain_admission'][4] == 'error'
    assert all(r[3] >= 0 for r in stages.values()) and response.elapsed_s >= 0
    assert root._active.acquire(blocking=False)
    root._active.release()
    assert records == ['prepare_request', 'prepare_response']


@pytest.mark.parametrize('requested,expected', [(30., 'SEARCH_INCOMPLETE'),
    (61., 'PREPARATION_INPUT_IDENTITY_MISMATCH'), (0., 'PREPARATION_INPUT_IDENTITY_MISMATCH')])
def test_core_uses_same_budget_and_never_activates_empty_search(requested, expected):
    request, context, inputs = query_and_inputs(requested)
    invoked = []
    def planning(*args, **kwargs):
        invoked.append(kwargs)
        return SimpleNamespace(status='SEARCH_INCOMPLETE', plan=None, trace=())
    owner = JointOperationPreparation(None, SimpleNamespace(reserve_s=.2, prepare=planning),
        context._model, EnvelopeLimits(5., (.1, .1, .1), (.01, .02), .005),
        max_center_checks=64, budget=PreparationBudget(60.), clock=lambda:10.)
    response = owner.prepare(request, PrepareJointOperationV2.Response(), inputs,
        deadline=40., revalidate=lambda *args:True)
    assert response.reason == expected and not response.success and not response.plan.plan_digest
    assert bool(invoked) is (requested == 30.)
    if invoked:assert invoked[0]['absolute_deadline'] == 40.


def test_expired_core_deadline_is_inactive_without_native_call():
    request, context, inputs = query_and_inputs(30.)
    def forbidden(*args, **kwargs):pytest.fail('expired request called planner')
    owner = JointOperationPreparation(None, SimpleNamespace(reserve_s=.2, prepare=forbidden),
        context._model, EnvelopeLimits(5., (.1, .1, .1), (.01, .02), .005),
        max_center_checks=64, budget=PreparationBudget(60.), clock=lambda:40.)
    response = owner.prepare(request, PrepareJointOperationV2.Response(), inputs,
        deadline=40., revalidate=forbidden)
    assert response.reason == 'CANCELLED_OR_DEADLINE' and not response.plan.plan_digest


def test_stage_timing_reports_returned_refusal_and_exception_without_interpreting_verdict():
    clock = SimpleNamespace(now=10.)
    trace = []
    timer = StageTimings('fixture', trace, clock=lambda:clock.now)
    def refusal():clock.now += .25;return False
    assert timer.call('refusal', refusal) is False
    def failure():clock.now += .5;raise RuntimeError('expected fixture failure')
    with pytest.raises(RuntimeError):timer.call('failure', failure)
    assert trace == [('timing', 'fixture', 'refusal', .25, 'completed'),
                     ('timing', 'fixture', 'failure', .5, 'error')]


def test_native_stage_timing_preserves_plan_bytes_and_deadlines():
    bundle = ports()
    original = prepare(bundle)
    _, _, endpoints, connections, certifier, clock, _ = bundle
    def measured(function, duration):
        def run(*args, **kwargs):
            result = function(*args, **kwargs)
            clock.now += duration
            return result
        return run
    endpoints.solve = measured(endpoints.solve, .125)
    connections.connect = measured(connections.connect, .25)
    certifier.certify = measured(certifier.certify, .375)
    result = prepare(bundle)
    assert result.status == 'PLAN_READY' and result.plan == original.plan
    times = {r[2]:r[3] for r in result.trace if r[0] == 'timing'}
    assert times == dict(local_paths=.125, align_connection=.25, carry_connection=.25,
                        time_parameterization=0., whole_chain_certification=.375)
    assert endpoints.calls[0][1]['deadline'] == pytest.approx(14.8)
    assert certifier.calls[0][2]['deadline'] == 15.
