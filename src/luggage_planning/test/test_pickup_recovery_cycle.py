"""Shared workflow conformance; ports are declared scripts, not physical motion."""
from dataclasses import asdict, replace
import hashlib
import json
from types import SimpleNamespace

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from luggage_msgs.action import ExecutePickupViewV2
from luggage_msgs.msg import DetectedLuggage, JointOperationPlanV2, PickupViewPlanV2, SuctionSupportProbeV2, SuctionTopDomainV2
from luggage_msgs.srv import PrepareJointOperationV2, ReprepareAfterPickupViewV2
from luggage_planning.joint_operation.pickup_recovery import (
    PickupPreparationAttempt, PickupPreparationRecovery, PickupRecoveryPorts, PickupRecoverySettings,
)
from luggage_planning.joint_operation.view_proposals import PickupViewSampling
from pickup_continuation_fixtures import case_archive, fixture_policy


def settings(count=2):
    return PickupRecoverySettings(count, 10., PickupViewSampling((.25,), (0.,), (1.,), .1, 3., 4),
                                  fixture_policy(), (4, 4), 2)


def domain_message(key, generation):
    result = SuctionTopDomainV2(domain_sha256=hashlib.sha256(key.encode()).hexdigest(),
        contact_model_hash='c'*64, generation=generation, box_id=key, valid=True)
    result.header.frame_id = 'world'; result.header.stamp.sec = generation
    return result


def probe(request_id, domain, query_id):
    result = SuctionSupportProbeV2(query_id=query_id, domain_sha256=domain.domain_sha256,
        contact_model_hash=domain.contact_model_hash, unproven_cell_count=1)
    result.placement.identity.request_id = request_id
    return result


def attempt(request_id, detection, domain, *, success=False, queries=('q1', 'q2'), reason='SEARCH_INCOMPLETE'):
    request = PrepareJointOperationV2.Request(request_id=request_id, detection=detection,
        observation_id=domain.domain_sha256, generation=domain.generation)
    response = PrepareJointOperationV2.Response(success=success, reason=reason, trace_json='[]')
    if success: response.plan = JointOperationPlanV2(request_id=request_id, plan_digest='a'*64)
    else: response.unresolved_contacts = [probe(request_id, domain, key) for key in queries]
    return PickupPreparationAttempt(request, response, detection, domain)


@pytest.fixture
def workflow_case(case_archive):
    f = SimpleNamespace(archive=case_archive, clock=[0.], calls=[], actions=[], acquisitions=[], continuations=[],
        fallbacks=[], action_modes=[], reply_modes=[], fallback_success=True, expires_on=None, count=0)
    f.domain = domain_message('original', 1); f.detection = DetectedLuggage(id='original', header=f.domain.header)
    f.initial = attempt('initial', f.detection, f.domain)
    f.settings = settings()
    def record(name, value):
        raw = value if isinstance(value, bytes) else serialize_message(value)
        path = case_archive.directory/(f'{len(f.calls):04d}_'+name+'.bin')
        path.write_bytes(raw); f.calls.append(path.name)
    f.record = record
    def execute(goal, deadline):
        f.actions.append(deserialize_message(serialize_message(goal), type(goal)))
        record('view_goal', goal)
        mode = f.action_modes.pop(0) if f.action_modes else 'success'
        submitted = mode != 'planning_failure'
        f.count += int(submitted)
        result = ExecutePickupViewV2.Result(success=mode=='success', submitted=submitted,
            motion_terminal=mode!='nonterminal', may_have_payload=mode=='loaded',
            settled_stamp_valid=mode!='no_settle', settled_stamp_ns=(len(f.actions)+1)*10**9,
            dispatched_motions=f.count)
        result.plan = PickupViewPlanV2(request_id=goal.preparation.request_id,
            task_request_id=goal.preparation.task_request_id, domain_sha256=goal.preparation.domain_sha256,
            query_id=goal.preparation.query_id)
        if mode == 'wrong_plan':
            result.success = True; result.plan.domain_sha256 = 'f'*64
        if mode == 'no_settle': result.success = True
        wrapped = ExecutePickupViewV2.Impl.GetResultService.Response(status=4 if result.success else 6, result=result)
        record('view_result', wrapped)
        f.clock[0] += 20. if f.expires_on == 'action' else .25
        return wrapped
    def observe(stamp, deadline):
        f.acquisitions.append((stamp, deadline))
        observed = domain_message('new-'+str(len(f.acquisitions)), 3+len(f.acquisitions))
        detected = DetectedLuggage(id=observed.box_id, header=observed.header)
        f.observed = detected, observed
        record('new_detection', detected); record('new_domain', observed)
        f.clock[0] += .25
        return f.observed
    def continue_after(request, deadline):
        f.continuations.append(deserialize_message(serialize_message(request), type(request)))
        record('continue_request', request)
        mode = f.reply_modes.pop(0) if f.reply_modes else 'success'
        detected, observed = f.observed
        output = ReprepareAfterPickupViewV2.Response(associated=mode!='ambiguous',
            success=mode in ('success', 'no_gain'), reason='SEARCH_INCOMPLETE', trace_json='[]')
        output.detection, output.top_domain = detected, observed
        if output.success: output.plan = JointOperationPlanV2(request_id=request.request_id, plan_digest='b'*64)
        elif mode == 'gap': output.unresolved_contacts = [probe(request.request_id, observed, 'new-gap')]
        output.information_json = json.dumps(dict(gained_samples=0 if mode == 'no_gain' else 1))
        record('continue_response', output)
        f.clock[0] += .25
        return output
    def prepare_current(detection, domain, deadline):
        f.fallbacks.append((detection.id, domain.domain_sha256, deadline-f.clock[0]))
        output = attempt('fallback-'+str(len(f.fallbacks)), detection, domain,
                         success=f.fallback_success, queries=('decidable-failed-gap',))
        record('fallback_request', output.request); record('fallback_response', output.response)
        f.clock[0] += .25
        return output
    f.ports = PickupRecoveryPorts(execute, observe, continue_after, prepare_current)
    for name, value in (('initial_request', f.initial.request), ('initial_response', f.initial.response),
                        ('initial_detection', f.detection), ('initial_domain', f.domain)):
        record(name, value)
    case_archive.save('workflow_fixture', dict(settings=asdict(f.settings), clock=f.clock[0],
                      ports='scripted Action/acquisition/preparation; no native motion or suction permission'))
    try:
        yield f
    finally:
        case_archive.save('workflow_terminal', dict(clock=f.clock[0], actions=len(f.actions),
            observations=f.acquisitions, continuations=len(f.continuations), fallbacks=f.fallbacks,
            task_keys=[g.pickup_task_id for g in f.actions], count=f.count,
            action_modes_left=f.action_modes, reply_modes_left=f.reply_modes))


def run(f):
    ids = iter('workflow-'+str(i) for i in range(100))
    workflow = PickupPreparationRecovery(f.settings, f.ports, record=f.record,
        clock=lambda: f.clock[0], request_id=lambda: next(ids))
    try:
        output = workflow.run(f.initial, pickup_task_id='stable-task')
    except Exception as error:
        f.archive.save('workflow_exception', dict(type=type(error).__name__, message=str(error)))
        raise
    for name, message in zip(('plan', 'detection', 'domain'), output): f.record('returned_'+name, message)
    return output


@pytest.mark.parametrize('configuration', ['absent', 'zero', 'enabled'])
def test_healthy_initial_plan_needs_no_extra_effect(workflow_case, configuration):
    f = workflow_case
    f.initial = attempt('ready', f.detection, f.domain, success=True)
    f.record('changed_initial_request', f.initial.request); f.record('changed_initial_response', f.initial.response)
    if configuration == 'absent': f.settings = None
    elif configuration == 'zero': f.settings = PickupRecoverySettings(0, 0., None, None, (), 0)
    assert run(f)[0] == f.initial.response.plan
    assert not f.actions and not f.acquisitions and not f.continuations


@pytest.mark.parametrize('damage', ['absent', 'zero', 'ik', 'backend', 'budget', 'no_probes',
                                  'domain', 'model', 'request', 'no_gap'])
def test_only_matching_localized_observation_need_can_emit_view(workflow_case, damage):
    f = workflow_case
    if damage == 'absent': f.settings = None
    elif damage == 'zero': f.settings = PickupRecoverySettings(0, 0., None, None, (), 0)
    elif damage in ('ik', 'backend', 'budget'): f.initial.response.reason = damage.upper()+'_FAILED'
    elif damage == 'no_probes': f.initial.response.unresolved_contacts = []
    else:
        for item in f.initial.response.unresolved_contacts:
            if damage == 'domain': item.domain_sha256 = 'f'*64
            elif damage == 'model': item.contact_model_hash = 'f'*64
            elif damage == 'request': item.placement.identity.request_id = 'unowned'
            elif damage == 'no_gap': item.unproven_cell_count = 0
    f.record('changed_initial_response', f.initial.response)
    with pytest.raises(RuntimeError, match='PREPARE:'): run(f)
    assert not f.actions and not f.acquisitions


def test_triggered_view_uses_actual_settle_new_observation_and_runtime_result(workflow_case):
    f = workflow_case
    plan, detection, domain = run(f)
    assert plan.plan_digest == 'b'*64 and domain is f.observed[1] and detection is f.observed[0]
    assert len(f.actions) == len(f.acquisitions) == len(f.continuations) == 1
    goal, request = f.actions[0], f.continuations[0]
    assert goal.preparation.query_id == 'q1' and goal.pickup_task_id == 'stable-task'
    assert request.previous_view.request_id == goal.preparation.request_id
    assert request.settled_stamp_ns == f.acquisitions[0][0]
    assert request.detection.id == detection.id and request.generation == domain.generation
    assert f.continuations[0].budget_s < f.actions[0].budget_s


def test_two_views_keep_one_task_and_discard_superseded_probes(workflow_case):
    f = workflow_case; f.reply_modes = ['gap', 'success']
    run(f)
    assert len(f.actions) == 2 and all(g.pickup_task_id == 'stable-task' for g in f.actions)
    assert [g.preparation.query_id for g in f.actions] == ['q1', 'new-gap']
    assert f.actions[1].preparation.domain_sha256 == domain_message('new-1', 4).domain_sha256
    assert f.actions[1].preparation.task_request_id == f.continuations[0].request_id
    assert f.actions[1].budget_s < f.actions[0].budget_s and f.count == 2


def test_count_exhaustion_tries_other_decidable_targets_with_same_deadline(workflow_case):
    f = workflow_case; f.settings = settings(1); f.reply_modes = ['gap']
    result = run(f)
    assert result[0].request_id.startswith('fallback') and len(f.actions) == 1 and len(f.fallbacks) == 1
    assert f.fallbacks[0][1] != f.domain.domain_sha256 and f.fallbacks[0][2] < 10.


def test_original_target_failure_without_new_gap_uses_current_observation_first(workflow_case):
    f = workflow_case; f.reply_modes = ['pure_ik']
    run(f)
    assert len(f.actions) == 1 and len(f.fallbacks) == 1


def test_failed_planning_cannot_repeat_same_probe_without_charging_motion(workflow_case):
    f = workflow_case; f.action_modes = ['planning_failure', 'success']
    run(f)
    assert [g.preparation.query_id for g in f.actions] == ['q1', 'q2'] and f.count == 1
    assert len(f.acquisitions) == 1


def test_terminal_failed_dispatch_consumes_available_view_unit(workflow_case):
    f = workflow_case; f.settings = settings(1); f.action_modes = ['abort']
    with pytest.raises(RuntimeError, match='PREPARE:'): run(f)
    assert len(f.actions) == 1 and f.count == 1 and not f.acquisitions


def test_ambiguous_new_observation_never_replaces_original_context(workflow_case):
    f = workflow_case; f.reply_modes = ['ambiguous', 'success']
    run(f)
    assert len(f.actions) == 2
    assert f.actions[1].preparation.domain_sha256 == f.domain.domain_sha256
    assert f.actions[1].preparation.task_request_id == f.initial.request.request_id


def test_no_gain_relies_on_independent_owner_result(workflow_case):
    f = workflow_case; f.reply_modes = ['no_gain']
    assert run(f)[0].plan_digest == 'b'*64
    assert len(f.actions) == 1 and not f.fallbacks


@pytest.mark.parametrize('mode', ['nonterminal', 'loaded', 'wrong_plan', 'no_settle'])
def test_invalid_terminal_or_plan_never_observes_or_replans(workflow_case, mode):
    f = workflow_case; f.action_modes = [mode]
    with pytest.raises(RuntimeError, match='PICKUP_RECOVERY_'): run(f)
    assert len(f.actions) == 1 and not f.acquisitions and not f.fallbacks


def test_expired_recovery_never_acquires_after_deadline(workflow_case):
    f = workflow_case; f.expires_on = 'action'
    with pytest.raises(RuntimeError, match='PREPARE:'): run(f)
    assert len(f.actions) == 1 and not f.acquisitions


@pytest.mark.parametrize('damage', ['schema', 'missing_policy', 'count', 'time', 'grid', 'ik'])
def test_configuration_never_invents_production_policy(case_archive, damage):
    value = settings()
    record = dict(schema='pickup_view_recovery_v1', **asdict(value))
    if damage == 'schema': record['schema'] = 'other'
    elif damage == 'missing_policy': record.pop('association_policy')
    elif damage == 'count': record['max_extra_view_motions'] = True
    elif damage == 'time': record['total_recovery_budget_s'] = 0.
    elif damage == 'grid': record['information_sample_shape'] = [0, 4]
    elif damage == 'ik': record['max_ik_seeds'] = 0
    case_archive.save('config_input', record)
    with pytest.raises((ValueError, KeyError)): PickupRecoverySettings.from_record(record)


def test_disabled_config_needs_no_calibrated_policy(case_archive):
    value = dict(schema='pickup_view_recovery_v1', max_extra_view_motions=0)
    case_archive.save('config_input', value)
    assert PickupRecoverySettings.from_record(value).max_extra_view_motions == 0
