"""Latest-map orchestration with real CDR scenes and explicit native doubles.

Recipes describe admitted fixture snapshots; these are not sensed ULD data or
physical native certification. No IK, retimer or controller is involved.
"""
import copy
from dataclasses import asdict, fields, replace
import hashlib
import json
from types import SimpleNamespace

import pytest
from moveit_msgs.msg import PlanningScene
from rclpy.serialization import deserialize_message, serialize_message

from luggage_mapping.observer import ObserverVolume
from luggage_mapping.source import OperationMapSource
from luggage_mapping.surface import ConfirmedSurface, freeze
from luggage_planning.joint_operation.contracts import seal_operation_plan
from luggage_planning.joint_operation.issuance import IssuedOperationRegistry
from luggage_planning.joint_operation.scenes import build_operation_scenes
from luggage_planning.placement_prior.snapshot import build_private_snapshot, proposal_scene_hash
from luggage_planning.joint_operation.preparation_refresh import (
    LatestMapFinalizer, RefreshCapture, require_source_progress,
)
from test_joint_operation_planner import ports, prepare


def source(revision):
    return OperationMapSource('scene', 'hull', 3, revision, revision, revision*10**9,
        'a'*64, 'ros', 4, hashlib.sha256(str(revision).encode()).hexdigest())


def capture(revision, baseline, transform, *, blocked=False, support_changed=False):
    current = source(revision)
    record = dict(scene_revision='scene', geometry_hash='hull', map_revision=revision,
        content_digest=current.content_sha256, frame_id='container_link', resolution_m=.1,
        origin_xy_container=[-2., -2.], floor_z_container=.7, ceiling_z_container=2.7,
        state=[['free']*40 for _ in range(40)],
        surface_z_container=[[.7]*40 for _ in range(40)], confidence=[[1.]*40 for _ in range(40)])
    if support_changed:record['confidence'][20][30] = .99
    cells = bytearray(b'\1'*(40*40*20))
    if blocked:
        cells[0], cells[1] = 0, 2  # Remote unknown and occupied, not target-box cells.
    volume = ObserverVolume(current, .1, (-2., -2., .7), (40, 40, 20), bytes(cells), (True,)*len(cells))
    surface = ConfirmedSurface(freeze(record), b'explicit-admitted-fixture-surface',
        freeze(dict(source='observer', observer_source=asdict(current))))
    snap = build_private_snapshot(surface, baseline, 'model', transform, collision_margin_m=.05, volume=volume)
    return RefreshCapture(surface, volume, current, baseline, snap)


def setup(*, changed=True, blocked=False, support_changed=False, attempts=2):
    bundle = ports()
    result = prepare(bundle)
    _, candidate, endpoints, connections, certifier, clock, _ = bundle
    old_baseline = deserialize_message(candidate.scenes.pick.scene_cdr, PlanningScene)
    # Phase fixture is used only as a full baseline; the owned box is replaced
    # again by the real phase builder. All unrelated geometry is retained.
    old = capture(7, old_baseline, candidate.scenes.pick.world_from_container)
    scenes = build_operation_scenes(old.snapshot, pickup_object_id='pickup_box',
        geometry=candidate.predicted_attachment, world_from_pick_box=candidate.world_from_pick_box,
        world_from_settled_box=candidate.world_from_settled_box)
    candidate = replace(candidate, scenes=scenes)
    plan = seal_operation_plan(replace(result.plan, identity=replace(result.plan.identity,
        snapshot_id=hashlib.sha256(repr(old.snapshot.identity).encode()).hexdigest()), plan_digest=''))
    baseline = copy.deepcopy(old_baseline)
    if changed:
        extra = copy.deepcopy(baseline.world.collision_objects[0]);extra.id = 'new-measured-obstacle'
        baseline.world.collision_objects.append(extra)
    fresh = capture(8 if changed else 7, baseline, old.snapshot.world_from_container,
        blocked=blocked, support_changed=support_changed)
    state = SimpleNamespace(current=fresh.source, captures=[], reads=[], observation=True)
    def acquire(deadline):state.captures.append(deadline);return fresh
    def read(deadline):state.reads.append(deadline);return baseline
    owner = LatestMapFinalizer(original_surface=old.surface, original_source=old.source,
        original_snapshot=old.snapshot, baseline_hash=proposal_scene_hash(old_baseline),
        capture=acquire, read_baseline=read, source_current=lambda value, deadline:value == state.current,
        observation_current=lambda deadline:state.observation, certification=certifier,
        attempts=attempts, clock=clock)
    certifier.calls.clear()
    state.initial_endpoint_calls, state.initial_connection_calls = len(endpoints.calls), len(connections.calls)
    return SimpleNamespace(owner=owner, plan=plan, candidate=candidate, old=old, fresh=fresh, state=state,
        certifier=certifier, clock=clock, endpoints=endpoints, connections=connections,
        inputs=SimpleNamespace(snapshot=old.snapshot))


def finalize(f):return f.owner.finalize(f.plan, f.candidate, f.inputs, 30.)


def test_latest_certification_changes_only_map_identity_and_receipt():
    f = setup(blocked=True)
    result = finalize(f)
    assert result.reason == 'PLAN_READY' and result.map_source == f.fresh.source
    final = result.plan
    assert final.identity.map_revision == 8 and final.identity.snapshot_id != f.plan.identity.snapshot_id
    assert final.plan_digest != f.plan.plan_digest
    assert len(final.legs) == 6 and final.legs is f.plan.legs
    changed = {'identity', 'certificate_id', 'plan_digest'}
    assert all(getattr(final, field.name) == getattr(f.plan, field.name)
               for field in fields(final) if field.name not in changed)
    assert replace(final.identity, map_revision=f.plan.identity.map_revision,
                   snapshot_id=f.plan.identity.snapshot_id) == f.plan.identity
    assert len(f.certifier.calls) == 1
    draft, refreshed, kwargs = f.certifier.calls[0]
    assert draft.legs is f.plan.legs and draft.certificate_id == 'pending-native-certification'
    assert kwargs['deadline'] == 30.
    for phase in ('pick', 'carry', 'retreat'):
        scene = deserialize_message(refreshed.scenes.for_phase(phase).scene_cdr, PlanningScene)
        ids = {obj.id for obj in scene.world.collision_objects}
        assert {'new-measured-obstacle', 'pickup_box_neighbor',
                'placement-prior-occ-voxel-0', 'placement-prior-occ-voxel-1'} <= ids
    assert len(f.endpoints.calls) == f.state.initial_endpoint_calls
    assert len(f.connections.calls) == f.state.initial_connection_calls
    registry = IssuedOperationRegistry(max_plans=2, max_bytes=10**6, lifetime_s=10.)
    registry.issue(final, map_source=result.map_source)
    with pytest.raises(ValueError, match='ISSUED_MAP_SOURCE_IDENTITY_MISMATCH'):
        registry.issue(final, map_source=f.old.source)


def test_unchanged_source_uses_existing_gate_without_capture_or_certification():
    f = setup(changed=False)
    result = finalize(f)
    assert result.plan is f.plan and result.map_source == f.old.source
    assert not f.state.captures and not f.certifier.calls


def test_baseline_change_requires_new_certificate_even_when_o_has_not_changed():
    f = setup(changed=False)
    baseline = copy.deepcopy(f.fresh.baseline);baseline.name = 'new-world-semantics'
    fresh = capture(7, baseline, f.old.snapshot.world_from_container)
    f.owner.read_baseline = lambda deadline:baseline
    f.owner.capture = lambda deadline:fresh
    result = finalize(f)
    assert result.reason == 'PLAN_READY' and result.plan.identity.map_revision == 7
    assert result.plan.identity.snapshot_id != f.plan.identity.snapshot_id
    assert len(f.certifier.calls) == 1 and result.plan.legs is f.plan.legs


@pytest.mark.parametrize('damage', ['model', 'transform', 'surface_hash', 'volume_source', 'record_revision'])
def test_misbound_complete_capture_is_inactive(damage):
    from luggage_planning.placement_prior.contracts import Transform
    f = setup()
    if damage == 'model':fresh = replace(f.fresh, snapshot=replace(f.fresh.snapshot, model_hash='other'))
    if damage == 'transform':
        fresh = replace(f.fresh, snapshot=replace(f.fresh.snapshot,
            world_from_container=Transform((.1, 0., 0.), (0., 0., 0., 1.))))
    if damage == 'surface_hash':
        fresh = replace(f.fresh, snapshot=replace(f.fresh.snapshot, surface_content_hash='bad'))
    if damage == 'volume_source':fresh = replace(f.fresh, volume=replace(f.fresh.volume, source=f.old.source))
    if damage == 'record_revision':
        record = f.fresh.surface.record();record['map_revision'] = 9
        fresh = replace(f.fresh, surface=replace(f.fresh.surface, record_json=freeze(record)))
    f.owner.capture = lambda deadline:fresh
    result = finalize(f)
    assert result.reason == 'REFRESH_CAPTURE_IDENTITY_MISMATCH' and result.plan is None
    assert not f.certifier.calls


@pytest.mark.parametrize('cell_state,reason', [(0, 'UNKNOWN'), (2, 'OCCUPIED')])
def test_new_target_volume_obstruction_refuses_even_if_native_double_would_pass(cell_state, reason):
    f = setup()
    cells = bytearray(f.fresh.volume.states);cells[20+40*30] = cell_state
    fresh = replace(f.fresh, volume=replace(f.fresh.volume, states=bytes(cells)))
    f.owner.capture = lambda deadline:fresh
    result = finalize(f)
    assert result.reason == 'REFRESH_PLACEMENT_VOLUME_' + reason and result.plan is None
    assert not f.certifier.calls


@pytest.mark.parametrize('damage', ['refused', 'identity', 'candidate', 'draft', 'phases', 'empty', 'old_receipt'])
def test_native_receipt_cannot_be_reused_or_misbound(damage):
    f = setup(blocked=True)
    def alter(receipt):
        return dict(refused=lambda:replace(receipt, passed=False),
            identity=lambda:replace(receipt, identity=f.plan.identity),
            candidate=lambda:replace(receipt, candidate_id='other'),
            draft=lambda:replace(receipt, draft_digest=f.plan.plan_digest),
            phases=lambda:replace(receipt, phase_scene_digests=f.candidate.scenes.digests),
            empty=lambda:replace(receipt, certificate_id=' '),
            old_receipt=lambda:replace(receipt, identity=f.plan.identity,
                draft_digest=f.plan.plan_digest, phase_scene_digests=f.candidate.scenes.digests))[damage]()
    f.certifier.alter = alter
    result = finalize(f)
    assert result.reason == 'REFRESH_NATIVE_CERTIFICATE_REFUSED' and result.plan is None


@pytest.mark.parametrize('key,value', [('epoch', 4), ('config_digest', 'b'*64), ('geometry_hash', 'new'),
    ('validity_generation', 5), ('scene_revision', 'new'), ('map_revision', 6),
    ('observed_revision', 6), ('last_integrated_stamp_ns', 6*10**9), ('content_sha256', 'b'*64)])
def test_reset_configuration_or_backwards_source_refuses(key, value):
    original = source(7)
    with pytest.raises(ValueError, match='REFRESH_SOURCE_'):
        require_source_progress(original, replace(original, **{key:value}))


def test_changed_support_refuses_before_native_certification():
    f = setup(support_changed=True)
    result = finalize(f)
    assert result.reason == 'REFRESH_PLACEMENT_SUPPORT_CHANGED' and result.plan is None
    assert not f.certifier.calls


def test_new_integration_without_changed_keys_can_retain_map_revision():
    original = source(7)
    current = replace(original, observed_revision=8, last_integrated_stamp_ns=8*10**9,
        content_sha256='b'*64)
    require_source_progress(original, current)


@pytest.mark.parametrize('mode', ['never_stable', 'advances_once', 'baseline_changes'])
def test_source_changes_during_native_certification_retry_exact_original_legs(mode):
    f = setup()
    f.state.current = source(9)
    if mode == 'advances_once':
        second = capture(9, f.fresh.baseline, f.fresh.snapshot.world_from_container)
        def acquire(deadline):
            f.state.captures.append(deadline)
            return f.fresh if len(f.state.captures) == 1 else second
        f.owner.capture = acquire
        f.owner.source_current = lambda value, deadline:len(f.certifier.calls) >= 2
    elif mode == 'baseline_changes':
        f.owner.source_current = lambda value, deadline:value == f.fresh.source
        changed = copy.deepcopy(f.fresh.baseline);changed.name = 'changed-again'
        f.owner.read_baseline = lambda deadline:changed
    result = finalize(f)
    assert len(f.certifier.calls) == 2 and len(f.state.captures) == 2
    assert all(draft.legs is f.plan.legs for draft, _, _ in f.certifier.calls)
    assert result.reason == ('PLAN_READY' if mode == 'advances_once' else 'REFRESH_SOURCE_UNSTABLE')
    assert (result.plan is not None) is (mode == 'advances_once')
    if result.plan:assert result.plan.identity.map_revision == 9 and result.map_source == source(9)


@pytest.mark.parametrize('mode', ['deadline', 'observation', 'capture_failure', 'cert_timeout', 'late_observation', 'late_deadline'])
def test_faults_return_inactive_without_fallback_to_old_certificate(mode):
    f = setup()
    if mode == 'deadline':f.clock.now = 30.
    if mode == 'observation':f.state.observation = False
    if mode == 'capture_failure':
        def failed(deadline):raise ValueError('EXPIRED_OBSERVER_LEASE')
        f.owner.capture = failed
    if mode == 'cert_timeout':
        def failed(*args, **kwargs):raise TimeoutError('native deadline')
        f.certifier.certify = failed
    if mode in ('late_observation', 'late_deadline'):
        def change(receipt):
            if mode == 'late_observation':f.state.observation = False
            else:f.clock.now = 30.
            return receipt
        f.certifier.alter = change
    result = finalize(f)
    assert result.plan is None and result.map_source is None and result.reason != 'PLAN_READY'


@pytest.mark.parametrize('attempts', [0, 9, 1.5, True])
def test_retry_budget_is_explicit_and_bounded(attempts):
    with pytest.raises(ValueError, match='BOUNDED_REFRESH_ATTEMPTS_REQUIRED'):setup(attempts=attempts)


def test_latest_capture_preserves_epoch_but_does_not_request_old_revision():
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    calls = []
    captured = SimpleNamespace(surface='surface', volume='volume', source='source')
    root = SimpleNamespace(revision='scene', observer=SimpleNamespace(
        capture=lambda **kwargs:(calls.append(kwargs) or captured)))
    request = SimpleNamespace(request_id='req', expected_map_epoch=3, expected_map_revision=7)
    assert JointOperationPrepareNode._capture_map(root, request, 20., latest=True) == ('surface', 'volume', 'source')
    JointOperationPrepareNode._capture_map(root, request, 20.)
    assert [c['expected_revision'] for c in calls] == [None, 7]
    assert all(c['expected_epoch'] == 3 and c['deadline'] == 20. for c in calls)


@pytest.mark.parametrize('mode', ['pass', 'native_refuses', 'observation_revoked', 'capture_failure'])
def test_real_root_finalization_binds_latest_issuance_and_releases_lock(mode):
    import threading
    import numpy as np
    from luggage_msgs.srv import PrepareJointOperationV2
    from luggage_planning.joint_operation.adapters import operation_plan_to_message
    from luggage_planning.joint_operation.prepare_node import JointOperationPrepareNode
    from luggage_planning.joint_operation.preparation_profile import PreparationBudget
    from test_preparation_profile import query_and_inputs
    f = setup(blocked=True)
    request, context, _ = query_and_inputs(30.)
    request.request_id = f.plan.identity.request_id
    request.expected_map_epoch = f.old.source.epoch
    calls, records, leases = [], [], []
    def capture_map(query, deadline, **kwargs):
        leases.append(kwargs.get('latest', False))
        if kwargs.get('latest'):
            if mode == 'capture_failure':raise ValueError('OBSERVER_CAPTURE_FAILED')
            return f.fresh.surface, f.fresh.volume, f.fresh.source
        return f.old.surface, f.old.volume, f.old.source
    observation = SimpleNamespace(binding='frozen-binding')
    def retain(*args, **kwargs):
        revoked = mode == 'observation_revoked' and bool(kwargs.get('expected_observation'))
        return SimpleNamespace(retained=not revoked, observation=observation, reason='OBSERVATION_REVOKED')
    def run(query, response, inputs, **kwargs):
        result = kwargs['finalize'](f.plan, f.candidate, inputs, kwargs['deadline'])
        response.reason, response.trace_json = result.reason, json.dumps(result.trace)
        if result.plan:
            response.plan = operation_plan_to_message(result.plan, result.plan.identity, result.plan.plan_digest)
            response.success = True
        return response
    if mode == 'native_refuses':f.certifier.alter = lambda receipt:replace(receipt, passed=False)
    root = SimpleNamespace(_active=threading.Lock(), budget=PreparationBudget(60.),
        model=SimpleNamespace(model_sha256='model', urdf=SimpleNamespace(tcp=lambda *args:np.eye(4)), path='fixture'),
        _current=lambda *args, **kwargs:context.domain,
        _record=lambda kind, value:records.append(kind), _capture_map=capture_map,
        _scene=lambda deadline:f.fresh.baseline if True in leases else f.old.baseline,
        _placements=lambda *args:(), observed_contacts=SimpleNamespace(retain=retain),
        observer=SimpleNamespace(current=lambda source, **kwargs:source == f.fresh.source),
        preparation=SimpleNamespace(prepare=run, planner=SimpleNamespace(certification=f.certifier)),
        refresh_attempts=2, revision='scene', margin=.05, container=f.old.snapshot.world_from_container,
        pickup_id='pickup_box', store=SimpleNamespace(current=lambda:None),
        issued=SimpleNamespace(issue=lambda plan, **kwargs:calls.append((plan, kwargs))),
        get_logger=lambda:SimpleNamespace(error=lambda text:None))
    response = JointOperationPrepareNode._prepare(root, request, PrepareJointOperationV2.Response())
    assert response.success is (mode == 'pass'), response.reason
    assert bool(calls) is (mode == 'pass')
    assert root._active.acquire(blocking=False)
    root._active.release()
    assert records == ['prepare_request', 'prepare_response']
    if calls:
        plan, bound = calls[0]
        assert plan.identity.map_revision == response.plan.map_revision == 8
        assert bound['map_source'] == f.fresh.source and bound['contact_binding'] == observation.binding
        assert plan.legs == f.plan.legs
    else:assert not response.plan.plan_digest


@pytest.mark.parametrize('accept', [True, False])
def test_real_core_uses_selected_candidate_and_serializes_finalized_identity(accept):
    from luggage_msgs.srv import PrepareJointOperationV2
    from luggage_perception.directed_suction_service import DirectedSuctionService
    from luggage_planning.joint_operation.contacts import contact_query, contact_evidence
    from luggage_planning.joint_operation.contracts import OperationBarrier, JointOperationPlan
    from luggage_planning.joint_operation.planner import timed_operation_leg
    from luggage_planning.joint_operation.preparation import JointOperationPreparation
    from luggage_planning.placement_prior.contracts import PlacementCandidate, Transform
    from luggage_planning.joint_operation.preparation_profile import PreparationBudget
    from luggage_planning.joint_operation.preparation_refresh import FinalizedPreparation
    from st09_contracts.payload_envelope import EnvelopeLimits
    from test_joint_operation_candidates import placed_pose, service_response
    from test_preparation_profile import query_and_inputs
    request, context, inputs = query_and_inputs(30.)
    service = DirectedSuctionService(clock=lambda:0.);service.replace_context(context)
    class Contacts:
        def evaluate(self, domain, detection, queries, model, **kwargs):
            query = contact_query(domain, detection, queries, model, budget_s=1.)
            return contact_evidence(service.handle(query, service_response()), query, domain, model)
    selected = []
    def planning(identity, joints, candidates, **kwargs):
        candidate = next(candidates);selected.append(candidate)
        rows = ((0.,)*6, (.01,)*6)
        phases = ('pick','pick','carry','carry','carry','retreat')
        names = ('align','pick_approach','pick_lift','carry','place_descent','place_retreat')
        legs = tuple(timed_operation_leg(name, phase, ((i*.01,)*6, ((i+1)*.01,)*6), identity, .2, .1)
                     for i, (name, phase) in enumerate(zip(names, phases)))
        plan = seal_operation_plan(JointOperationPlan(identity, candidate.candidate_id, 'fixture-certificate',
            candidate.world_from_pick_box,candidate.world_from_pick_tool,
            candidate.world_from_release_box,candidate.world_from_release_tool,candidate.world_from_settled_box,
            candidate.predicted_attachment,rows[0],legs,
            (OperationBarrier(2,'attachment_confirmed'), OperationBarrier(5,'release_confirmed')),''))
        return SimpleNamespace(status='PLAN_READY',plan=plan,trace=())
    def finalizing(plan, candidate, current, deadline):
        assert candidate is selected[0] and current is inputs and deadline == 40.
        if not accept:return FinalizedPreparation(None,None,'REFRESH_SOURCE_UNSTABLE')
        final = seal_operation_plan(replace(plan, identity=replace(plan.identity,map_revision=8,snapshot_id='fresh'),
            certificate_id='new-fixture-certificate',plan_digest=''))
        return FinalizedPreparation(final,None,'PLAN_READY',(('map_finalization','fixture'),))
    inputs = replace(inputs, placements=(PlacementCandidate('slot', placed_pose(0.)),),
        world_from_reference_tool=Transform((0.,-.8,1.),(1.,0.,0.,0.)))
    owner = JointOperationPreparation(Contacts(),SimpleNamespace(reserve_s=.2,prepare=planning),
        context._model,EnvelopeLimits(5.,(.1,.1,.1),(.01,.02),.005),max_center_checks=64,
        budget=PreparationBudget(60.),clock=lambda:10.)
    def forbidden(*args):pytest.fail('O finalization fell through G strict gate')
    response = owner.prepare(request,PrepareJointOperationV2.Response(),inputs,deadline=40.,
        revalidate=forbidden,finalize=finalizing)
    assert response.success is accept, response.reason
    if accept:assert response.plan.map_revision == 8 and response.plan.snapshot_id == 'fresh'
    else:assert response.reason == 'REFRESH_SOURCE_UNSTABLE' and not response.plan.plan_digest
