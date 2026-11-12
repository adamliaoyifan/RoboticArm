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
from luggage_planning.joint_operation_contracts import seal_operation_plan
from luggage_planning.joint_operation_issuance import IssuedOperationRegistry
from luggage_planning.joint_operation_scenes import build_operation_scenes
from luggage_planning.placement_prior_snapshot import build_private_snapshot, proposal_scene_hash
from luggage_planning.preparation_refresh import (
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


@pytest.mark.parametrize('mode', ['never_stable', 'advances_once', 'baseline_changes'])
def test_source_changes_during_native_certification_retry_exact_original_legs(mode):
    f = setup()
    f.state.current = source(9)
    if mode == 'advances_once':
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
    from luggage_planning.joint_operation_prepare_node import JointOperationPrepareNode
    calls = []
    captured = SimpleNamespace(surface='surface', volume='volume', source='source')
    root = SimpleNamespace(revision='scene', observer=SimpleNamespace(
        capture=lambda **kwargs:(calls.append(kwargs) or captured)))
    request = SimpleNamespace(request_id='req', expected_map_epoch=3, expected_map_revision=7)
    assert JointOperationPrepareNode._capture_map(root, request, 20., latest=True) == ('surface', 'volume', 'source')
    JointOperationPrepareNode._capture_map(root, request, 20.)
    assert [c['expected_revision'] for c in calls] == [None, 7]
    assert all(c['expected_epoch'] == 3 and c['deadline'] == 20. for c in calls)
