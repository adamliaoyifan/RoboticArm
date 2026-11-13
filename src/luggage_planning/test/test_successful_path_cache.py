from dataclasses import replace
import pytest
from test_joint_operation_planner import ports, prepare
from luggage_planning.joint_operation.path_cache import SuccessfulPathCache


def test_success_only_roundtrip_and_new_identity(tmp_path):
    bundle=ports(); original=prepare(bundle).plan
    cache=SuccessfulPathCache(str(tmp_path))
    assert not cache.remember(original,success=False,motion_terminal=True,may_have_payload=False)
    assert cache.proposal(bundle[0],original.start_joints,bundle[1]) is None
    assert cache.remember(original,success=True,motion_terminal=True,may_have_payload=False)
    identity=replace(bundle[0],request_id='another',map_revision=200)
    draft=cache.proposal(identity,original.start_joints,bundle[1]).plan
    assert draft.identity == identity
    assert draft.certificate_id == 'pending-native-certification'
    assert draft.legs[0].knots == original.legs[0].knots
    assert cache.proposal(replace(identity,model_sha256='changed'),original.start_joints,bundle[1]) is None
    assert cache.proposal(identity,(.2,)*6,bundle[1]) is None


@pytest.mark.parametrize('native_accepts',[True,False])
def test_hit_always_certified_and_refusal_falls_back(tmp_path,native_accepts):
    bundle=ports(); plan=prepare(bundle).plan
    cache=SuccessfulPathCache(str(tmp_path))
    cache.remember(plan,success=True,motion_terminal=True,may_have_payload=False)
    planner=bundle[-1];planner.path_cache=cache
    endpoint_count=len(bundle[2].calls)
    if not native_accepts:
        cert=bundle[4];original=cert.certify
        def refuse_cached(draft,candidate,**kwargs):
            if len(bundle[2].calls)==endpoint_count:
                return replace(original(draft,candidate,**kwargs),passed=False,reason='CURRENT_COLLISION')
            return original(draft,candidate,**kwargs)
        cert.certify=refuse_cached
    result=prepare(bundle)
    assert result.plan is not None
    if native_accepts:
        assert len(bundle[2].calls)==endpoint_count
        assert any(row[1]=='SUCCESSFUL_PATH_RECERTIFIED' for row in result.trace)
    else:
        assert len(bundle[2].calls)>endpoint_count
        assert any(row[1]=='CACHED_PATH_REFUSED_FALLBACK' for row in result.trace)


def test_corruption_is_a_miss(tmp_path):
    bundle=ports();plan=prepare(bundle).plan
    cache=SuccessfulPathCache(str(tmp_path))
    cache.remember(plan,success=True,motion_terminal=True,may_have_payload=False)
    next(tmp_path.glob('*.gz')).write_bytes(b'broken')
    assert cache.proposal(bundle[0],plan.start_joints,bundle[1]) is None


@pytest.mark.parametrize('native_accepts', [True, False])
def test_size_update_reuses_only_knots_and_certifies_current_geometry(tmp_path, native_accepts):
    """Port behaviour only: this double does not establish physical containment."""
    import hashlib
    from st09_contracts.attachment_geometry import freeze_geometry
    from test_joint_operation_scenes import baseline, compose
    bundle = list(ports())
    old = prepare(bundle).plan
    cache = SuccessfulPathCache(str(tmp_path))
    cache.remember(old, success=True, motion_terminal=True, may_have_payload=False)
    geometry = old.predicted_attachment
    enlarged = freeze_geometry((.552, .40, .25), geometry.t_tool_box_translation,
        geometry.t_tool_box_quaternion_xyzw, geometry.conservative.limits)
    scenes = compose(replace(baseline(), map_revision=300), enlarged)
    bundle[0] = replace(bundle[0], request_id='fresh', observation_stamp_ns=50, map_revision=300,
        snapshot_id=hashlib.sha256(repr(scenes.base_identity).encode()).hexdigest())
    bundle[1] = replace(bundle[1], predicted_attachment=enlarged, scenes=scenes)
    bundle[2].evidence = replace(bundle[2].evidence, attachment_digest=enlarged.digest(),
        phase_scene_digests=scenes.digests)
    bundle[-1].path_cache = cache
    native = bundle[4].certify
    first = len(bundle[4].calls)

    def inspect(draft, candidate, **kwargs):
        assert draft.identity == bundle[0]
        assert draft.predicted_attachment == candidate.predicted_attachment == enlarged
        receipt = native(draft, candidate, **kwargs)
        return replace(receipt, passed=False, reason='CURRENT_COLLISION') if not native_accepts else receipt

    bundle[4].certify = inspect
    result = prepare(bundle)
    assert len(bundle[4].calls) > first
    assert (result.plan is not None) == native_accepts
    assert any(row[1] == ('SUCCESSFUL_PATH_RECERTIFIED' if native_accepts
                         else 'CACHED_PATH_REFUSED_FALLBACK') for row in result.trace)
