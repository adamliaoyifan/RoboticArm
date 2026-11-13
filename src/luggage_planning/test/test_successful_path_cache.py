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
