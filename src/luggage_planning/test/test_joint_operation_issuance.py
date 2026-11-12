"""Functional issuance ownership/retention; no collision-certification claim."""
from dataclasses import asdict, replace
import json

import pytest

from luggage_planning.joint_operation.contracts import seal_operation_plan
from luggage_planning.joint_operation.issuance import IssuedOperationRegistry
from test_joint_operation_contracts import example_plan


def plan(request='req', generation=2, certificate='cert'):
    draft=example_plan()
    return seal_operation_plan(replace(draft,certificate_id=certificate,
        identity=replace(draft.identity,request_id=request,generation=generation),
        legs=tuple(replace(leg,trajectory_request_id=request) for leg in draft.legs)))


def registry(**settings):
    options=dict(max_plans=16,max_bytes=16*1024*1024,lifetime_s=300.)
    options.update(settings)
    return IssuedOperationRegistry(**options)


def contact_binding(p):
    from luggage_perception.frozen_suction_observation import FrozenObservationBinding
    i=p.identity
    return FrozenObservationBinding(i.request_id,i.observation_id,'a'*32,i.box_id,i.generation,
        i.observation_stamp_ns,i.contact_model_hash,'b'*64,'c'*64,'d'*64)


def test_owner_preserves_independent_contact_provenance_without_read_renewal():
    now=[10.];store=registry(clock=lambda:now[0],lifetime_s=5.);p=plan();b=contact_binding(p)
    store.issue(p,contact_binding=b)
    assert store.contact_binding(p) is b
    now[0]=14.;store.issue(p,contact_binding=b);assert store.contact_binding(p)==b
    now[0]=15.;assert store.contact_binding(p) is None and not store.current(p)


@pytest.mark.parametrize('field,value',[('request_id','other'),('observation_id','other'),
    ('box_id','other'),('generation',4),('stamp_ns',999),('contact_model_hash','other')])
def test_issuance_cannot_bind_a_different_contact_observation(field,value):
    store=registry();p=plan();b=replace(contact_binding(p),**{field:value})
    with pytest.raises(ValueError,match='BINDING_IDENTITY_MISMATCH'):store.issue(p,contact_binding=b)
    assert not store.current(p)


def test_existing_issued_plan_cannot_silently_change_or_drop_its_observation():
    store=registry();p=plan();b=contact_binding(p);store.issue(p,contact_binding=b)
    for different in (None,replace(b,retention_id='e'*32)):
        with pytest.raises(ValueError,match='REBIND_REFUSED'):store.issue(p,contact_binding=different)
    assert store.contact_binding(p) is b


def test_issued_byte_budget_includes_observation_provenance():
    p=plan();size=len(json.dumps(asdict(p),sort_keys=True,separators=(',',':'),allow_nan=False).encode())
    store=registry(max_bytes=size)
    with pytest.raises(ValueError,match='EXCEEDS_BYTE_LIMIT'):store.issue(p,contact_binding=contact_binding(p))
    assert not store.current(p)


def test_only_owner_publication_makes_exact_plan_current():
    store=registry();p=plan()
    assert not store.current(p) and store.lookup(p.plan_digest) is None
    store.issue(p)
    assert store.current(p) and store.lookup(p.plan_digest) is p
    assert not registry().current(p), 'process restart must lose old issuance'


@pytest.mark.parametrize('field,value',[
    ('certificate_id','forged'),('candidate_id','other'),('plan_digest','0'*64)])
def test_altered_client_payload_does_not_inherit_issuance(field,value):
    store=registry();p=plan();store.issue(p)
    assert not store.current(replace(p,**{field:value}))
    assert store.current(p)


def test_self_consistent_resealed_client_plan_is_still_not_issued():
    store=registry();p=plan();store.issue(p)
    forged=seal_operation_plan(replace(p,certificate_id='client-resealed',plan_digest=''))
    assert not store.current(forged)


def test_checks_and_duplicate_publication_never_renew_lifetime():
    now=[10.];store=registry(clock=lambda:now[0],lifetime_s=5.);p=plan();store.issue(p)
    now[0]=14.;assert store.current(p);store.issue(p)
    now[0]=15.;assert not store.current(p)
    assert store._bytes==0


def test_successful_replacement_retires_old_plan_and_rejects_older_generation():
    store=registry();old=plan();new=plan(generation=3);store.issue(old);store.issue(new)
    assert not store.current(old) and store.current(new)
    with pytest.raises(ValueError,match='OLDER_ISSUED_GENERATION'):store.issue(old)
    assert store.current(new)
    replacement=plan(generation=3,certificate='new-certificate');store.issue(replacement)
    assert not store.current(new) and store.current(replacement)


def test_count_bound_evicts_oldest_without_lookup_promoting_it():
    store=registry(max_plans=2);a,b,c=(plan(r) for r in ('a','b','c'))
    store.issue(a);store.issue(b);assert store.current(a);store.issue(c)
    assert not store.current(a) and store.current(b) and store.current(c)
    assert len(store._entries)==2


def test_byte_bound_and_oversized_refusal_preserve_valid_publication():
    a,b=plan('a'),plan('b')
    size=len(json.dumps(asdict(a),sort_keys=True,separators=(',',':'),allow_nan=False).encode())
    store=registry(max_bytes=size);store.issue(a);store.issue(b)
    assert not store.current(a) and store.current(b) and store._bytes==size
    oversized=plan('huge-request-id'*100)
    with pytest.raises(ValueError,match='EXCEEDS_BYTE_LIMIT'):store.issue(oversized)
    assert store.current(b) and store._bytes==size


@pytest.mark.parametrize('limits',[dict(max_plans=0),dict(max_plans=True),dict(max_bytes=0),
    dict(lifetime_s=0.),dict(lifetime_s=float('nan')),dict(lifetime_s=True)])
def test_explicit_resource_configuration_is_validated(limits):
    with pytest.raises(ValueError):registry(**limits)


def test_failed_issue_does_not_replace_original():
    store=registry();p=plan();store.issue(p)
    with pytest.raises(ValueError):store.issue(replace(p,certificate_id='unsealed'))
    assert store.current(p)


def test_expiry_while_checking_payload_cannot_return_positive(monkeypatch):
    import luggage_planning.joint_operation.issuance as module
    now=[10.];store=registry(clock=lambda:now[0],lifetime_s=5.);p=plan();store.issue(p)
    original=module.admit_operation_plan
    def slow(*args):
        result=original(*args);now[0]=15.;return result
    monkeypatch.setattr(module,'admit_operation_plan',slow)
    assert not store.current(p)

@pytest.mark.parametrize('lifetime,expected',[(300.,False),(960.,True)])
def test_issued_storage_survives_composed_execution_without_lookup_renewal(lifetime,expected):
    now=[0.];store=registry(clock=lambda:now[0],lifetime_s=lifetime);p=plan()
    store.issue(p,contact_binding=contact_binding(p))
    now[0]=310.;assert store.current(p)==expected
    now[0]=lifetime;assert not store.current(p)
