"""Observed contact diversity and exact-position recovery ordering."""
from dataclasses import replace
import math

import numpy as np
import pytest

from luggage_perception.observed_contact_footprint import check_observed_contact_footprint
from luggage_perception.directed_suction_service import DirectedSuctionService
from luggage_planning.joint_operation.candidates import (
    _contact_matrix, group_placement_positions, propose_contact, propose_contacts,
)
from luggage_planning.placement_prior.contracts import PlacementCandidate, Transform
from luggage_planning.joint_operation.contacts import contact_evidence, contact_query
from test_directed_suction_service import request_and_context
from test_joint_operation_candidates import placed_pose


@pytest.mark.parametrize('yaw',[0.,math.pi/2,math.pi,-math.pi/2])
def test_alternate_contacts_preserve_primary_and_use_only_full_observed_support(yaw):
    _,context=request_and_context()
    domain,model=context.domain,context._model
    args=dict(world_from_place_box=placed_pose(yaw),
        desired_world_from_tool=Transform((0.,-.8,1.),(1.,0.,0.,0.)),max_center_checks=64)
    primary=propose_contact(domain,model,query_id='primary',**args)
    alternatives=tuple(propose_contacts(domain,model,query_id='alternatives',max_proposals=3,**args))
    assert len(alternatives)==3
    assert alternatives[0].center_uv_top_m==primary.center_uv_top_m
    assert alternatives[0].yaw_rad==primary.yaw_rad
    assert len({p.center_uv_top_m for p in alternatives})==3
    assert sum(p.checked_centres for p in alternatives)<=64
    for proposal in alternatives:
        assert proposal.reason=='OBSERVED_FOOTPRINT_PROPOSAL'
        assert check_observed_contact_footprint(domain,model,
            _contact_matrix(domain,proposal.center_uv_top_m,proposal.yaw_rad)).contained
    request,_=request_and_context()
    query=contact_query(domain,request.detection,[p.to_message() for p in alternatives],model,budget_s=1.)
    service=DirectedSuctionService(clock=lambda:0.)
    service.replace_context(context)
    from luggage_msgs.srv import EvaluateDirectedSuctionV2
    response=service.handle(query,EvaluateDirectedSuctionV2.Response())
    evidence=contact_evidence(response,query,domain,model)
    assert evidence.complete and evidence.contacts
    assert evidence.domain_sha256==domain.digest()
    assert len(evidence.verdicts)==len(alternatives)


@pytest.mark.parametrize('checks',[1,2,8])
def test_contacts_share_one_geometric_budget_and_never_fill_missing_support(checks):
    _,context=request_and_context()
    domain,model=context.domain,context._model
    common=dict(query_id='bounded',world_from_place_box=placed_pose(0.),
        desired_world_from_tool=Transform((0.,-.8,1.),(1.,0.,0.,0.)),
        max_center_checks=checks,max_proposals=4)
    candidates=tuple(propose_contacts(domain,model,**common))
    assert sum(p.checked_centres for p in candidates)<=checks
    assert len(candidates)<=min(4,checks)
    unknown=replace(domain,support=bytes(len(domain.support)))
    refused=tuple(propose_contacts(unknown,model,**common))
    assert refused and all(p.reason!='OBSERVED_FOOTPRINT_PROPOSAL' for p in refused)
    assert not np.any(unknown.cells())


def test_alternatives_do_not_cross_an_observed_internal_hole():
    _,context=request_and_context()
    domain,model=context.domain,context._model
    cells=domain.cells().copy()
    centre=tuple(n//2 for n in cells.shape)
    cells[centre]=False
    domain=replace(domain,support=cells.astype(np.uint8).tobytes())
    candidates=tuple(propose_contacts(domain,model,query_id='hole',world_from_place_box=placed_pose(0.),
        desired_world_from_tool=Transform((0.,-.8,1.),(1.,0.,0.,0.)),
        max_center_checks=64,max_proposals=4))
    assert domain.cells()[centre]==False
    assert any(p.reason=='OBSERVED_FOOTPRINT_PROPOSAL' for p in candidates)
    for proposal in candidates:
        if proposal.reason=='OBSERVED_FOOTPRINT_PROPOSAL':
            assert check_observed_contact_footprint(domain,model,
                _contact_matrix(domain,proposal.center_uv_top_m,proposal.yaw_rad)).contained


def test_existing_headings_are_grouped_without_snapping_or_changing_admitted_targets():
    a=PlacementCandidate('deep-0',placed_pose(0.))
    b=PlacementCandidate('shallow',Transform((.5,-.8,.85),(0.,0.,0.,1.)))
    c=PlacementCandidate('deep-90',placed_pose(math.pi/2))
    near=PlacementCandidate('almost-deep',Transform((.8+1e-9,-.8,.85),(0.,0.,0.,1.)))
    original=(a,b,near,c)
    result=group_placement_positions(original)
    assert result==(a,c,b,near)
    assert all(any(item is candidate for item in original) for candidate in result)
    assert original==(a,b,near,c)
