from copy import deepcopy
import pytest
from luggage_gazebo.prior_functional_evidence import summarize_prior_cycle


def trial():
    return dict(fail_code='', place_state='HOME', segments_planned=2, segments_succeeded=2,
                vac_attach=True,lost_payload=False,inside_inner_box=True, extras=dict(
        placement_prior=dict(adopted=True,candidate_id='offset-a'),
        authorized_pick=dict(authorized=True),verify=dict(settled=True),
        selector=dict(winner_method='route-graph',probe=dict(mode='verify',verified=1,
            reason='OK',unavailable=0,certification_id='cert-a',admitted_map_revision=13,
            validated_map_revision=13)),submitted_legs=[dict(ok=True),dict(ok=True)],
        map_confirmation=dict(confirmed=True),integrate=dict(success=True,
            occupancy_height_ok=True,expected_map_revision=15,resulting_map_revision=16)))


def test_single_verified_cycle_is_separate_from_campaign_and_does_not_mutate():
    value=trial(); before=deepcopy(value)
    report=summarize_prior_cycle(value)
    assert report['functional_cycle_passed']
    assert report['f3_campaign']=='not_evaluated'
    assert report['performance']=='not_evaluated'
    assert value==before


@pytest.mark.parametrize('path,value', [
    (('fail_code',),'MAP_CONFIRMATION_REQUIRED'),
    (('extras','selector','probe','verified'),0),
    (('extras','selector','probe','certification_id'),''),
    (('extras','selector','probe','validated_map_revision'),14),
    (('extras','map_confirmation','confirmed'),False),
    (('extras','integrate','occupancy_height_ok'),False),
    (('extras','integrate','resulting_map_revision'),15),
    (('extras','submitted_legs'),[dict(ok=True)]),
    (('extras','submitted_legs'),[dict(ok=True),dict(ok=False)]),
    (('extras','placement_prior','adopted'),False),
    (('extras','authorized_pick','authorized'),False),
])
def test_missing_or_failed_boundary_refuses_case(path,value):
    case=trial(); container=case
    for key in path[:-1]: container=container[key]
    container[path[-1]]=value
    assert not summarize_prior_cycle(case)['functional_cycle_passed']


def test_empty_evidence_is_not_a_pass():
    assert not summarize_prior_cycle({})['functional_cycle_passed']
