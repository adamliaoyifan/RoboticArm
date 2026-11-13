"""Recovery facts do not turn binary missing support into a view permission."""
from types import SimpleNamespace

from luggage_planning.joint_operation.search_evidence import PreparationSearchEvidence


def domain(support):
    return SimpleNamespace(support=bytes(support),digest=lambda:'a'*64)


def test_unknown_support_and_bounded_contact_search_remain_unresolved():
    evidence=PreparationSearchEvidence(domain([0,0,0,0]))
    evidence.proposal(SimpleNamespace(reason='CONTACT_CENTRE_SEARCH_INCOMPLETE',checked_centres=2))
    record=evidence.record('SEARCH_INCOMPLETE')
    assert record['domain_sha256']=='a'*64
    assert record['supported_cells']==0 and record['unproven_cells']==4
    assert record['proposals_checked']==1 and record['centres_checked']==2
    assert record['proposals_accepted']==record['quality_batches_complete']==0
    assert record['furthest_stage']=='CONTACT_PROPOSAL'
    assert record['observation_deficiency']=='NOT_ESTABLISHED'


def test_complete_quality_refusal_is_distinct_from_unavailable_or_invalid_receipt():
    evidence=PreparationSearchEvidence(domain([1,1,0,1]))
    evidence.quality(SimpleNamespace(complete=True,reason='CONTACT_QUALITY_BATCH_VALID',contacts=(),
        verdicts=(('q',False,'invalidfrac','b'*64),)))
    evidence.quality(SimpleNamespace(complete=False,reason='CONTACT_QUALITY_BACKEND_UNAVAILABLE',
        contacts=('untrusted',),verdicts=(('untrusted',True,'untrusted',''),)))
    record=evidence.record('SEARCH_INCOMPLETE')
    assert record['quality_batches_complete']==record['quality_batches_incomplete']==1
    assert record['quality_contacts_refused']==1 and record['quality_contacts_accepted']==0
    assert record['quality_contact_reasons']=={'invalidfrac':1}
    assert record['furthest_stage']=='DIRECTED_QUALITY'
    assert record['observation_deficiency']=='NOT_ESTABLISHED'


def test_later_proposals_do_not_hide_native_candidate_progress_or_authorize_views():
    evidence=PreparationSearchEvidence(domain([1,1]))
    evidence.candidate_yielded()
    evidence.proposal(SimpleNamespace(reason='CONTACT_CENTRE_SEARCH_INCOMPLETE',checked_centres=1))
    evidence.quality(SimpleNamespace(complete=True,reason='CONTACT_QUALITY_BATCH_VALID',
        contacts=(),verdicts=()))
    failed=evidence.record('SEARCH_INCOMPLETE')
    passed=evidence.record('PLAN_READY')
    assert failed['furthest_stage']==passed['furthest_stage']=='NATIVE_CANDIDATE'
    assert failed['native_candidates_yielded']==1
    assert failed['terminal_reason']=='SEARCH_INCOMPLETE'
    assert passed['terminal_reason']=='PLAN_READY'
    assert failed['observation_deficiency']==passed['observation_deficiency']=='NOT_ESTABLISHED'
