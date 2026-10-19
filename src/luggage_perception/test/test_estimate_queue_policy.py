from dataclasses import replace
import pytest
from luggage_perception.estimate_queue_policy import EstimateEvidence, retain_measured_pending

MEASURE=EstimateEvidence(1_000_000_000,2,'box','measure',1)
HOLD=replace(MEASURE,stamp_ns=1_100_000_000,source='hold_track',cargo_count=0)


def keep(pending=MEASURE,incoming=HOLD,now=1_200_000_000):
    return retain_measured_pending(pending,incoming,now_ns=now,max_age_ns=1_000_000_000)


def test_same_instance_fresh_measurement_survives_continuity_only():
    assert keep()
    assert not keep(incoming=replace(HOLD,source='measure',cargo_count=1))


@pytest.mark.parametrize('incoming',[
    replace(HOLD,generation=3),replace(HOLD,instance_id='other'),
    replace(HOLD,cargo_count=2),replace(HOLD,source='reject_clutter'),
    replace(HOLD,source='empty'),replace(HOLD,source='source_unmatched'),
    replace(HOLD,stamp_ns=900_000_000),None])
def test_new_identity_or_uncertain_evidence_supersedes_pending(incoming):
    assert not keep(incoming=incoming)


@pytest.mark.parametrize('pending',[
    replace(MEASURE,generation=0),replace(MEASURE,instance_id=''),
    replace(MEASURE,cargo_count=2),replace(MEASURE,source='hold_track'),
    replace(MEASURE,stamp_ns=0),None])
def test_missing_or_nonmeasured_identity_is_not_retained(pending):
    assert not keep(pending=pending)


@pytest.mark.parametrize('now',[999_999_999,2_000_000_001])
def test_clock_rollback_or_age_expiry_restores_latest_policy(now):
    assert not keep(now=now)
