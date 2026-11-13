"""Durable explicit task budgets; deterministic clock, no production defaults."""
import json
import pytest
from luggage_planning.joint_operation.view_budget import PickupViewBudgetLedger


def ledger(tmp_path,clock,capacity=2):
    return PickupViewBudgetLedger(tmp_path/'budget.json',clock=lambda:clock[0],capacity=capacity)


def test_retry_and_restart_preserve_dispatch_count_and_original_deadline(tmp_path):
    clock=[100.];owner=ledger(tmp_path,clock)
    owner.open('box','a'*64,max_motions=2,total_budget_s=10.)
    clock[0]=102.;owner.dispatch('box')
    owner.open('box','a'*64,max_motions=2,total_budget_s=10.)
    assert owner.status('box')==(1,2.) and owner.remaining('box')==8.
    restored=ledger(tmp_path,clock)
    assert restored.status('box')==(1,2.)
    clock[0]=109.;restored.open('box','a'*64,max_motions=2,total_budget_s=10.)
    assert restored.remaining('box')==1.
    clock[0]=110.
    with pytest.raises(ValueError,match='TIME_LIMIT'):restored.remaining('box')
    assert json.loads((tmp_path/'budget.json').read_text())['box']['opened_wall_s']==100.


@pytest.mark.parametrize('limit',[0,1,2])
def test_exact_motion_bound_including_failed_dispatches(tmp_path,limit):
    owner=ledger(tmp_path,[100.])
    if limit==0:
        with pytest.raises(ValueError,match='MOTION_LIMIT'):
            owner.open('box','a'*64,max_motions=limit,total_budget_s=10.)
    else:
        owner.open('box','a'*64,max_motions=limit,total_budget_s=10.)
        for _ in range(limit):owner.dispatch('box')  # Failure/cancel never refunds.
        with pytest.raises(ValueError,match='MOTION_LIMIT'):owner.dispatch('box')
    assert owner.status('box')[0]==limit


@pytest.mark.parametrize('change',['task','domain','count','time'])
def test_retry_cannot_rename_acquisition_or_raise_limits(tmp_path,change):
    owner=ledger(tmp_path,[100.]);owner.open('box','a'*64,max_motions=2,total_budget_s=10.)
    with pytest.raises(ValueError,match='RENAME|REBIND'):
        owner.open('other' if change=='task' else 'box','b'*64 if change=='domain' else 'a'*64,
            max_motions=3 if change=='count' else 2,total_budget_s=11. if change=='time' else 10.)


def test_capacity_and_clock_rollback_never_forget_consumed_budget(tmp_path):
    clock=[100.];owner=ledger(tmp_path,clock,capacity=1)
    owner.open('box','a'*64,max_motions=2,total_budget_s=10.);owner.dispatch('box')
    with pytest.raises(ValueError,match='CAPACITY'):
        owner.open('other','b'*64,max_motions=2,total_budget_s=10.)
    clock[0]=99.
    with pytest.raises(ValueError,match='CLOCK_ROLLBACK'):owner.remaining('box')
    assert owner.rows['box']['dispatched']==1
