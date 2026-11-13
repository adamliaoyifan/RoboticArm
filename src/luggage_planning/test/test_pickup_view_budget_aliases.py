"""New matched acquisitions inherit the original persisted recovery budget."""
from dataclasses import replace
import json

import pytest

from luggage_planning.joint_operation.view_budget import PickupViewBudgetLedger
from pickup_continuation_fixtures import case_archive, planar_case, planar_view


def opened(tmp_path, archive, *, limit=2, aliases=128):
    clock = [100.]
    ledger = PickupViewBudgetLedger(tmp_path/'budget.json', clock=lambda: clock[0],
                                    max_acquisitions_per_task=aliases)
    archive.ledger = (ledger.path, clock)
    case = planar_case(); proof = archive.associate(case)
    ledger.open('original-task', case[0].digest(), max_motions=limit, total_budget_s=10.)
    archive.save('ledger_before', json.loads(ledger.path.read_text()))
    return ledger, clock, case, proof


def save_after(archive, ledger):
    archive.save('ledger_after', json.loads(ledger.path.read_text()))


def test_alias_and_restart_preserve_task_dispatch_count_and_deadline(tmp_path, case_archive):
    owner, clock, case, proof = opened(tmp_path, case_archive)
    owner.dispatch('original-task'); clock[0] = 102.
    owner.adopt_acquisition('original-task', proof)
    before = owner.path.read_bytes()
    clock[0] = 103.; owner.adopt_acquisition('original-task', proof)
    assert owner.path.read_bytes() == before
    restored = PickupViewBudgetLedger(owner.path, clock=lambda: clock[0])
    row = restored.open('original-task', case[2].digest(), max_motions=2, total_budget_s=10.)
    save_after(case_archive, restored)
    assert row['dispatched'] == 1 and row['opened_wall_s'] == 100.
    assert restored.remaining('original-task') == 7.
    assert row['acquisitions'][case[2].digest()] == proof.digest()


@pytest.mark.parametrize('change', ['task', 'count', 'time', 'unknown-domain'])
def test_new_alias_does_not_allow_rename_or_limits_change(tmp_path, case_archive, change):
    owner, clock, case, proof = opened(tmp_path, case_archive)
    owner.adopt_acquisition('original-task', proof)
    before = owner.path.read_bytes()
    with pytest.raises(ValueError, match='RENAME|REBIND'):
        owner.open('new-task' if change == 'task' else 'original-task',
                   'f'*64 if change == 'unknown-domain' else case[2].digest(),
                   max_motions=3 if change == 'count' else 2,
                   total_budget_s=11. if change == 'time' else 10.)
    assert owner.path.read_bytes() == before
    save_after(case_archive, owner)


@pytest.mark.parametrize('damage', ['parent', 'rejected', 'cross-task', 'rebind', 'capacity'])
def test_alias_requires_original_parent_unique_task_and_proof(tmp_path, case_archive, damage):
    owner, clock, case, proof = opened(tmp_path, case_archive, aliases=1 if damage == 'capacity' else 128)
    if damage == 'parent': proof = replace(proof, before_domain_sha256='f'*64)
    elif damage == 'rejected': proof = replace(proof, accepted=False, reason='AMBIGUOUS_REPORTED_CANDIDATES')
    elif damage == 'cross-task':
        owner.open('other-task', case[2].digest(), max_motions=2, total_budget_s=10.)
    elif damage == 'rebind':
        owner.adopt_acquisition('original-task', proof)
        proof = replace(proof, settled_stamp_ns=proof.settled_stamp_ns+1)
    before = owner.path.read_bytes()
    case_archive.save('attempted_adoption_proof', proof)
    with pytest.raises(ValueError, match='PARENT|VERIFIED|RENAME|REBIND|CAPACITY'):
        owner.adopt_acquisition('original-task', proof)
    assert owner.path.read_bytes() == before
    save_after(case_archive, owner)


def test_last_view_can_adopt_measurements_but_cannot_dispatch_again(tmp_path, case_archive):
    owner, clock, case, proof = opened(tmp_path, case_archive, limit=1)
    owner.dispatch('original-task'); clock[0] = 102.
    owner.adopt_acquisition('original-task', proof)
    assert owner.rows['original-task']['acquisitions'][case[2].digest()] == proof.digest()
    with pytest.raises(ValueError, match='MOTION_LIMIT'): owner.dispatch('original-task')
    assert owner.status('original-task') == (1, 2.)
    save_after(case_archive, owner)


@pytest.mark.parametrize('now,reason', [(110., 'TIME_LIMIT'), (99., 'CLOCK_ROLLBACK')])
def test_expiry_and_clock_rollback_do_not_authorize_new_acquisition(tmp_path, case_archive, now, reason):
    owner, clock, case, proof = opened(tmp_path, case_archive)
    before = owner.path.read_bytes(); clock[0] = now
    with pytest.raises(ValueError, match=reason): owner.adopt_acquisition('original-task', proof)
    assert owner.path.read_bytes() == before
    save_after(case_archive, owner)


def test_old_persisted_row_migrates_without_renewing_budget(tmp_path, case_archive):
    owner, clock, case, proof = opened(tmp_path, case_archive)
    owner.dispatch('original-task')
    assert 'acquisitions' not in json.loads(owner.path.read_text())['original-task']
    clock[0] = 104.
    restored = PickupViewBudgetLedger(owner.path, clock=lambda: clock[0])
    restored.adopt_acquisition('original-task', proof)
    save_after(case_archive, restored)
    assert restored.status('original-task') == (1, 4.)
    assert restored.remaining('original-task') == 6.
    assert restored.rows['original-task']['opened_wall_s'] == 100.


def test_alias_chain_requires_each_measured_predecessor(tmp_path, case_archive):
    owner, clock, case, proof = opened(tmp_path, case_archive)
    owner.adopt_acquisition('original-task', proof)
    third_domain, third_view = planar_view(newer=True, camera_x=.075,
        domain_changes=dict(stamp_ns=5_000_000_000, generation=9, instance_id='third-id', box_id='third-id'))
    second_proof = case_archive.associate((*case[2:], third_domain, third_view), settled_stamp_ns=4_000_000_000)
    assert second_proof.accepted
    clock[0] = 105.; owner.adopt_acquisition('original-task', second_proof)
    save_after(case_archive, owner)
    assert len(owner.rows['original-task']['acquisitions']) == 3
    assert owner.remaining('original-task') == 5.
