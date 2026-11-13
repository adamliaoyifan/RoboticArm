"""Offline ownership/journal and typed map transition gates; not execution proof."""
from dataclasses import replace
from types import SimpleNamespace
import json

import pytest

from luggage_planning.joint_operation.runtime import OperationJournal, JointOperationRuntime
from luggage_planning.joint_operation.map_source import OperationMapSource, OperationMapRefresh
from luggage_planning.planning_contract.attachment import ATTACHMENT_MAX_AGE_NS


def test_runtime_archive_budget_counts_lossless_stored_bytes_and_preserves_raw_hash(tmp_path):
    import gzip,hashlib,threading
    runtime=JointOperationRuntime.__new__(JointOperationRuntime)
    runtime._record_lock=threading.RLock();runtime.directory=tmp_path
    runtime.records=[];runtime.bytes=0;runtime.evidence_limit=1000
    runtime.machine=SimpleNamespace(state='fixture',may_have_payload=False)
    runtime.owner=SimpleNamespace(motion_lock=threading.Lock())
    raw=b'repeated exact frozen scene'*1000
    runtime.record('native_request',raw)
    row=runtime.records[0];stored=(tmp_path/row['file']).read_bytes()
    assert gzip.decompress(stored)==raw
    assert row['sha256']==hashlib.sha256(raw).hexdigest()
    assert row['stored_sha256']==hashlib.sha256(stored).hexdigest()
    assert runtime.bytes==row['stored_bytes']==len(stored)<runtime.evidence_limit<len(raw)
    runtime.evidence_limit=runtime.bytes
    with pytest.raises(ValueError,match='EVIDENCE_CAP'):runtime.record('next',b'next')
    assert len(runtime.records)==1


def test_operation_journal_retains_unresolved_across_restart_and_completed_replay(tmp_path):
    path=tmp_path/'owner.json'
    owner=OperationJournal(str(path),capacity=2)
    owner.write(dict(stage='VACUUM_EFFECT_PENDING',may_have_payload=True))
    restored=OperationJournal(str(path),capacity=2)
    assert restored.value['active']==owner.value['active']
    plan=SimpleNamespace(plan_digest='a'*64,identity=SimpleNamespace(observation_stamp_ns=1_000_000_000))
    owner.finish(plan,2_000_000_000)
    restored=OperationJournal(str(path),capacity=2)
    assert restored.value['active'] is None and restored.used(plan.plan_digest)
    owner.finish(plan,3_000_000_000)
    assert len(owner.value['used'])==1
    assert not path.with_name(path.name+'.pending').exists()


def test_operation_journal_capacity_refuses_instead_of_forgetting_live_replay(tmp_path):
    journal=OperationJournal(str(tmp_path/'owner.json'),capacity=1)
    a=SimpleNamespace(plan_digest='a'*64,identity=SimpleNamespace(observation_stamp_ns=1))
    b=SimpleNamespace(plan_digest='b'*64,identity=SimpleNamespace(observation_stamp_ns=2))
    journal.finish(a,3)
    journal.write(dict(stage='active second'))
    with pytest.raises(ValueError,match='CAPACITY'):journal.finish(b,3)
    assert journal.used(a.plan_digest) and journal.value['active']
    journal.finish(b,ATTACHMENT_MAX_AGE_NS+2)
    assert journal.used(b.plan_digest) and not journal.used(a.plan_digest)


def map_inputs():
    original=OperationMapSource('scene','hull',2,7,8,10_000_000_000,'a'*64,'ros',3,'b'*64)
    current=replace(original,map_revision=8,observed_revision=9,last_integrated_stamp_ns=11_000_000_000,content_sha256='c'*64)
    identity=SimpleNamespace(scene_revision='scene',geometry_hash='hull',map_revision=7)
    snapshot=SimpleNamespace(scene_revision='scene',geometry_hash='hull',map_revision=8,surface_content_hash='d'*64)
    return original,current,identity,snapshot


def test_ordinary_current_map_revision_only_authorizes_new_native_check():
    original,current,identity,snapshot=map_inputs()
    proof=OperationMapRefresh(original,current,'d'*64)
    assert proof.admits(identity,snapshot)
    assert not proof.admits(identity,replace_snapshot(snapshot,surface_content_hash='e'*64))


def replace_snapshot(snapshot,**changes):
    return SimpleNamespace(**dict(vars(snapshot),**changes))


@pytest.mark.parametrize('field,value',[('epoch',3),('config_digest','e'*64),('geometry_hash','other'),
    ('scene_revision','other'),('validity_generation',4),('map_revision',6),
    ('observed_revision',7),('last_integrated_stamp_ns',9_000_000_000)])
def test_map_reset_rebound_or_time_reversal_does_not_authorize_native_refresh(field,value):
    original,current,identity,snapshot=map_inputs()
    current=replace(current,**{field:value})
    proof=OperationMapRefresh(original,current,'d'*64)
    assert not proof.admits(identity,replace_snapshot(snapshot,map_revision=current.map_revision))


def runtime_boundary():
    """No ROS effects: exercise actual runtime ownership/finalization methods."""
    import threading
    runtime=object.__new__(JointOperationRuntime)
    runtime._active=threading.Lock()
    runtime._uncertain=True
    runtime._restored_unresolved=False
    runtime.session=object()
    runtime.machine=SimpleNamespace(may_have_payload=False,plan=None)
    runtime.pending_motion_command=None
    runtime.directory=None
    runtime.cleanup_budget=1.
    runtime.owner=SimpleNamespace(motion_lock=threading.Lock(),finish=lambda session:True)
    runtime.ports=SimpleNamespace(rpc=SimpleNamespace(unresolved=lambda:False),
        finalize_world=lambda:None)
    runtime.journal=SimpleNamespace(write=lambda active:None)
    runtime.record=lambda *args:None
    return runtime


def test_late_terminal_poll_is_serialized_before_consuming_controller_result():
    import threading
    runtime=runtime_boundary()
    entered=threading.Event()
    proceed=threading.Event()
    consumed=[]
    runtime.machine.may_have_payload=True
    def poll():
        consumed.append('poll')
        entered.set()
        assert proceed.wait(timeout=2.)
        return None
    runtime.motion=SimpleNamespace(poll_terminal=poll)
    worker=threading.Thread(target=runtime.poll_terminal)
    worker.start()
    try:
        assert entered.wait(timeout=2.)
        runtime.poll_terminal()
        assert consumed==['poll']
    finally:
        proceed.set()
        worker.join(timeout=2.)
    assert not worker.is_alive() and not runtime._active.locked()
    assert runtime.session is not None


def test_failed_durable_carry_checkpoint_aborts_callback_and_keeps_motion_owner():
    runtime=runtime_boundary()
    runtime._active.acquire()
    runtime.machine.may_have_payload=True
    runtime.handle=SimpleNamespace(request=SimpleNamespace(plan=SimpleNamespace(plan_digest='a'*64)))
    def disk_full(active):raise OSError('disk full')
    runtime.journal.write=disk_full
    outcomes=[]
    handle=SimpleNamespace(request=runtime.handle.request,is_cancel_requested=False,
                           abort=lambda:outcomes.append('abort'))
    result=SimpleNamespace(success=False,reason='attachment refused',outcome='CARRY_FAULT',
                           motion_terminal=True)
    session=runtime.session
    runtime.finish_action(handle,result,None)
    assert outcomes==['abort'] and not runtime._active.locked()
    assert runtime.session is session and runtime._uncertain
    assert result.may_have_payload and 'FINALIZATION_UNRESOLVED:disk full' in result.reason


def test_callback_terminal_transport_failure_still_releases_callback_mutex():
    runtime=runtime_boundary()
    runtime._active.acquire()
    runtime.machine.may_have_payload=True
    runtime.handle=SimpleNamespace(request=SimpleNamespace(plan=SimpleNamespace(plan_digest='b'*64)))
    def broken_transport():raise RuntimeError('terminal response lost')
    handle=SimpleNamespace(request=runtime.handle.request,is_cancel_requested=False,abort=broken_transport)
    result=SimpleNamespace(success=False,reason='holding',outcome='CARRY_FAULT',motion_terminal=True)
    with pytest.raises(RuntimeError,match='terminal response lost'):
        runtime.finish_action(handle,result,None)
    assert not runtime._active.locked() and runtime.session is not None


def test_preparation_window_and_replay_retention_share_original_acquisition_bound(tmp_path):
    from luggage_planning.joint_operation.preparation_profile import execution_observation_age_limit_ns
    from luggage_planning.attachment.attachment_provider import MeasuredAttachmentProvider
    bound = execution_observation_age_limit_ns(300.)
    assert bound == ATTACHMENT_MAX_AGE_NS + 300_000_000_000
    provider = MeasuredAttachmentProvider('scene','model',max_age_ns=bound)
    assert provider.max_age_ns == bound
    journal = OperationJournal(str(tmp_path/'window.json'),capacity=1,max_acquisition_age_ns=bound)
    first = SimpleNamespace(plan_digest='first',identity=SimpleNamespace(observation_stamp_ns=1))
    second = SimpleNamespace(plan_digest='second',identity=SimpleNamespace(observation_stamp_ns=2))
    journal.finish(first,195_000_000_000)
    with pytest.raises(ValueError,match='CAPACITY'):
        journal.finish(second,bound+1)  # first still at its exact admission boundary
    assert journal.used('first')
    journal.finish(second,bound+2)
    assert not journal.used('first') and journal.used('second')

@pytest.mark.parametrize('case',('converges','collision','exhausted','cancelled','deadline'))
def test_barrier_rechecks_only_valid_source_changes_with_bounded_attempts(case):
    import time
    runtime=object.__new__(JointOperationRuntime)
    runtime.refresh_attempts=3
    calls=[];records=[]
    def validate(index):
        calls.append(index)
        accepted=case=='converges' and len(calls)==2
        runtime.ports.retryable_change=case!='collision' and not accepted
        return SimpleNamespace(receipt=SimpleNamespace(accepted=accepted,
            reason='NATIVE_SUFFIX_CHECKED' if accepted else 'refused'))
    runtime.ports=SimpleNamespace(validate=validate,retryable_change=False)
    runtime.record=lambda name,data:records.append(json.loads(data))
    result=runtime.validate_with_refresh(5,deadline=time.monotonic()+(-1 if case=='deadline' else 10),
        cancelled=lambda:case=='cancelled')
    assert len(calls)==({'converges':2,'exhausted':3}.get(case,1))
    assert result.receipt.accepted==(case=='converges')
    assert len(records)==len(calls) and all(row['index']==5 for row in records)
