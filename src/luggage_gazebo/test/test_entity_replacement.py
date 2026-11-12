from types import SimpleNamespace
import subprocess
import importlib.util
import json
from pathlib import Path

from luggage_gazebo.entity_replacement import probe_model, replace_entity


POSE = [1., 2., 3., 0., 0., 0.]


def run(states, *, delete_error=None, create_error=None):
    now = [0.]
    events = []
    sequence = iter(states)
    last = [states[-1]]

    def probe(budget):
        assert budget > 0
        last[0] = next(sequence, last[0])
        events.append(last[0])
        return dict(status=last[0], pose=POSE)

    def delete(budget):
        events.append('delete')
        return delete_error

    def create(pose, budget):
        assert pose == POSE
        events.append('create')
        return create_error

    result = replace_entity(probe=probe, delete=delete, create=create,
        pose_matches=lambda a, b: a == b, timeout=1., persistence=.2,
        clock=lambda: now[0], sleep=lambda seconds: now.__setitem__(0, now[0] + seconds))
    return result, events


def test_delete_ack_is_not_disappearance_and_create_waits():
    result, events = run(['present', 'present', 'present', 'absent', 'absent', 'present'])
    assert result['success']
    assert events[:6] == ['present', 'delete', 'present', 'present', 'absent', 'create']
    assert result['elapsed_s'] >= .4


def test_successful_create_ack_with_missing_entity_never_returns_success():
    result, events = run(['present', 'absent', 'absent'])
    assert result['reason'] == 'CREATE_UNCONFIRMED'
    assert events.count('create') == 1


def test_probe_failure_is_not_evidence_of_deletion():
    result, events = run(['present', 'unavailable'])
    assert result['reason'] == 'DELETE_UNCONFIRMED'
    assert 'create' not in events


def test_initial_absence_cannot_be_replaced_by_cached_pose():
    result, events = run(['absent'])
    assert result['reason'] == 'INITIAL_ENTITY_UNCONFIRMED'
    assert 'delete' not in events


def test_entity_disappearing_during_persistence_never_passes():
    result, _ = run(['present', 'absent', 'present', 'absent'])
    assert result['reason'] == 'CREATE_UNCONFIRMED'


def test_service_refusal_stops_without_retry():
    result, events = run(['present'], delete_error='refused')
    assert result['reason'] == 'DELETE_REFUSED'
    assert events.count('delete') == 1
    assert 'create' not in events


def test_fresh_query_distinguishes_missing_from_malformed(monkeypatch):
    monkeypatch.setattr(subprocess, 'run', lambda *a, **kw: SimpleNamespace(
        returncode=0, stdout='No model named <box> was found', stderr=''))
    assert probe_model('box', 1.)['status'] == 'absent'
    assert probe_model('other', 1.)['status'] == 'unavailable'


def test_fresh_query_does_not_accept_failed_command(monkeypatch):
    monkeypatch.setattr(subprocess, 'run', lambda *a, **kw: SimpleNamespace(
        returncode=1, stdout='[1 2 3] [0 0 0]', stderr='failure'))
    assert probe_model('box', 1.)['status'] == 'unavailable'


def call_filter(monkeypatch, tmp_path, states):
    path = Path(__file__).resolve().parents[1] / 'scripts/pickup_box_spawner_node.py'
    spec = importlib.util.spec_from_file_location('carry_ready_spawner', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    from luggage_gazebo import entity_replacement
    sequence = iter(states)
    monkeypatch.setattr(entity_replacement, 'probe_model',
        lambda model, timeout: dict(status=next(sequence), pose=POSE))
    trace = tmp_path / 'trace.jsonl'
    monkeypatch.setenv('ELFIN_CARRY_REPLACEMENT_TRACE', str(trace))
    operations = []
    node = SimpleNamespace(_current_model='box', _spawn_size=[.3, .4, .2], _rigid_attachment=None,
        _spawn_mass=5., _spawn_visual='mesh', _carry_bitmask=-1,
        _current_box=SimpleNamespace(pose='stale cached pose'), _place_persist_sec=0.,
        _box_sdf=lambda *a, **kw: 'exact body sdf',
        _pose_mismatch=lambda actual, expected: None,
        _delete_model=lambda model, **kw: operations.append(('delete', model)),
        _spawn_model=lambda model, pose, **kw: operations.append(('create', pose)))
    response = module.PickupBoxSpawner.handle_set_p04_carry_filter(
        node, SimpleNamespace(data=True), SimpleNamespace(success=False, message=''))
    return response, node, operations, json.loads(trace.read_text())


def test_real_callback_refuses_missing_entity_without_mutation(monkeypatch, tmp_path):
    response, node, operations, trace = call_filter(monkeypatch, tmp_path, ['absent'])
    assert not response.success
    assert node._carry_bitmask == -1
    assert operations == []
    assert trace['reason'] == 'INITIAL_ENTITY_UNCONFIRMED'


def test_real_callback_spawns_at_fresh_pose_and_records_observed_transition(monkeypatch, tmp_path):
    response, node, operations, trace = call_filter(
        monkeypatch, tmp_path, ['present', 'absent', 'present'])
    assert response.success
    assert node._carry_bitmask != -1
    assert operations[1][1].position.x == POSE[0]
    assert operations[1][1].position.y == POSE[1]
    assert trace['reason'] == 'READY'
    assert [r['stage'] for r in trace['trace']] == [
        'before_delete', 'delete_ack', 'after_delete', 'create_ack', 'after_create']
