"""Exercise the hardware driver boundary after a completed pick session.

Motion/authorization/session outcomes are injected; this is no hardware or
vacuum execution test. A changed selected candidate must never reach place.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


def load_driver():
    path = Path(__file__).parents[1] / 'scripts/hardware_pick_driver.py'
    spec = importlib.util.spec_from_file_location('hardware_prior_retry_driver', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_boundary(module, monkeypatch, *, bound_id, picked_id, attachment_present=True):
    calls, logs = [], []
    logger = NS(**{name: lambda text: logs.append(text) for name in ('info','warn','warning','error')})
    attachment = NS(box_id='measured-box', generation=2) if attachment_present else None
    choice = NS(candidate_id=bound_id, slot_id='bound-slot') if bound_id else None
    prior = NS(success=True, measured=True)
    detection = NS(stamp_ns=100, candidates=())
    fake = NS(_args=NS(skip_observe=True, observe_pose='unused', settle_sec=0.,
                      detect_only=False, dry_run=False, plan_only=False, trace_out='',
                      use_vacuum=True, release=True, frame_scene_revision='scene'),
        get_logger=lambda: logger, wait_graph=lambda: (True,'injected ready'),
        detect_until_authorized=lambda: NS(authorized=True, box=object(),
            decision=NS(action='authorize', reason='injected', attempts=1)),
        _detection_view=lambda box: detection, _log_detection=lambda *args: None,
        _load_contact_model=lambda: object(), _session_config=lambda model: object(),
        _typed=NS(set_placement_prior=lambda result: None), _frame_scene_revision='scene',
        _planning_recorder=None, _placement_prior_result=None, _placement_prior_choice=None,
        _placement_prior_client=None, _measured_attachment=attachment,
        _frame_box_identity=lambda: ('measured-box',2), _frame_request_id=lambda kind: 'place',
        _place_planner=NS(plan=lambda *args,**kw: (calls.append(('plan',kw)) or
                                                NS(success=True,selection_id='route'))),
        vacuum=lambda enable: (calls.append(('vacuum',enable)) or (True,'injected')),
        remove_scene_box=lambda: calls.append(('remove',None)))
    if choice:
        fake._placement_prior_client=NS(prepare=lambda *args: prior,
            bind=lambda *args: (detection,choice))
    monkeypatch.setattr(module, 'RosPickSessionPorts', lambda *args,**kw:
                        NS(close=lambda: calls.append(('close',None))))
    monkeypatch.setattr(module, 'PickSession', lambda *args,**kw:
        NS(run_request=lambda view: NS(completed=True, exit_code=0,
           reason_code='OK', detail='', selected_candidate_id=picked_id)))
    result = module.HardwarePickDriver.run(fake)
    return result,calls,logs


def test_changed_candidate_stops_before_place_and_preserves_vacuum(monkeypatch):
    code,calls,logs=run_boundary(load_driver(),monkeypatch,bound_id='A',picked_id='B')
    assert code == 10
    assert calls == [('close',None)]
    assert any('pickup retry changed the prior binding' in text for text in logs)


@pytest.mark.parametrize('bound_id', ['B', None])
def test_matching_rebound_candidate_or_ordinary_path_can_reach_place(monkeypatch,bound_id):
    code,calls,_=run_boundary(load_driver(),monkeypatch,bound_id=bound_id,picked_id='B')
    assert code == 0
    assert [kind for kind,_ in calls] == ['plan','vacuum','remove','close']
    selected=calls[0][1]['selected_choice']
    assert (selected.candidate_id if selected else None) == bound_id


def test_missing_measured_attachment_never_reaches_place_or_release(monkeypatch):
    code,calls,_=run_boundary(load_driver(),monkeypatch,bound_id='A',picked_id='A',attachment_present=False)
    assert code == 10 and calls == [('close',None)]
