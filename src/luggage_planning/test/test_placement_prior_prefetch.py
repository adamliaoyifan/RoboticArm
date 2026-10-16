"""Template prefetch retains provenance and never replaces measured checking."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from luggage_planning.placement_prior import compute_prior
from luggage_planning.placement_prior_prefetch import PrefetchTemplate, reusable_placements
from luggage_planning.placement_prior_worker import PriorWorker
from test_placement_prior import request, SensorWall, await_result
from test_placement_prior_requests import confirmed_grid


def test_template_request_is_explicitly_hypothetical():
    template = PrefetchTemplate((.4,.3,.2), [0.,0.,.1,1.,0.,0.,0.])
    message = template.request(confirmed_grid())
    assert not message.measured
    assert list(message.template_box_wdh) == [.4,.3,.2]
    assert len(message.template_box_from_tcp) == 1
    assert message.candidate_limit == 32 and message.budget_s == 10.
    with pytest.raises(ValueError):
        PrefetchTemplate((.4,.3,0), [0.,0.,.1,1.,0.,0.,0.])
    with pytest.raises(ValueError):
        PrefetchTemplate((.4,.3,.2), [0.]*7)


def test_template_geometry_reused_but_measured_collision_is_rechecked():
    measured = request()
    hypothetical = replace(measured, measured=False)
    prior = compute_prior(hypothetical, SensorWall())
    slots = reusable_placements((hypothetical, prior), measured.snapshot, measured.box_wdh, 1)
    assert slots == measured.placements
    assert prior.adopt(measured) == ()
    collision = SensorWall()
    result = compute_prior(replace(measured, placements=slots), collision)
    assert len(collision.calls) == 2
    assert [c.grasp.candidate_id for c in result.adopt(measured)] == ['offset']
    assert reusable_placements((hypothetical,prior), measured.snapshot, (.5,.3,.2), 1) == ()
    assert reusable_placements((hypothetical,prior),
                              replace(measured.snapshot,map_revision=99), measured.box_wdh, 1) == ()


def test_old_timeout_cannot_cancel_new_foreground_job():
    worker = PriorWorker(SensorWall())
    try:
        old = worker.submit(request())
        new = worker.submit(replace(request(), request_id='foreground'))
        assert not worker.invalidate(generation=old)
        generation, result, error = await_result(worker)
        assert generation == new and error is None
        assert result.adopt(replace(request(), request_id='foreground'))
    finally:
        assert worker.close()


def test_timer_waits_for_confirmation_and_runs_once_per_snapshot():
    path = Path(__file__).parents[1]/'scripts/place_prior_node.py'
    spec = importlib.util.spec_from_file_location('prior_prefetch_node_test',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    current = [None]
    calls = []
    reader = SimpleNamespace(service_is_ready=lambda: True)
    store = SimpleNamespace(current=lambda: current[0], set_confirmation_required=lambda value: None)
    fake = SimpleNamespace(_template=PrefetchTemplate((.4,.3,.2),[0,0,.1,1,0,0,0]),
        _lock=threading.Lock(), _store=store, _foreground_active=False,
        _prefetch_attempt=None, _get_scene=reader, _get_map_parameters=reader,
        _collision=SimpleNamespace(ready=lambda:True), _invalidate_if_changed=lambda:None,
        _call=lambda *args:SimpleNamespace(values=[SimpleNamespace(type=1,bool_value=True)]),
        get_logger=lambda:SimpleNamespace(info=lambda text:None))
    def prepare(message,response,*,background):
        calls.append((message,background))
        response.reason='READY'
        return response
    fake._prepare=prepare
    tick=lambda:module.PlacePriorNode._prefetch_tick(fake)
    tick()
    assert calls == []
    current[0]=confirmed_grid()
    fake._foreground_active=True
    tick()
    assert calls == []
    fake._foreground_active=False
    tick();tick()
    assert len(calls)==1 and calls[0][1] and not calls[0][0].measured
    current[0]=confirmed_grid(floor=.3)
    tick()
    assert len(calls)==2
