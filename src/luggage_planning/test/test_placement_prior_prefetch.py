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
from luggage_planning.placement_prior_schedule import SnapshotQuietGate
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


@pytest.mark.parametrize('quiet_s', [0., 2.])
def test_timer_waits_for_confirmation_and_runs_once_per_snapshot(monkeypatch, quiet_s):
    path = Path(__file__).parents[1]/'scripts/place_prior_node.py'
    spec = importlib.util.spec_from_file_location('prior_prefetch_node_test',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    now=[0.]
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:now[0]))
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
    fake._quiet_gate=SnapshotQuietGate(quiet_s)
    fake._invalidate_if_changed=lambda:fake._quiet_gate.observe(
        current[0].identity if current[0] else None,now[0])
    tick=lambda:module.PlacePriorNode._prefetch_tick(fake)
    tick()
    assert calls == []
    current[0]=confirmed_grid()
    fake._foreground_active=True
    tick()
    assert calls == []
    fake._foreground_active=False
    if quiet_s:
        now[0]=quiet_s-.01
        tick()
        assert calls==[]
    now[0]=quiet_s
    tick();tick()
    assert len(calls)==1 and calls[0][1] and not calls[0][0].measured
    current[0]=confirmed_grid(floor=.3)
    tick()
    if quiet_s:
        assert len(calls)==1
        now[0]+=quiet_s
        tick()
    assert len(calls)==2


def test_changed_size_preserves_only_regenerated_geometry_and_rechecks_collision():
    from luggage_planning.placement_prior_requests import enumerate_placements
    from luggage_planning.placement_prior_prefetch import rank_regenerated_placements
    from luggage_planning.placement_prior import CollisionVerdict
    confirmed = confirmed_grid()
    original = request()
    # Representative measured dimensions from corrected_projection acquisition03.
    measured_size = (.5056767675344938, .3155214191690675, .21751788507479985)
    fresh = enumerate_placements(confirmed, measured_size, 8)
    template_slots = enumerate_placements(confirmed, (.53, .35, .22), 32)
    matching = next(slot for slot in reversed(fresh)
                    if slot.slot_id in {item.slot_id for item in template_slots})
    hypothetical = replace(original, measured=False, box_wdh=(.53,.35,.22),
                           placements=template_slots)
    preferred_template = next(slot for slot in template_slots if slot.slot_id==matching.slot_id)
    cached = SimpleNamespace(choices=(SimpleNamespace(placement=preferred_template),))
    assert preferred_template.container_from_box != matching.container_from_box
    assert reusable_placements((hypothetical,cached),original.snapshot,measured_size,8)==()
    ranked = rank_regenerated_placements((hypothetical,cached),original.snapshot,fresh)
    assert ranked[0] is matching
    assert {id(slot) for slot in ranked} == {id(slot) for slot in fresh}
    assert all(slot.container_from_box.translation[2] == pytest.approx(.25+measured_size[2]/2)
               for slot in ranked)

    class NewlyBlocked:
        def __init__(self): self.calls=[]
        def check(self,snapshot,tcp,box,size,**kwargs):
            self.calls.append((box,size))
            return CollisionVerdict(False,'MEASURED_COLLISION')
    port=NewlyBlocked()
    measured=replace(original,box_wdh=measured_size,placements=ranked,candidate_limit=32)
    result=compute_prior(measured,port)
    assert not result.choices
    assert len(port.calls)==len(ranked)*len(measured.grasps)
    assert all(size==measured_size for _,size in port.calls)


def test_regenerated_order_never_adds_missing_slots_or_crosses_snapshot_identity():
    from luggage_planning.placement_prior_prefetch import rank_regenerated_placements
    from luggage_planning.placement_prior_contracts import PlacementCandidate,Transform
    original=request();other=PlacementCandidate('other',Transform((.1,0,.1),(0,0,0,1)))
    current=(other,*original.placements)
    hypothetical=replace(original,measured=False)
    cached=SimpleNamespace(choices=(SimpleNamespace(placement=original.placements[0]),))
    assert rank_regenerated_placements((hypothetical,cached),original.snapshot,current)==tuple(reversed(current))
    assert rank_regenerated_placements((hypothetical,cached),original.snapshot,(other,))==(other,)
    for changed in (replace(original.snapshot,map_revision=99),
                    replace(original.snapshot,model_hash='new-model'),
                    replace(original.snapshot,scene_cdr=b'changed-world')):
        assert rank_regenerated_placements((hypothetical,cached),changed,current)==current
    assert rank_regenerated_placements((original,cached),original.snapshot,current)==current


def test_measured_decode_uses_fresh_size_before_prefetch_order(monkeypatch):
    import luggage_planning.placement_prior_requests as requests
    from luggage_planning.placement_prior_contracts import Snapshot
    from test_placement_prior_requests import query
    message=query();message.measured=True;message.slots=[];message.max_slots=8
    snapshot=Snapshot('cal',3,'hull','model',request().snapshot.world_from_container,b'occ',b'world')
    confirmed=confirmed_grid();size=(.5056767675344938,.3155214191690675,.21751788507479985)
    fresh=requests.enumerate_placements(confirmed,size,8)
    old=requests.enumerate_placements(confirmed,(.53,.35,.22),32)
    desired=next(slot for slot in reversed(fresh) if slot.slot_id in {s.slot_id for s in old})
    hypothetical=replace(request(),snapshot=snapshot,measured=False,box_wdh=(.53,.35,.22),placements=old)
    cached=SimpleNamespace(choices=(SimpleNamespace(placement=next(s for s in old if s.slot_id==desired.slot_id)),))
    # Detection admission has its own genuine-patch tests. Isolate the decoder's
    # ordering boundary while retaining the production geometry generator.
    monkeypatch.setattr(requests,'measured_detection_view',lambda _message:SimpleNamespace(box_xyz=(0,0,0),box_quat=(0,0,0,1),box_size=size))
    monkeypatch.setattr(requests,'candidates_for_prior',lambda *args,**kwargs:request().grasps)
    decoded=requests.decode_request(message,snapshot,confirmed,prefetched=(hypothetical,cached))
    assert decoded.measured and decoded.box_wdh==size
    assert decoded.placements[0]==desired
    assert set(decoded.placements)==set(fresh)
