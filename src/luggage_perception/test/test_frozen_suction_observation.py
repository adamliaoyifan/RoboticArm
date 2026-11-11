"""Synthetic measured-plane cases; no inferred support or physical-seal claim."""
import copy
from dataclasses import asdict, fields, replace
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.frozen_suction_observation import FrozenSuctionObservations
from test_directed_suction_quality import context_inputs


def contact(uv=(0.,0.),tilt=0.):
    result=np.eye(4)
    result[:3,:3]=(Rotation.from_euler('x',np.pi)*Rotation.from_euler('y',tilt)).as_matrix()
    result[:3,3]=(*uv,1.)
    return result


def relative(domain,pose):
    box=np.eye(4);box[:3,:3]=Rotation.from_quat(domain.box_quaternion_xyzw).as_matrix()
    box[:3,3]=domain.box_xyz_world
    return np.linalg.inv(box)@pose


@pytest.fixture
def observed_case(request):
    domain,hmap,model=context_inputs()
    clock=SimpleNamespace(now=0.)
    store=FrozenSuctionObservations(max_records=3,max_bytes=8*1024**2,lifetime_s=10.,clock=lambda:clock.now)
    counter=0
    if os.environ.get('FROZEN_SUCTION_EVIDENCE_DIR'):
        initial=Path(os.environ['FROZEN_SUCTION_EVIDENCE_DIR'])/request.node.name
        initial.mkdir(parents=True,exist_ok=False)
        metadata=asdict(domain);metadata.pop('support')
        (initial/'acquisition.json').write_text(json.dumps({'domain':metadata,'model':model.as_dict()},indent=2))
        np.savez_compressed(initial/'acquisition_samples.npz',support=domain.cells(),**{
            f.name:getattr(hmap,f.name) for f in fields(hmap)
            if isinstance(getattr(hmap,f.name),np.ndarray)})
    def retain(request_id='operation',observation_id='view',**changes):
        nonlocal domain,hmap,model
        domain=changes.get('domain',domain);hmap=changes.get('hmap',hmap);model=changes.get('model',model)
        return store.retain(request_id,observation_id,DirectedSuctionContext(domain,hmap,model))
    def check(binding,pose=None,**changes):
        nonlocal counter
        pose=contact() if pose is None else pose
        options=dict(attachment_geometry_sha256='a'*64,measured_box_wdh=domain.box_wdh,
            box_from_contact_frame=tuple(relative(domain,pose).ravel()),current_model=model,deadline=100.)
        options.update(changes)
        output=None
        if os.environ.get('FROZEN_SUCTION_EVIDENCE_DIR'):
            output=Path(os.environ['FROZEN_SUCTION_EVIDENCE_DIR'])/request.node.name/str(counter)
            output.mkdir(parents=True,exist_ok=False)
            owned=store.lookup(binding)
            actual_domain=owned.domain if owned else domain
            actual_hmap=owned._height_map if owned else hmap
            metadata=asdict(actual_domain);metadata.pop('support')
            (output/'input.json').write_text(json.dumps({'binding':asdict(binding),'domain':metadata,
                'current_model':options['current_model'].as_dict(),'clock':clock.now,
                'retained_context_available':owned is not None,
                'retained_model':owned._model.as_dict() if owned else None,
                'options':{k:v for k,v in options.items() if k!='current_model'},
                'fixture':'synthetic measured same-acquisition top'},indent=2))
            np.savez_compressed(output/'samples.npz',support=actual_domain.cells(),**{
                f.name:getattr(actual_hmap,f.name) for f in fields(actual_hmap)
                if isinstance(getattr(actual_hmap,f.name),np.ndarray)})
        counter+=1
        try:
            result=store.check(binding,**options)
            if output:(output/'result.json').write_text(json.dumps(asdict(result),indent=2))
            return result
        except Exception as exc:
            if output:(output/'exception.json').write_text(json.dumps({'type':type(exc).__name__,'reason':str(exc)}))
            raise
    return SimpleNamespace(domain=domain,hmap=hmap,model=model,clock=clock,store=store,retain=retain,check=check)


@pytest.mark.parametrize('tilt',[0.,.025,-.025])
def test_actual_pose_preserved_with_original_observed_frame(observed_case,tilt):
    f=observed_case;binding=f.retain();actual=contact((.031,-.027),tilt)
    result=f.check(binding,actual)
    assert result.accepted,result.reason
    assert np.array(result.world_from_contact_frame_at_observation).reshape(4,4)==pytest.approx(actual,abs=1e-15)
    assert dict(result.metrics)['valid_coverage']==1.
    assert result.normal_difference_rad==pytest.approx(abs(tilt),abs=1e-12)


def test_acquisition_box_rotation_is_used_instead_of_live_world_pose(observed_case):
    f=observed_case
    domain=replace(f.domain,box_xyz_world=(.02,-.01,.85),
        box_quaternion_xyzw=tuple(Rotation.from_euler('z',.71).as_quat()))
    binding=f.retain(domain=domain)
    actual=contact((.02,-.01))
    result=f.check(binding,actual)
    assert result.accepted,result.reason
    assert np.array(result.world_from_contact_frame_at_observation).reshape(4,4)==pytest.approx(actual,abs=1e-15)


def test_actual_offset_rechecks_surface_instead_of_old_grasp_score(observed_case):
    f=observed_case
    u=f.hmap.origin_u+(np.arange(f.hmap.nu)+.5)*f.hmap.cell_size_m
    f.hmap.smooth_h[u>.08,:]=.012
    binding=f.retain()
    old=f.check(binding,contact((-.12,0.)))
    actual=f.check(binding,contact((.12,0.)))
    assert old.accepted,old.reason
    assert not actual.accepted
    assert actual.reason != old.reason


@pytest.mark.parametrize('kind',['interior_hole','edge_notch','unknown_quality','edge_overhang'])
def test_whole_footprint_and_quality_each_remain_required(observed_case,kind):
    f=observed_case;pose=contact()
    if kind in ('interior_hole','edge_notch'):
        support=f.domain.cells().copy();support[70 if kind=='interior_hole' else 45,45]=False
        domain=replace(f.domain,support=support.astype(np.uint8).tobytes())
        binding=f.retain(domain=domain)
    else:
        if kind=='unknown_quality':f.hmap.valid_count[:]=0
        else:pose=contact((.3,0.))
        binding=f.retain()
    result=f.check(binding,pose)
    assert not result.accepted,result.reason


def test_frozen_samples_and_model_do_not_alias_retained_input(observed_case):
    f=observed_case;context=DirectedSuctionContext(f.domain,f.hmap,f.model)
    binding=f.store.retain('operation','view',context)
    original=copy.deepcopy(f.model)
    f.hmap.smooth_h[:]=.5
    context._height_map.smooth_h=np.ones(f.hmap.shape)*.5
    context._model.boundary_margin_m+=.1
    result=f.check(binding,current_model=original)
    assert result.accepted,result.reason


@pytest.mark.parametrize('kind',['box_dimensions','geometry_digest','policy','binding','pose_nan','pose_scale','pose_last_row','wrong_face'])
def test_invalid_or_changed_actual_input_refuses(observed_case,kind):
    f=observed_case;binding=f.retain();pose=contact();changes={}
    if kind=='box_dimensions':changes['measured_box_wdh']=(.71,.45,.3)
    elif kind=='geometry_digest':changes['attachment_geometry_sha256']='arbitrary'
    elif kind=='policy':
        changed=copy.deepcopy(f.model);changed.boundary_margin_m+=.005;changes['current_model']=changed
    elif kind=='binding':binding=replace(binding,observation_id='other')
    elif kind=='pose_nan':pose[0,0]=np.nan
    elif kind=='pose_scale':pose[0,0]=2.
    elif kind=='pose_last_row':pose[3,0]=1.
    else:pose[:3,:3]=np.eye(3)
    if kind=='wrong_face':
        result=f.check(binding,pose,**changes);assert not result.accepted
    else:
        with pytest.raises(ValueError):f.check(binding,pose,**changes)


def test_lifetime_read_and_duplicate_retention_do_not_renew(observed_case):
    f=observed_case;binding=f.retain()
    f.clock.now=9.
    assert f.store.lookup(binding) is not None
    assert f.retain()==binding
    f.clock.now=10.
    assert f.store.lookup(binding) is None
    with pytest.raises(ValueError):f.check(binding)
    with pytest.raises(ValueError):f.retain()


def test_same_request_observation_cannot_be_rebound(observed_case):
    f=observed_case;binding=f.retain()
    with pytest.raises(ValueError):f.retain(domain=replace(f.domain,stamp_ns=f.domain.stamp_ns+1))
    assert f.store.lookup(binding) is not None


def test_explicit_revocation_and_resource_eviction_refuse_old_binding(observed_case):
    f=observed_case;binding=f.retain();f.store.revoke('operation','view')
    assert f.store.lookup(binding) is None
    renewed=f.retain();assert renewed.retention_id!=binding.retention_id
    assert f.store.lookup(binding) is None
    f.retain(observation_id='second');f.retain(observation_id='third');f.retain(observation_id='fourth')
    assert f.store.lookup(renewed) is None


@pytest.mark.parametrize('kind',['revoke','expire','deadline'])
def test_source_invalidated_during_quality_cannot_grant_positive(observed_case,kind,monkeypatch):
    from luggage_perception.suction_patch_evaluator import SuctionPatchEvaluator
    f=observed_case;binding=f.retain();original=SuctionPatchEvaluator._evaluate_batch
    def delayed(*args,**kw):
        value=original(*args,**kw)
        if kind=='revoke':f.store.revoke('operation','view')
        else:f.clock.now=11. if kind=='expire' else 2.
        return value
    monkeypatch.setattr(SuctionPatchEvaluator,'_evaluate_batch',delayed)
    result=f.check(binding,deadline=1. if kind=='deadline' else 100.)
    assert not result.accepted


def test_record_larger_than_resource_budget_is_never_retained(observed_case):
    f=observed_case;store=FrozenSuctionObservations(max_records=1,max_bytes=1,lifetime_s=10.)
    with pytest.raises(ValueError):store.retain('operation','view',DirectedSuctionContext(f.domain,f.hmap,f.model))
