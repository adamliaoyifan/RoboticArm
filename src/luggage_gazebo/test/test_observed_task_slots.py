"""Explicit stack intent filters the existing full observed query before caps."""
import math
import pytest
from test_joint_operation_observer import configuration,snapshot
from luggage_gazebo.observed_task_slots import observed_task_slots
from luggage_planning.attachment.attachment_geometry_policy import load_attachment_geometry_policy


@pytest.mark.parametrize('mode',('floor','stack'))
def test_task_layer_uses_same_observed_surface_and_volume(tmp_path,mode):
    config,_,_=configuration(tmp_path,size=(.8,.8,1.),floor=.53,resolution=.02)
    nx,ny,nz=config.dimensions_xyz
    states=[1]*(nx*ny*nz)
    for z in range(10):
        for y in range(12,28):
            for x in range(12,28):states[x+nx*(y+ny*z)]=2
    captured=snapshot(config,states)
    dimensions=(.12,.10,.10)
    slots,report=observed_task_slots(captured,dimensions,mode,
        limits=load_attachment_geometry_policy(),margin=.02,vertical_margin=0.)
    assert report['retained']>0 and report['feasible_requested_layer']>0
    assert report['source']['epoch']==captured.source.epoch
    assert report['finite_capacity']==nx*ny*4
    expected=.53 if mode=='floor' else .73
    assert all(slot.translation[2]-.05==pytest.approx(expected) for slot in slots)
    assert len({round(2*math.atan2(s.quaternion_xyzw[2],s.quaternion_xyzw[3])/(math.pi/2))%4 for s in slots})==4


def test_unknown_volume_cannot_become_stack_or_floor_by_task_intent(tmp_path):
    config,_,_=configuration(tmp_path,size=(.4,.4,.6),floor=.53,resolution=.02)
    captured=snapshot(config,[0]*math.prod(config.dimensions_xyz))
    for mode in ('floor','stack'):
        slots,report=observed_task_slots(captured,(.12,.10,.10),mode,
            limits=load_attachment_geometry_policy(),margin=.02,vertical_margin=0.)
        assert not slots and report['feasible_requested_layer']==0
