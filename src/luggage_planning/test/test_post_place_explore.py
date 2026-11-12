"""Post-view adoption requires new source acquisition, not predicted FOV gain."""
from types import SimpleNamespace as NS
import time
import numpy as np
import pytest
from luggage_planning.joint_operation.post_place_explore import PostPlaceExplorer
from luggage_planning.placement_prior.contracts import Transform
from luggage_planning.interior_view_scorer import CameraIntrinsics


@pytest.mark.parametrize('fresh',(True,False))
def test_new_view_must_integrate_after_settle_and_release(monkeypatch,fresh):
    import luggage_planning.joint_operation.post_place_explore as module
    initial=NS(source=NS(last_integrated_stamp_ns=5),accepted=False)
    stale=NS(source=NS(last_integrated_stamp_ns=9),accepted=True)
    latest=NS(source=NS(last_integrated_stamp_ns=11),accepted=True)
    captures=iter([initial,stale,latest if fresh else stale,stale])
    identity=Transform((0.,0.,0.),(0.,0.,0.,1.));records=[];motion=[]
    ports=NS(released=True,release_stamp_ns=1,deadline=time.monotonic()+10,cancelled=lambda:False,
        container=identity,revision='scene',snapshot=object(),
        joints=lambda:NS(positions=(0.,)*6,stamp_ns=1),capture=lambda:next(captures),
        model=NS(urdf=NS(tcp=lambda *a:np.eye(4)),path=()),
        plan=NS(identity=NS(request_id='operation'),legs=[NS(knots=[NS(positions=(0.,)*6)])]*4))
    def target(captured):
        ports.last_release_coverage=dict(accepted=captured.accepted,expected_columns=1,
            matched_columns=int(captured.accepted),missing_points_container=[] if captured.accepted else [(0.,0.,1.)])
        return captured.accepted,1
    ports.observed_target=target
    explorer=object.__new__(PostPlaceExplorer);explorer.ports=ports;explorer.config={}
    explorer.max_views=1;explorer.velocity_scale=.2;explorer.acceleration_scale=.1
    explorer.camera_geometry=lambda:(CameraIntrinsics(100,100,50,50),identity,identity)
    explorer.ik=NS(refine=lambda *a,**k:NS(success=True,joints=(.01,0.,0.,0.,0.,0.)))
    explorer.connection=NS(connect=lambda *a,**k:NS(success=True,positions=((0.,)*6,(.01,0.,0.,0.,0.,0.))))
    explorer.checkpoint=lambda stage:None;explorer.settled=lambda:10
    explorer.record=lambda name,data:records.append(name)
    explorer.move=lambda leg:motion.append(leg) or True
    monkeypatch.setattr(module,'generate_interior_downward_views',lambda config:[dict(
        name='one',valid_geometry=True,camera_xyz=(0.,0.,2.),orientation_quat=(1.,0.,0.,0.))])
    if fresh:assert explorer.observe(initial,2) is latest
    else:
        with pytest.raises(ValueError,match='BOUNDED_POST_PLACE_VIEWS_EXHAUSTED'):explorer.observe(initial,2)
    assert len(motion)==1 and motion[0].name=='post_place_explore'
    assert records.count('explore_view_observation.json')==(1 if fresh else 0)
