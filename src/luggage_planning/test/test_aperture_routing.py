"""Geometric/assembly tests. Port doubles do not certify physical motion."""
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from luggage_planning.aperture_geometry import ApertureGeometry, box_vertices, transform_vertices
from luggage_planning.joint_operation_aperture import ApertureRouter
from luggage_planning.joint_operation_connection import GeometricConnection
from luggage_planning.joint_operation_ik import SnapshotIKResult
from luggage_planning.placement_prior_contracts import Transform


def aperture():
    return ApertureGeometry(((-.7,-1.,.5),(-.7,1.,.5),(-.7,1.,2.),(-.7,-1.,2.)),(1.,0.,0.))


def geometry(offset=(.1,.12,-.15), dimensions=(.5,.4,.3)):
    return SimpleNamespace(conservative=SimpleNamespace(dimensions_wdh=dimensions,
        t_tcp_box_translation=offset,t_tcp_box_quaternion_xyzw=(0.,0.,0.,1.)))


@pytest.mark.parametrize('heading',range(4))
def test_offset_box_region_and_exact_plane_for_four_headings(heading):
    opening = aperture()
    tool = Transform((0.,0.,1.),tuple(Rotation.from_euler('z',heading*np.pi/2).as_quat()))
    payload = geometry()
    region = opening.region(payload,tool)
    points = opening.proposals(region,(0.,.8,1.),6)
    assert 1 < len(points) <= 6
    for point in points:
        assert point[0] == pytest.approx(-.7)
        tcp = Transform(point,tool.quaternion_xyzw)
        box = Transform(payload.conservative.t_tcp_box_translation,(0.,0.,0.,1.))
        vertices = transform_vertices(tcp.compose(box),box_vertices(payload.conservative.dimensions_wdh))
        assert vertices[:,1].min() >= -1.-1e-9 and vertices[:,1].max() <= 1.+1e-9
        assert vertices[:,2].min() >= .5-1e-9 and vertices[:,2].max() <= 2.+1e-9


def test_upright_formula_and_tool_projection_are_grasp_bound():
    opening = aperture()
    tool = Transform((0.,0.,1.),(0.,0.,0.,1.))
    region = opening.region(geometry(),tool)
    # Contact-to-box lateral offset +0.12m shifts the legal contact left.
    assert region.lower_uv == pytest.approx((-.92,-.45))
    assert region.upper_uv == pytest.approx((.68,.75))
    head = opening.region(geometry(),tool,rigid_vertices=((0.,0.,.35),))
    assert head.upper_uv[1] == pytest.approx(.4)
    too_wide = opening.region(geometry(dimensions=(.5,3.,.3)),tool)
    assert too_wide.empty and opening.proposals(too_wide,(0.,0.,1.),6) == ()


def test_tilt_and_rotated_opening_frame_are_not_flattened():
    tool = Transform((0.,0.,1.),tuple(Rotation.from_euler('x',.1).as_quat()))
    assert aperture().region(geometry(),tool) != aperture().region(geometry(),Transform((0.,0.,1.),(0.,0.,0.,1.)))
    # A vertical opening facing +Y uses a lateral axis pointing -X.
    opening = ApertureGeometry(((1.,-.7,.5),(-1.,-.7,.5),(-1.,-.7,2.),(1.,-.7,2.)),(0.,1.,0.))
    point, = opening.proposals(opening.region(geometry(),tool),(0.,0.,1.),1)
    assert point[1] == pytest.approx(-.7)


@pytest.mark.parametrize('corners,normal',[
    (((0.,0.,0.),)*4,(1.,0.,0.)),
    (((-.7,-1.,.5),(-.7,1.,.5),(-.6,1.,2.),(-.7,-1.,2.)),(1.,0.,0.)),
    (((-.7,-1.,.5),(-.7,1.,.5),(-.7,1.,2.),(-.7,-1.,2.)),(0.,0.,1.)),
])
def test_invalid_aperture_never_generates_waypoints(corners,normal):
    with pytest.raises(ValueError):ApertureGeometry(corners,normal)


class Model:
    position_tolerance_m = .002
    path = ()
    def __init__(self): self.urdf = self
    def tcp(self,q,path):
        matrix = np.eye(4)
        matrix[:3,3] = q[:3]
        return matrix


class IK:
    def __init__(self): self.calls=[];self.reject=False
    def refine(self,scene,target,seed,**kwargs):
        self.calls.append((scene,target,seed,kwargs))
        if self.reject:return SnapshotIKResult(True,False,'COLLISION')
        return SnapshotIKResult(True,True,'double',tuple(target.translation)+tuple(seed[3:]))


class Connections:
    def __init__(self):self.calls=[];self.fail=0
    def connect(self,scene,start,goal,**kwargs):
        self.calls.append((scene,start,goal,kwargs))
        if self.fail:
            self.fail-=1
            return GeometricConnection(False,False,'SEARCH_INCOMPLETE')
        count=max(1,int(np.ceil(np.max(np.abs(np.asarray(start)-goal))/.009)))
        points=(tuple(start),)+tuple(tuple(np.asarray(start)+(np.asarray(goal)-start)*i/count)
                                    for i in range(1,count))+(tuple(goal),)
        return GeometricConnection(True,True,'double',points,len(points))


def fixture():
    scene=SimpleNamespace(world_from_container=Transform((0.,0.,0.),(0.,0.,0.,1.)))
    candidate=SimpleNamespace(scenes=SimpleNamespace(carry=scene),predicted_attachment=geometry(),
        world_from_release_tool=Transform((.2,0.,1.),(0.,0.,0.,1.)))
    ik,connections=IK(),Connections()
    router=ApertureRouter(aperture(),Model(),ik,connections,rigid_vertices=(),
        max_points=2,max_seeds=2)
    return candidate,ik,connections,router


def route(bundle,**kwargs):
    candidate,_,_,router=bundle
    trace=[]
    args=dict(request_id='gate',deadline=10.,cancelled=lambda:False,trace=trace)
    args.update(kwargs)
    router.clock=lambda:0.
    result=router.connect(candidate,(-1.5,0.,1.,0.,0.,6.4),(.2,0.,1.,0.,0.,6.5),**args)
    return result,trace


def test_ordered_plane_witness_joins_and_constraints_preserve_joint6_winding():
    bundle=fixture()
    result,trace=route(bundle)
    assert result.success and result.complete
    connections=bundle[2]
    assert len(connections.calls)==4
    assert [bool(c[3].get('path_constraints')) for c in connections.calls]==[False,True,True,False]
    assert connections.calls[1][2][0]==pytest.approx(-.7)
    assert all(a[2]==b[1] for a,b in zip(connections.calls,connections.calls[1:]))
    assert result.positions[0][-1]==6.4 and result.positions[-1][-1]==6.5
    assert any(row[0]=='aperture_selected' for row in trace)


def test_candidate_failure_expands_instead_of_direct_carry_fallback():
    bundle=fixture();bundle[2].fail=1
    result,trace=route(bundle)
    assert result.success and len(bundle[2].calls)>4
    assert any(row[0]=='aperture_edge' for row in trace)
    rejected=fixture();rejected[1].reject=True
    result,_=route(rejected)
    assert not result.success and result.reason=='APERTURE_SEARCH_INCOMPLETE'
    assert not rejected[2].calls


@pytest.mark.parametrize('kwargs',[dict(deadline=-1.),dict(cancelled=lambda:True)])
def test_deadline_and_cancel_do_not_start_native_work(kwargs):
    bundle=fixture();result,_=route(bundle,**kwargs)
    assert not result.success and not bundle[1].calls and not bundle[2].calls


def test_inside_start_does_not_force_exit_and_reentry():
    candidate,ik,connections,router=fixture();router.clock=lambda:0.
    result=router.connect(candidate,(0.,0.,1.,0.,0.,0.),(.2,0.,1.,0.,0.,0.),
        request_id='inside',deadline=1.,cancelled=lambda:False,trace=[])
    assert result.success and not ik.calls and len(connections.calls)==1


def test_normal_halfspace_above_roof_is_not_treated_as_inside():
    candidate,ik,connections,router=fixture();router.clock=lambda:0.
    trace=[]
    result=router.connect(candidate,(0.,0.,3.,0.,0.,0.),(.2,0.,1.,0.,0.,0.),
        request_id='above',deadline=1.,cancelled=lambda:False,trace=trace)
    assert result.success and ik.calls and len(connections.calls)==4
    assert not any(row[1]=='ALREADY_INSIDE_NO_FORCED_REENTRY' for row in trace if row[0]=='aperture')


def test_independent_plane_fk_witness_rejects_missing_or_wrong_region():
    candidate,_,_,router=fixture()
    def draft(rows):
        leg=SimpleNamespace(knots=tuple(SimpleNamespace(positions=q) for q in rows))
        return SimpleNamespace(legs=(None,None,None,leg))
    start=(-1.5,0.,1.,0.,0.,0.)
    end=(.2,0.,1.,0.,0.,0.)
    assert not router.has_plane_witness(draft((start,end)),candidate)
    assert not router.has_plane_witness(draft((start,(-.7,3.,1.,0.,0.,0.),end)),candidate)
    assert router.has_plane_witness(draft((start,(-.7,0.,1.,0.,0.,0.),end)),candidate)
