"""Validated request boundary; independent of ROS and simulator truth.

Poses are row-major container_link absolute transforms of geometric box centre.
Support IDs refer to this immutable OCC/surface snapshot, never commanded points.
A confirmed attachment is immutable for the lifetime of an attached request.
"""
from dataclasses import dataclass, asdict, replace
from collections import Counter
import time
import numpy as np
from scipy.spatial.transform import Rotation
from shapely.geometry import box as polygon_box
from shapely.ops import unary_union
from candidates import matrix, samples, interior_samples, round_robin, _action_stream
from geometry import pose
from planning_types import digest
from support_geometry import measure_support


def rigid_pose(value, name):
    a=np.asarray(value,dtype=float)
    if a.size!=16 or not np.isfinite(a).all():raise ValueError(name+':INVALID_POSE')
    a=a.reshape(4,4)
    if (not np.allclose(a[3],[0,0,0,1],rtol=0,atol=1e-9) or
        not np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3),rtol=0,atol=1e-7) or
        abs(np.linalg.det(a[:3,:3])-1)>1e-7):raise ValueError(name+':NON_RIGID_POSE')
    return a


@dataclass(frozen=True)
class PlacementRequest:
    request_id: str
    box: object  # immutable BoxState
    snapshot: object  # immutable SceneSnapshot; same OCC contract as perception
    start_q: tuple
    final_box_poses: tuple
    support_source_ids: tuple
    start_attached: bool = False
    confirmed_box_T_tool: tuple | None = None
    frame_id: str = 'container_link'

    def __post_init__(self):
        if self.frame_id!='container_link' or self.snapshot.frame_id!=self.frame_id:
            raise ValueError('REQUEST_FRAME_MISMATCH')
        if not self.request_id:raise ValueError('REQUEST_ID_REQUIRED')
        if len(self.start_q)!=6 or not np.isfinite(self.start_q).all():raise ValueError('INVALID_START_JOINTS')
        if len(self.box.dimensions)!=3 or not np.isfinite(self.box.dimensions).all() or min(self.box.dimensions)<=0:
            raise ValueError('INVALID_BOX_DIMENSIONS')
        if len(self.box.com_local)!=3 or not np.isfinite(self.box.com_local).all():raise ValueError('INVALID_COM')
        rigid_pose(self.box.container_T_box,'box')
        if not self.final_box_poses:raise ValueError('FINAL_BOX_POSES_REQUIRED')
        for value in self.final_box_poses:rigid_pose(value,'final_box_pose')
        known={p.source_id for p in self.snapshot.surfaces}
        if not self.support_source_ids or not set(self.support_source_ids)<=known:
            raise ValueError('UNKNOWN_OR_EMPTY_SUPPORT_IDS')
        if len(set(self.support_source_ids))!=len(self.support_source_ids):raise ValueError('DUPLICATE_SUPPORT_IDS')
        if type(self.start_attached) is not bool:raise ValueError('INVALID_ATTACHMENT_MODE')
        if self.start_attached!=(self.confirmed_box_T_tool is not None):raise ValueError('CONFIRMED_ATTACHMENT_REQUIRED_ONLY_FOR_ATTACHED_START')
        if self.confirmed_box_T_tool is not None:rigid_pose(self.confirmed_box_T_tool,'confirmed_box_T_tool')

    @property
    def revision(self):return digest(asdict(self))


def target_actions(assets,request,config,emit=None,deadline=None):
    """Generate exact targets, different handoffs/grasps/pushes; no IK claims.

    Current family: inverted, axis-aligned box, planar coplanar support, +container
    X push. Other families fail explicitly. All obstacles remain in the snapshot.
    A caller can supply several targets for its permitted final-pose adjustment.
    """
    start=time.monotonic();deadline=min(start+config.search_budget_s,deadline or float('inf'))
    reasons=Counter();witnesses=[];streams=[]
    selected=tuple(p for p in request.snapshot.surfaces if p.source_id in request.support_source_ids)
    for final_value in request.final_box_poses:
        final=matrix(final_value);bottom=final[2,3]-request.box.dimensions[2]/2
        patches=tuple(p for p in selected if abs(p.surface_z_container-bottom)<=config.contact_tolerance_m)
        if not patches:reasons['TARGET_NOT_ON_SUPPORT']+=1;continue
        # Do not mix different support heights into a flat support model.
        if max(p.surface_z_container for p in patches)-min(p.surface_z_container for p in patches)>config.contact_tolerance_m:
            reasons['NON_COPLANAR_SUPPORT_OUTSIDE_ACTION_FAMILY']+=1;continue
        yaw=None
        for angle in config.box_yaws_deg:
            expected=Rotation.from_euler('z',angle,degrees=True).as_matrix()@Rotation.from_euler('x',np.pi).as_matrix()
            if np.allclose(final[:3,:3],expected,rtol=0,atol=1e-7):yaw=angle;break
        if yaw is None:reasons['TARGET_ORIENTATION_OUTSIDE_ACTION_FAMILY']+=1;continue
        support=measure_support(final,request.box.dimensions,patches,config,request.box.com_local)
        if not support.accepted:reasons['FINAL_'+support.reason]+=1;continue
        rz=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
        length,width=np.abs(rz[:2,:2])@request.box.dimensions[:2]
        # Tilted yaw needs a face-normal push family; reject rather than use its AABB.
        if abs(np.sin(np.deg2rad(2*yaw)))>1e-7:
            reasons['TARGET_ORIENTATION_OUTSIDE_ACTION_FAMILY']+=1;continue
        bounds=unary_union([polygon_box(*p.bounds_xy) for p in patches]).bounds
        xmin,ymin,xmax,ymax=bounds
        if not ymin+width/2<=final[1,3]<=ymax-width/2:
            reasons['TARGET_LATERAL_OUTSIDE_SUPPORT']+=1;continue
        # Keep the input snapshot ID: the full request independently binds chosen
        # support IDs; only the private geometric view changes its surfaces.
        view=replace(request.snapshot,surfaces=patches)
        contact=np.array(assets['contact']['footprint_size_xy_m']);edge=assets['contact']['boundary_margin_m']
        for tool_yaw in config.tool_yaws_deg:
            rt=Rotation.from_euler('z',tool_yaw,degrees=True).as_matrix()
            bmax=length-float(np.abs(rt[0,:2])@contact)/2-edge
            forward=max(body.moved(pose([0,0,0],rt)).corners()[:,0].max()
                        for body in assets['tool'])
            low=max(config.support_area_ratio_min*length,length/2+config.support_com_margin_m)
            high=bmax-forward-config.collision_margin_m
            if high<low:reasons['HANDOFF_WINDOW_EMPTY']+=1;continue
            for depth in interior_samples(low,high,config.handoff_step_m):
                grasp_values=tuple(reversed(samples(depth+forward+config.collision_margin_m,bmax,config.grasp_step_m)))
                if request.start_attached:
                    tcp=final@matrix(request.confirmed_box_T_tool)
                    if not np.allclose(tcp[:3,:3],rt,rtol=0,atol=1e-7):continue
                    # Offset from the front face is invariant during straight push.
                    b=final[0,3]+length/2-tcp[0,3]
                    grasp_values=(b,) if depth+forward+config.collision_margin_m<=b<=bmax else ()
                if grasp_values:
                    streams.append(_action_stream(assets,view,request.box,config,(yaw,),(tool_yaw,),
                        final[1,3],(depth,),grasp_values,reasons,witnesses,deadline,emit,final_value))
    actions=[]
    for action in round_robin(streams):
        if request.start_attached and not np.allclose(action.box_T_tool,request.confirmed_box_T_tool,rtol=0,atol=1e-7):
            reasons['FIXED_GRASP_OUTSIDE_ACTION_FAMILY']+=1;continue
        actions.append(action)
        if emit:emit(dict(event='target_geometry_candidate',action_id=action.action_id,target=action.placement.final_box_pose))
        if len(actions)>=config.max_action_candidates or time.monotonic()>=deadline:break
    return actions,dict(reasons=dict(reasons),witnesses=witnesses,
        termination='BUDGET_EXHAUSTED' if time.monotonic()>=deadline else
                    'CANDIDATE_LIMIT' if len(actions)>=config.max_action_candidates else 'SEARCH_EXHAUSTED',
        count=len(actions),elapsed_s=time.monotonic()-start)
