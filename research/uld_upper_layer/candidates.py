"""Joint grasp/handoff/push geometry search; no arm or execution success claims."""
from collections import Counter, deque
import time
import numpy as np
from scipy.spatial.transform import Rotation
from shapely.geometry import Point
from shapely.ops import unary_union
from shapely.geometry import box as polygon_box
from geometry import Box, pose
from ideal_snapshot import query_box
from planning_types import ActionCandidate, PlacementCandidate, digest
from support_geometry import measure_support
from action import boundary_failure
from tool_contact_geometry import derive_push_surface


def matrix(value):
    return np.asarray(value, float).reshape(4,4)


def flat(value):
    return tuple(np.asarray(value, float).ravel())


def samples(low, high, step):
    if high < low - 1e-9:
        return ()
    count = int(np.floor((high-low)/step + 1e-7))
    values=list(low+np.arange(count+1)*step)
    # Boundary samples matter for narrow geometric windows. Floating-point
    # arithmetic must not silently discard the legal high endpoint when its
    # distance from low is just below an integer multiple of step.
    if high-values[-1]>1e-9:values.append(high)
    else:values[-1]=high
    return tuple(values)


def tool_collision(assets, snapshot, tcp, clearance):
    for tool in assets['tool']:
        body = tool.moved(tcp)
        failure = boundary_failure(body, assets, clearance)
        if failure:
            return failure
        status, hits = query_box(snapshot, body, clearance)
        if status != 'FREE':
            return dict(reason='TOOL_'+status, body=body.name, hits=hits,
                        tcp=tcp.tolist())
    return None


def _action_stream(assets, snapshot, state, config, yaw_values, tool_yaw_values, lateral, depth_values, grasp_values, reasons, witnesses, deadline=None, emit=None, final_pose=None, fixed_grasp=None):
    """Return immutable geometric candidates and bounded diagnostic records.

    Grasp is selected on the original top face. The final box is inverted by
    Rx(pi), so that same material face rests on the upward panel and supports.
    All fixed tool bodies must stay clear of actual lower cuboids, not a shelf.
    """
    started = time.monotonic()
    push_geometry=derive_push_surface(assets)
    surface_T_tool=np.linalg.inv(matrix(push_geometry.tool_T_push_surface))
    if emit:emit(dict(event='geometry_stream_input',box_yaws_deg=yaw_values,
        tool_yaws_deg=tool_yaw_values,lateral_m=lateral,depths_m=depth_values,grasps_m=grasp_values))
    candidates = []
    windows = handoff_windows(assets, state, config)
    top = max(p.surface_z_container for p in snapshot.surfaces)
    patches = tuple(p for p in snapshot.surfaces if abs(p.surface_z_container-top)<=config.contact_tolerance_m)
    support_union = unary_union([polygon_box(*p.bounds_xy) for p in patches])
    xmin, ymin, xmax, ymax = support_union.bounds
    size = np.array(state.dimensions)
    contact_width = np.array(assets['contact']['footprint_size_xy_m'])
    edge = assets['contact']['boundary_margin_m']
    state_pose = matrix(state.container_T_box)
    # Necessary windows are a complete cheap matrix, even when later search budgets expire.
    for yaw in yaw_values:
        rz = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
        box_rotation = rz @ Rotation.from_euler('x', np.pi).as_matrix()
        projected = np.abs(rz[:2,:2]) @ size[:2]
        length, width = projected
        # Final positions are inside actual top envelope, not a hidden answer.
        final_xs = samples(xmin+length/2+config.collision_margin_m,
                          xmax-length/2-config.collision_margin_m, config.placement_step_m)
        ys = samples(ymin+width/2+config.collision_margin_m,
                     ymax-width/2-config.collision_margin_m, config.placement_step_m)
        if final_pose is not None:
            requested=matrix(final_pose)
            if not np.allclose(requested[:3,:3],box_rotation,rtol=0,atol=1e-7):
                reasons['TARGET_ORIENTATION_OUTSIDE_ACTION_FAMILY']+=1;continue
            if abs(requested[2,3]-(top+size[2]/2))>config.contact_tolerance_m:
                reasons['TARGET_NOT_ON_SUPPORT']+=1;continue
            final_xs=(float(requested[0,3]),)
        for tool_yaw in tool_yaw_values:
            rt = Rotation.from_euler('z', tool_yaw, degrees=True).as_matrix()
            footprint_x = np.abs(rt[0,:2]) @ contact_width
            footprint_y = np.abs(rt[1,:2]) @ contact_width
            bmax = length-footprint_x/2-edge
            forward = max(body.moved(pose([0,0,0],rt)).corners()[:,0].max()
                          for body in assets['tool'])
            dmin = max(config.support_area_ratio_min*length,
                       length/2+config.support_com_margin_m)
            dmax = bmax-forward-config.collision_margin_m
            if dmax < dmin or footprint_y+2*edge > width:
                reasons['HANDOFF_WINDOW_EMPTY'] += 1; continue
            # Interleave spatial placements and grasp/depth rather than exhaust
            # expensive final-pose paths at one point. Further expansion occurs
            # in the full trajectory search, using this bounded geometry list.
            for y in (lateral,):
                for depth in depth_values:
                    front = xmin+depth
                    handoff = pose([front-length/2,y,top+size[2]/2], box_rotation)
                    failure = boundary_failure(Box('payload', handoff[:3,3], box_rotation, size/2),assets,config.collision_margin_m)
                    if failure:
                        reasons['HANDOFF_APERTURE']+=1;continue
                    metrics = measure_support(handoff,size,patches,config,state.com_local)
                    if not metrics.accepted:
                        reasons[metrics.reason]+=1;continue
                    for b in grasp_values:
                        if deadline is not None and time.monotonic()>=deadline:return
                        tcp = (pose([front-b,y,top],rt) if fixed_grasp is None
                               else handoff@matrix(fixed_grasp))
                        failure = tool_collision(assets,snapshot,tcp,config.collision_margin_m)
                        # Verify the actual withdrawal legs including downward
                        # separation. Upward-carried box remains on lower cargo.
                        if not failure:
                            end = tcp.copy(); end[2,3] -= config.release_drop_m
                            end2 = end.copy(); end2[0,3] -= config.withdraw_m
                            for start,finish in ((tcp,end),(end,end2)):
                                count=max(1,int(np.ceil(np.linalg.norm(finish[:3,3]-start[:3,3])/
                                                       config.handoff_step_m)))
                                for t in np.linspace(0,1,count+1):
                                    waypoint = start.copy()
                                    waypoint[:3,3] = (1-t)*start[:3,3]+t*finish[:3,3]
                                    failure=tool_collision(assets,snapshot,waypoint,config.collision_margin_m)
                                    if failure: break
                                if failure:break
                        if failure:
                            reasons[failure['reason']]+=1
                            if len(witnesses)<32:witnesses.append(failure)
                            continue
                        box_to_tool=(np.linalg.inv(handoff)@tcp if fixed_grasp is None
                                     else matrix(fixed_grasp).copy())
                        pick=state_pose@box_to_tool
                        for final_x in final_xs:
                            if deadline is not None and time.monotonic()>=deadline:return
                            if final_x < handoff[0,3]+config.collision_margin_m:continue
                            final=(pose([final_x,y,top+size[2]/2],box_rotation) if final_pose is None
                                   else matrix(final_pose).copy())
                            final_metrics=measure_support(final,size,patches,config,state.com_local)
                            if not final_metrics.accepted:continue
                            payload=Box('payload',final[:3,3],box_rotation,size/2)
                            if boundary_failure(payload,assets,config.collision_margin_m):
                                reasons['PAYLOAD_HULL']+=1;continue
                            # Panel contact normal +X and panel footprint inside
                            # the rear face. Contact is intentional; all other
                            # fixed bodies must clear cargo and the actual hull.
                            push_r=Rotation.from_euler('y',np.pi/2).as_matrix()
                            low=top+contact_width[0]/2+edge
                            high=top+size[2]-contact_width[0]/2-edge
                            for z in interior_samples(low,high,config.push_height_step_m):
                                push_start=pose([front-length,y,z],push_r)@surface_T_tool
                                distance=final_x-handoff[0,3]
                                push_end=push_start.copy();push_end[0,3]+=distance
                                push_control_start=push_start.copy();push_control_start[0,3]+=config.push_contact_stroke_m
                                push_control_end=push_end.copy();push_control_end[0,3]+=config.push_contact_stroke_m
                                failure=tool_collision(assets,snapshot,push_control_start,config.collision_margin_m)
                                if not failure:failure=tool_collision(assets,snapshot,push_control_end,config.collision_margin_m)
                                if failure:
                                    reasons[failure['reason']]+=1;continue
                                pid=digest(dict(pose=flat(final),snapshot=snapshot.snapshot_id,
                                                config=config.revision))[:20]
                                placement=PlacementCandidate(pid,snapshot.snapshot_id,
                                    config.revision,flat(final),final_metrics)
                                aid=digest(dict(pid=pid,grasp=flat(box_to_tool),
                                                handoff=flat(handoff),push=flat(push_start),
                                                push_geometry=push_geometry.geometry_id))[:20]
                                contact_tcp=tcp.copy()
                                contact_tcp[2,3]-=config.support_contact_stroke_m
                                yield ActionCandidate(aid,placement,flat(box_to_tool),
                                    flat(pick),flat(handoff),flat(tcp),flat(contact_tcp),metrics,flat(push_start),
                                    flat(push_end),flat(push_control_start),flat(push_control_end),(1.,0.,0.),distance,tool_yaw,
                                    push_geometry.tool_T_push_surface,push_geometry.geometry_id,
                                    ('SUPPORT_AND_FIXED_TOOL_CHECKED',
                                     'FULL_ARM_NOT_EVALUATED','PUSH_PHYSICS_NOT_EVALUATED'))


def interior_samples(low,high,step):
    midpoint=(low+high)/2
    return tuple(sorted(samples(low,high,step),key=lambda value:abs(value-midpoint)))


def round_robin(streams):
    active=deque(streams)
    while active:
        stream=active.popleft()
        try:yield next(stream);active.append(stream)
        except StopIteration:pass


def support_ordered_positions(positions,x,z,rotation,dimensions,patches,config,com_local):
    """Try stronger measured support first, retaining every lateral position.

    A global row midpoint may straddle an actual gap between cargo. This is a
    geometry-only search heuristic, not a stability or collision certificate.
    Final eligible-plan ranking still prioritizes full-arm collision clearance.
    """
    if not positions:return ()
    center=(min(positions)+max(positions))/2
    def key(y):
        metrics=measure_support(pose([x,y,z],rotation),dimensions,patches,config,com_local)
        # Ignore sub-nanometre arithmetic noise when geometries have equal area
        # or margin; never let it choose an arbitrary edge over an interior.
        return (-round(metrics.contact_area_ratio,10),-round(metrics.com_margin_m,10),
                abs(y-center),y)
    return tuple(sorted(positions,key=key))


def generate_actions(assets, snapshot, state, config, emit=None, deadline=None):
    """Interleave orientations, positions, handoffs and grasps before repeats.

    Stronger geometric support then interior lateral/depth samples and the grasp
    with greatest tool/support clearance are tried first. No successful pose or
    arm result is fed in; gaps are measured from this snapshot.
    Full trajectory validation and final scoring remain separate gates.
    """
    started=time.monotonic();reasons=Counter();witnesses=[];candidates=[]
    deadline=min(started+config.search_budget_s,float('inf') if deadline is None else deadline)
    top=max(p.surface_z_container for p in snapshot.surfaces)
    surfaces=tuple(p for p in snapshot.surfaces
                   if abs(p.surface_z_container-top)<=config.contact_tolerance_m)
    patches=[polygon_box(*p.bounds_xy) for p in surfaces]
    xmin,ymin,_,ymax=unary_union(patches).bounds
    orientations=[];size=np.array(state.dimensions)
    contact=np.array(assets['contact']['footprint_size_xy_m'])
    edge=assets['contact']['boundary_margin_m']
    for yaw in config.box_yaws_deg:
        rz=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
        length,width=np.abs(rz[:2,:2])@size[:2]
        ys=interior_samples(ymin+width/2+config.collision_margin_m,
                            ymax-width/2-config.collision_margin_m,config.placement_step_m)
        for tool_yaw in config.tool_yaws_deg:
            rt=Rotation.from_euler('z',tool_yaw,degrees=True).as_matrix()
            bmax=length-float(np.abs(rt[0,:2])@contact)/2-edge
            forward=max(body.moved(pose([0,0,0],rt)).corners()[:,0].max() for body in assets['tool'])
            low=max(config.support_area_ratio_min*length,length/2+config.support_com_margin_m)
            high=bmax-forward-config.collision_margin_m
            if high<low or float(np.abs(rt[1,:2])@contact)+2*edge>width:
                reasons['HANDOFF_WINDOW_EMPTY']+=1;continue
            positions=[]
            lateral_order=support_ordered_positions(ys,xmin+(low+high)/2-length/2,
                top+size[2]/2,rz@Rotation.from_euler('x',np.pi).as_matrix(),size,
                surfaces,config,state.com_local)
            for y in lateral_order:
                variants=[]
                for depth in interior_samples(low,high,config.handoff_step_m):
                    # Greater b leaves more clearance between tool and support;
                    # preserve the entire footprint's configured edge margin.
                    grasps=reversed(samples(depth+forward+config.collision_margin_m,bmax,config.grasp_step_m))
                    for grasp in grasps:
                        variants.append(_action_stream(assets,snapshot,state,config,(yaw,),
                            (tool_yaw,),y,(depth,),(grasp,),reasons,witnesses,deadline,emit))
                positions.append(round_robin(variants))
            orientations.append(round_robin(positions))
    stream=round_robin(orientations)
    while len(candidates)<config.max_action_candidates:
        if time.monotonic()>=deadline:break
        try:
            candidate=next(stream);candidates.append(candidate)
            if emit:emit(dict(event='geometry_candidate',count=len(candidates),
                              elapsed_s=time.monotonic()-started,action=candidate))
        except StopIteration:
            return candidates,dict(windows=handoff_windows(assets,state,config),reasons=dict(reasons),
                witnesses=witnesses,termination=('BUDGET_EXHAUSTED' if time.monotonic()>=deadline
                                                else 'SEARCH_EXHAUSTED'),elapsed_s=time.monotonic()-started)
    return candidates,dict(windows=handoff_windows(assets,state,config),reasons=dict(reasons),
        witnesses=witnesses,termination='BUDGET_EXHAUSTED',elapsed_s=time.monotonic()-started)


def handoff_windows(assets, state, config):
    """Complete cheap necessary-condition matrix, independent of search budget."""
    result=[]
    footprint=np.array(assets['contact']['footprint_size_xy_m'])
    for yaw in config.box_yaws_deg:
        r=Rotation.from_euler('z',yaw,degrees=True).as_matrix()
        length=float(np.abs(r[0,:2])@np.array(state.dimensions[:2]))
        for tool_yaw in config.tool_yaws_deg:
            rt=Rotation.from_euler('z',tool_yaw,degrees=True).as_matrix()
            bmax=length-float(np.abs(rt[0,:2])@footprint)/2-assets['contact']['boundary_margin_m']
            forward=max(body.moved(pose([0,0,0],rt)).corners()[:,0].max() for body in assets['tool'])
            low=max(config.support_area_ratio_min*length,length/2+config.support_com_margin_m)
            high=float(bmax-forward-config.collision_margin_m)
            result.append(dict(box_yaw_deg=yaw,tool_yaw_deg=tool_yaw,
                insertion_min_m=low,insertion_max_m=high,feasible=bool(high>=low),
                source='full_width_necessary_condition'))
    return result
