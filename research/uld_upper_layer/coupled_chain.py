"""Back-seed constrained insertion from each collision-free handoff IK branch.

An independently reachable approach pose does not establish that its elbow and
wrist branch can continue through insertion. Solve the tight chain backwards;
the free joint-space hop must arrive at that actual chain endpoint.
"""
import numpy as np
from candidates import matrix
from path_planning import cartesian_path


def handoff_branches(scene,action,approach,seeds,config,emit,goals=None):
    handoff=matrix(action.handoff_tool_pose)
    contact=matrix(action.support_contact_tool_pose)
    lifted=handoff.copy();lifted[2,3]+=config.placement_lift_m
    scene.payload['allow_support']=True
    if goals is None:
        goals,diagnostic=scene.endpoint_solutions(contact,seeds,
            config.collision_margin_m,config.path_attempt_budget_s)
    else:diagnostic=dict(source='full_collision_screened_handoff_branches')
    emit(dict(event='handoff_ik_branches',action_id=action.action_id,
              count=len(goals),diagnostic=diagnostic))
    for index,q in enumerate(goals):
        scene.payload['allow_support']=True
        failure=scene.check_state(q,config.collision_margin_m,True)
        if failure:
            emit(dict(event='branch_rejected',branch=index,stage='handoff_endpoint',diagnostic=failure));continue
        lowering,detail=cartesian_path(scene,q,lifted,config,True)
        if lowering is None:
            emit(dict(event='branch_rejected',branch=index,stage='reverse_lower',diagnostic=detail));continue
        insertion,detail=cartesian_path(scene,lowering[-1],approach,config,True)
        if insertion is None:
            emit(dict(event='branch_rejected',branch=index,stage='reverse_insert',diagnostic=detail));continue
        emit(dict(event='constrained_branch_connected',branch=index,
                  approach_q=insertion[-1],handoff_q=q))
        yield dict(branch=index,approach_q=insertion[-1],
                   insertion_path=list(reversed(insertion)),
                   lowering_path=list(reversed(lowering)))


def push_branches(scene,state,action,goals,config,emit):
    """Connect the moving-box push backwards before choosing its entry branch.

    Endpoint reachability alone cannot select a branch for a constrained push.
    Reverse the declared straight box motion together with the TCP, then reverse
    the resulting joint path. The caller still certifies forward interpolation.
    """
    world=scene.world_container
    handoff=matrix(action.handoff_box_pose)
    final=matrix(action.placement.final_box_pose)
    start=matrix(action.push_start_control_tool_pose)
    pre=matrix(action.push_start_tool_pose).copy()
    pre[:3,3]-=config.precontact_distance_m*np.asarray(action.push_direction)
    def box_at(t):
        pose=handoff.copy()
        pose[:3,3]=(1-t)*final[:3,3]+t*handoff[:3,3]
        return world@pose
    for index,q in enumerate(goals):
        scene.set_payload(state.dimensions,'world',world@final,allow_support=True)
        reverse,detail=cartesian_path(scene,q,start,config,'push_plate',box_at)
        if reverse is None:
            emit(dict(event='push_branch_rejected',branch=index,stage='reverse_push',diagnostic=detail))
            continue
        scene.set_payload(state.dimensions,'world',world@handoff,allow_support=True)
        entry,detail=cartesian_path(scene,reverse[-1],pre,config,'push_plate')
        if entry is None:
            emit(dict(event='push_branch_rejected',branch=index,stage='reverse_push_contact',diagnostic=detail))
            continue
        failure=scene.check_state(entry[-1],config.collision_margin_m,False)
        if failure:
            emit(dict(event='push_branch_rejected',branch=index,stage='push_pre',diagnostic=failure))
            continue
        yield dict(branch=index,pre_q=entry[-1],contact_path=list(reversed(entry)),
                   push_path=list(reversed(reverse)))
