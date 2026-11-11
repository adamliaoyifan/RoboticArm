"""Back-seed constrained insertion from each collision-free handoff IK branch.

An independently reachable approach pose does not establish that its elbow and
wrist branch can continue through insertion. Solve the tight chain backwards;
the free joint-space hop must arrive at that actual chain endpoint.
"""
import numpy as np
from candidates import matrix
from path_planning import cartesian_path


def handoff_branches(scene,action,approach,seeds,config,emit):
    handoff=matrix(action.handoff_tool_pose)
    contact=matrix(action.support_contact_tool_pose)
    lifted=handoff.copy();lifted[2,3]+=config.placement_lift_m
    scene.payload['allow_support']=True
    goals,diagnostic=scene.endpoint_solutions(contact,seeds,
        config.collision_margin_m,config.path_attempt_budget_s)
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
