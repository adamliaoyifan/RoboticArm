"""Necessary full-action endpoint screening before expensive prefix planning.

Sampled IK rejection is a search result, never a proof of unreachability. The
scene includes the released box at its hypothetical handoff/final positions.
"""
import numpy as np
from candidates import matrix
from planning_types import digest


def push_screen_key(snapshot,state,action,seeds,config):
    """Reuse only an identical sampled endpoint problem, including seed order.

    The grasp relation is absent deliberately: this screen uses a released
    world box, never an attached payload. Scope includes all other inputs.
    """
    return digest(dict(snapshot=snapshot.snapshot_id,dimensions=state.dimensions,
        handoff=action.handoff_box_pose,final=action.placement.final_box_pose,
        push_start=action.push_start_tool_pose,push_end=action.push_end_tool_pose,
        push_control_start=action.push_start_control_tool_pose,
        push_control_end=action.push_end_control_tool_pose,
        push_geometry=action.push_contact_geometry_id,tool_T_push_surface=action.tool_T_push_surface,
        direction=action.push_direction,seeds=[list(q) for q in seeds],
        config=config.revision))


def screen_push(scene,state,action,seeds,config):
    scene.validate_push_geometry(action)
    world=scene.world_container
    handoff=matrix(action.handoff_box_pose)
    pre=matrix(action.push_start_tool_pose).copy()
    pre[:3,3]-=config.precontact_distance_m*np.array(action.push_direction)
    result={}
    try:
        for name,box,target,contact in (
            ('push_pre',handoff,pre,False),
            ('push_end',matrix(action.placement.final_box_pose),matrix(action.push_end_control_tool_pose),'push_plate')):
            scene.set_payload(state.dimensions,'world',world@box,allow_support=True)
            goals,diagnostic=scene.endpoint_solutions(target,seeds,
                config.collision_margin_m,config.path_attempt_budget_s)
            accepted=[];rejected=[]
            for q in goals:
                failure=scene.check_state(q,config.collision_margin_m,contact)
                if failure:rejected.append(dict(q=q,diagnostic=failure))
                else:accepted.append(q)
            result[name]=dict(q_solutions=accepted,diagnostic=diagnostic,rejected=rejected)
            if not accepted:return dict(valid=False,reason='NO_SAMPLED_PUSH_ENDPOINT_SOLUTION',
                stage=name,exhaustive=False,endpoints=result)
        return dict(valid=True,endpoints=result,scope='NECESSARY_ENDPOINTS_NOT_CONNECTED_PATH')
    finally:
        scene.set_payload(state.dimensions,'world',world@matrix(state.container_T_box),allow_support=True)
