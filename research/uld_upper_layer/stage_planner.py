"""Connected joint/Cartesian stage planning for a single joint action candidate.

All intermediate poses are derived from the measured box and candidate grasp.
The private collision scene changes contact mode explicitly at each boundary.
No function in this module commands the simulator or a physical object.
"""
from dataclasses import asdict
import time
import numpy as np
from candidates import matrix
from path_planning import cartesian_path, joint_path, time_path, certify_path


def shifted(transform, delta):
    result=transform.copy();result[:3,3]+=delta
    return result


def plan_action(scene,state,action,start_q,seeds,config,emit):
    """Backtrack pick joint branches as well as coupled handoff branches."""
    from action_screen import screen_push
    started=time.monotonic()
    scene.set_payload(state.dimensions,'world',
        scene.world_container@matrix(state.container_T_box),allow_support=True)
    initial_failure=scene.check_state(start_q,config.collision_margin_m)
    if initial_failure:return dict(valid=False,reason='INITIAL_STATE_INVALID',diagnostic=initial_failure,stages=[])
    screened=screen_push(scene,state,action,seeds,config)
    emit(dict(event='necessary_push_endpoint_screen',result=screened))
    if not screened['valid']:return dict(valid=False,reason=screened['reason'],diagnostic=screened,stages=[])
    pre_pick=shifted(matrix(action.pick_tool_pose),[0,0,config.precontact_distance_m])
    goals,diagnostic=scene.endpoint_solutions(pre_pick,[start_q]+seeds,
        config.collision_margin_m,config.path_attempt_budget_s)
    emit(dict(event='pick_ik_branches',q_solutions=goals,diagnostic=diagnostic))
    last=None;attempted=0
    for index,goal in enumerate(sorted(goals,key=lambda q:np.linalg.norm(np.array(q)-start_q))):
        if time.monotonic()-started>=config.search_budget_s:break
        attempted+=1
        emit(dict(event='pick_branch_input',branch=index,pick_pre_q=goal))
        last=_plan_from_pick_branch(scene,state,action,start_q,seeds,config,emit,
                                    goal,screened,started)
        emit(dict(event='pick_branch_output',branch=index,valid=last['valid'],
                  diagnostic=last.get('diagnostic'),reason=last.get('reason')))
        if last['valid']:
            last['pick_joint_branch']=index
            last['pick_joint_branches_attempted']=attempted
            return last
    return dict(valid=False,reason='NO_CONNECTED_PICK_BRANCH',
                pick_joint_branches_attempted=attempted,exhaustive=False,
                diagnostic=last,stages=[])


def _plan_from_pick_branch(scene,state,action,start_q,seeds,config,emit,
                            pick_pre_q,screened,started):
    stages=[];q=list(start_q)
    box_pose=matrix(state.container_T_box)
    grasp=matrix(action.box_T_tool)
    handoff=matrix(action.handoff_box_pose)
    pick=matrix(action.pick_tool_pose)
    tcp_handoff=matrix(action.handoff_tool_pose)
    push_start=matrix(action.push_start_tool_pose)
    push_end=matrix(action.push_end_tool_pose)
    world=scene.world_container

    def payload(mode, box=box_pose, support=False):
        scene.set_payload(state.dimensions,mode,world@box,grasp,support)

    def stage(name,target,kind='cartesian',contact=False,speed=None,box_motion=None,goal_q=None,goal_solutions=None,prepared_path=None):
        nonlocal q
        boundary=dict(stage=name,start_q=q,target_container=target.tolist(),
                      payload_mode=scene.payload['mode'],contact_tool=contact)
        emit(dict(event='stage_input',**boundary))
        if time.monotonic()-started>=config.search_budget_s:
            return dict(reason='ACTION_BUDGET_EXHAUSTED',**boundary)
        path=None;diagnostics=[]
        if prepared_path is not None:
            if not np.allclose(prepared_path[0],q,rtol=0,atol=1e-8):
                raise ValueError('PREPARED_BRANCH_START_MISMATCH')
            path=prepared_path;detail=dict(reason='REVERSED_CONSTRAINED_CHAIN')
        elif kind=='joint':
            if goal_solutions is not None:
                goals=goal_solutions;ik=dict(source='screened_push_pre_endpoints')
            elif goal_q is None:
                goals,ik=scene.endpoint_solutions(target,[q]+seeds,
                    config.collision_margin_m,config.path_attempt_budget_s)
            else:goals=[goal_q];ik=dict(source='backseeded_constrained_chain')
            for goal in sorted(goals,key=lambda v:np.linalg.norm(np.array(v)-q)):
                path,detail=joint_path(scene,q,goal,config,contact)
                diagnostics.append(detail)
                if path:break
            detail=dict(ik=ik,connections=diagnostics,
                        certificate=diagnostics[-1].get('certificate') if path else None)
        else:
            path,detail=cartesian_path(scene,q,target,config,contact,box_motion)
        if not path:
            result=dict(reason='STAGE_PATH_FAILED',**boundary,diagnostic=detail)
            emit(dict(event='stage_rejected',**result));return result
        timed=time_path(scene,path,config,speed)
        # The actual planned interpolation (cubic controller input) is checked,
        # not just IK waypoints. Push uses a declared nominal payload reference.
        motion=None
        if box_motion is not None:
            distances=[0.]
            for a,b in zip(path,path[1:]):
                ta=scene.urdf.tcp(a,scene.urdf.path_to('suction_contact_frame'))
                tb=scene.urdf.tcp(b,scene.urdf.path_to('suction_contact_frame'))
                distances.append(distances[-1]+np.linalg.norm(tb[:3,3]-ta[:3,3]))
            total=distances[-1]
            motion=lambda index,t:box_motion((distances[index]+t*(distances[index+1]-distances[index]))/total)
        emit(dict(event='sweep_input',stage=name,trajectory=timed))
        certificate=detail.get('certificate') or certify_path(scene,timed,config,contact,motion,
                                 deadline=started+config.search_budget_s,interval_bounds=box_motion is None)
        if not certificate['valid']:
            result=dict(reason='SWEEP_REJECTED',**boundary,certificate=certificate)
            emit(dict(event='stage_rejected',**result));return result
        record=dict(name=name,trajectory=timed,certificate=certificate,planning_diagnostic=detail,
                    payload_mode=scene.payload['mode'],contact_tool=contact,
                    target_container=target.tolist())
        stages.append(record);q=path[-1]
        emit(dict(event='stage_accepted',stage=name,end_q=q,
                  duration_s=timed['time_from_start'][-1],certificate=certificate))
        return None

    payload('world',support=True)
    failure=scene.check_state(q,config.collision_margin_m)
    if failure:return dict(valid=False,reason='INITIAL_STATE_INVALID',diagnostic=failure,stages=stages)
    pre_pick=shifted(pick,[0,0,config.precontact_distance_m])
    failure=stage('pick_pre',pre_pick,'joint',goal_q=pick_pre_q)
    if not failure:failure=stage('pick_contact',pick,contact=True,speed=config.contact_speed_mps)
    if failure:return dict(valid=False,diagnostic=failure,stages=stages)
    payload('attached',support=True)
    failure=stage('pick_lift',shifted(pick,[0,0,config.pick_lift_m]),contact=True)
    if failure:return dict(valid=False,diagnostic=failure,stages=stages)
    scene.payload['allow_support']=False
    opening=np.array(scene.assets['scene']['container']['opening']['aperture']['corners'])
    corners=np.array(scene.payload['mesh'].vertices)
    # Move the inverted box's leading edge outside the actual aperture before
    # insertion. This is derived from candidate geometry, not a preset answer.
    leading=(corners@handoff[:3,:3].T+handoff[:3,3])[:,0].max()
    approach=shifted(tcp_handoff,[opening[:,0].min()-config.outside_aperture_margin_m-leading,
                                  0,config.placement_lift_m])
    from coupled_chain import handoff_branches
    prefix_count=len(stages);carry_q=list(q);connected=False;failure=None
    for branch in handoff_branches(scene,action,approach,[q]+seeds,config,emit):
        del stages[prefix_count:];q=list(carry_q)
        scene.payload['allow_support']=False
        failure=stage('invert_and_approach',approach,'joint',contact=True,goal_q=branch['approach_q'])
        if not failure:
            scene.payload['allow_support']=True
            failure=stage('insert_up',shifted(tcp_handoff,[0,0,config.placement_lift_m]),
                          contact=True,prepared_path=branch['insertion_path'])
        if not failure:
            failure=stage('lower_to_support',tcp_handoff,contact=True,speed=config.contact_speed_mps,
                          prepared_path=branch['lowering_path'])
        if not failure:connected=True;break
        emit(dict(event='branch_route_rejected',branch=branch['branch'],diagnostic=failure))
    if not connected:
        return dict(valid=False,reason='NO_CONNECTED_HANDOFF_BRANCH',diagnostic=failure,stages=stages)
    payload('world',handoff,True)
    down=shifted(tcp_handoff,[0,0,-config.release_drop_m])
    failure=stage('release_down',down,contact=True,speed=config.contact_speed_mps)
    if not failure:failure=stage('withdraw',shifted(down,[-config.withdraw_m,0,0]))
    pre_push=shifted(push_start,[-config.precontact_distance_m,0,0])
    if not failure:failure=stage('push_pre',pre_push,'joint',
        goal_solutions=screened['endpoints']['push_pre']['q_solutions'])
    if not failure:failure=stage('push_contact',push_start,contact=True,speed=config.contact_speed_mps)
    if failure:return dict(valid=False,diagnostic=failure,stages=stages)
    nominal=lambda t:world@shifted(handoff,[action.push_distance_m*t,0,0])
    failure=stage('push',push_end,contact=True,speed=config.push_speed_mps,box_motion=nominal)
    if failure:return dict(valid=False,diagnostic=failure,stages=stages)
    payload('world',matrix(action.placement.final_box_pose),True)
    failure=stage('retreat',shifted(push_end,[-config.precontact_distance_m,0,0]),contact=True)
    return dict(valid=failure is None,diagnostic=failure,stages=stages,
                action=asdict(action),elapsed_s=time.monotonic()-started,
                certificate_scope='sampled_full_arm_and_payload_nominal_contact_modes',
                physical_execution='not_evaluated')
