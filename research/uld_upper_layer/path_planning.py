"""OMPL free motion and seed-continuous Cartesian motion in a private FCL scene."""
import time
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def cartesian_path(scene,start_q,target_container,config,contact_tool=False,box_motion=None):
    from luggage_planning.offline_ik import refine_to_pose
    start=scene.container_world@scene.urdf.tcp(start_q,scene.urdf.path_to('suction_contact_frame'))
    target=np.asarray(target_container).reshape(4,4)
    distance=np.linalg.norm(target[:3,3]-start[:3,3])
    angle=Rotation.from_matrix(start[:3,:3].T@target[:3,:3]).magnitude()
    # A conservative bounding radius scales rotation into point displacement.
    radius=max(np.linalg.norm(b.center)+np.linalg.norm(b.half) for b in scene.assets['tool'])
    count=max(1,int(np.ceil((distance+angle*radius)/config.cartesian_step_m)))
    rotations=Slerp([0.,1.],Rotation.from_matrix([start[:3,:3],target[:3,:3]]))
    q=list(start_q);path=[q];started=time.monotonic()
    for index,t in enumerate(np.linspace(0,1,count+1)[1:]):
        desired=np.eye(4);desired[:3,3]=(1-t)*start[:3,3]+t*target[:3,3]
        desired[:3,:3]=rotations(t).as_matrix()
        world=scene.world_container@desired
        next_q,info=refine_to_pose(scene.urdf,scene.urdf.path_to('suction_contact_frame'),
            world[:3,3],Rotation.from_matrix(world[:3,:3]).as_quat(),q)
        if next_q is None:return None,dict(reason='CARTESIAN_IK',index=index,ik=info)
        if np.max(np.abs(np.array(next_q)-q))>np.pi:
            return None,dict(reason='CARTESIAN_BRANCH_JUMP',index=index)
        if box_motion is not None:scene.payload['world_pose']=box_motion(float(t))
        failure=scene.check_state(next_q,config.collision_margin_m,contact_tool)
        if failure:return None,dict(index=index,**failure)
        q=next_q;path.append(q)
    return path,dict(reason='CARTESIAN_CONNECTED',elapsed_s=time.monotonic()-started)


def robot_radius(scene):
    radius=sum(np.linalg.norm(j['xyz']) for j in scene.urdf.joints.values())
    radius+=max(np.linalg.norm(b.center)+np.linalg.norm(b.half) for b in scene.assets['tool'])
    if hasattr(scene,'robot_parts'):
        radius+=max(np.max(np.linalg.norm(mesh.vertices,axis=1))+np.linalg.norm(local[:3,3])
                    for _,mesh,local in scene.robot_parts.values())
    if getattr(scene,'payload',None) is not None:
        radius+=np.linalg.norm(scene.payload['size'])/2
        if scene.payload['box_T_tool'] is not None:radius+=np.linalg.norm(scene.payload['box_T_tool'][:3,3])
    return radius


def joint_path(scene,start_q,goal_q,config,contact_tool=False):
    from ompl import base as ob,geometric as og
    from interval_certificate import certify_joint_intervals
    started=time.monotonic();deadline=started+config.path_attempt_budget_s;active_deadline=deadline
    limits=np.array(scene.urdf.joint_limits())
    space=ob.RealVectorStateSpace(6);bounds=ob.RealVectorBounds(6)
    for i in range(6):bounds.setLow(i,limits[i,0]);bounds.setHigh(i,limits[i,1])
    space.setBounds(bounds)
    setup=og.SimpleSetup(space);witness=[];cache={};radius=robot_radius(scene)
    def valid(state):
        if time.monotonic()>active_deadline:return False
        q=[state[i] for i in range(6)]
        failure=scene.check_state(q,config.collision_margin_m,contact_tool)
        if failure and not witness:witness.append(failure)
        return failure is None
    def key(a,b):return tuple(sorted((tuple(a),tuple(b))))
    class Motion(ob.MotionValidator):
        def __init__(self,si):super().__init__(si)
        def checkMotion(self,a,b):
            qa=[a[i] for i in range(6)];qb=[b[i] for i in range(6)]
            identity=key(qa,qb)
            if identity not in cache:
                from payload_dynamics import screen_path
                effort=screen_path(scene,[qa,qb],config,active_deadline)
                if not effort['valid']:
                    cache[identity]=effort
                    if not witness:witness.append(effort)
                    return False
                cache[identity]=certify_joint_intervals(scene,[qa,qb],config,radius,
                                                       contact_tool,active_deadline)
            detail=cache[identity]
            if not detail['valid'] and not witness:witness.append(detail)
            return detail['valid']
    setup.setStateValidityChecker(valid)
    si=setup.getSpaceInformation();motion=Motion(si);si.setMotionValidator(motion)
    start=space.allocState();goal=space.allocState()
    for i in range(6):start[i]=start_q[i];goal[i]=goal_q[i]
    if not valid(start):return None,dict(reason='START_INVALID',witness=witness)
    if not valid(goal):return None,dict(reason='GOAL_INVALID',witness=witness)
    setup.setStartAndGoalStates(start,goal);si.setup()
    result=None;path_evidence={};path_source='certified_direct_joint_segment'
    def snapshot_path(path):
        points=[[path.getState(j)[i] for i in range(6)] for j in range(path.getStateCount())]
        edges=[]
        for index,(a,b) in enumerate(zip(points,points[1:])):
            proof=cache.get(key(a,b))
            edges.append(dict(index=index,cached=proof is not None,
                              valid=proof['valid'] if proof else False,
                              reason=proof.get('reason') if proof else 'MISSING_CACHE_KEY',
                              checks=proof.get('checks') if proof else None))
        return dict(positions=points,edges=edges,remaining_budget_s=deadline-time.monotonic())
    if motion.checkMotion(start,goal):result=[list(start_q),list(goal_q)]
    elif time.monotonic()<deadline:
        planner=og.RRTConnect(si)
        planner.setRange(config.ompl_range_rad)
        setup.setPlanner(planner)
        outcome=setup.solve(max(0.,deadline-time.monotonic()))
        if outcome and setup.haveExactSolutionPath():
            path_evidence['before_simplification']=snapshot_path(setup.getSolutionPath())
            remaining=deadline-time.monotonic()
            if remaining>0 and config.path_simplification_budget_s>0:
                simplification_started=time.monotonic()
                active_deadline=min(deadline,simplification_started+config.path_simplification_budget_s)
                try:setup.simplifySolution(max(0.,active_deadline-time.monotonic()))
                finally:active_deadline=deadline
                path_evidence['simplification_elapsed_s']=time.monotonic()-simplification_started
            path=setup.getSolutionPath()
            path_evidence['after_simplification']=snapshot_path(path)
            edited=path_evidence['after_simplification']
            original=path_evidence['before_simplification']
            if all(edge['valid'] for edge in edited['edges']):
                result=edited['positions'];path_source='certified_simplified_path'
            elif all(edge['valid'] for edge in original['edges']):
                # The Python copy cannot be mutated by the native simplifier.
                # Preserve the already certified solution if editing loses proof.
                result=original['positions'];path_source='certified_rrt_path_simplification_rejected'
            else:
                return None,dict(reason='RETURNED_PATH_MISSING_INTERVAL_PROOF',
                                 witness=witness,path_evidence=path_evidence)
    if result is None:return None,dict(reason='PATH_BUDGET_EXHAUSTED',witness=witness)
    proofs=[cache.get(key(a,b)) for a,b in zip(result,result[1:])]
    if any(p is None or not p['valid'] for p in proofs):
        return None,dict(reason='RETURNED_PATH_MISSING_INTERVAL_PROOF',witness=witness,path_evidence=path_evidence)
    return result,dict(reason='CONNECTED_WITH_INTERVAL_PROOFS',witness=witness,
        path_source=path_source,path_evidence=path_evidence,
        certificate=dict(valid=True,checks=sum(p['checks'] for p in proofs),
                         method='pairwise_ancestry_Lipschitz_interval_bounds',
                         interpolation='monotone_joint_cubic_same_locus',edges=proofs))


def time_path(scene,path,config,tcp_speed=None):
    """Stop-at-knot cubic segments, matching position+zero-velocity controller input.

    Smoothstep has max normalized velocity 1.5, acceleration 6. Durations
    conservatively enforce source joint velocities and configured acceleration.
    """
    velocities=[]
    for i in range(1,7):
        element=scene.urdf.root.find(f"joint[@name='elfin_joint{i}']/limit")
        velocities.append(float(element.get('velocity'))*config.joint_speed_scaling)
    velocities=np.array(velocities);times=[0.]
    tcp_path=scene.urdf.path_to('suction_contact_frame')
    for a,b in zip(path,path[1:]):
        delta=np.abs(np.array(b)-a)
        duration=max(float(np.max(1.5*delta/velocities)),
                     float(np.max(np.sqrt(6*delta/config.joint_acceleration_radps2))),.001)
        model=getattr(scene,'payload_dynamics',None)
        if model is not None:
            duration=max(duration,float(np.max(1.5*delta*model.damping/model.reserve)))
        if tcp_speed:
            pa=scene.urdf.tcp(a,tcp_path)[:3,3];pb=scene.urdf.tcp(b,tcp_path)[:3,3]
            duration=max(duration,1.5*np.linalg.norm(pb-pa)/tcp_speed,
                         np.sqrt(6*np.linalg.norm(pb-pa)/config.push_acceleration_mps2))
        times.append(times[-1]+duration)
    return dict(positions=path,velocities=[[0.]*6 for _ in path],time_from_start=times,
                interpolation='cubic_zero_velocity_knots')


def certify_path(scene,timed,config,contact_tool=False,box_motion=None,deadline=None,interval_bounds=False):
    """Bound all link-point travel using chain-length Lipschitz subdivision.

    The 1mm sampling gate is evidence at the declared granularity, not a claim
    of exact continuous collision detection. Mesh queries run at every sample.
    """
    radius=robot_radius(scene)
    checks=0;path=timed['positions']
    if interval_bounds:
        from interval_certificate import certify_joint_intervals
        return certify_joint_intervals(scene,path,config,radius,contact_tool,deadline)
    for index,(a,b) in enumerate(zip(path,path[1:])):
        count=max(1,int(np.ceil(radius*np.sum(np.abs(np.array(b)-a))/config.sweep_step_m)))
        for t in np.linspace(0,1,count+1):
            progress=t*t*(3-2*t)
            q=(1-progress)*np.array(a)+progress*np.array(b)
            if box_motion is not None:
                scene.payload['world_pose']=box_motion(index,progress)
            if deadline is not None and time.monotonic()>deadline:
                return dict(valid=False,reason='CERTIFICATION_BUDGET_EXHAUSTED',checks=checks)
            # Both members of a self-collision pair can move between samples.
            # A 2*step guard keeps the declared margin between checked states.
            failure=scene.check_state(q,config.collision_margin_m+2*config.sweep_step_m,contact_tool)
            checks+=1
            if failure:return dict(valid=False,segment=index,fraction=float(t),checks=checks,**failure)
    return dict(valid=True,checks=checks,step_bound_m=config.sweep_step_m)
