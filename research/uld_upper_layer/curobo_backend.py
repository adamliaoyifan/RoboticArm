"""GPU proposal admission: exact source endpoints, efforts, and FCL intervals.

GPU spline timing is deliberately discarded. stage_planner retimes this path
using the existing controller interpolation and certifies that executed locus.
"""
import time
import numpy as np
from curobo_scene import make_request
from curobo_client import CuroboClient


def admit(scene,path,start,goal,config,contact_tool,deadline):
    from interval_certificate import certify_joint_intervals
    from payload_dynamics import screen_path
    from path_planning import robot_radius
    q=np.asarray(path,dtype=float)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or not np.isfinite(q).all():
        return None,dict(reason='INVALID_GPU_PATH')
    # Float32 endpoint rounding is normalized only within this explicit tolerance;
    # source endpoints and every resulting edge are then checked from scratch.
    errors=[float(np.max(np.abs(q[0]-start))),float(np.max(np.abs(q[-1]-goal)))]
    if max(errors)>1e-5:return None,dict(reason='GPU_ENDPOINT_MISMATCH',errors_rad=errors)
    q[0]=start;q[-1]=goal
    limits=np.asarray(scene.urdf.joint_limits())
    if np.any(q<limits[:,0]) or np.any(q>limits[:,1]):return None,dict(reason='GPU_JOINT_LIMIT')
    for index,state in enumerate(q):
        if time.monotonic()>=deadline:return None,dict(reason='GPU_ADMISSION_BUDGET_EXHAUSTED')
        failure=scene.check_state(state,config.collision_margin_m,contact_tool)
        if failure:return None,dict(reason='GPU_STATE_REJECTED',index=index,witness=failure)
    effort=screen_path(scene,q.tolist(),config,deadline)
    if not effort['valid']:return None,dict(reason='GPU_EFFORT_REJECTED',witness=effort)
    proof=certify_joint_intervals(scene,q.tolist(),config,robot_radius(scene),contact_tool,deadline)
    if not proof['valid']:return None,dict(reason='GPU_INTERVAL_REJECTED',certificate=proof)
    return q.tolist(),dict(reason='CONNECTED_WITH_INTERVAL_PROOFS',path_source='curobo_fast_source_certified',
                          endpoint_rounding_rad=errors,certificate=proof)


def joint_path(scene,start_q,goal_q,config,contact_tool=False):
    started=time.monotonic();deadline=started+config.path_attempt_budget_s
    try:
        # Endpoint failures precede expensive device/model initialization.
        for name,q in (('START',start_q),('GOAL',goal_q)):
            failure=scene.check_state(q,config.collision_margin_m,contact_tool)
            if failure:return None,dict(reason=name+'_INVALID',witness=[failure])
        request=make_request(scene,start_q,goal_q,config,contact_tool)
        client=getattr(scene,'_curobo_client',None)
        runtime_key=(config.curobo_python,config.curobo_e2e_root)
        if client is None or not client.finalizer.alive or getattr(scene,'_curobo_runtime_key',None)!=runtime_key:
            if client is not None:client.close()
            client=CuroboClient(config.curobo_python)
            scene._curobo_client=client;scene._curobo_runtime_key=runtime_key
        result=client.query(request,deadline-time.monotonic())
        if not result.get('success'):
            return None,dict(reason='CUROBO_PROPOSAL_FAILED',gpu=result,elapsed_s=time.monotonic()-started)
        path,detail=admit(scene,result.get('positions'),start_q,goal_q,config,contact_tool,deadline)
        detail.update(gpu=result,request_identity=request['identity'],elapsed_s=time.monotonic()-started)
        return path,detail
    except (OSError,ValueError,RuntimeError) as exc:
        return None,dict(reason='CUROBO_BACKEND_ERROR',error=str(exc),elapsed_s=time.monotonic()-started)
