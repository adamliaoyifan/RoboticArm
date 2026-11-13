"""Conservative distance proofs for a complete joint interpolation interval.

Shared ancestor joints rotate/translate both robot surfaces rigidly together;
they cannot change their separation. Nonshared joints each move at most one of
the surfaces, with point travel bounded by radius*abs(delta_angle).
"""
import time
import numpy as np


def certify_joint_intervals(scene,path,config,radius,contact_tool,deadline=None):
    checks=intervals=virtual_samples=0
    for index,(a,b) in enumerate(zip(path,path[1:])):
        pending=[(np.array(a,float),np.array(b,float))]
        while pending:
            if deadline is not None and time.monotonic()>deadline:
                return dict(valid=False,reason='CERTIFICATION_BUDGET_EXHAUSTED',checks=checks)
            low,high=pending.pop();mid=(low+high)/2
            failure=scene.check_state(mid,config.collision_margin_m,contact_tool);checks+=1
            if failure:return dict(valid=False,segment=index,checks=checks,**failure)
            delta=np.abs(high-low)/2
            if hasattr(scene,'relative_joint_indices'):
                proven=True
                for distance,pair in scene.sample_distances:
                    if hasattr(scene,'relative_motion_bound'):
                        bound=scene.relative_motion_bound(pair,delta)
                    else:
                        axes=scene.relative_joint_indices(pair)
                        bound=radius*sum(delta[j] for j in axes)
                    if distance<config.collision_margin_m+bound:
                        proven=False;break
            else:
                # Generic behavioural fixtures without source ancestry metadata.
                proven=scene.sample_clearance_m>=config.collision_margin_m+2*radius*sum(delta)
            if proven:
                intervals+=1
                virtual_samples+=max(1,int(np.ceil(2*radius*sum(delta)/config.sweep_step_m)))
            elif np.array_equal(mid,low) or np.array_equal(mid,high):
                return dict(valid=False,reason='UNCERTIFIABLE_CLEARANCE_BOUND',checks=checks)
            else:pending.extend(((mid,high),(low,mid)))
    return dict(valid=True,checks=checks,certified_intervals=intervals,
                virtual_1mm_samples=virtual_samples,step_bound_m=config.sweep_step_m,
                method='pairwise_ancestry_Lipschitz_interval_bounds')
