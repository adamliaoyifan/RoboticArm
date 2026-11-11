"""Collision-first deterministic ordering of scored complete nominal plans.

This produces no execution permission. Robust-payload and current-state admission
remain separate gates. Missing scores and stale input/model/policy identities fail
closed; different IK traces cannot inflate the count for one geometric action.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from plan_clearance import plan_digest


def _same_pose(a,b,translation_m,rotation_deg):
    a=np.asarray(a).reshape(4,4);b=np.asarray(b).reshape(4,4)
    distance=float(np.linalg.norm(a[:3,3]-b[:3,3]))
    angle=float(np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude()))
    return distance<=translation_m+1e-10 and angle<=rotation_deg+1e-10


def equivalent_actions(a,b,policy):
    def local_push(action):
        return (np.linalg.inv(np.array(action['handoff_box_pose']).reshape(4,4))@
                np.array(action['push_start_tool_pose']).reshape(4,4)@
                np.array(action['tool_T_push_surface']).reshape(4,4))
    pairs=((a['placement']['final_box_pose'],b['placement']['final_box_pose'],policy.placement_dedup_translation_m),
           (a['box_T_tool'],b['box_T_tool'],policy.grasp_dedup_translation_m),
           (a['handoff_box_pose'],b['handoff_box_pose'],policy.handoff_dedup_translation_m),
           (local_push(a),local_push(b),policy.push_dedup_translation_m))
    return (a['push_contact_geometry_id']==b['push_contact_geometry_id'] and
            all(_same_pose(x,y,limit,policy.pose_dedup_rotation_deg) for x,y,limit in pairs))


def _rejection(plan,snapshot_id,config,policy,model_identity):
    from physical_executor import STAGES
    try:
        score=plan['clearance_score']
        if not plan.get('valid') or not score.get('valid'):return 'INCOMPLETE_OR_INVALID_NOMINAL_PLAN'
        if (tuple(s['name'] for s in plan['stages'])!=STAGES or
            not all(s['certificate'].get('valid') for s in plan['stages'])):
            return 'INCOMPLETE_OR_INVALID_NOMINAL_PLAN'
        if (score['snapshot_id']!=snapshot_id or score['config_revision']!=config.revision or
            score['clearance_policy_revision']!=policy.clearance_revision or score['model_identity']!=model_identity):
            return 'STALE_SCORE_INPUT_OR_MODEL'
        if (plan['action']['placement']['snapshot_id']!=snapshot_id or
            plan['action']['placement']['config_revision']!=config.revision):
            return 'STALE_ACTION_INPUT'
        if score['plan_digest']!=plan_digest(plan) or score['action_id']!=plan['action']['action_id']:
            return 'SCORE_PLAN_BINDING_MISMATCH'
        if (len(score['stages'])!=len(plan['stages']) or
            not all(s.get('valid') for s in score['stages']) or
            [s['stage'] for s in score['stages']]!=[s['name'] for s in plan['stages']]):
            return 'INCOMPLETE_SCORE_STAGE_COVERAGE'
        values=[score[k] for k in ('robot_lower_m','robot_upper_m','payload_lower_m',
                                  'min_com_margin_m','min_support_area_ratio')]
        if not all(math.isfinite(v) for v in values):return 'INVALID_SCORE_METRIC'
        if (score['robot_upper_m']<score['robot_lower_m'] or
            score['robot_upper_m']-score['robot_lower_m']>policy.clearance_precision_m+1e-10):
            return 'INSUFFICIENT_SCORE_PRECISION'
        if min(score['robot_lower_m'],score['payload_lower_m'])<config.collision_margin_m-1e-10:
            return 'COLLISION_MARGIN_BELOW_POLICY'
        if (score['min_com_margin_m']<config.support_com_margin_m-1e-10 or
            score['min_support_area_ratio']<config.support_area_ratio_min-1e-10):
            return 'SUPPORT_BELOW_POLICY'
        if not score.get('robot_witness'):return 'MISSING_COLLISION_SCORE_WITNESS'
        if not equivalent_actions(plan['action'],plan['action'],policy):return 'INVALID_ACTION_POSE'
    except (KeyError,TypeError,ValueError):return 'SCORE_OR_PLAN_CONTRACT_INVALID'
    return None


def rank_nominal_plans(plans,snapshot_id,config,policy,model_identity):
    """Form collision tiers anchored at each remaining best proved clearance.

    Tier span is <= collision_tie_m. This avoids the nontransitive pairwise
    comparator 'within 1mm -> compare support'. Secondary metrics are COM, area,
    then stable action/path identities. Deduplication follows the ranked order.
    """
    pending=[];rejected=[]
    for index,plan in enumerate(plans):
        reason=_rejection(plan,snapshot_id,config,policy,model_identity)
        if reason:rejected.append(dict(index=index,reason=reason))
        else:pending.append(index)
    ordered=[];tiers=[]
    while pending:
        anchor=max(plans[i]['clearance_score']['robot_lower_m'] for i in pending)
        tier=[i for i in pending if anchor-plans[i]['clearance_score']['robot_lower_m']<=policy.collision_tie_m+1e-10]
        pending=[i for i in pending if i not in tier]
        tier.sort(key=lambda i:(-plans[i]['clearance_score']['min_com_margin_m'],
            -plans[i]['clearance_score']['min_support_area_ratio'],plans[i]['action']['action_id'],
            plans[i]['clearance_score']['plan_digest']))
        kept=[]
        for index in tier:
            duplicate=next((j for j in ordered if equivalent_actions(plans[index]['action'],plans[j]['action'],policy)),None)
            if duplicate is not None:rejected.append(dict(index=index,reason='EQUIVALENT_GEOMETRIC_ACTION',kept_index=duplicate))
            else:ordered.append(index);kept.append(index)
        tiers.append(dict(anchor_lower_m=anchor,indices=kept))
    return dict(ordered_indices=ordered,selected_index=ordered[0] if ordered else None,
        ordered_plan_digests=[plans[i]['clearance_score']['plan_digest'] for i in ordered],
        tiers=tiers,rejected=rejected,achieved=len(ordered),target=config.solution_count_target,
        minimum=config.minimum_executable_solutions,nominal_minimum_met=len(ordered)>=config.minimum_executable_solutions,
        selection_policy_revision=policy.revision,
        scope='scored_nominal_plans_only_not_robust_or_execution_admission')
