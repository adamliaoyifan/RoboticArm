"""Validate and, when permitted, reroute free-box noncontact motion.

Only a producer-declared joint route without tool contact can be rerouted.
Cartesian release/withdraw/push semantics and every stage endpoint are fixed.
Each repair is independently rechecked in a plain source scene before scoring.
No simulator or robot command is issued here.
"""
from copy import deepcopy
from dataclasses import replace
import time
import numpy as np
from full_arm import FullArmScene
from envelope_planning import FreeBoxEnvelopeScene
from payload_tube import certify_payload_noncontact_tube
from path_planning import joint_path, time_path


def prepare_free_box_motion(assets, snapshot, state, plan, config, selection_policy,
                            execution_policy, deadline, emit, query_observer=None):
    candidate = deepcopy(plan)
    repairs = []
    def reject(reason, diagnostic=None):
        candidate['valid'] = False
        return dict(valid=False, reason=reason, diagnostic=diagnostic,
                    plan=candidate, repairs=repairs)
    from physical_executor import STAGES
    if (not candidate.get('valid') or tuple(s['name'] for s in candidate['stages']) != STAGES or
            not all(s['certificate'].get('valid') for s in candidate['stages'])):
        return reject('COMPLETE_NOMINAL_PLAN_REQUIRED')
    while time.monotonic() < deadline:
        validation_scene = FullArmScene(assets, snapshot, config)
        validation_scene.query_observer = query_observer
        proof = certify_payload_noncontact_tube(validation_scene, state, candidate, config,
            selection_policy, execution_policy, deadline, emit=emit)
        emit(dict(event='free_box_envelope_validation', certificate=proof))
        if proof['noncontact_clearance_certified']:
            candidate['noncontact_tube'] = proof
            candidate['envelope_repairs'] = repairs
            candidate['execution_admitted'] = False
            return dict(valid=True, plan=candidate, repairs=repairs)
        failed = proof['stages'][-1]
        stage = next(s for s in candidate['stages'] if s['name'] == failed['stage'])
        if (stage.get('motion_kind') != 'joint' or stage['payload_mode'] != 'world' or
                stage.get('box_motion_model') != 'fixed_world' or
                stage.get('box_reference_container') is None or
                stage['contact_tool'] is not False):
            return reject('ENVELOPE_STAGE_CANNOT_BE_REROUTED', proof)
        if any(r['stage'] == stage['name'] for r in repairs):
            return reject('ENVELOPE_REPAIR_NOT_CERTIFIED', proof)
        if failed.get('reason') != 'ENVELOPE_SAMPLE_UNPROVED':
            return reject('ENVELOPE_VALIDATION_INCOMPLETE', proof)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        routing_config = replace(config, path_attempt_budget_s=min(config.path_attempt_budget_s, remaining))
        scene = FreeBoxEnvelopeScene(assets, snapshot, config, selection_policy.distance_numeric_error_m,
            execution_policy.push_lateral_error_m, np.deg2rad(execution_policy.push_rotation_error_deg))
        scene.query_observer = query_observer
        scene.set_payload(state.dimensions, 'world',
            scene.world_container @ np.array(stage['box_reference_container']).reshape(4, 4),
            np.array(candidate['action']['box_T_tool']).reshape(4, 4), allow_support=True)
        trajectory = stage['trajectory']; start = trajectory['positions'][0]; goal = trajectory['positions'][-1]
        emit(dict(event='free_box_envelope_route_input', stage=stage['name'], start_q=start,
                  goal_q=goal, budget_s=routing_config.path_attempt_budget_s, failure=failed))
        path, detail = joint_path(scene, start, goal, routing_config, stage['contact_tool'])
        repair = dict(stage=stage['name'], path_found=path is not None, diagnostic=detail)
        repairs.append(repair)
        emit(dict(event='free_box_envelope_route_output', **repair))
        if not path:
            return reject('ENVELOPE_ROUTE_NOT_FOUND', detail)
        if not np.array_equal(path[0], start) or not np.array_equal(path[-1], goal):
            return reject('ENVELOPE_ROUTE_ENDPOINT_CHANGED')
        stage['trajectory'] = time_path(scene, path, config)
        stage['certificate'] = detail['certificate']
        stage['planning_diagnostic'] = detail
        emit(dict(event='free_box_envelope_route_trajectory', stage=stage['name'],
                  trajectory=stage['trajectory'], certificate=stage['certificate']))
    return reject('ENVELOPE_PREPARATION_DEADLINE')
