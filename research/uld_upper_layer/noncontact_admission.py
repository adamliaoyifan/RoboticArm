"""Consumer checks for the partial noncontact certificate, never execution admission."""
import math
import numpy as np
from plan_clearance import plan_digest

FREE_BOX_STAGES = ('release_down', 'withdraw', 'push_pre', 'push_contact', 'push', 'retreat')


def noncontact_rejection(plan, snapshot_id, config, selection_policy, execution_policy, model_identity):
    try:
        proof = plan['noncontact_tube']
        if proof['noncontact_clearance_certified'] is not True:
            return 'NONCONTACT_TUBE_NOT_CERTIFIED'
        if (proof['plan_digest'] != plan_digest(plan) or proof['snapshot_id'] != snapshot_id or
                proof['config_revision'] != config.revision or proof['model_identity'] != model_identity or
                proof['execution_policy_revision'] != execution_policy.revision or
                proof['clearance_policy_revision'] != selection_policy.clearance_revision):
            return 'STALE_NONCONTACT_TUBE'
        if (proof['translation_error_m'] != execution_policy.push_lateral_error_m or
                proof['rotation_error_deg'] != execution_policy.push_rotation_error_deg or
                proof['distance_numeric_error_m'] != selection_policy.distance_numeric_error_m or
                proof['error_frame'] != 'world' or
                proof['rotation_model'] != 'SO3_geodesic_ball_about_geometric_box_centre'):
            return 'NONCONTACT_TUBE_DOMAIN_MISMATCH'
        if (tuple(proof['requested_stages']) != FREE_BOX_STAGES or
                tuple(s['stage'] for s in proof['stages']) != FREE_BOX_STAGES):
            return 'INCOMPLETE_NONCONTACT_TUBE'
        for row in proof['stages']:
            if (row['noncontact_clearance_certified'] is not True or
                    not math.isfinite(row['lower_m']) or row['lower_m'] < config.collision_margin_m or
                    row['checks'] < 1 or row['certified_intervals'] < 1):
                return 'NONCONTACT_TUBE_MARGIN_UNPROVED'
            if row['stage'] == 'push':
                direction = np.asarray(row['transverse_direction_world'])
                if (row['translation_model'] != 'perpendicular_ball' or direction.shape != (3,) or
                        not np.isfinite(direction).all() or not np.isclose(np.linalg.norm(direction), 1.)):
                    return 'NONCONTACT_TUBE_DOMAIN_MISMATCH'
            elif row['translation_model'] != 'isotropic_ball':
                return 'NONCONTACT_TUBE_DOMAIN_MISMATCH'
        if proof['execution_admitted'] is not False or proof['intended_contact_uncertainty'] != 'not_evaluated':
            return 'NONCONTACT_TUBE_SCOPE_INVALID'
    except (KeyError, TypeError, ValueError, AttributeError):
        return 'MISSING_OR_INVALID_NONCONTACT_TUBE'
    return None
