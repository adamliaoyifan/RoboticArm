"""Shared production placement admission, independent of ROS and map backend.

Inputs are an admitted surface and same-snapshot XYZ verifier. This is a set of
packing proposals, not a robot-motion certificate. Native whole-chain planning
remains responsible for each bound grasp, directed placement and route.
"""
import math

from luggage_description.container_geometry import normalize_descriptor
from luggage_packing.placement_solver import placement_constraint_verdict, solve_placement
from luggage_packing.placement_volume_query import KnownFreeVolume

DIRECTED_PLACEMENT_YAWS = (0., math.pi / 2, math.pi, 3 * math.pi / 2)
NOMINAL_SIDE_GAP_M = 0.0  # User permits final side contact, never penetration.


def ranking_only_atlas(annotate):
    """A center-TCP prior cannot rule out all legal eccentric grasps."""
    if annotate is None:
        return None

    def rank(candidates):
        verdicts = {row['candidate_id']: (row['feasible'], row['reason'], row.get('verdict'))
                    for row in candidates}
        annotated, report = annotate(candidates)
        for candidate in annotated:
            feasible, reason, verdict = verdicts[candidate['candidate_id']]
            if feasible and not candidate['feasible']:
                candidate.setdefault('detail', {})['center_atlas_rejection_hint'] = candidate['reason']
            candidate['feasible'], candidate['reason'] = feasible, reason
            if verdict is not None:
                candidate['verdict'] = verdict
        return annotated, dict(report or {}, authority='ranking_only')
    return rank


def placement_query(surface, box_size, *, allowed_yaws=DIRECTED_PLACEMENT_YAWS,
                    params=None, placed_aabbs=(), volume_verifier=None,
                    atlas_annotate=None):
    """Common hard gates and bounded directed pool for Compute and prepare.

    ``placed_aabbs`` may contain only admitted measured/committed geometry, in
    floor-relative coordinates (or named oriented-box records as solver accepts).
    Caller-proposed placements must not be passed as observed obstacles.
    """
    hull = normalize_descriptor(surface['geometry_descriptor'])
    for yaw in allowed_yaws:
        if (not math.isfinite(yaw)
                or abs(math.remainder(yaw, math.pi / 2)) > 1e-6):
            raise ValueError('placement requires directed quarter-turn headings')
    cfg = dict(same_layer_clearance_m=NOMINAL_SIDE_GAP_M,
               retain_heading_diversity=True, verify_floor_prior_volume=True,
               payload_symmetry='none', top_n=2000, keep_rejected=200)
    cfg.update(params or {})
    # These are admission invariants, not optional solver tuning switches.
    cfg.update(retain_heading_diversity=True, verify_floor_prior_volume=True,
               payload_symmetry='none')
    verifier = volume_verifier if volume_verifier is not None else KnownFreeVolume(None)

    def validate(candidate):
        return placement_constraint_verdict(
            candidate, surface['inner_size'][2], placed_aabbs=placed_aabbs,
            inner_size=surface['inner_size'], smallest_size=box_size, hull=hull,
            floor_z=hull.floor_z,
            same_layer_clearance_m=cfg['same_layer_clearance_m'],
            support_tol=cfg.get('support_tol', .05), corridor_as_ranking=True)

    result = solve_placement(surface, box_size, allowed_yaws=allowed_yaws,
        params=cfg, candidate_validator=validate, volume_verifier=verifier,
        atlas_annotate=ranking_only_atlas(atlas_annotate))
    result['admission_policy'] = dict(name='shared_observed_volume',
        nominal_side_gap_m=cfg['same_layer_clearance_m'],
        straight_corridor='ranking_only', center_atlas='ranking_only',
        motion_authority='bound_full_model_trajectory')
    return result
