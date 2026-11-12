# 2026-11-12 -- Prior purpose and removal boundary

- role: reviews
- agent: codex
- model: gpt-6
- cli: codex
- status: done

## Summary

User asks if the prior exists to substitute missing observations with5cm
separation and whether it can be removed. Source audit distinguishes three
independent mechanisms. No runtime or source behavior changed in this reply.

Placement prior is a read-only candidate/grasp hint provider. It precomputes
slot proposals and pick offsets before expensive complete path validation;
optional background templates hypothesize next-box size/grasps, not environment
free space. The surface store requires measured, confirmed OCC and matching
revision/acquisition. Unknown space remains blocked. Prior choices do not
certify motion; missing/stale hints fall back to ordinary planning in the
legacy consumer. Thus no observation is not the purpose or valid replacement
rule, and5cm is not intrinsic to the prior algorithm.

O collision_margin_m=0.05m is a separate inherited per-face margin. Complete
operation preparation and runtime directly use the shared snapshot converter
and ranking utility, without requiring the prior sidecar service. Disabling
the sidecar therefore does not remove O inflation or undo early candidate
ranking/truncation influenced by that value. Prior service is optional, but
blindly deleting placement_prior_* modules would break shared dependencies.

The packing solver separately has SAME_LAYER_CLEARANCE_M=0.05m and the
structural proposal default gap_min=0.05m. PlacementPlanner exposes a clearance
parameter, but the new prepare enumerate_placements currently calls
 generate_candidates with only top_n and therefore inherits solver defaults.
These actual box spacing rules are distinct from O collision padding; removing
O padding alone is not a packing-density policy change. No deployed override
or calibrated replacement distance is asserted.

Recommended discussion boundary: optional speculative sidecar may be disabled
while keeping measured-O slot generation, atlas seed consumption and complete
MoveIt certification. Remove the unexplained blanket5cm assumption from the
policy design; review measured error and geometry terms, explicit spacing,
unknown coverage and distinct release gap before implementing replacement.
Do not choose an arbitrary new small clearance or erase unknowns to pass.

## Pointers

- `docs/architecture/placement_prior.md`
- `docs/plans/placement_prior_g1.md`
- `src/luggage_planning/luggage_planning/placement_prior_surface.py`
- `src/luggage_planning/luggage_planning/placement_prior_requests.py`
- `src/luggage_planning/luggage_planning/placement_prior_ranking.py`
- `src/luggage_planning/luggage_planning/joint_operation_prepare_node.py`
- `src/luggage_planning/luggage_planning/joint_operation_runtime_ports.py`
- `src/luggage_packing/luggage_packing/placement_solver.py`
- `src/luggage_packing/luggage_packing/placement_structural_candidates.py`
- `docs/agents/reviews/2026-11-12_1236_occupancy-margin-provenance.md`
