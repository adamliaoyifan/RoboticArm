# 2026-10-02 -- Place yaw plan review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The rejected plan identifies a real missing contract: placement must use the measured post-attachment box-to-TCP transform instead of reusing pre-pick PCA yaw. Its proposed atlas yaw rejection, message placement, Cartesian transit, executor retreat, and perception-only writer should not be adopted. Fold the useful attachment and target-synthesis work into ST09-A/B/C under the existing unified MoveIt plus directed-graph route contract.

## Acceptance

- Preserve `placement_yaw` as the box pose selected by packing and represent `theta + pi` as an execution-equivalent heading candidate.
- Record the measured full `T_tcp_box`, dimensions, acquisition stamp, frame identities, request correlation, and validity/provenance after attachment; do not reduce the contract to scalar `psi` plus a translation.
- Derive every TCP waypoint from `T_world_box * inverse(T_tcp_box)` and certify the actual attached payload along the complete timed trajectory.
- Use atlas yaw bins only as seed/prior classes; failure to match a bin cannot prove an orientation is invalid, and a seed may never authorize execution.
- Keep MoveIt plus the directed graph as the only route producer; waypoint generation and execution may not add Cartesian, OMPL, seeded-hop, or retreat planning bypasses.
- Missing measured attachment evidence must fail before place planning and release, with a named diagnostic and no mutation effect.

## Risks

- Treating quaternion sign or `theta + pi` as different physical orientations can reject valid wrist-continuous routes.
- A scalar yaw offset loses tilt and eccentric suction geometry needed for collision and release verification.
- Making the current two-bin atlas an orientation allow-list would encode incomplete sampled reachability as a permanent product restriction.
- Adding a new field only to the legacy service would conflict with the ST09 typed-v2 migration and require another interface migration.

## Pointers

- `docs/plans/frame_migration_st09_planner_convergence.md`
- `docs/plans/frame_migration_st09_parallel_execution.md`
- `docs/architecture/placement.md`
- `src/luggage_planning/luggage_planning/waypoint_generator.py`
- `src/luggage_planning/luggage_planning/atlas_runtime.py`
