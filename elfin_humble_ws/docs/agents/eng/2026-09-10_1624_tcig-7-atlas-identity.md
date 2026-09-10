# 2026-09-10 -- TCIG-7 hull-aware atlas identity

- role: eng
- agent: codex-tcig-atlas-eng
- model: gpt-5.6-sol
- cli: codex
- status: done
- parent: TCIG-20260904
- subtask: TCIG-7
- base_revision: 4425f227a74b789944e002a1adc8b2144553116c
- started_at: 2026-09-10T15:26:11+08:00
- completed_at: 2026-09-10T16:24:36+08:00

## Summary

Implemented hull-masked atlas construction/query, clipped reachable-volume metrics, runtime geometry identity invalidation, exact floor/debug geometry, a ROS 2 builder adapter, and deterministic migration of all four checked-in legacy atlases.

## Requirement

- Skip IK for hull-invalid full payloads, exclude inactive cells from denominators, and fail closed on missing, stale, or changed geometry identity.
- Keep the ROS-free kernel testable and the historical rospy builder unchanged under `ros1_reference`.

## Changed

- `src/luggage_planning/luggage_planning/atlas_builder.py`
- `src/luggage_planning/luggage_planning/reachability_atlas.py`
- `src/luggage_planning/luggage_planning/container_geometry_runtime.py`
- `src/luggage_planning/luggage_planning/atlas_migration.py`
- `src/luggage_planning/data/reachability_atlas/s20_container_collision_aware*`
- `src/luggage_msgs/msg/ContainerOpeningEstimate.msg`
- ROS 2 adapters, packaging, and focused planning tests

## Verification

- Description suite: 143 passed; planning suite: 258 passed, 14 subtests passed.
- Required three-package `colcon build` and `colcon test`: pass; 417 tests, 0 errors, 0 failures.
- Four migrated atlas pairs load with exact geometry hash `cb41f55f243786d50ab30997291083ab40467507abcf78a00e55da77ffc00856`.
- `git diff --check`: pass.

## Result

- pass at `c499e818d754e845e3840edc45a03a3b421da496`.

## Pointers

- `docs/agents/discuss/2026-09-10_1524_tcig-7-atlas-identity-g1.md`
- `docs/status/evidence/true_container_inner_geometry/8edc404/g7/summary.md`
