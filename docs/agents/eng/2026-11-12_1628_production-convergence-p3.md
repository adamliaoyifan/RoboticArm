# 2026-11-12 -- Native-grid regional transport foundation

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P3
- base_revision: 5ea0132b3c5f107241661d6d2382490fe33aa2e7
- started_at: 2026-11-12T16:19:36+08:00
- completed_at: 2026-11-12T16:27:13.984406+08:00

## Summary

Native-resolution half-open region blocks preserve identity, grid origin and
explicit coverage. Full O capture now uses the same exact region reader; G can
slice an already admitted full response into the identical block representation.
No full-model simulation baseline or measured network improvement claimed.

## Requirement

Plan ac681ff4 generation1. No resampling, unqueried-free assumption, guessed
radius, cross-revision stitching, or fixed page-count cap. Complete volume remains
required for native motion certification; staged candidate ROI scheduling is not
enabled. Latency policy remains baseline-first per latest user answer.

## Changed

- Added luggage_mapping.regions with NativeGrid, CellRegion, CellBlock and explicit
  admitted-volume adapter. Outside-coverage reads refuse. Original grid is retained;
  block cannot masquerade as a complete-volume DTO. Inactive cells remain unknown.
- O regional requests start cursor0 and retain exact bounds, identity, generation
  and caller deadline on every page. Counts follow region size and native65536 cap.
  Malformed pages and changes fail without a partial return. Lease release retained.
- Installed-origin checker includes the new module. Updated cargo-grid architecture.

## Verification

101 deterministic tests pass, zero skipped, in1.92s through production source-test
wrapper; file list/command and immutable fixtures in result.json. Real ROS O/G DTOs
match scalar region output. Source-reader whole capture and failure release tested.
Full149x197x148 fixture reads4,344,244cells in67pages;0.713s is Python transport-double
time only. No owner server, native endpoint, controller or Gazebo process ran.

Initial fixture numpy.bool_ response error and later real G numpy.uint32 adapter
refusal preserved with source/log/JUnit. Adapter now explicitly normalizes only
integer transport dimensions; fractional/bool dimensions still refuse. Rebuild13
packages6.53s, before/after source hashes unchanged;38installed checks pass.

## Result

- pass at c65dfcd5eb0ef18d7d40d53734809703791d453d for this bounded transport unit.
- Full-chain production client, staged candidate ROI use, and full-model simulated
  success with stage timings remain open; no operation reliability claim.

## Pointers

- `docs/plans/PRODUCTION_CONVERGENCE_P3_EXECUTION_20261112.md`
- `docs/agents/discuss/2026-11-12_1619_production-convergence-p3.md`
- `docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P3/run_01/result.json`
