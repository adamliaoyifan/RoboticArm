# 2026-11-12 -- Observed full-model Gazebo operation integration

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- base_revision: 5086d84f2873db9b36ec94d8e67efeb83481c74f
- started_at: 2026-11-12T17:35:41+08:00
- completed_at: n/a

## Summary

Integrate an explicit observed O/full-model simulation pipeline. Full-cycle
baseline remains open until an actual observed controller cycle passes.

## Requirement

Follow docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md at983e80e1.
No guessed OCC, no online simulator truth, no unilateral collision margin change.

## Changed

Explicit sim_world pipeline selection; new grouped profile; one O owner and
observed optical depth adapter; same full native model for all operation ports;
explicit start diagnostic driver; installed source checks.

## Verification

- Production source runner:32 deterministic selection/depth cases pass.
- scripts/production/build.sh --evidence docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/build_01:13 packages pass,6.48s,zero concurrent source changes.
- Full-model sensor/controller integration: pending.

## Result

Open. Composition is implementation, not a baseline success witness.

## Pointers

- docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/
