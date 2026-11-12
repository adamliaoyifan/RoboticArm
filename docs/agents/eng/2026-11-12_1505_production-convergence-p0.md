# 2026-11-12 -- Production convergence P0 implementation

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P0
- base_revision: 9652fb5a7a312cec8ae65d97d9244ca04e7daa5d
- started_at: 2026-11-12T14:57:28+08:00

## Summary

Added a named production build/run boundary and actual import/path/content gates.
Fresh sequential Humble build installed12 packages in58.93s without changing old
prefixes or any input during the build. No robot, simulator or map started.

## Requirement

Plan dc9b3c9f, ST-4-PRODUCTION-P0 generation1; executable installation prerequisite
of the user-authorized production convergence plan. Whole-chain G2 remains open.

## Changed

`scripts/production/` clears inherited workspace overlays, builds into the named
production prefix and records before/after source provenance. Runtime wrapper
sources only Humble and the production local setup. Checker imports current
modules, checks origin/source bytes, executable placement, ament lookup/resources.
Focused behavioral fixtures reject missing/stale/wrong-prefix files and imports.

## Verification

- Fresh build:12 packages, zero failure, zero concurrent changed inputs.
- Installed boundary:32/32 checks pass; new prepare/runtime modules and generated
  operation/O types import from install-production.
- Behavioral fixtures:5/5 pass; valid matching imports/files, stale module/script,
  wrong-origin imports, missing imports/executables and external symlink refusal.
- Full checker subprocess using a nonexistent prefix fails as expected.
- Poisoned inherited Python/ament/CMake/colcon paths:32/32 checks still pass through
  production wrapper, demonstrating actual isolation from the parent overlay.
- Installed hardware/prepare launch argument parsing:2/2 pass without node start.
- Evidence1,581,340 bytes, below128MiB gate. No motion/map/model qualification claim.

## Result

P0 deterministic installation gate passes. Map consumers, four-heading admission,
default complete-operation client and full-model simulation remain separate work.

## Pointers

- docs/plans/PRODUCTION_CONVERGENCE_P0_EXECUTION_20261112.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P0/run_01/result.json
- docs/agents/discuss/2026-11-12_1457_production-convergence-p0.md
- scripts/production/README.md
