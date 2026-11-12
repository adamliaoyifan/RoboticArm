# 2026-11-12 -- P1 selected map owner and shared O snapshot consumption

- status: approved
- dispatch_ready: yes
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P1
- generation: 1
- owner: codex/gpt-6
- base_revision: 7db985cb
- authorization: User "按计划执行", G3 P1 and source-resolution requirement.

## Bounded scope

Create a lower-level luggage_mapping package for immutable surface/source and O
snapshot transport, removing packing's need to import planning. Preserve existing
planning imports as explicit compatibility exports. Both ComputePlacement and
complete prepare use that O reader, owner bytes/resolution, exact identity and
dimension-derived progress checks; no independent consumer resolution or64-page
cap. Keep one request deadline and releases on all paths.

Add integrated hardware placement backend selector, default observer, explicit
grid_g. Resolve deployment resolution once, generate immutable O geometry/effective
configuration from current scene and pass exact files to owner/readers. O branch
does not create G; G branch does not create O and enables volume evidence. Existing
G transport and selector need not be rewritten in this unit. This prerequisite
does not claim full G/O route parity or default complete-operation client readiness.
No geometry priors, thresholds, release policy, maps or controller execution changed.

## Acceptance and procedures

| ID | Owner | Depends | Base | Scope | Tests | Passing evidence |
|---|---|---|---|---|---|---|
| ST-4-PRODUCTION-P1 | codex/gpt-6 | ST-4-PRODUCTION-P0 | 7db985cb | Above | Below | Pending |

System Python + Humble, production prefix. Deterministic tests run once, AND over
all named positive and negative cases; coverage, not per-operation reliability.

1. `scripts/production/run.sh python3 -m pytest -q --import-mode=importlib
   src/luggage_planning/test/test_map_owner_launch.py
   src/luggage_planning/test/test_joint_operation_observer.py
   src/luggage_planning/test/test_hardware_plan_only_launch.py
   src/luggage_planning/test/test_joint_operation_map_source.py`
   with source packages on an explicitly recorded test-only PYTHONPATH (inside the
   wrapper command), plugin autoload disabled. Tests execute actual launch callbacks
   and reader page assembly, not source spelling. Every case passes.
2. Matrix: O/G at default1cm and explicit2cm; exact one selected producer; O uses
   source optical cloud and configured frame, G uses cargo semantic cloud. Exact
   generated config resolution/hull reaches consumers; backend invalid/missing
   required inputs and mismatched resolution refuse. Generated files cannot be
   silently overwritten under the same digest.
3. Shared reader tests: same assembly at O source resolution; unknown/inactive
   retained; stalled/mixed-identity/short pages reject and release. Synthetic
   4,344,244-state paged service fixture completes67 pages, not64. Preserve exact
   boundary service requests/results for failures; no live67-RPC claim.
4. `scripts/production/build.sh --evidence <P1/run_01/build>` then installed checker
   at `<P1/run_01/install.json>` and both launch --show-args succeed. New mapping
   imports and compatibility exports retain the same class identity.

Evidence root docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P1/.
Before build, preserve test inputs/config/result/JUnit; build logs and before/after
hashes as P0. Test failure bundles retain request/response fixture and exception,
single-call window, JSON/text <=16MiB; build<=32MiB, total<=128MiB. Capture health
requires complete artifacts before rerun. No native/model/motion score. No nodes
or sim started; teardown not_needed. P2 admission and regional reads remain next.

## Pointers

- docs/plans/PRODUCTION_PLACEMENT_CONVERGENCE_G3_20261112.md
- docs/architecture/cargo_grid.md
- docs/architecture/placement.md
- docs/architecture/production_orchestration.md
- docs/agents/discuss/2026-11-12_1457_production-convergence-p0.md
