# 2026-09-10 -- Undispatched plan rebaseline review

- role: reviews
- agent: codex-reviews-main
- model: gpt-5
- cli: codex
- status: done

## Summary

The undispatched SIM-R1, TCIG, LRF, and Todo 5 acceptance work is not uniformly
obsolete. TCIG's physical-hull work remains necessary and most of its gates
remain correct. SIM-R1's operator/state-machine architecture remains correct,
but its reuse assumptions and camera-map boundary predate the EXP-A1 findings
and the accepted depth-primary/D555 architecture. LRF's original serial chain
cannot be dispatched after the later LRF-P1 stop decision. The Todo 5 guide's
two-box smoke remains historical wiring evidence, but its capacity and
zero-regression instructions are not a current acceptance baseline. All four
documents need a reviewed rebaseline before any missing runnable rows are
created.

## Disposition

| Work | Decision | Required update before dispatch |
|---|---|---|
| SIM-R1-1 | retain complete | Carry accepted revision `713cfbb` forward; do not reopen its pure contracts unless a compatibility test fails. |
| SIM-R1-2 | revise materially | Treat EXP-A1 gaps as implementation scope, not reusable baseline: candidate-specific coverage, finite quaternion/dimension checks, duplicate rejection, atlas/hash/revision/stamp correlation, and an `ExplorationPolicy` adapter. Add external TCIG-7 as a passing dependency. |
| SIM-R1-3 | revise materially | Replace cloud/latest-frame language with canonical aligned depth plus same-grid CameraInfo, exact integer acquisition stamp, bounded local deprojection, stamped TF, settled-view correlation, and fail-closed mismatch behavior. Bind runtime acceptance to the D555 DSIM passing revision and TCIG-2. |
| SIM-R1-4 | retain concept, revise dependencies | Keep the thin `PlanNextCargoView` action and stop-and-look ordering; require revised SIM-R1-2/3 and accepted TCIG-7 runtime atlas identity. |
| SIM-R1-5 | mostly retain | Explicit Start and production state-machine gates remain valid. Rebind to current ROS 2 package state and keep it independent of physical calibration. |
| SIM-R1-6 | split acceptance | Static sim/hardware composition may prove identical production nodes and backend-only adapters. Live hardware/hand-eye/TF parity remains a later hardware gate after the board is ready; it must not block simulation integration. |
| SIM-R1-INTEGRATION | revise materially | Run only after PF-R10/PF-R7, D555 DSIM, TCIG integration, and revised R1-2..R1-6. Use canonical depth, not a Gazebo camera point cloud, for the persistent three-box loop. |
| TCIG-1 | retain complete | Carry accepted `7af4022` implementation forward. |
| TCIG-4 | retain complete | Carry accepted remediation revision `10a93e8` forward. |
| TCIG-2/3/5/6/7 | retain requirements, rebind | Gates remain necessary; refresh exact base, ownership, runtime interface names, and scheduling. Current center-only active voxels, rectangular placement enumeration, hard-coded metric denominators, rectangular EMS/replay, and incomplete atlas identity show these tasks are not already complete. |
| TCIG-INTEGRATION | revise dependencies | Re-run G1-G7 on one current revision and bind its simulation E2E to the canonical D555 depth pipeline. Hardware calibration is not required for the simulation result. |
| LRF-A1 | park; old chain not runnable | LRF-P1's later closeout says `stop this spike` and forbids shadow wiring. A learned NBV study needs a new research generation and a stable heuristic baseline after SIM-R1-2/4 plus TCIG-7; P1 completion alone is insufficient. |
| LRF-PL1 | park; old chain not runnable | Require stable TCIG-3/5/6/7 and an accepted deterministic packing baseline. It may only rank hard-gate-approved candidates. Do not infer readiness from LRF-P1. |
| Todo 5 B4 | replace acceptance section | Retain the n=2 result only as `capacity_claim_valid=false` wiring smoke. A new pack-to-full run waits for PF/DSIM simulation stability and TCIG-2/3/4/5 integration, uses exact hull denominators, and must end in genuine `BIN_FULL`. |
| Todo 5 B5 | redefine and rerun | Pin the exact no-packing baseline, commands, metrics, seed/profile, and current D555 depth path; compare on one clean revision. The old place-N=3 result does not by itself satisfy the guide's no-packing regression. |

## Recommended execution order

1. Finish PF-R10 and PF-R7, then D555 DSIM; this is the current simulation
   priority and produces the sensor/runtime baseline.
2. Rebaseline and dispatch TCIG-2, TCIG-5, and TCIG-7 where ownership permits;
   run TCIG-3 after TCIG-2 and TCIG-6 after TCIG-2/4/5/7, then TCIG integration.
3. Re-run Todo 5 B4/B5 on the integrated depth-plus-hull baseline.
4. Dispatch revised SIM-R1-2 and SIM-R1-5, then R1-3/R1-4/R1-6 and the
   persistent three-box integration according to the refreshed dependencies.
5. Reconsider LRF-A1 only after the deterministic heuristic NBV baseline is
   accepted; reconsider LRF-PL1 only after deterministic packing acceptance.

## Acceptance for the plan refresh

- A distinct Codex consensus reviews the revised SIM-R1/TCIG/LRF/packing
  dependency graph and confirms no circular dependency.
- Each unfinished executable item has one concrete owner/model, generation,
  exact current base, exact approved plan revision, and explicit
  `dispatch_ready` state before it appears in `OPEN.md`.
- Completed SIM-R1-1, TCIG-1, and TCIG-4 Results are referenced rather than
  rerun or silently rewritten.
- No camera-native Gazebo `PointCloud2`, latest-TF fallback, D455 naming, or
  unapplied hardware calibration appears as a simulation prerequisite.
- Packing capacity metrics use the exact TCIG hull volume/floor area and never
  the legacy rectangular constants.
- Research tasks remain advisory-only and cannot bypass deterministic geometry,
  collision, reachability, map identity, or motion gates.

## Risks

- Dispatching the old SIM-R1-2 text would either omit EXP-A1 hard gates or
  encourage a second incompatible exploration implementation.
- Running B4 now could produce a visually successful but invalid capacity
  claim because legacy rectangular denominators remain in active scripts.
- The early LRF-P1 thread says `continue-research`, while the later closeout
  says `stop this spike`; the later budget closeout must govern dispatch.
- The current shared worktree contains active PF-R10 changes, so no refreshed
  runnable row should bind to today's dirty `HEAD`.

## Pointers

- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/agents/discuss/2026-09-04_2021_exp-a1-nbv-readiness-audit.md`
- `docs/plans/true_container_inner_geometry.md`
- `docs/plans/learning_research_feasibility.md`
- `docs/status/evidence/learning_research/LRF-P1/CLOSEOUT.md`
- `docs/plans/pack_eval_test_guide.md`
- `docs/status/packing_eval_carryon_n2.md`
- `docs/plans/d555_sim_depth_pipeline_execution.md`
