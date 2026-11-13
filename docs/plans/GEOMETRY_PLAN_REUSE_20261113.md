# 2026-11-13 — Measured geometry and reusable whole-operation planning

- owner: eng/codex/gpt-6
- base_revision: ed2c47f434497226601a0c2ea7fc712a4db52e98
- authorization: user requests all four tasks from the preceding design discussion

## Scope and order

1. Actual selected-pose admission shared with placement search: continuous quarter-turn footprint, support, hull, same-source native volume. Search lattice membership is not admission.
2. Bounded pre-pick same-instance estimation and frozen geometry; impact-based compatibility/recertification decisions, original acquisition timestamps retained. Seal domain is observed support, never an inferred full-body mask. Fixed physical attachment after seal.
3. Measured structural LiDAR return association before O: preserve ULD collision geometry and structure ray terminals. No fixed floor deletion band, GT online, or rounding-as-accuracy. Uncalibrated inputs fail closed/retain original obstacles.
4. Successful trajectory reuse: bounded cache as proposal, compatible model/endpoints plus mandatory current-scene native validation and certificate reissue. Failure falls back to ordinary planning. Persist only successful execution records, with atomic storage and explicit schema.

## Acceptance

Each unit has deterministic behavioural tests, one invocation per matrix; all expected decisions must match (AND over deterministic cases, not a statistical success-rate claim). Exact commands will be recorded with results. Unit environment is workspace Python/ROS Humble imports and native source resolution. Preserve baseline source snapshots and unrelated dirty changes; commits stage only owned files/hunks.

Placement: four headings times floor/stack; harmless 2.108mm parity-crossing update keeps original XYZ; positive-volume wall/occupied/unknown intersection and insufficient support refuse. Generated feasible targets and exact-pose validator agree. Reuse: identical certified envelope keeps path, enlarged envelope requests validation, failed validation replans, ambiguous identity/occluded shrink does not silently shrink; after-seal grasp unchanged. LiDAR: observed structure residual fit, terminal preservation, ambiguous nearby object retained, inadequate/mismatched calibration never suppresses hits. Cache: round-trip persistence, exact endpoints/model binding, current collision rejection and planner fallback, corrupt/incompatible entries ignored, successful-execution-only promotion.

Integration: focused affected suites followed by the existing explicit joint_operation Gazebo profile, single world, full robot, RGB-D/O default and qualified atlas, floor/floor/stack three-box run. One successful functional witness is required; no statistical reliability claim. LiDAR qualification is reported separately and cannot be claimed from RGB-D success. Rehearsal failures before execution are unscored and bounded by 30min per attempt; preserve artifacts before repair. Product changes invalidate prior scored evidence and require a fresh run. No production latency threshold is invented; record per-stage timings and cache hits/misses.

## Evidence and dump matrix

Evidence root: docs/status/evidence/geometry_plan_reuse/20261113/ . Every command records revision, dirty state, environment, input identities, timing and result. Deterministic failures retain exact fixture parameters and assertion traceback. Integration reuses the established three-box boundary recorder and audit, with cache/geometry-decision diagnostics added.

| Boundary | T1 | Trigger | T2 | Window | Format/budget | Inspect |
|---|---|---|---|---|---|---|
| placement | target/dimensions/map/reason | unexpected verdict | exact surface/volume/target | one case | JSON/NPZ, 20MB | pytest case replay |
| perception geometry | instance/stamps/extents/decision | mismatch/uncertainty | bounded contributing frames and frozen model | estimation window | JSON/NPZ, 100MB | focused test/replay |
| LiDAR structure | calibration/TF/source counts | wrong association | points, planes, labels, ray endpoints | one scan plus calibration | JSON/NPZ, 100MB | focused replay |
| path reuse/native planning | key/endpoints/hit/verdict/time | native rejection/fallback | cached path, native request/scene and output | one planning attempt | typed CDR/JSON, 250MB | existing joint-plan tools |
| execution | plan/events/controller/O versions | fault/timeout | established three-box recorder | failing leg plus event boundaries | CDR/JSON/CSV, 2GB per case | existing three-box audit |

Total retained integration budget 8GB plus prior untouched evidence; no deletion of failed evidence before diagnosis. Capture health requires nonempty joinable inputs/outputs and replay identifiers; missing payload is incomplete evidence. Debug overhead is reported, not compared as clean latency. Record exact run command and source/install hash verification before scoring; stop stack using scripts/stop_sim.sh after owned runs.
