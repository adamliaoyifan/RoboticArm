# 2026-09-09 -- D555 simulation depth-pipeline dispatch

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Approved a four-task execution plan for replacing the active Gazebo camera
cloud path with D555-profile colour-aligned depth, consumer-local deprojection,
and acquisition-stamped optical-to-world TF. All work is assigned to
`cursor/grok-4.6`, but the rows are intentionally held with
`dispatch_ready: no` until their exact prerequisites exist.

PF-R9 generation 2 currently owns overlapping dirty perception files. DSIM-1
generation 1 must be superseded by a higher generation bound to the eventual
PF-R9 passing commit before release. DSIM-INTEGRATION also remains behind
PF-R7 fixed-revision certification unless separate authority supersedes
PF-R10/PF-R7.

## Acceptance

- Active ROS 2 Gazebo graph contains no camera-native or preprocessed full
  camera cloud; Livox and semantic output clouds remain allowed.
- Canonical products are 640x360 at 15 Hz, colour-aligned, common truthful
  optical frame, runtime K within 2 percent of HB-1, and 16UC1 millimetres.
- Every world-XYZ path uses local deprojection with same-grid
  `CameraInfo` and TF at the exact acquisition stamp.
- `pickup_observe` passes the full catalog with required optical Z at least
  0.30 m, valid-depth ratio at least 0.95, and image margin at least 10 px.
- Integration passes the 120-second ratio gates, three consecutive Gate-4
  short-six runs without lowered geometry bars, the three-pose world oracle,
  full package tests, and zero-residual simulation teardown.

## Consensus

- Codex agent: `codex-d555-sim-consensus`
- Thread: `docs/agents/discuss/2026-09-09_1455_d555-sim-depth-consensus.md`
- Result: reached

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| DSIM-1 | `cursor/grok-4.6` | none plus R0 | Backend bridge and D555 profile | DS1-C0 through DS1-C5 | Static, adapter boundary, 30-second runtime |
| DSIM-2 | `cursor/grok-4.6` | DSIM-1 | Observe-pose geometry | DS2-C0 through DS2-C2 | Catalog, IK/collision/settle, near negative |
| DSIM-3 | `cursor/grok-4.6` | DSIM-1 | Cloud cleanup and stamped TF | DS3-C0 through DS3-C3 | Inventory, fault matrix, graph audit |
| DSIM-INTEGRATION | `cursor/grok-4.6` | DSIM-1,DSIM-2,DSIM-3 plus R2 | Whole-chain regression | DSI-C0 through DSI-C4 | 120-second run, Gate-4 x3, world oracle, package suites |

## Risks

- Current mailbox tasks are visible but not runnable; Cursor must not claim
  them while `dispatch_ready: no`.
- HE-2 is not a data-path prerequisite, but absolute sim-to-hardware world
  parity cannot be claimed until the calibrated transform is applied.
- D555 enclosure collision and mass placement remain outside scope because
  this plan does not establish housing placement relative to the optical
  datum.

## Pointers

- `docs/plans/d555_sim_depth_pipeline_execution.md`
- `docs/agents/discuss/2026-09-09_1522_d555-sim-dsim1-backend-profile.md`
- `docs/agents/discuss/2026-09-09_1522_d555-sim-dsim2-observe-geometry.md`
- `docs/agents/discuss/2026-09-09_1522_d555-sim-dsim3-cloud-tf-closure.md`
- `docs/agents/discuss/2026-09-09_1522_d555-sim-integration.md`
- `02e8afdab9eda85e8dc50837ee58b911c4d8251a`
