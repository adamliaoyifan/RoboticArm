# SIM-R1 Production Orchestrator and Exploration Review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: SIM-R1-20260904
- subtask: n/a
- revision: 7af40220c9e86feb7f56908d9fcde389aa48dd9d

## Summary

The user confirmed the production flow and modular exploration boundary. The
robot remains motionless until a visible explicit Start confirmation, explores
the empty container from live depth, returns to `pick_observe_pose`, and then
runs one pick-place cycle for each exact request-ID operator confirmation.
Simulation and hardware use the same algorithms; test code may only inject
inputs and inspect outputs. The legacy ROS 1 orchestrator moves to reference.

The selected exploration approach is prior-guided stop-and-look NBV. Its policy
is replaceable, including by a learned model, while map integration and all
geometry, timing, reachability, collision, motion, revision, and termination
hard gates remain coordinator-owned and invariant.

## Acceptance

- No startup/readiness event, `run`, or `step` produces motion before the
  explicit `start` command.
- Initial exploration uses online depth only and returns to pickup observe
  before publishing an operator request.
- Exact request IDs authorize one cycle; stale/duplicate/wrong IDs cause no
  motion.
- Three confirmed boxes are placed persistently without clearing earlier boxes.
- Policy replacement requires configuration only and cannot bypass hard gates.
- Fake inputs exist only in isolated tests; no production fake/simulation branch
  enters algorithm functions.
- Eval truth remains outside the online graph.

## Consensus

- Codex agent: `codex-sim-r1-consensus`
- Model: `gpt-5.5`
- Thread: `docs/agents/discuss/2026-09-04_1801_sim-r1-gpt55-consensus.md`
- Result: reached

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| SIM-R1-1 | `codex-sim-r1-eng/gpt-5.5` | none | interfaces and ROS-free orchestrator contracts | explicit Start and exact pickup authorization fail closed | plain pytest and interface build |
| SIM-R1-2 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1 | policy/coordinator plugin core | heuristic/model proposals share hard gates | policy fault matrix |
| SIM-R1-3 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1, TCIG-2 | stamped live-depth integration | one correlated view produces one revision | mapper unit and ROS adapter tests |
| SIM-R1-4 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-2,SIM-R1-3,TCIG-7 | thin ROS 2 exploration action | stop-look-integrate loop and cancellation semantics | action integration tests |
| SIM-R1-5 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1 | production rclpy orchestrator and legacy move | motionless launch and operator-driven cycles | state, adapter and launch tests |
| SIM-R1-6 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-4,SIM-R1-5 | backend launch parity | no fake runtime and same production nodes | launch/static checks |
| SIM-R1-INTEGRATION | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1..6 | complete workflow | persistent three-box Fortress close loop | full E2E and teardown |

## Risks

- SIM-R1-3 and SIM-R1-4 must wait for the corresponding TCIG map and atlas
  revisions and must not edit actively owned TCIG files.
- Existing `IntegrateCargoView` lacks sufficient stamp/revision correlation.
- Existing `luggage_bringup` remains catkin and its historical scripts require
  explicit classification during the ROS 2 migration.

## Pointers

- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/architecture/perception_architecture.md`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/container_geometry.md`
- `docs/status/phase2_interfaces.md`

## Open

- None. The approved plan must now be published at a reproducible revision
  before `SIM-R1-1` is dispatched.
