# 2026-09-16 -- Hardware executor safety dispatch

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

The user selected the HW wave from the dirty-tree audit: exclusive trajectory
goal ownership, fail-closed CPS completion polling, and controller-limit
preflight capped at 80 percent. Existing uncommitted executor changes are the
accepted starting point; unrelated dirty files are out of scope. Real-cell
motion is operator-gated and is not authorized by this implementation task.

## Acceptance

- Atomic admission admits exactly one owner; overlapping goals are rejected as
  busy, cannot reconnect the hardware, and cannot clear or set another goal's
  cancellation token. Ownership is released on every terminal or exception
  path.
- Nonzero or malformed `HRIF_IsBlendingDone` and `HRIF_IsMotionDone` responses
  stop execution, preserve the CPS code in diagnostics, put the interface in
  `ERROR`, and never advance the waypoint queue.
- Full hardware readiness requires six finite positive velocity and
  acceleration limits. Every generated MoveJ command is preflighted before the
  first waypoint at no more than `controller_limit_fraction=0.8` of the
  controller minima; jerk is read for diagnostics only.
- Focused tests cover 100 eight-contender admission rounds, ten overlapping ROS
  simulation action sequences, both completion APIs, invalid response payloads,
  missing limits, and the 80 percent profile matrix. Every rejected preflight
  produces zero waypoint calls.
- On a clean worktree at the passing commit, the package builds and its complete
  test suite passes under ROS 2 Humble. `git diff --check` and
  `scripts/check_agent_contract.sh` pass. Evidence records the exact commit and
  dirty count zero.
- Real-cell qualification remains `not_evaluated`; a later authorized campaign
  uses ten dry no-vacuum cycles and requires 10/10 per named segment and 10/10
  cycles, zero CPS errors/timeouts, and final TCP error at most 5 mm.

## Subtasks

| ID | Owner agent/model | Depends on | Base revision | Scope | Acceptance | Required tests | Commit evidence |
|---|---|---|---|---|---|---|---|
| HW-1 | `codex/gpt-5` | none | `558731137bf3f39a7d69bdae0f0cb42e7f01a7a8` | `elfin_trajectory_executor` goal admission, CPS polling, and controller-limit preflight | All offline criteria above; real motion `not_evaluated` | Focused pytest, ROS action integration, package colcon build/test, contract checks | Passing commit and `docs/status/evidence/hardware_executor_safety/` |

## Risks

- The primary worktree contains unrelated changes. Stage only the executor
  safety paths and new task/evidence records; verify the passing commit from a
  clean satellite worktree.
- Sequential `HRIF_WayPoint` stutter and a future buffered `ServoEsJ` backend
  are not fixed or certified by this wave.
- A controller without readable velocity or acceleration limits will now
  reject hardware motion by design.

## Pointers

- `docs/agents/reviews/2026-09-16_1227_dirty-tree-defect-audit.md`
- `docs/agents/discuss/2026-09-16_1228_dirty-tree-defect-audit.md`
- `docs/agents/reviews/2026-09-15_2158_elfin-real-trajectory-control-review.md`
- `.cursor/rules/debug-evidence.mdc`
