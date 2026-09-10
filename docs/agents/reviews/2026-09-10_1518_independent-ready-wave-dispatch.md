# 2026-09-10 -- Independent ready wave dispatch

- role: reviews
- agent: codex-reviews-main
- model: gpt-5
- cli: codex
- status: done

## Summary

Approved four independent implementation tasks with no unfinished product
prerequisites. TCIG-5 is bounded metric migration for Cursor/Grok-4.6.
TCIG-2, TCIG-7, and SIM-R1-5 are cross-package semantic tasks for three
distinct Codex GPT-5.6 eng owners. All work is isolated from active PF-R10;
dependent integrations, learning research, and packing runtime acceptance stay
undispatched.

## Acceptance

- Each owner completes the exact checkpoints and commands in the approved
  execution overlay on an isolated worktree and returns one clean passing
  commit with evidence.
- TCIG-5 cannot claim TCIG-6-owned replay floor/reachability metrics.
- TCIG-2 exposes typed geometry identity/physical volume and tests its pure
  surface validator without implementing TCIG-3.
- TCIG-7 migrates or rejects legacy atlas identity explicitly and does not
  extend rospy exploration code.
- SIM-R1-5 emits no motion before explicit Start; if it needs Gazebo, it waits
  for the shared simulator lock and records zero residual processes.

## Consensus


- Codex agent: `codex-independent-ready-consensus`
- Thread: `docs/agents/discuss/2026-09-10_1518_independent-ready-wave-consensus.md`
- Result: reached

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| TCIG-5 | `cursor-grok-b/grok-4.6` | accepted TCIG-1 | Exact G5 metric helper and entry points | Exact denominators and identity; replay legacy fields non-authoritative | Description, packing, Gazebo, direct bringup tests; colcon current ROS 2 packages |
| TCIG-2 | `codex-tcig-map-eng/gpt-5.6-sol` | accepted TCIG-1 | Weighted cargo map, surface validator, typed stats | G2 volume conservation and fail-closed identity | Description/perception/mapper contract plus colcon |
| TCIG-7 | `codex-tcig-atlas-eng/gpt-5.6-sol` | accepted TCIG-1 | Hull-aware atlas and runtime identity | Active-mask/hash/migration/runtime G7 gates | Description/planning/messages plus colcon |
| SIM-R1-5 | `codex-sim-r1-eng/gpt-5.6-sol` | accepted SIM-R1-1 | ROS 2 bringup and explicit-Start orchestrator | R5 no-motion/request/fault/idempotence gates | SIM-R1 contracts, bringup tests/build, 60-second no-Start |

## Risks

- TCIG-4 is accepted off-branch but absent from this execution base; later
  integration must merge it explicitly.
- `cursor-grok-b` is file-only with a stale runtime heartbeat, so its runnable
  row is durable dispatch but not proof of immediate live execution.
- The shared primary worktree contains active PF-R10 edits; only the plan,
  consensus, and this note may enter the reproducible plan commit.

## Pointers

- `docs/plans/independent_ready_wave_20260910.md`
- `docs/agents/discuss/2026-09-10_1518_independent-ready-wave-consensus.md`
- `docs/plans/true_container_inner_geometry.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
