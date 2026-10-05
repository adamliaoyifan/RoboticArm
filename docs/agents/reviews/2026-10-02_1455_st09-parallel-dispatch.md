# 2026-10-02 -- ST09 parallel dispatch preparation

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: open

## Summary

Accepted C0 commit `ba0505dc0d5c180310ee778a5888771f5a8d9cde` as the implementation base and froze the convergence and parallel plans at `7a31fc666d06358be43dc190abd1e3817b27027a`. Created decision-complete ST09-A through ST09-E and ST09-Q task threads with exact owners, bases, acceptance criteria, file boundaries and evidence requirements. They remain `dispatch_ready: no` until the still-claimed generation-5 owner acknowledges the recorded stop request; this prevents concurrent ownership of the same ST09 lineage.

## Acceptance

- Generation 5 owner records that work, processes and simulation have stopped and its checkpoint/evidence are preserved.
- Reviews supersedes the generation-5 thread without changing its historical requirement.
- Reviews flips the prepared task threads to `dispatch_ready: yes`; each exact owner uses `agent_start.sh` before editing.
- A, B, C and D may start in the first wave; E and Q enter as concurrency becomes available. ST09-I and ST09-R are created only at their declared dependency boundary.

## Subtasks

| ID | Owner agent/model | Depends on | Base revision | Scope | Acceptance | Required tests | Commit evidence |
|---|---|---|---|---|---|---|---|
| ST09-A | `codex-placement/gpt-6` | C0 satisfied | `ba0505d` | Placement geometry and ordering | Parallel plan ST09-A | `tests/st09_parallel/A` plus saved replay | Clean commit and A evidence |
| ST09-B | `claude-route/glm-5.3` | C0 satisfied | `ba0505d` | Graph and MoveIt route | Parallel plan ST09-B | `tests/st09_parallel/B` plus mock MoveIt | Clean commit and B evidence |
| ST09-C | `codex-certifier/gpt-6` | C0 satisfied | `ba0505d` | Certification and stop policy | Parallel plan ST09-C | `tests/st09_parallel/C` plus mock controller | Clean commit and C evidence |
| ST09-D | `claude-map/glm-5.3` | C0 satisfied | `ba0505d` | Observation, map and attachment provider | Parallel plan ST09-D | `tests/st09_parallel/D` plus saved sensor inputs | Clean commit and D evidence |
| ST09-E | `codex-evidence/gpt-6` | C0 satisfied | `ba0505d` | Runner, dumps and replay | Parallel plan ST09-E | `tests/st09_parallel/E` | Clean commit and E evidence |
| ST09-Q | `claude-qp/glm-5.3` | C0 assets | `555dbc5` | Isolated QP diagnostics | Parallel plan ST09-Q core | Discovered QP suite plus diagnostics | Clean experiment commit and Q evidence |

## Risks

- Generation 5 remains formally claimed until its concrete owner acknowledges the stop request.
- Cursor and Claude routes are file-only unless their sessions register a supported dispatcher; mailbox creation does not prove they started.
- The primary workspace contains unrelated dirty files, so every owner must use its isolated worktree and exact base.

## Pointers

- `docs/plans/frame_migration_st09_parallel_execution.md`
- `docs/plans/frame_migration_st09_planner_convergence.md`
- `docs/agents/discuss/2026-10-01_1104_frame-stage-09-g5-cursor-handoff.md`
- `docs/agents/discuss/2026-10-02_1449_st09-a-placement.md`
- `docs/agents/discuss/2026-10-02_1449_st09-b-route.md`
- `docs/agents/discuss/2026-10-02_1457_st09-b-route-claude-g2.md`
- `docs/agents/discuss/2026-10-02_1449_st09-c-certifier.md`
- `docs/agents/discuss/2026-10-02_1449_st09-d-map-attachment.md`
- `docs/agents/discuss/2026-10-02_1449_st09-e-evidence.md`
- `docs/agents/discuss/2026-10-02_1449_st09-q-qp-experiment.md`
