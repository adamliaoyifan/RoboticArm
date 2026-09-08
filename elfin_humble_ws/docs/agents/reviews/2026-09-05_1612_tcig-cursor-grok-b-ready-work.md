# 2026-09-05 16:12 -- TCIG work for cursor-grok-b

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done

## Summary

The distinct `eng/cursor-grok-b/grok-4.6` session removes the resource
conflict that previously kept Cursor-owned TCIG work behind the original
PF-R7 test worker. TCIG-1 has a passing exact Result at `7af4022`, so TCIG-4,
TCIG-5, and TCIG-7 satisfy their formal dependencies. TCIG-4 was selected as
the first assignment and dispatched after explicit user authorization.

## Decision

- TCIG-4 is ready and preferred first. Its exclusive corridor, waypoint, and
  audit files are clean, and the TCIG-1 swept-box and payload-erosion APIs it
  consumes are present.
- TCIG-5 is independently ready, but should follow TCIG-4 because one concrete
  Cursor session must own only one runnable row at a time.
- TCIG-7 is technically dependency-ready but should wait until TCIG-4/5 because
  it has a broader message/runtime interface and is an external dependency of
  SIM-R1 exploration work.
- TCIG-6 remains blocked by TCIG-2, TCIG-4, TCIG-5, and TCIG-7.
- TCIG-2 and TCIG-3 retain their approved Codex ownership; TCIG-2 is ready and
  TCIG-3 waits for TCIG-2.

## Dispatch Conditions

- Route to `to_agent=cursor-grok-b`, `to_model=grok-4.6`, role `eng`.
- Use the approved plan revision `bd942eba3120cf521010c5ba628b2489f5e31546`.
- Create an isolated worktree from the exact dispatch base. The primary
  workspace is dirty and shared by active PF/MPF agents.
- Keep coordination in the primary workspace mailbox and preserve TCIG-4's
  exclusive file list and Gate G4 unchanged.

## Dispatch

- Thread: `docs/agents/discuss/2026-09-05_1619_tcig-4-insertion-corridor.md`
- Mailbox: `Q-20260905-6`
- Owner: `eng/cursor-grok-b/grok-4.6/cursor`
- Base revision: `a3dba5e`
- Plan revision: `bd942eba3120cf521010c5ba628b2489f5e31546`
- Isolation: dedicated worktree required; PF-R7 remains with the original
  `test/cursor/grok-4.6` worker.

## Pointers

- `docs/plans/true_container_inner_geometry.md`
- `docs/agents/discuss/2026-09-04_1708_tcig-1-geometry-kernel.md`
- `docs/agents/discuss/2026-09-05_1603_cursor-grok-b-session-identity.md`
- `docs/agents/RUNTIME.md`
