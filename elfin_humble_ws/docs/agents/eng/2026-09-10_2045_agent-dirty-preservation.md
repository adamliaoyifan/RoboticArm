# 2026-09-10 -- Agent dirty-work preservation

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Audited every registered Git worktree before the switch to `ros2_humble` as
the development baseline. Preserved meaningful uncommitted work on a separate
WIP branch without modifying or stashing any agent worktree. Clean agent
worktrees and content already integrated by `e25c7c9` were not duplicated as
patches.

The snapshot retains primary PF-R10 and pendant-replay implementation, tests,
documents, and derived evidence; LRF-P1 residual/locked-pose work; and TCIG-7
CLI executable-mode corrections. Runtime leases and the 10.6 GB raw MCAP were
excluded from Git. The MCAP identity and cleanup warning are recorded in the
archive README.

All three patches pass reverse-apply checks in their source worktrees. The
focused primary PF-R10/pendant suite passed 110 tests with one skip, and the
LRF-P1 OOD suite passed 9 tests. No simulator or robot was started.

## Pointers

- `docs/status/evidence/agent_dirty_archive/2026-09-10/README.md`
- `docs/status/evidence/agent_dirty_archive/2026-09-10/patches/primary-master.patch`
- `docs/status/evidence/agent_dirty_archive/2026-09-10/patches/lrf-p1.patch`
- `docs/status/evidence/agent_dirty_archive/2026-09-10/patches/tcig7-post-close.patch`
