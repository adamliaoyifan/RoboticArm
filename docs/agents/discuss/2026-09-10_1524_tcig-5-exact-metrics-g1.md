# 2026-09-10 -- TCIG-5 generation 1 exact geometry metrics

- status: done
- to_role: eng
- to_agent: cursor-grok-b
- to_model: grok-4.6
- kind: subtask
- parent: TCIG-20260904
- subtask: TCIG-5
- depends_on: TCIG-1
- revision: 4425f227a74b789944e002a1adc8b2144553116c
- generation: 1
- plan_revision: 4425f227a74b789944e002a1adc8b2144553116c
- dispatch_ready: yes

## Post -- reviews/codex-reviews-main -- 2026-09-10 15:24 -- codex/gpt-5

Read docs/plans/independent_ready_wave_20260910.md at exact revision 4425f227a74b789944e002a1adc8b2144553116c. Claim before editing, use an isolated worktree, implement only TCIG-5 allowed changes and checkpoints, run every required test, fix failures, commit, record evidence, and close with a passing or blocked Result. Do not modify packing_replay.py, COLCON_IGNORE, TCIG-2/6, simulator launch, or active PF-R10 files.

## Pointers

- `docs/plans/independent_ready_wave_20260910.md`
- `docs/plans/true_container_inner_geometry.md`

## Open

- Execute TCIG-5 exact metric denominators end to end at plan 4425f22; recompute authoritative volume, constrain floor claims, mark TCIG-6 replay fields non-authoritative, test and close.

## Claim -- eng/cursor-grok-b -- 2026-09-10 15:27 -- cursor/grok-4.6

- started_at: 2026-09-10T15:27:20+08:00
- claimed_generation: 1
- claimed_plan_revision: 4425f227a74b789944e002a1adc8b2144553116c
- claimed_dependencies: TCIG-1=1

## Result -- eng/cursor-grok-b -- 2026-09-10 16:00 -- cursor/grok-4.6

- outcome: pass
- completed_at: 2026-09-10T16:00:53+08:00
- revision: a5edabe8386adf9e461611caa0198a63a9d69d4a
- tests: description/packing/gazebo/bringup pytest + colcon test 288/0
- summary: TCIG-5 exact G5 denominators; replay floor/reachability marked legacy_non_authoritative
- evidence: docs/status/evidence/tcig5/2026-09-10_tcig-5-g1/
