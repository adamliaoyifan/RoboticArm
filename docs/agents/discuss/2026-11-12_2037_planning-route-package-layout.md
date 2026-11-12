# 2026-11-12 -- Planning route package layout

- status: done
- to_role: eng
- to_agent: codex
- to_model: gpt-6
- kind: subtask
- parent: PLANNING-ROUTE-LAYOUT-20261112
- subtask: PACKAGE-LAYOUT
- depends_on: none
- revision: dfbcc39fa8c5e58defc46d7fdb33f3f30bb38e35
- generation: 1
- plan_revision: bd46a9f03f6aaba0e5cc746134856ee8611e175b
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-11-12 20:37 -- codex/gpt-6

Mechanically group72 route modules, update active callers and install paths, preserve concurrent dirty logic, verify frozen AST and behavioral matrix under planbd46a9f0.

## Pointers

- `docs/plans/PLANNING_ROUTE_LAYOUT_20261112.md`

## Open

- Mechanically group72 route modules, update active callers and install paths, preserve concurrent dirty logic, verify frozen AST and behavioral matrix under planbd46a9f0.

## Claim -- eng/codex -- 2026-11-12 20:37 -- codex/gpt-6

- started_at: 2026-11-12T20:37:45+08:00
- claimed_generation: 1
- claimed_plan_revision: bd46a9f03f6aaba0e5cc746134856ee8611e175b
- claimed_dependencies: none

## Result -- eng/codex -- 2026-11-12 21:00 -- codex/gpt-6

- outcome: pass
- completed_at: 2026-11-12T21:00:26+08:00
- revision: e9e7cb9e6c7f48040bd79a25140d67f89c526edb
- tests: Baseline536pass/11skip unchanged;extended544pass/11skip;56native MoveIt pass;188installed checks pass;13packages stable-source build;frame16pass/60subtests;no Gazebo/hardware success claimed.
- summary: Grouped72 planning modules into four packages, migrated active callers and frame inventory, added route index and stale-install pruning; preserved concurrent dirty/untracked implementation separately.
- evidence: docs/status/evidence/planning_route_layout/20261112/summary.json
