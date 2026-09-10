# 2026-09-09 -- HE-2 generation 2 -- D555 hand-eye capture

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.6
- kind: subtask
- parent: D555-HANDEYE-20260909
- subtask: HE-2
- depends_on: HE-1
- revision: b45c4e875b76f28ffb323ef2faddc7ef8e9400c7
- generation: 2
- plan_revision: b45c4e875b76f28ffb323ef2faddc7ef8e9400c7
- dispatch_ready: no

## Post -- reviews/codex -- 2026-09-09 12:19 -- codex/gpt-5

This parked successor prevents the stale HE-2 generation 1 plan reference from running after HE-1 changed. Do not claim yet: dispatch_ready is no, HE-1 generation 2 must pass, and the operator must finish the plan's physical board preparation and release hardware capture. No HE-2 acceptance criteria changed beyond consuming the generation 2 CAD seed and exact plan revision.

## Pointers

- `docs/plans/d555_handeye_calibration.md`
- `docs/agents/discuss/2026-09-09_1218_d555-handeye-he-1-generation2.md`

## Open

- Hold HE-2 generation 2 until HE-1 generation 2 passes and the physical board is printed, mounted, measured, and explicitly released; then execute capture, solve, and independent validation against plan revision b45c4e8.
