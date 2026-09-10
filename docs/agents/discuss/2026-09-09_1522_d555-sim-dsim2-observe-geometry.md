# 2026-09-09 -- DSIM-2 D555 pickup observe geometry

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.6
- kind: subtask
- parent: D555-SIM-DEPTH-20260909
- subtask: DSIM-2
- depends_on: DSIM-1
- revision: 02e8afdab9eda85e8dc50837ee58b911c4d8251a
- generation: 1
- plan_revision: 02e8afdab9eda85e8dc50837ee58b911c4d8251a
- dispatch_ready: no

## Post -- reviews/codex -- 2026-09-09 15:22 -- codex/gpt-5

Dispatch is held on DSIM-1. Do not claim early. Read docs/plans/d555_sim_depth_pipeline_execution.md at plan revision 02e8afdab9eda85e8dc50837ee58b911c4d8251a. Measure the old approximately 0.24 m failing baseline before changing the simulation pose; then find a reachable, collision-free, settled pose satisfying optical-Z at least 0.30 m, valid-depth ratio at least 0.95, image margin at least 10 px, and support-point retention for every valid catalog model. Do not lower the 0.26 m near clip or change camera extrinsics/estimator bars. Own tests, evidence, commit, teardown, and closure.

## Pointers

- `docs/plans/d555_sim_depth_pipeline_execution.md`

## Open

- HOLD: after DSIM-1 passes, execute pickup_observe geometry checkpoints DS2-C0 through DS2-C2 for the full valid catalog.
