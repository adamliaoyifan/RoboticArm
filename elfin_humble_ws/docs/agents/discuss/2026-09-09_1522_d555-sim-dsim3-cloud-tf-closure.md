# 2026-09-09 -- DSIM-3 camera-cloud and stamped-TF closure

- status: open
- to_role: eng
- to_agent: cursor
- to_model: grok-4.6
- kind: subtask
- parent: D555-SIM-DEPTH-20260909
- subtask: DSIM-3
- depends_on: DSIM-1
- revision: 02e8afdab9eda85e8dc50837ee58b911c4d8251a
- generation: 1
- plan_revision: 02e8afdab9eda85e8dc50837ee58b911c4d8251a
- dispatch_ready: no

## Post -- reviews/codex -- 2026-09-09 15:22 -- codex/gpt-5

Dispatch is held on DSIM-1. Do not claim early. Read docs/plans/d555_sim_depth_pipeline_execution.md at plan revision 02e8afdab9eda85e8dc50837ee58b911c4d8251a. Remove or migrate every active ROS 2/eval camera-cloud reference, preserve Livox and semantic output clouds, trace every world-XYZ path to aligned depth plus same-grid CameraInfo plus exact-stamp TF, and execute the full fail-closed matrix including a newer-TF-present/exact-TF-missing negative control. Own tests, evidence, commit, runtime graph audit, teardown, and closure.

## Pointers

- `docs/plans/d555_sim_depth_pipeline_execution.md`

## Open

- HOLD: after DSIM-1 passes, execute active-consumer cleanup and stamped-TF checkpoints DS3-C0 through DS3-C3.
