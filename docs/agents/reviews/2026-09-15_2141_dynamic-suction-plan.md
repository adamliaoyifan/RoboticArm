# 2026-09-15 -- Dynamic surface and suction-patch plan

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Produced a decision-complete, software-only plan to remove fixed pickup-position influence from top-surface estimation and to authorize attach only from a locally flat, single-plane suction-panel footprint. The user approved dispatch of the sequential implementation and integration work to `claude/glm-5.3`.

## Acceptance

- Identical sensor input is invariant to `scene_tf` pickup-source/workspace values when hardware crop gates are off.
- Dynamic-position top geometry meets the specified 324-case synthetic and 45-case recorded-data accuracy matrices.
- Every candidate crossing a >=6 mm two-plane step is rejected, and every >=10 mm no-patch real fixture authorizes zero motion.
- Candidate-aware planning performs at most three attempts and never moves laterally before vacuum release and an >=80 mm recovery retreat.
- The supervised valid-patch real-cell subset achieves at least 9/10 DI0 seals and retained 0.35 m pick retreats with zero unsafe event.
- All failure cases preserve replayable bounded evidence at the first divergent boundary.

## Subtasks

| ID | Owner agent/model | Depends on | Base revision | Scope | Acceptance | Required tests | Commit evidence |
|---|---|---|---|---|---|---|---|
| ST-1 | `claude/glm-5.3` | none | `8f4d0a0dc8c449197caf146bd3a6ec6cf2fba627` | Dynamic instance-derived top geometry | Plan gates A0-A5 | ROS-free, property, replay and launch tests | Clean passing commit and evidence pointer |
| ST-2 | `claude/glm-5.3` | ST-1 | same | Local suctionability map, discontinuity rejection and message contract | Plan gates B0-B7 | Synthetic patch matrix, message/build and replay tests | Clean passing commit and evidence pointer |
| ST-3 | `claude/glm-5.3` | ST-2 | same | Candidate waypoint selection and bounded seal-failure recovery | Plan gates C0-C7 | Planning, vacuum state-machine and fault-injection tests | Clean passing commit and evidence pointer |
| INTEGRATION | `claude/glm-5.3` | ST-1,ST-2,ST-3 | same | Clean build, offline and supervised real-cell acceptance | Plan gates D0-D8 and E evidence gate | Colcon, acceptance runner, no-motion and real-cell matrix | Tested integration commit and complete manifests |

## Risks

- Current URDF gives only a coarse 0.18 x 0.18 m panel envelope; the plan requires a versioned explicit hardware config and fails closed if it is absent.
- Conservative visual flatness thresholds can reduce availability; relaxing them requires a higher plan generation and repeated acceptance.
- Bbox-fill is less selective than a pixel instance mask, so the plan includes depth-component isolation and multi-plane connected scoring.
- Retry motion remains disabled until all state-machine and fault-injection gates pass.

## Pointers

- `docs/plans/dynamic_top_surface_and_suction_patch.md`
- `8f4d0a0dc8c449197caf146bd3a6ec6cf2fba627`
