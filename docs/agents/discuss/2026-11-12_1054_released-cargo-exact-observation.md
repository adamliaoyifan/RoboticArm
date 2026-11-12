# 2026-11-12 -- released-cargo-exact-observation

- status: superseded
- to_role: eng
- to_agent: codex
- to_model: gpt-6
- kind: subtask
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-OBS-JOIN
- depends_on: none
- revision: 7b697e00
- generation: 1
- plan_revision: HEAD
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-11-12 10:54 -- codex/gpt-6

Implement ST-4-OBS-JOIN only under docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md: exact-acquisition RGB/depth/mask/calibration/TF, immutable source pixel/point partition, read-only ROS service, finite functional AND matrix, isolated bindings/install parity and case-boundary evidence. No physical identity, map substitution or controller effects. Implement test fix commit and close this bounded prerequisite; broad integration stays open.

## Open

- Implement ST-4-OBS-JOIN only under docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md: exact-acquisition RGB/depth/mask/calibration/TF, immutable source pixel/point partition, read-only ROS service, finite functional AND matrix, isolated bindings/install parity and case-boundary evidence. No physical identity, map substitution or controller effects. Implement test fix commit and close this bounded prerequisite; broad integration stays open.

## Claim -- eng/codex -- 2026-11-12 10:54 -- codex/gpt-6

- started_at: 2026-11-12T10:54:25+08:00
- claimed_generation: 1
- claimed_plan_revision: HEAD
- claimed_dependencies: none


## Reply -- eng/codex -- 2026-11-12 11:05 -- codex/gpt-6

Owner acknowledges stop of this generation: dispatch accidentally stored the symbolic plan revision HEAD rather than immutable18f35ccb. No scope change is intended; create generation2 with that exact plan commit. Preserve current source WIP and run_01 evidence.33 pure observation cases passed;27 ROS cases failed in fixture construction before source ingress because CameraInfo float fields received mixed integer literals. No product or physical action claim; no runtime map consumption change.
## StopAck -- eng/codex -- 2026-11-12 11:05 -- codex/gpt-6

- acknowledged_at: 2026-11-12T11:05:35+08:00
- reason: Superseded by generation2 exact immutable plan18f35ccb; same owner stops stale generation, WIP and run_01 retained.
