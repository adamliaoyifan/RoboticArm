# 2026-11-12 -- released-cargo-exact-observation-g2

- status: done
- to_role: eng
- to_agent: codex
- to_model: gpt-6
- kind: subtask
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-OBS-JOIN
- depends_on: none
- revision: 7b697e00
- generation: 2
- plan_revision: 18f35ccb
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-11-12 11:04 -- codex/gpt-6

Administrative replacement of generation1 symbolic plan reference. Same immutable scope under plan18f35ccb docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md. Implement exact-source observation partition and read-only ROS service; full finite behavioral matrix, isolated install/IDL parity and failure captures. No physical identity or O replacement enabled. Preserve run_01 fixture failure and current WIP; implement test fix commit close bounded OBS-JOIN only.

## Open

- Administrative replacement of generation1 symbolic plan reference. Same immutable scope under plan18f35ccb docs/plans/RELEASED_CARGO_ASSOCIATION_20261112.md. Implement exact-source observation partition and read-only ROS service; full finite behavioral matrix, isolated install/IDL parity and failure captures. No physical identity or O replacement enabled. Preserve run_01 fixture failure and current WIP; implement test fix commit close bounded OBS-JOIN only.

## Claim -- eng/codex -- 2026-11-12 11:04 -- codex/gpt-6

- started_at: 2026-11-12T11:04:56+08:00
- claimed_generation: 2
- claimed_plan_revision: 18f35ccb
- claimed_dependencies: none
## Result -- eng/codex -- 2026-11-12 11:14 -- codex/gpt-6

- outcome: pass
- completed_at: 2026-11-12T11:14:30+08:00
- revision: cf3192f6acfddd90f8e6716a3e9b3a38e34f8991
- tests: 62 observation cases pass;62 related regressions pass;5 installed ROS cases pass;2 isolated packages build; source/install parity. Additional global staged inventory has unrelated F14 deleted-asset pointer; preserved and not claimed clean.
- summary: Closed bounded OBS-JOIN: immutable exact RGBD/mask/calibration/TF point/pixel partition and read-only service; no cross-release physical identity, O replacement or whole-cycle completion. Conditional matching and O provenance remain next prerequisites.
- evidence: docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-OBS-JOIN/run_03/manifest.json
- evidence: docs/agents/eng/2026-11-12_1115_cargo-observation-partition.md
