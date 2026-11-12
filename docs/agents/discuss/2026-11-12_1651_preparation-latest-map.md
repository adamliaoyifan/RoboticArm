# 2026-11-12 -- preparation-latest-map

- status: done
- to_role: eng
- to_agent: codex
- to_model: gpt-6
- kind: subtask
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PREPARATION-LATEST-MAP
- depends_on: ST-4-PRODUCTION-P4-PROFILING
- revision: ca1f8f65e3011ad6f745356eda8d2f1dde7572b2
- generation: 1
- plan_revision: 61d8fb614a676494d8b74393f642d7b279714cd4
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-11-12 16:51 -- codex/gpt-6

Implement authorized latest O map full recertification of unchanged original candidate/trajectory, latest identity/certificate issuance, unchanged support proof, bounded stable-source retry. Keep existing age1000ms and fail closed. No default-client or full-model baseline claim.

## Pointers

- `docs/plans/PRODUCTION_PREPARATION_LATEST_MAP_20261112.md`

## Open

- Implement authorized latest O map full recertification of unchanged original candidate/trajectory, latest identity/certificate issuance, unchanged support proof, bounded stable-source retry. Keep existing age1000ms and fail closed. No default-client or full-model baseline claim.

## Claim -- eng/codex -- 2026-11-12 16:55 -- codex/gpt-6

- started_at: 2026-11-12T16:55:40+08:00
- claimed_generation: 1
- claimed_plan_revision: 61d8fb614a676494d8b74393f642d7b279714cd4
- claimed_dependencies: ST-4-PRODUCTION-P4-PROFILING=1

## Result -- eng/codex -- 2026-11-12 17:14 -- codex/gpt-6

- outcome: pass
- completed_at: 2026-11-12T17:14:46+08:00
- revision: f11ec48b219134433c6d2884c42b66cd3c82c6c2
- tests: 401 deterministic cases plus final73 compatibility cases pass;41 installed checks;13-package production build pass; external native opt-in excluded
- summary: Latest O full-chain recertification preserves six timed legs and physical geometry, binds new identity/source at issuance; age1000ms and strict G retained. Native/full-model baseline remains open.
- evidence: docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PREPARATION-LATEST-MAP/run_01/result.json
