# 2026-11-12 -- gazebo-operation-integration

- status: open
- to_role: eng
- to_agent: codex
- to_model: gpt-6
- kind: subtask
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- depends_on: ST-4-PREPARATION-LATEST-MAP
- revision: 5086d84f2873db9b36ec94d8e67efeb83481c74f
- generation: 1
- plan_revision: 983e80e1
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-11-12 17:35 -- codex/gpt-6

Wire explicit observed O full-model Gazebo operation pipeline and client; rehearse startup, execute diagnostic cycle with complete failure evidence; keep baseline open until actual success.

## Pointers

- `docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md`

## Open

- Wire explicit observed O full-model Gazebo operation pipeline and client; rehearse startup, execute diagnostic cycle with complete failure evidence; keep baseline open until actual success.

## Claim -- eng/codex -- 2026-11-12 17:35 -- codex/gpt-6

- started_at: 2026-11-12T17:35:41+08:00
- claimed_generation: 1
- claimed_plan_revision: 983e80e1
- claimed_dependencies: ST-4-PREPARATION-LATEST-MAP=1

## Post -- eng/codex -- 2026-11-12 18:07 -- codex/gpt-6

Three unscored startup attempts preserved. Review completed: (1) missing launch
argument declarations; (2) client read identity before launch materialized it;
(3) controller activation transient sampled once, plus actual Best Effort /
Reliable domain incompatibility. No product controller goal issued. Next bounded
startup attempt uses measured controller readiness and actual DDS delivery test,
with same full-model/O policy. Baseline remains open. Source/archive/CDR/log
pointers: ST-4-GAZEBO-OPERATION/rehearsal_01..03 and offline_01/qos_transport.log.

## Post -- eng/codex -- 2026-11-12 18:18 -- codex/gpt-6

Startup rehearsal_04 passed, 3,525 sensor/state messages, both controllers active,
all required endpoints present, clean owned teardown. cycle_01 executed both
MoveGroup observation motions and generated sensor O, then stopped before prepare:
detection at50.556s lacked measured height, exact domain invalid; first subsequent
valid domain51.216s. Evidence and source archive preserved, zero residual processes.
Diagnostic client now waits within the existing30s observation budget for complete
measured geometry and an exact header/domain join; no admission relaxation.
11 behavioural readiness cases pass. Latest-only prepare/context association race
remains a source concern to measure at its actual boundary. User confirms online
IK diagnosis first, but final success baseline requires requalified full-model atlas.

## Post -- eng/codex -- 2026-11-12 18:25 -- codex/gpt-6

cycle_02 passed domain admission and frozen retention, then GetSnapshot refused
REVISION_UNAVAILABLE. Client erroneously used GetValidity's transition-time
revision1 while actual O receipts were revision5. Fix binds a real current
snapshot lease (same1000ms/allow_stale=false), retains it through preparation and
releases in finally. 13 client/readiness behavioural cases pass. No movement of
product box, no collision-policy change. Evidence: cycle_02/failure_summary.json.
Separately, observed O INTEGRATED receipts with zero free/hit keys advance the
global stamp; they cannot establish regional ULD freshness. Keep this limitation
visible for the map-age design review instead of claiming fresh ULD coverage.
