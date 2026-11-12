# 2026-11-12 -- Configured preparation watchdog and stage timing

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PRODUCTION-P4-PROFILING
- base_revision: c47e7d3430ba4ac25984219cadfc5f91bf708b24
- started_at: 2026-11-12T16:30:58+08:00
- completed_at: 2026-11-12T16:43:47.444853+08:00

## Summary

Root/core preparation share an explicit watchdog instead of duplicated10s caps.
Existing traces carry measured stages, outcomes and correlation IDs. Full-model
simulated success, default complete-operation production entry remain open.

## Requirement

Plan6310a332 generation1: baseline and stage timings before production SLA. Keep
one request deadline, current search caps, certification reserve and motion gates.
300s default is an overrideable diagnostic watchdog, not a throughput promise.

## Changed

- preparation_profile provides shared validated PreparationBudget and StageTimings.
- Root times admission, frozen evidence, map, scene, candidates, preparation and
  issuance; core times proposal, quality, bound candidate, atlas, revalidation;
  native planner times local paths, connections, timing and certification.
- A returned refusal is completed, not success; normal verdicts remain authoritative.
  Nested stages must not be summed. Exceptions record error at attempted boundary.
- Return total includes issuance. Internal response evidence intentionally precedes
  issuance to preserve the existing capture gate; final RPC is captured externally.
- Prepare launch exposes watchdog; installed checker includes new module. Updated
  closest normative preparation boundary documentation; no ROS schema version added.

## Verification

233 distinct deterministic cases pass (232 closed matrix plus one added real
proposal/quality/binding coverage;257 executions), zero skipped selected tests.
One existing native transport opt-in explicitly deselected, not claimed covered.
Real root callback accepts30s under cap60 and refuses under cap20; nonfinite/zero
budgets refuse with inactive plan, valid trace and released lock. Expired core
never calls planner. Identical frozen input yields exactly equal plans before
and after timing instrumentation. Native branch recovery/timeout/refusal retained.

Initial fixture compared separately timestamped scenes; failure log/JUnit/input
retained, corrected to use the same immutable candidate. Rebuild13packages6.57s,
unchanged input hashes,39installed checks pass. Installed launch exposes watchdog.
No external owner/native endpoint/controller/Gazebo process or robot execution.

## Result

- pass at81dc2c2af7b7019acf831ed48162b9ce8adc7b28 for this bounded prerequisite.
- Source review identifies potential conflict: default O max_age1000ms and final
  exact-source identity versus multi-second planning. No live failure-rate claim.
  User now authorizes latest-map full recertification of retained trajectories;
  implement as a separate claim/plan. Existing age gate remains until measured.

## Pointers

- `docs/plans/PRODUCTION_CONVERGENCE_P4_PROFILING_20261112.md`
- `docs/agents/discuss/2026-11-12_1630_production-p4-profiling.md`
- `docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PRODUCTION-P4-PROFILING/run_01/result.json`
