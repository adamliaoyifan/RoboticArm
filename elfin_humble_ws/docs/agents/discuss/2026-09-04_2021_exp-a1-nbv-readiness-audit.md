# 2026-09-04 -- EXP-A1 NBV baseline readiness audit

- status: done
- to_role: test
- to_agent: codex
- to_model: gpt-5
- kind: subtask
- parent: SIM-R1-20260904
- subtask: EXP-A1
- depends_on: none
- revision: 1e8ea460aa7abdce6c7c66dc4020317e777d8747

## Post -- reviews/codex -- 2026-09-04 20:21 -- codex/gpt-5

Audit the existing ROS-free NBV baseline at source revision 408f6d5dd9aeb8536377f0f14d06155be7a904d5. Run every command and mandatory probe in the dispatch note, preserve machine-readable evidence, classify unknown hard-gate evidence as BLOCKED, and close this thread yourself. This task must not repair findings or touch active SIM-R1 production files.

## Pointers

- `docs/agents/reviews/2026-09-04_2022_exp-a1-nbv-readiness-dispatch.md`
- `docs/architecture/production_orchestration.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`

## Open

- Execute the immutable-snapshot EXP-A1 audit exactly as specified; do not modify production code or start ROS/simulation, and report audit_outcome separately from nbv_readiness.
## Claim -- test/codex -- 2026-09-04 20:25 -- codex/gpt-5

- started_at: 2026-09-04T20:25:25+08:00
## Result -- test/codex -- 2026-09-04 20:36 -- codex/gpt-5

- outcome: pass
- completed_at: 2026-09-04T20:36:21+08:00
- revision: 408f6d5dd9aeb8536377f0f14d06155be7a904d5
- tests: focused pytest retained collection failure: missing ament_index_python; evidence-local exp_a1_probe.py completed strict JSON: 3 PASS, 2 FAIL, 2 BLOCKED; privileged dependency scan 0 hits; git diff --check pass; scripts/check_agent_contract.sh pass
- summary: EXP-A1 audit complete: audit_outcome=pass, nbv_readiness=blocked by CargoNBV non-candidate-specific coverage, missing duplicate/quaternion/dimension gates, missing atlas hash/revision/stamp correlation, and missing ExplorationPolicy adapter.
- evidence: docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/audit_report.md
- evidence: docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/probe_results.json

