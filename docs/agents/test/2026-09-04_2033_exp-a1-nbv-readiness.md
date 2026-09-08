# 2026-09-04 20:33 -- EXP-A1 NBV baseline readiness audit

- role: test
- agent: codex
- model: gpt-5
- cli: codex
- status: done
- parent: SIM-R1-20260904
- subtask: EXP-A1
- revision: 408f6d5dd9aeb8536377f0f14d06155be7a904d5

## Summary

Completed the immutable-snapshot EXP-A1 audit without changing production
source or starting ROS/simulation. The audit result is complete, but NBV
baseline readiness is blocked by candidate-specific CargoNBV scoring gaps,
missing duplicate/quaternion/dimension gates, missing atlas hash/revision/stamp
correlation, and absence of a direct `ExplorationPolicy` adapter.

## Result

- audit_outcome: pass
- nbv_readiness: blocked
- focused_pytest: collection blocked by missing `ament_index_python` in the
  audit environment; logs retained.
- mandatory_probe_summary: 3 PASS, 2 FAIL, 2 BLOCKED.
- privileged_dependency_scan: 0 ROS/Gazebo/GT/spawner matches in audited
  planning modules.
- hygiene: `git diff --check` passed; `scripts/check_agent_contract.sh` passed.

## Pointers

- `docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/audit_report.md`
- `docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/probe_results.json`
- `docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/focused_pytest.log`
- `docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/focused_pytest_rosenv.log`
- `docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/privileged_dependency_scan.txt`
- `docs/status/evidence/sim_r1/2026-09-04_202530_exp-a1/geometry_contract_scan.txt`
