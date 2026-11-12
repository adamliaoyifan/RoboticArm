# 2026-11-12 -- Latest O map recertification of retained operation

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-PREPARATION-LATEST-MAP
- base_revision: ca1f8f65e3011ad6f745356eda8d2f1dde7572b2
- started_at: 2026-11-12T16:55:40+08:00
- completed_at: 2026-11-12T17:14:46.034980+08:00

## Summary

Authorized latest O publication now recertifies the exact selected original six
legs against a fresh full volume and baseline, without IK/search/retiming or
physical pose changes. Issuance retains the newly certified source. This bounded
unit does not complete default production migration or full-model simulation.

## Requirement

Plan61d8fb61 generation1: latest complete O certification under the original
absolute deadline, exact local support-proof reuse and known-free nominal volume,
unchanged frozen grasp and actual start admission, bounded stable-source retry.
Keep observer age1000ms and explicit G strict final gate.

## Changed

- packing placement_recheck: source-resolution complete support patch equality,
  positive boundary overlap, directed quarter turns, fresh known-free volume.
- planning preparation_refresh: narrow read-only capture/scene/source/observation
  ports, three phase rebuild, shared exact receipt predicate, map/snapshot rebind.
- prepare root/core: retain only the selected candidate, consume final identity,
  update issuance source, share frozen-observation/current-start checks. Legacy
  serialization still checks the original expected identity.
- Configurable preparation_refresh_attempts default2 (range1..8), launch exposure,
  timing/identity diagnostics, closest architecture and installed-module checks.
- Owner source review establishes observed_revision can advance while map_revision
  stays constant (observer.cpp changed_keys); this is admitted, freshness unchanged.

## Verification

- Frozen exact command matrix in closed-matrix/manifest.json:401 deterministic
  cases pass,68 new,zero selected skips,one external-native opt-in deselected.
- Final narrow compatibility command in legacy-identity-check/manifest.json:
  73 cases pass after preserving original G serialization admission.
- scripts/production/build.sh --evidence <root>/build-final:13 packages pass,
  6.53s,source before/after unchanged.
- scripts/production/run.sh /usr/bin/python3 scripts/production/check_install.py
  --output <root>/install-final.json:41 installed origin/hash checks pass.
- scripts/production/run.sh ros2 launch luggage_planning
  joint_operation_prepare.launch.py --show-args:watchdog and retry budget visible.
- Owned staged diff whitespace passes. Whole-workspace git diff --check fails
  on three unrelated historical thread EOFs; exact paths in workspace-whitespace.log.
- scripts/check_agent_contract.sh fails on164 preexisting documentation issues;
  every exact file/missing field is retained in agent-contract.log. Repository-wide
  contract compliance is not claimed.
- Initial three reason-case assertions failed; original inputs/log/JUnit preserved
  losslessly before correction. Unsafe volumes were already refused.

## Result

- pass at f11ec48b219134433c6d2884c42b66cd3c82c6c2 for offline preparation-finalization scope.
- Native transport/controller-sampled backend integration: not run in this unit.
- Full-model simulated/real whole-cycle baseline: not evaluated, remains open.
- Exact support proof is deliberately conservative: local height/state/confidence
  changes require a fresh placement request. Continuous updates may exhaust the
  explicit retry budget; no production success rate/latency improvement claimed.
- No controller, robot, Gazebo, native owner or map writer started.

## Pointers

- `docs/plans/PRODUCTION_PREPARATION_LATEST_MAP_20261112.md`
- `docs/agents/discuss/2026-11-12_1651_preparation-latest-map.md`
- `docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-PREPARATION-LATEST-MAP/run_01/result.json`
