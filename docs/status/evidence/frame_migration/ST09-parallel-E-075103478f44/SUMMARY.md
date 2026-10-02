# ST09-E Generation 3 Evidence

- base_revision: `075103478f442383457d4335766c881a87e82355`
- task: `FRAME-MIGRATION-20260929 / ST09-E / generation 3 lineage repair`
- evidence_status: pass for ST09-E offline tooling acceptance

## Reports

- `sample_manifest.json`: all required `offline`, `graph`, and `rehearsal` samples are content-validated in the synthetic fixture package.
- `schema_validation_report.json`: fixed conflict codes covered: `UNKNOWN_SCHEMA`, `DATUM_CONFLICT`, `IDENTITY_MISMATCH`, `MIXED_SOURCE_CONFLICT`, `OVERLAY_HASH_MISMATCH`.
- `removal_sensitivity_report.json`: generated from the acceptance manifest; every required stream/sample removal expects `not_evaluated` and exit `2`.
- `trajectory_replay_report.json`: valid fixed trajectory passes; truncated, missing positions, missing velocities, missing accelerations, wrong joint order, and non-monotonic time all fail.
- `fault_replay_report.json`: frozen planning, execution, attachment, scene, and map faults reproduce reason and zero unexpected effect deltas.
- `two_box_join_report.json` and `three_box_join_report.json`: strict pick/place/release/integration/map_publish chain and exact revision handoff counts pass.

## Commands

- `python3 -m pytest tests/st09_parallel/E -q`: pass, 11 tests.
- `python3 -m pytest tests/st09_parallel/C tests/st09_parallel/E -q`: pass, 30 tests.
- `python3 -m pytest tests/st09_parallel/E/test_evidence_runner.py::test_replay_shell_entrypoint_refuses_live_domain -q`: pass.
- `bash scripts/check_agent_contract.sh`: pass.
- `python3 scripts/check_frame_contracts.py --mode migration --output /tmp/st09-e2-frame-contracts-log`: exit 1 with existing pending frame-contract migration state; recorded under `frame_contracts_migration/` and not counted as ST09-E2 success.

## Lineage

This evidence was refreshed in a clean worktree rooted at `075103478f442383457d4335766c881a87e82355`. The replay shell entrypoint `scripts/frame_migration_st09_replay.sh` is retained from that base and covered by the focused live-domain refusal test.

## Residual

The frame-contract checker still reports `existing_base_helper` and pending interfaces. This is the pre-existing migration proof state called out by the dispatch and is recorded only as residual status.
