# 2026-09-11 -- Todo 5 B/C completion

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Completed the cargo occupancy-map commit loop and ROS-free placement solver
contract. Cargo commits are idempotent and revisioned once per mutation;
occupied volume, detached ledger snapshots, surface height, world-to-container
pose conversion, and generated ROS service fields are covered. Placement now
applies hull, aperture, overlap, and corridor gates before ranking/retention,
with stable `BIN_FULL` diagnostics and fixed JSON acceptance cases.

## Pointers

- Commit `507eaa041005c0e441c592730543db113ad54b99`
- `docs/status/evidence/todo5_bc_20260911/summary.md`
- `src/luggage_perception/test/test_cargo_map_commit_contract.py`
- `src/luggage_packing/test/fixtures/compute_placement_cases.json`
- `src/luggage_packing/test/test_compute_placement_fixture.py`
