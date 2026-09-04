# 2026-09-04 — PF-A1 privileged input audit

- role: test
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-A1
- revision: a001be7 plus this PF-A1/A2 commit

## Summary

Static/data-flow audit of Humble online pickup nodes against `a001be7`.
Detector, segmenter, point filter, waypoint, and placement do not import
`GetCurrentBox` or parse spawned pose/size. `/luggage/current_box` is epoch
id/generation only; hardware may omit it (generation 0) or publish the same
JSON without geometry. Eval drivers remain the only GetCurrentBox clients.
Vacuum still consumes spawned pose/size for sim attach; that is classified
sim-backend and does not feed height/planning geometry.

## Commands

- `PYTHONPATH=src/luggage_perception python3 -m pytest -q src/luggage_perception/test/eval/test_pf_a1_static_audit.py`
- `python3 scripts/pf_a1_privileged_input_audit.py --out docs/status/evidence/platform_free_height/pf-a1_privileged_inputs/inventory.json`

## Evidence

- `docs/status/evidence/platform_free_height/pf-a1_privileged_inputs/`

## Result

- pass: no unresolved privileged geometry on the online height path; task-state has a credible hardware provider.
- residual (not a PF-R7 height blocker): vacuum_controller uses current_box pose/size/mass; hardware attach must use pressure/GPIO, not spawned truth.

## Pointers

- `docs/plans/platform_free_height_remediation.md`
- `src/luggage_perception/luggage_perception/eval/pf_a1_static_audit.py`
