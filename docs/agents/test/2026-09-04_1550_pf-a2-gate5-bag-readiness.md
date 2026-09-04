# 2026-09-04 — PF-A2 Gate 5 bag readiness checker

- role: test
- agent: cursor
- model: grok-4.6
- cli: cursor
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-A2
- revision: a001be7 plus this PF-A1/A2 commit

## Summary

Defined the Gate 5 rosbag contract and a ROS-free manifest checker with
stable reason codes. Synthetic fixtures cover missing topic, wrong type,
empty stream, non-overlap, missing TF, reference leak, and accuracy-claim
refusal. A valid fixture is ready with `gate5_accuracy: not_claimed`. No
real bag was present; Gate 5 accuracy is not passed.

## Commands

- `PYTHONPATH=src/luggage_perception python3 -m pytest -q src/luggage_perception/test/eval/test_gate5_bag_readiness.py`
- `python3 scripts/gate5_bag_readiness.py --manifest src/luggage_perception/test/eval/fixtures/gate5/valid.json`
- `python3 scripts/gate5_bag_readiness.py --manifest .../valid.json --claim-accuracy` (exit 1)

## Evidence

- `docs/status/evidence/platform_free_height/pf-a2_gate5_readiness/`
- `src/luggage_perception/test/eval/fixtures/gate5/`

## Result

- pass: checker rejects invalid metadata/time/TF/reference cases deterministically.
- Gate 5 accuracy: not claimed (no real rosbag).

## Pointers

- `docs/plans/platform_free_height_gate5_bag_contract.md`
- `src/luggage_perception/luggage_perception/eval/gate5_bag_readiness.py`
