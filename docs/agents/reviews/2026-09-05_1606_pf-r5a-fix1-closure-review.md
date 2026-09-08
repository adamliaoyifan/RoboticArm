# 2026-09-05 16:06 -- PF-R5A-FIX1 closure review

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done
- parent: PFH-R5-CLOSURE-20260905
- subtask: PF-R5A-FIX1
- reviewed_revision: 192a6aa08f93febf8855970e5057fe3e49cb2f73
- plan_revision: 28370bc511d5ed564dc709a661af4f6bea2679eb

## Summary

FIX1 correctly repairs tier-separated caching, unknown-visual rejection, and
clear-before-validation ordering, with a clean three-file implementation
commit. It is not yet accepted because a reference failure still advances the
spawner RNG state before validation, contrary to the required zero-state-
mutation behavior and deterministic fail-closed semantics.

## Finding

1. `handle_spawn_next` calls `_sample_box`, `_entry_pose`, and the random visual
   choice before `_gt_size` validates the prospective reference. Those calls
   advance `self._rng`. On `MeshReferenceError`, sequence/current model/topics
   are preserved but RNG is not. A retry can therefore select a different
   candidate and succeed, making a missing or invalid asset intermittent and
   weakening reproducibility. The handler spy test does not snapshot or assert
   RNG state. An independent probe returned `rng_preserved=False`.

## Accepted Parts

- Cache key includes visual id, resolved tier, and reference version.
- Unknown visual ids are rejected before path normalization.
- No clear/delete/create/topic publication or current instance replacement
  occurs on reference failure.
- Code commit `192a6aa` contains exactly the three declared files.

## Required Remediation

- Snapshot RNG state before candidate sampling and restore it on mesh-reference
  failure, or provide an equivalent design that leaves retry selection
  deterministic without committing candidate state.
- Extend the handler test to assert exact RNG-state equality before and after
  failure and prove repeated failed calls select the same invalid candidate.
- Preserve the valid path, six pinned references, online isolation, and all
  accepted FIX1 behavior.

## Verification

- PF-R5A and FIX1 focused suites: 14 passed.
- `luggage_gazebo`: 47 passed.
- `luggage_description`: 135 passed with `ROS_LOG_DIR` redirected to writable
  `/tmp` for the live ROS tests.
- `luggage_perception`: 458 non-GPU tests passed. The complete run reached 459
  passed and failed only the two CUDA-required vintage inference cases because
  the review sandbox exposes no GPU; the owner reports both passed in its GPU
  environment.
- RNG-state probe: request failed, sequence remained 41, external calls stayed
  zero, but RNG state changed.
- `scripts/check_agent_contract.sh`: pass.
- `git diff --check`: pass.

## Decision

PF-R5A-FIX1 remains acceptance-pending until PF-R5A-FIX2 passes. PF-R5B stays
blocked.

## Resolution

PF-R5A-FIX2 at `5d677ac` restores exact RNG state on reference failure and
adds deterministic repeated-failure coverage. This finding is closed.

## Pointers

- `docs/agents/reviews/2026-09-05_1550_pf-r5a-closure-review.md`
- `docs/agents/discuss/2026-09-05_1551_pf-r5a-fix1-review-remediation.md`
- `src/luggage_gazebo/test/test_pf_r5a_fix1_spawn_fail_closed.py`
