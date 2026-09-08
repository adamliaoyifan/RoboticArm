# 2026-09-05 15:50 -- PF-R5A closure review

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: done
- parent: PFH-R5-CLOSURE-20260905
- subtask: PF-R5A
- reviewed_revision: 24060c75815e3c9646b5d2186b21744e99d69532
- plan_revision: 28370bc511d5ed564dc709a661af4f6bea2679eb

## Summary

PF-R5A is not accepted at `24060c7`. The pure resolver tests pass, but the
spawner valid path has a tier-omitting cache key, unknown visual ids silently
resolve to the loafbrr asset, and the failed-spawn path clears the current
world model before validating the prospective mesh reference. The result
commit also contains the concurrently owned MPF-1 implementation.

## Findings

1. `PickupBoxSpawner._observable_reference` caches by
   `(visual_id, OBSERVABLE_REFERENCE_VERSION)` and omits the size tier. After
   one tier is resolved, later tiers of the same visual reuse the first tier's
   dimensions and SHA-256. A direct small-then-large probe returned the small
   reference object for both requests. This changes the valid GT path and
   invalidates the claimed run8 reuse boundary.
2. `sized_model_name` normalizes every unknown visual id to
   `suitcase_loafbrr`. Consequently `resolve_observable_reference` accepts an
   unknown visual, reads the loafbrr STL, and records the requested unknown id.
   This violates the required unknown visual/tier fail-closed behavior.
3. `handle_spawn_next` calls `handle_clear` before it samples and validates the
   prospective reference. A bad STL can therefore delete the existing Gazebo
   model and publish empty current-box state before returning
   `MESH_REFERENCE_UNAVAILABLE`. The claimed pre-mutation behavior is false.
4. The ten new tests exercise only the pure resolver. They do not cover cache
   separation or service-handler mutation/publication behavior, so the three
   defects above pass the focused suite.
5. Commit `24060c7` also adds the MPF-1 mailbox implementation, tests,
   evidence, and role note while MPF-1 remains claimed by `codex/gpt-5`.
   Ownership and revision provenance must be reconciled before either task
   cites this as an isolated result commit.

The MPF-1 owner completed its thread during this review and explicitly adopted
`24060c7` as its own tested Result revision. The collision is therefore
traceable and requires no history rewrite, although `24060c7` remains a shared
rather than single-purpose result commit.

## Required Remediation

- Restore a cache key containing both visual id and resolved tier; add a
  same-process, same-visual, cross-tier regression proving dimensions and
  hashes differ correctly.
- Reject unknown visual ids before path normalization; test unknown visual and
  unknown tier independently.
- Resolve and validate the prospective reference before clearing/deleting the
  current instance. Add a handler-level test proving no delete, create,
  current-box publication, size-eval publication, sequence/state replacement,
  or GetCurrentBox contamination on reference failure.
- Keep valid six-asset values and online paths unchanged. Rerun affected
  package tests and report an exact repair revision.
- Coordinate the accidental MPF-1 inclusion with its active owner; do not
  revert or overwrite that owner's work.

## Verification

- `test_pf_r5a_gt_fail_closed.py`: 10 passed.
- Unknown-visual probe resolved
  `unknown_visual` to `suitcase_loafbrr_small/meshes/suitcase.stl`.
- Cache probe requested loafbrr small then large and received the same small
  reference object, width, tier, and hash for both.
- `git diff --check 28370bc..24060c7`: pass.

## Decision

PF-R5A remains acceptance-pending. PF-R5B must remain blocked until the repair
passes and the exact-revision provenance issue is reconciled.

## Resolution

Accepted after PF-R5A-FIX1 at `192a6aa` and PF-R5A-FIX2 at `5d677ac` repaired
all findings and passed closure review.

## Pointers

- `docs/plans/platform_free_height_closure.md`
- `docs/agents/discuss/2026-09-05_1533_pf-r5a-gt-fail-closed.md`
- `docs/agents/discuss/2026-09-05_1518_mpf-1-generation-lifecycle.md`
