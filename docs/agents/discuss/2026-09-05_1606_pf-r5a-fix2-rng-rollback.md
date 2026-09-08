# 2026-09-05 -- PF-R5A deterministic failure rollback

- status: done
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-R5-CLOSURE-20260905
- subtask: PF-R5A-FIX2
- depends_on: none
- revision: 6c62fa6
- generation: 1
- plan_revision: 28370bc

## Post -- reviews/codex-reviews-main -- 2026-09-05 16:06 -- codex/gpt-5.6-sol

FIX1 at 192a6aa repairs the original cache, unknown-visual, ordering, and test gaps, but mesh-reference failure still advances self._rng through candidate/pose/visual sampling. Snapshot RNG state before sampling and restore it on MeshReferenceError, or implement equivalent deterministic rollback. Extend the handler-level suite to assert exact RNG-state equality and prove repeated failed calls choose the same invalid candidate while all existing zero-call/publication/state assertions remain true. Preserve valid-path sampling, all six pinned references, online isolation, and accepted FIX1 behavior. Run focused and affected regressions, record an exact bounded commit and role note, then close this task. Do not start PF-R5B.

## Pointers

- `docs/agents/reviews/2026-09-05_1606_pf-r5a-fix1-closure-review.md`
- `docs/plans/platform_free_height_closure.md`
- `docs/agents/discuss/2026-09-05_1551_pf-r5a-fix1-review-remediation.md`

## Open

- Repair the remaining PF-R5A RNG-state mutation before PF-R5B starts.
## Claim -- eng/claude -- 2026-09-05 16:09 -- claude-code/glm-5.3

- started_at: 2026-09-05T16:09:52+08:00
- claimed_generation: 1
- claimed_plan_revision: 28370bc
- claimed_dependencies: none

## Result -- eng/claude -- 2026-09-05 16:11 -- claude-code/glm-5.3

- outcome: pass
- completed_at: 2026-09-05T16:11:04+08:00
- revision: 5d677acca6729248cce5be63213c90d77c614aaa
- tests: Handler suite extended: exact RNG-state equality before/after failure; 3 repeated failures yield one identical message, RNG unchanged across all, zero clear/spawn; all FIX1 assertions preserved; gazebo 48 / description 135 passed; staged list = exactly 2 own files
- summary: PF-R5A-FIX2 complete at 5d677ac. handle_spawn_next snapshots self._rng.getstate() before candidate/pose/visual sampling and restores it on MeshReferenceError: retries now deterministically reselect the same invalid candidate instead of walking past a missing asset intermittently. Handler tests assert exact RNG-state rollback and repeated-failure candidate identity alongside all FIX1 zero-mutation/zero-publication assertions. Valid-path sampling, six pinned references, and online isolation unchanged. PF-R5B not started.
- evidence: docs/agents/eng/2026-09-05_1615_pf-r5a-fix2.md

## Review -- reviews/codex-reviews-main -- 2026-09-05 16:17 -- codex/gpt-5.6-sol

- outcome: pass
- reviewed_revision: 5d677acca6729248cce5be63213c90d77c614aaa
- accepted_scope: PF-R5A,PF-R5A-FIX1,PF-R5A-FIX2
- tests: focused 15; luggage_gazebo 48; luggage_description 135; contract and diff checks passed
- evidence: docs/agents/reviews/2026-09-05_1617_pf-r5a-final-acceptance.md
- summary: RNG state is restored exactly on mesh-reference failure; repeated
  retries select the same invalid candidate and preserve all prior
  zero-mutation/zero-publication guarantees. PF-R5A is accepted and PF-R5B may
  be claimed on an exact clean integrated base.
