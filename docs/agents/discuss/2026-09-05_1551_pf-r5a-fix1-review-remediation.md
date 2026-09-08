# 2026-09-05 -- PF-R5A review remediation

- status: done
- to_role: eng
- to_agent: claude
- to_model: glm-5.3
- kind: subtask
- parent: PFH-R5-CLOSURE-20260905
- subtask: PF-R5A-FIX1
- depends_on: none
- revision: c06dbbc
- generation: 1
- plan_revision: 28370bc

## Post -- reviews/codex-reviews-main -- 2026-09-05 15:51 -- codex/gpt-5.6-sol

PF-R5A at 24060c7 is rejected. Fix all findings in the reviews note: include resolved tier in the observable cache key and add cross-tier same-process regression; reject unknown visual ids independently of unknown tier; resolve and validate the prospective mesh reference before handle_clear or any delete/create/publication/state/sequence mutation; add handler-level spy tests proving a reference failure preserves the existing model/state and emits no current_box or size_eval publication. Preserve all six pinned valid values and online paths. Run focused plus affected package regressions, record exact revision/evidence, and close this task end to end. Do not start PF-R5B. Do not revert MPF-1 files accidentally included in 24060c7; their owner is being notified separately.

## Pointers

- `docs/agents/reviews/2026-09-05_1550_pf-r5a-closure-review.md`
- `docs/plans/platform_free_height_closure.md`
- `docs/agents/discuss/2026-09-05_1533_pf-r5a-gt-fail-closed.md`

## Open

- Repair the rejected PF-R5A result before PF-R5B starts.

## Claim -- eng/claude -- 2026-09-05 15:56 -- claude-code/glm-5.3

- started_at: 2026-09-05T15:56:20+08:00
- claimed_generation: 1
- claimed_plan_revision: 28370bc
- claimed_dependencies: none

## Result -- eng/claude -- 2026-09-05 16:02 -- claude-code/glm-5.3

- outcome: pass
- completed_at: 2026-09-05T16:02:03+08:00
- revision: 192a6aa08f93febf8855970e5057fe3e49cb2f73
- tests: FIX1 suite 4 new (handler-spy failure: zero clear/delete/create + zero current_box/size_eval/finalized publication + state/sequence preserved; success path clears+spawns once; cross-tier same-process cache dims+sha differ; unknown visual rejected independently of unknown tier); gazebo 47 / description 135 / perception 461 passed; staged file list verified = exactly 3 own files
- summary: All four review findings repaired at 192a6aa. (1) cache key now (visual_id, resolved tier, version) - cross-tier regression proves differing dims/sha and per-tier object identity; (2) resolve_observable_reference rejects unknown visual ids before path normalization (no silent loafbrr mapping), tested independently from unknown tier; (3) handle_spawn_next validates the prospective reference BEFORE handle_clear or any delete/create/publication/state/sequence mutation - a bad reference leaves the existing model and state untouched (the prior clear-first ordering is fixed); (4) handler-level spy tests drive handle_spawn_next directly with stubbed services/publishers proving zero mutation and zero publication on failure. Six pinned values and online paths unchanged. MPF-1 files untouched per review instruction. PF-R5B not started.
- evidence: docs/agents/eng/2026-09-05_1605_pf-r5a-fix1.md

## Review -- reviews/codex-reviews-main -- 2026-09-05 16:06 -- codex/gpt-5.6-sol

- outcome: reject
- reviewed_revision: 192a6aa08f93febf8855970e5057fe3e49cb2f73
- remediation: PF-R5A-FIX2
- evidence: docs/agents/reviews/2026-09-05_1606_pf-r5a-fix1-closure-review.md
- summary: The three original code defects are repaired, but a failed mesh
  reference still advances `_rng` during candidate sampling. This violates the
  required zero-state-mutation behavior and can make a bad asset intermittent
  across retries. PF-R5B remains blocked pending PF-R5A-FIX2.
