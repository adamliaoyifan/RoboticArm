# 2026-09-05 -- Assignment token compression review

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Reviewed the open assignment queue and PF-R6 handoff chain. The main token waste is not the number of rows in `OPEN.md`; it is repeated natural-language restatement of requirements and verification procedure in runnable threads after the same facts already exist in approved plans, consensus notes, and evidence JSON.

Recommended compression:

- Use notification threads as pointers to canonical plans/evidence/check scripts instead of restating the full acceptance contract.
- Replace exploratory review verification text with a single task-owned verification command that emits JSON pass/fail data.
- After consensus is reached and the conclusion is copied into the plan, keep later routing posts to summary plus pointers; do not require later readers to replay the whole consensus exchange.

Current active claimed threads should not be rewritten in place. Apply this to new dispatches, superseding generations, or follow-up plan text.

## Risks

- Pointer-only posts are safe only when the target plan revision is exact and immutable for that generation.
- Verification-command compression must not hide acceptance criteria inside prose; the JSON schema needs explicit gates, thresholds, evidence paths, revision, dirty count, and teardown result.
- Existing active PF-R6/PF-R6-VERIFY threads remain verbose until closed or superseded because rewriting claimed requirements would violate the shared agent contract.

## Pointers

- `docs/agents/discuss/OPEN.md`
- `docs/agents/discuss/2026-09-05_1731_pf-r6-generation2-codex-takeover.md`
- `docs/agents/discuss/2026-09-05_1840_pf-r6-grok-validation.md`
- `docs/agents/discuss/2026-09-05_1956_pf-r6-ransac-method-research.md`
- `docs/agents/discuss/2026-09-05_1950_pf-r6-ransac-research-consensus.md`
- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/pf_r6_support_plane_method_research.md`
