# 2026-09-05 -- PF-R6 generation 2 Codex takeover

- status: superseded
- to_role: eng
- to_agent: codex
- to_model: gpt-5
- kind: subtask
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R6
- depends_on: PF-R4,PF-R5
- revision: 218f444406217bab779ce99333ec75fb8b24e1de
- generation: 2
- plan_revision: a001be7
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-09-05 17:31 -- codex/gpt-5

Take over PF-R6 from the blocked `claude/glm-5.3` generation 1 attempt.

Own PF-R6 end to end against `docs/plans/platform_free_height_remediation.md`
at plan revision `a001be7` plus the accepted root-cause consensus in
`docs/agents/reviews/2026-09-05_1731_pf-r6-root-cause-consensus.md`.

Start from current revision `218f444406217bab779ce99333ec75fb8b24e1de`.
Treat `1ce5f6c` as a partial candidate, not an accepted performance fix:
`stage_perf_probe.py` and the band-first support filter may remain if verified;
lazy raw processing must be proven with focused exact-stamp availability,
retry-semantic, and critical-path latency tests before acceptance.

Implementation order:

1. Establish repaired instrumentation for causal rates, exact join misses,
   backlog, buffer occupancy, stage timing, RSS trend, and first TOP_ONLY /
   first stable FULL_3D latency.
2. Run a controlled accepted-profile baseline before changing stamp semantics,
   QoS, queue depth, temporal hold, or preprocessor pairing.
3. Optimize only after the bottleneck is localized, following PF-R6 order:
   duplicate decode/TF removal, bounded same-stamp reuse, support candidate
   crop/downsample, early failure exits, and only then algorithm parameters.
4. Preserve all platform-free fail-closed and no-online-GT contracts.
5. If any online detection-path behavior changes, rerun PF-G6S and the original
   PF-R5 30-trial Gate 4 accuracy matrix before closing PF-R6.

Acceptance:

- Accepted semantic geometry output is at least 4 Hz on the accepted GPU
  profile.
- Per-stage P50/P95/max and active-window E2E latency are reported.
- Join buffers are bounded; callback lag and RSS do not grow monotonically.
- First TOP_ONLY and first stable FULL_3D latency after each box change are
  reported.
- Focused tests cover every retained optimization and changed online behavior.
- `scripts/stop_sim.sh` is run for any agent-started sim and residual count is
  zero.
- Write evidence under
  `docs/status/evidence/platform_free_height/<timestamp>_pfr6_generation2/`,
  write an eng role note, commit bounded PF-R6 changes, and close with
  `scripts/agent_complete.sh`.

## Pointers

- `docs/agents/discuss/2026-09-04_1453_pfh-remediation-r6.md`
- `docs/agents/reviews/2026-09-05_1724_pf-r6-profiling-conclusion-review.md`
- `docs/agents/reviews/2026-09-05_1731_pf-r6-root-cause-consensus.md`
- `docs/agents/eng/2026-09-05_1720_pf-r6-handoff.md`
- `docs/agents/eng/2026-09-05_1830_full-handoff-eng-successor.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1800_pfr6-profiling/`
- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/platform_free_height_test_plan.md`
- `src/luggage_perception/scripts/stage_perf_probe.py`
- `src/luggage_perception/scripts/semantic_segmenter_node.py`
- `src/luggage_perception/luggage_perception/semantic_segmenter.py`
- `src/luggage_perception/scripts/semantic_point_filter_node.py`

## Open

- Implement PF-R6 generation 2 and pass PF-G6S with the required regression
  boundary.
## Claim -- eng/codex -- 2026-09-05 17:34 -- codex/gpt-5

- started_at: 2026-09-05T17:34:37+08:00
- claimed_generation: 2
- claimed_plan_revision: a001be7
- claimed_dependencies: PF-R4=1,PF-R5=1

## Reply -- eng/codex -- 2026-09-05 18:40 -- codex/gpt-5

Per user direction, long-running validation for the current PF-R6 generation 2
checkpoint has been split to `test/cursor/grok-4.6` as
`Q-20260905-14` / `PF-R6-VERIFY` in
`docs/agents/discuss/2026-09-05_1840_pf-r6-grok-validation.md`.

This does not close or transfer ownership of PF-R6 itself. The regression task
is an independent validation of the shared-worktree checkpoint, including the
current `output.cargo_voxel_size: 0.005` setting, aggregated
`stage_perf_probe.py` timing, bounded-buffer diagnostics, short accepted-profile
Gate 4 evidence, and teardown residuals. If grok reports a failing criterion,
`eng/codex` will resume from the exact bottleneck evidence. If grok reports a
validation-pass candidate, `eng/codex` still owns final PF-R6 review and formal
closure, including full PF-G6S and original PF-R5 30-trial Gate 4 regression on
an exact committed revision.

## Reply -- eng/codex -- 2026-09-05 19:42 -- codex/gpt-5

After `test/cursor/grok-4.6` re-ran PF-R6-VERIFY and returned blocked, eng
summarized the bottleneck data and asked reviews to confirm the next
optimization direction before estimator changes:
`docs/agents/discuss/2026-09-05_1942_pf-r6-ransac-optimization-review.md`
(`Q-20260905-15`).

Current evidence: `active_output_hz` 3.557 fails the 4.0 bar, while
`top_surface_rate` 1.0, `full3d_rate` 0.961, false measured height 0, and
teardown residual 0 pass. The largest measured cost is detector support-plane
RANSAC: valid `geometry_ms` p50 210.783 ms / p95 276.818 ms, with
`support_ransac_ms` p50 126.008 ms / p95 183.909 ms. Proposed direction sent to
reviews: keep top RANSAC initially and replace only support-plane RANSAC with a
fail-closed robust `z` mode/median estimator guarded by the existing
same-stamp/status gates, annulus crop, side coverage, residual threshold, and
no-online-GT constraints. Handoff note:
`docs/agents/eng/2026-09-05_1942_pf-r6-ransac-handoff.md`.

## Supersede -- eng/claude -- 2026-09-08 10:47 -- claude/glm-5.3

Superseded by generation 3 in
`2026-09-08_1047_pf-r6-generation3-claude-user-directed.md` per the user
directive recorded in Q-20260907-1: owner routing `eng/codex/gpt-5` -> 
`eng/claude/glm-5.3`. The codex generation-2 claim of 2026-09-05 is stale
by that user decision; codex should acknowledge this stop at its next run.
The gen-2 checkpoint work (instrumentation, lazy raw transform, band-first
filter) carried into the generation-3 commit `f03ccc3`.
