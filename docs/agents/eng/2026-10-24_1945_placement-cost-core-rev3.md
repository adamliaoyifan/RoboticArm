- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- parent: PLACEMENT-GRID-LAYOUT-20261023
- subtask: COST-CORE
- base_revision: 92455834
- started_at: 2026-10-24T00:24:00+08:00
- completed_at: 2026-10-24T19:40:00+08:00

## Summary

Measurement-only correction round (reviews/2026-10-24_0000, user-bounded:
record timings, no optimization): per-stage timing instrumentation
(diagnostics excluded from all content identity), real 0/5/20-box load
fix with placed==requested assertion, and a 9-group timing table on
f3029889.  The closure check caught a real instrumentation double count
(repaint in two buckets) -- corrected analytically in the evidence and
fixed in code for future runs (5135ba7f).  Root-cause data: adjacency is
negligible; after-window rescan 60-72% and coverage books 28-40% at
0.01m; books_s tracks paint_blocks at a constant ~2.0-2.5us/block across
all resolutions/loads.  Standing acceptance unchanged: performance FAIL
(0.02/0.01 exceed 20s), chamfer not verified, online forbidden.

## Requirement

- reviews/2026-10-24_0000 findings 1-2 wording/load corrections and the
  user's narrowed scope: timing only, no functional optimization, no
  chamfer attempt this round; every bucket mutually exclusive with
  unattributed time reported; version separation between the running
  batch and later instrumentation; counter correlation presented as
  locating clues only.

## Changed

- `src/luggage_packing/luggage_packing/placement_layout_contracts.py` (diagnostics field, content_equal)
- `src/luggage_packing/luggage_packing/placement_layout_cost.py` (stage timers/counters; accounting fix)
- `src/luggage_packing/test/test_placement_layout_cost.py` (diagnostics + load tests)
- `scripts/g3/placement_cost_core_eval.py` (load fix, timing subcommand)

Commits f3029889 (ran the batch), 865fdb8b (books split, future runs),
5135ba7f (mutually exclusive buckets, future runs) on
agent/eng/placement-cost-g2.

## Verification

- 43 tests pass (41 prior + diagnostics identity + load counts);
  baseline solver/atlas/occ-crop 45 pass unchanged.
- timing: 9/9 groups measured, exit 0, one cold run each (no sampling
  inference); fixtures re-run exit 0, 540 cases, oracle 201/201.
- cost_digest pre/post instrumentation identical on three input classes
  (empty / 5-degree neighbour / platform) -- scope-limited, not
  extrapolated.
- Closure: corrected unattributed time <=2.7% per group; other_s is a
  residual bucket.

## Result

- pass for the measurement round at `f3029889`..`5135ba7f` (deliverable:
  instrumentation + load fix + timing table + corrected evidence
  wording).  Overall COST-CORE acceptance state remains: performance
  FAIL (blocking, offline), chamfer unverified, online forbidden --
  optimization candidates ranked in
  evidence rev3/timing/ANALYSIS.md await an authorized optimization
  round with pre/post same-input comparison.

## Pointers

- `docs/status/evidence/occ_owner_b1/placement_cost_core_g2_20261023/rev3/`
- `docs/agents/discuss/2026-10-23_1813_placement-cost-core-g2.md`
