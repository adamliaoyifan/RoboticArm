# 2026-09-07 -- PF-R8/PF-R9/PF-R10 planned and dispatched, then parked on consensus

- role: reviews
- agent: cursor
- model: opus5
- cli: cursor
- status: done

## Summary

Wrote `docs/plans/pf_r8_r9_perception_acceptance.md` (PF-R8 detection
availability, PF-R9 preprocessor throughput, PF-R10 gate4 integration), all
owned by `eng/claude/glm-5.3`, and dispatched `Q-20260907-3/4/5`. The mandatory
distinct-Codex requirement-consistency check then rejected the plan, so the
three rows are parked at `dispatch_ready: no` and **must not be claimed**.

I initially recorded `consensus: reached` on my own thread
(`2026-09-07_2010_...`) from a narrowed third-round prompt. That was wrong: the
same Codex reviewer had independently opened
`2026-09-07_2026_pf-r8-r9-plan-consistency.md` with a fuller audit that rejects
both `95522db` and the amended `bcafe2e`. My thread is now `superseded`; the
`2026-09-07_2026` thread is authoritative.

## Findings that survived verification

Two are load-bearing and change the plan's premise, not just its wording.

### The 5-frame temporal hold cannot bridge the observed dropouts

Recomputed from `failed_cases.jsonl`: of the 42 false-positive-only frames, 27
have no accepted in-region detection within +/-5 rows and 37 have none in the
prior 5 rows. Rows 0-20 and 122-142 are contiguous 21-row runs, and the median
row spacing is 0.263 s against a frame period of ~0.26-0.29 s at the measured
3.4-3.8 Hz, so those runs are genuine ~5 s dropouts rather than a filtered-capture
artefact (the 17 row gaps > 1 s are the ~5 s trial orchestration gaps).

Consequences:

1. Acceptance A3a as written is unsatisfiable. A causal 5-frame gate cannot emit
   a held bbox for every target frame, and using a *future* nearest neighbour as
   the reference could never justify an online hold.
2. More importantly, **repairing the temporal gate is necessary but not
   sufficient**. `temporal_window_frames: 5` bridges ~1.4 s; the failures are
   ~5 s. The dominant sub-problem is YOLO recall on the suitcase (in-region
   confidence p50 0.240, p25 0.124, 42/155 frames below the 0.04 threshold, in
   multi-second runs), which the plan treated as a secondary detail.

Repairing the gate is still correct — it is currently dead, since the persistent
false positive makes `had_cargo` true on every frame — but PF-R8's scope and
acceptance must be rebuilt around recall plus fail-closed behaviour.

### The A1 fixture cannot discriminate the predicate it is meant to validate

Every frozen expected-negative in A1 is border-touching and no edge-clipped true
positive is present, so a pure border test scores perfectly on it. The plan's own
Risks section calls border-only rejection unsafe. The fixture must contain a
labelled edge-clipped positive, or the plan must adopt border rejection
explicitly and drop the contradictory safety claim.

## Other open findings (from the authoritative thread)

- PF-R7 (`Q-20260904-2`) is unsuperseded and its dependency list excludes
  PF-R8/R9/R10, so it could certify the pre-fix chain. Needs supersession to
  depend on PF-R10, or an explicit cancel/replace. This touches
  `cursor/grok-4.6`'s row, so it is not mine to rewrite unilaterally.
- B1 lacks the matched-header cloud-versus-RGB receipt-lag distribution,
  unmatched fraction, and clock domain, which are exactly what B2's deadline
  must be derived from; B2 does not define the deadline clock or the behaviour
  when the justified wait reaches `camera_horizon_sec`.
- B3/B4 still lack an explicit scoring window and warmup interval.
- B5 authorises removing the whole-observation copy without preserving the
  immutable independent-copy contract; it needs a mutation-isolation test, and
  conditional acceptance (minimum retained point density, geometry
  non-regression) if decimation is selected.
- A4 equates the PF-R5 run8 value 0.9606 with a newly defined
  accepted-detection recall; the baseline predates the predicate and does not
  establish the same numerator.
- C2's first-versus-last-quartile means demonstrate net drift, not absence of
  monotonic growth; it needs the actual pass inequality or a permitted slope.

## Correct items (unchanged)

The two evidence corrections in section 1 of the plan were confirmed by the
Codex audit against the raw data: the `top_surface_rate` failure is the static
border false positive rather than FOV clipping, and the `active_output_hz`
failure is preprocessor throughput (2517 raw images against 435 preprocessed
frames) rather than downstream join loss (397/477 joins succeed). PF-R9's
direction and the PF-R8/PF-R9/PF-R10 decomposition shape were both accepted.

## Next step

PF-R8 needs a scope decision before another amendment round: whether raising
YOLO suitcase recall is in scope for `eng/claude/glm-5.3`, or whether PF-R8
narrows to the false-positive predicate plus gate repair with fail-closed
acceptance and recall handled separately. That decision is the user's, because
it changes the size of the task materially.

## Pointers

- `docs/plans/pf_r8_r9_perception_acceptance.md` (at `76e5307`)
- `docs/agents/discuss/2026-09-07_2026_pf-r8-r9-plan-consistency.md` (authoritative)
- `docs/agents/discuss/2026-09-07_2010_pf-r8-r9-perception-acceptance-consensus.md` (superseded)
- `docs/agents/discuss/2026-09-07_2039_pf-r8-cargo-detection-availability.md`
- `docs/agents/discuss/2026-09-07_2039_pf-r9-preprocessor-throughput.md`
- `docs/agents/discuss/2026-09-07_2039_pf-r10-gate4-integration.md`
- `docs/agents/reviews/2026-09-07_1956_pf-r6-gen3-current-issues-review.md`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/`

## Closed

Consensus was subsequently reached at plan revision `3460bff` and the three rows
were dispatched. Final state and the scope decision are recorded in
`docs/agents/reviews/2026-09-07_2118_pf-r8-r9-r10-dispatched.md`.

