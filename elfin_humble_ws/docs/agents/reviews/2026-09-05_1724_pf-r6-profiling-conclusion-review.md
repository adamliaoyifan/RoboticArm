# 2026-09-05 -- PF-R6 profiling conclusion review

- role: reviews
- agent: codex-reviews-main
- model: gpt-5.6-sol
- cli: codex
- status: open

## Summary

PF-R6 remains correctly blocked at partial implementation commit `1ce5f6c`,
but the evidence does not yet establish the stated exact-stamp/temporal-hold
root cause. The current probe observes causally impossible rate ordering, and
the implementation publishes held detections with the current input stamp.
Instrumentation must be repaired before selecting the next online-path
optimization. The band-first support filter appears behavior-preserving; the
lazy raw-cloud change needs a focused semantic and latency review.

## Findings

1. The measured rates cannot support the upstream-cause conclusion. In
   `optimized_100s.json`, `yolo=4.806 Hz` exceeds its only input
   `pre_rgb=4.421 Hz`, and `frame=3.501 Hz` exceeds required semantic cargo
   input `cargo=3.052 Hz`. These inversions show probe subscriber loss,
   boundary effects, or callback starvation. They do not prove a 28 percent
   exact-stamp mismatch or a cargo throughput ceiling.
2. The handoff's temporal-hold statement conflicts with the code.
   `SemanticSegmenter.update()` stores the current input stamp in every
   `SegmenterOutput`; the temporal gate changes mask/detection content only.
   `semantic_segmenter_node.py` publishes that current stamp. A held detection
   is therefore not evidence of an old-stamp replay.
3. PF-G6S instrumentation is incomplete. The probe does not report separate
   top/support fit times, exact join misses and occupancy, callback backlog,
   first TOP_ONLY/FULL_3D latency, or a defensible RSS trend. Detector
   diagnostics are null because that topic is emitted around service calls,
   not every stream frame. RSS is sampled repeatedly during each divisible
   five-second bucket and summarized only as first/last.
4. Baseline and optimized runs are not controlled A/B samples: input rates and
   cargo point counts differ materially. The observed frame change from 3.418
   to 3.501 Hz cannot be attributed to `1ce5f6c` yet.
5. The band-first support candidate filter commutes two predicates over finite
   points and is covered by the reported perception regression. It is a
   reasonable provisional optimization.
6. Lazy raw processing removes background work, but moves decode/TF onto the
   joined-frame critical path. It also changes retry semantics from one initial
   lookup plus three retries to three total attempts. Focused tests must prove
   unchanged exact-stamp support availability and measure critical-path impact
   before this change is accepted.
7. Any further semantic segmenter/filter change modifies the online detection
   path. Final PF-R6 closure must therefore rerun the original PF-R5 30-trial
   accuracy matrix as well as PF-G6S; the earlier run8 reuse boundary no longer
   applies.

## Proposed Resolution

- Treat `1ce5f6c` as a partial candidate revision, not an accepted performance
  fix.
- First add publisher-side or in-node counters for causal rates, exact stamp
  sets, join reasons/occupancy, stage timing, backlog, RSS trend, and first
  valid output. Use the existing preprocessor and semantic-filter diagnostics
  where possible.
- Localize the bottleneck with a controlled baseline before changing stamp
  semantics, QoS, queue depth, temporal hold, or preprocessor pairing.
- Because the original Claude owner has little context budget left and has
  already returned `outcome: blocked`, supersede the claimed legacy attempt and
  assign a higher-generation PF-R6 replacement to a fresh owner after the plan
  amendment is approved.

## Consensus Questions

1. Do the causal inversions and current-stamp implementation invalidate the
   claimed exact-stamp/held-stamp root cause?
2. Is the proposed instrumentation-first replacement sufficient to establish
   PF-G6S bottleneck ownership without changing sim/real semantics?
3. Should the band-first optimization be retained provisionally, and should
   lazy raw processing remain conditional on focused retry/latency evidence?
4. Should PF-R6 be replaced by a higher generation for a fresh owner, with the
   original Gate 4 matrix required after any further online-path change?

## Pointers

- `docs/agents/eng/2026-09-05_1720_pf-r6-handoff.md`
- `docs/agents/eng/2026-09-05_1830_full-handoff-eng-successor.md`
- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/platform_free_height_test_plan.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1800_pfr6-profiling/`
- `src/luggage_perception/scripts/stage_perf_probe.py`
- `src/luggage_perception/scripts/semantic_segmenter_node.py`
- `src/luggage_perception/luggage_perception/semantic_segmenter.py`
- `src/luggage_perception/scripts/semantic_point_filter_node.py`
