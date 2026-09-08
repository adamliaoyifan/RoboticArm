# PF-R6 generation 3 checkpoint (zmode_median production implementation)

- owner: `eng/claude/glm-5.3/claude` (user-directed takeover, formal
  generation 3 row pending reviews dispatch `Q-20260907-1`)
- date: 2026-09-07
- base: gen-2 dirty checkpoint on `f34d9171f8448d7bf92ce84897dc188a110216e2`
- machine: AMD Ryzen 9 9950X3D, RTX 5090, Linux 6.8.0-138, ROS Humble

## Change

`src/luggage_perception/luggage_perception/top_support_estimator.py`:
support-plane fitting stage replaced — `_zmode_support_plane` (adopted
`zmode_median`) instead of `_ransac_horizontal_plane`. Everything else in
`estimate_local_support` unchanged (band, annulus, min points, side
coverage, residual, reason codes, timing keys). Timing key
`support_ransac_ms` kept for series continuity; `support_fit_method`
records the estimator. No online fallback; top-plane RANSAC untouched.

One deliberate refinement over the research prototype: cluster selection
is the *dominant* dist_thresh window (max inlier count; windows within
5% of the max count tie-break to the highest center). The research
prototype's first-from-top rule let a thin Gaussian noise tail (>=80 of
26k points) win and biased support_z by ~5 mm; the count-first rule
matches the committed RANSAC preference and removes that bias. Verified
below on the research capture set.

`src/luggage_perception/test/test_pf_r6_zmode_support.py`: 14 focused
tests covering the codex adoption list — bin-boundary z, competing
plane, wrong-height clutter (full-sector and single-side), missing side,
insufficient points, non-finite, determinism, timing/method tag.

## Verification

- Focused suite: 69 passed
  (`test_pf_r6_zmode_support`, `test_top_support_estimator`,
  `test_platform_free_pipeline`, `test_pf_r6_detector_instrumentation`,
  `test_pf_r6_metrics`, `test_semantic_point_filter`).
- `colcon build --packages-select luggage_perception`: pass.
- Offline regression on the research 156-frame sim capture with the
  PRODUCTION estimator: valid 156/156 (rate 1.000), height err
  med/p95/max 3.68/10.72/10.73 mm (identical to baseline), fit p50
  0.102 ms / p95 0.137 ms.
- Live accepted-profile probes (this machine, `ROS_DOMAIN_ID=7`):
  - `stage_probe.json` (120 s, box present): valid `geometry_ms` p50
    65.339 / p95 73.207 / max 87.008 (was 210.783/276.818/342.598);
    `support_ransac_ms` (zmode) p50 0.604 / p95 0.846 / max 1.739 ms
    (was 126.008/183.909); `support_total_ms` p50 10.498 (was 135.524);
    detector RSS flat (179.5->176.4 MB).
  - `gate4_short6`: active 3.764, top 1.000, full3d 0.947, false
    measured height 0.
  - `gate4_short6_rerun`: active 3.753, top 0.804, full3d 1.000,
    support_z err p50 0.15 um / max 0.52 um, false 0.
  - `gate4_short6_fresh` (clean sim restart): active 3.515, top 0.680,
    full3d 1.000, false 0.
- Teardown residual 0 after every sim run; `check_agent_contract.sh`
  pass.

## Gate status: detector bottleneck resolved; PF-R6 acceptance still blocked upstream

`active_output_hz` is now bounded by stages outside the PF-R6 detector
scope (rates from the 120 s probe):

- `pre_rgb` 3.634 Hz (preprocessor, ~250 ms/frame) — the frame-rate
  ceiling; the detector join can only emit what arrives.
- 26 of 80 settled frames in the fresh run had near-empty cargo clouds
  (median 3605 vs 29574 cargo points, trials 0 and 3 only) ->
  `DETECT_TOP_UNOBSERVABLE`: observation/segmentation completeness, not
  geometry. All frames that did see cargo produced `full3d` support
  (full3d_rate 1.000 in both later runs).

Geometry itself dropped 210.8 -> 65.3 ms p50, so the detector no longer
constrains the 4.0 Hz bar. Remaining levers live in
`sensor_preprocessor`/segmentation observation timing — outside the
bounded PF-R6 perception files of this generation.

## Next decision (for user/reviews)

1. Extend PF-R6 scope to the preprocessor rate bottleneck (new subtask,
   e.g. PF-R8) and/or observation-settle timing (settle-sec, arm pose
   arrival), then rerun the 4.0 Hz gate; or
2. accept 3.5-3.8 Hz as upstream-bound and re-baseline the bar; or
3. hand the fresh-run observation-completeness variance (top 0.68-1.00)
   to the eval/test role for root-causing.
