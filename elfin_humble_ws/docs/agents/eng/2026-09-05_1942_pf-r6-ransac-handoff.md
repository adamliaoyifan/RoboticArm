# 2026-09-05 -- PF-R6 RANSAC handoff

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

PF-R6 generation 2 remains open and blocked on the `active_output_hz >= 4.0`
bar. Independent `test/cursor/grok-4.6` evidence localizes the largest cost to
detector support-plane RANSAC, so eng asked reviews to confirm a fail-closed
replacement direction before changing the estimator.

## Status

- PF-R6 owner remains `eng/codex/gpt-5`; do not close
  `docs/agents/discuss/2026-09-05_1731_pf-r6-generation2-codex-takeover.md`
  until final PF-G6S and original PF-R5 30-trial Gate 4 regression pass on an
  exact committed revision.
- `Q-20260905-14` / `PF-R6-VERIFY` was run by `test/cursor/grok-4.6` and
  returned `outcome: blocked`.
- `Q-20260905-15` is open for `reviews/codex-reviews-main/gpt-5.6-sol` to
  confirm or amend the proposed support-estimator optimization.
- `PF-R7` remains unclaimed and must stay blocked until PF-R6 has a passing
  Result.
- Current tree contains unrelated multi-agent dirty files. Preserve them; do
  not revert broad docs/scripts changes outside the bounded PF-R6 files.

## Current Changes

The current PF-R6 checkpoint includes these bounded perception changes:

- `src/luggage_perception/luggage_perception/pf_r6_metrics.py`: new pure metric
  helpers for active rate, percentiles, stamp deltas, stamp-match summaries, and
  first-latency summaries.
- `src/luggage_perception/test/test_pf_r6_metrics.py`: pure tests for those
  metric helpers.
- `src/luggage_perception/test/test_pf_r6_detector_instrumentation.py`: focused
  lazy raw lookup instrumentation tests without constructing a ROS node.
- `src/luggage_perception/luggage_perception/semantic_point_filter.py`: join
  diagnostics for cloud/mask counts, misses, stale drops, and latest stamp gap.
- `src/luggage_perception/scripts/semantic_point_filter_node.py`: bounded
  buffer diagnostics, stage timing, optional cargo voxel downsample, obstacle
  publish skip when unsubscribed, and stats payload expansion.
- `src/luggage_perception/scripts/luggage_detector_node.py`: stream stats topic,
  lazy raw lookup counters/status, raw buffer stats, and pipeline timing export.
- `src/luggage_perception/luggage_perception/top_support_estimator.py`: timing
  out-params around top/support crop, voxel, RANSAC, side coverage, and totals.
- `src/luggage_perception/luggage_perception/platform_free_pipeline.py`: carries
  estimator timing into `PipelineResult.timing`.
- `src/luggage_perception/scripts/stage_perf_probe.py`: subscribes filter stats,
  detector stream stats, and current box; reports stamp matches, first
  latencies, aggregated filter timing, stream timing, valid-stream timing, PCA
  reasons, and RSS.
- `src/luggage_perception/config/semantic_segmenter.yaml`: current working value
  is `semantic_point_filter.ros__parameters.output.cargo_voxel_size: 0.005`.
  The key must remain under `ros__parameters`; the first grok run proved the
  launch fails when it is outside that mapping.
- `src/luggage_perception/test/test_semantic_point_filter.py`: added
  `JoinStampTracker` diagnostic coverage.

## Current Data

- `active_output_hz`: 3.557, below the 4.0 PF-R6 bar.
- `top_surface_rate`: 1.0, passes.
- `full3d_rate`: 0.961, passes.
- `false_measured_height`: 0, passes.
- probe rates: `pre_rgb` 4.403 Hz, `yolo` 4.664 Hz, `depth_pts` 3.822 Hz,
  `cargo` 3.677 Hz, `frame` 3.518 Hz.
- valid detector `geometry_ms`: p50 210.783 ms, p95 276.818 ms, max
  342.598 ms.
- detector split: `support_total_ms` p50 135.524 ms, p95 194.7 ms;
  `support_ransac_ms` p50 126.008 ms, p95 183.909 ms.
- top split: `top_total_ms` p50 74.056 ms, p95 89.604 ms;
  `top_ransac_ms` p50 39.641 ms, p95 50.341 ms; `top_refine_ms` p50
  29.819 ms, p95 35.724 ms.
- semantic point filter: `process_total_ms` p50 68.569 ms, p95 78.107 ms;
  `publish_ms` p50 35.826 ms, p95 41.39 ms.
- detector RSS: 188068 kB to 279240 kB during the 120 s probe; recheck after
  the next fix.
- teardown residual: 0.

## Evidence Detail

Primary independent verification:

- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/RESULT.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/metrics_extract.json`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/stage_probe.json`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/gate4_short6/summary.json`
- `docs/agents/test/2026-09-05_1924_pfr6-verify.md`

Earlier local PF-R6 generation 2 evidence is under
`docs/status/evidence/platform_free_height/2026-09-05_1748_pfr6_generation2/`.
Notable intermediate runs:

- `gate4_short3/summary.json`: active output about 3.705 Hz before obstacle
  publish skip and cargo voxel tuning.
- `gate4_short3_obstacle_skip/summary.json`: active output about 3.302 Hz after
  skipping obstacle publish only; publish time improved, but rate did not.
- `gate4_short3_cargo_voxel/summary.json`: `output.cargo_voxel_size=0.01`
  reduced serialization but harmed small-box top detection; do not restore
  0.01 without new evidence.
- `stage_probe_aggregated_timing_eval6.json`: first local proof that valid
  `geometry_ms` was dominated by support RANSAC, but the grok 120 s probe is
  the stronger evidence.

## Verification So Far

- `python3 -m pytest -q src/luggage_perception/test/test_pf_r6_detector_instrumentation.py src/luggage_perception/test/test_pf_r6_metrics.py src/luggage_perception/test/test_semantic_point_filter.py`: passed earlier in the PF-R6 checkpoint.
- `python3 -m pytest -q src/luggage_perception/test/test_pf_r6_detector_instrumentation.py src/luggage_perception/test/test_pf_r6_metrics.py src/luggage_perception/test/test_semantic_point_filter.py src/luggage_perception/test/test_platform_free_pipeline.py src/luggage_perception/test/test_top_support_estimator.py`: `56 passed`.
- `colcon build --packages-select luggage_perception`: passed.
- Full `src/luggage_perception/test` was attempted before the final YAML fix:
  most tests passed, but `TestVintagePoseRegression` failed because CUDA device
  `0` was requested while CUDA was unavailable in that run. Treat this as
  environment-gated, not PF-R6 logic, unless it reproduces on the accepted GPU
  profile.
- `scripts/stop_sim.sh`: residual 0 after local and grok runs.

## Proposed Direction

RANSAC here is plane fitting, not edge extraction. The slowest stage is fitting
the support platform plane from 21k-31k annulus candidates. The proposed next
change is to keep the top estimator unchanged initially and replace only the
support-plane RANSAC stage with a fail-closed robust `z` estimator:

- keep existing same-stamp raw lookup, preprocessor status gate, source gate,
  height band, workspace crop, annulus crop, and side-coverage checks;
- select support height from the annulus candidates by dominant `z` bin or
  robust median/mode cluster instead of 200-iteration RANSAC;
- require `min_support_points`, `min_support_sides`, residual threshold, and
  plausible top-minus-support height before reporting measured support;
- return TOP_ONLY / `DETECT_SUPPORT_UNOBSERVABLE` or
  `DETECT_SUPPORT_UNSTABLE` on weak, multimodal, sparse, or side-insufficient
  evidence;
- do not add any online GT fallback or configured support shortcut in auto
  mode.

## Implementation Sketch

Do not implement this until reviews answers `Q-20260905-15`, unless the user
explicitly overrides the wait.

Candidate helper shape in
`src/luggage_perception/luggage_perception/top_support_estimator.py`:

- add a private helper such as `_robust_horizontal_support_z(candidates,
  top_estimate, config, timing=None)`;
- bin candidate `z` values at roughly the existing support distance threshold
  scale, for example 8-10 mm, or use a small median-centered cluster;
- choose the densest plausible cluster inside the existing height band;
- compute `support_z` as cluster median;
- compute residual as median absolute `z - support_z`;
- reject when cluster size is below `min_support_points`, residual exceeds a
  threshold tied to `support_ransac_dist_thresh`, cluster dominance is weak, or
  side coverage is below `min_support_sides`;
- preserve `SupportPlaneEstimate` fields: `support_z`, `residual`,
  `normal_alignment=1.0`, `side_coverage`, `inlier_count`, `confidence`,
  `reason`;
- keep `estimate_local_support` API stable; callers and ROS messages should
  not need schema changes.

Suggested focused tests:

- flat platform with all sides visible returns measured support near GT;
- dense lower floor outside the top-minus-support height band is ignored;
- wrong-height clutter inside annulus does not dominate unless it has a strong,
  low-residual, side-covered cluster;
- sparse flyers/outliers do not move support median beyond tolerance;
- one missing side can still pass when `min_support_sides=2`, but insufficient
  side coverage fails with `DETECT_SUPPORT_UNSTABLE`;
- too few candidates fails with `DETECT_SUPPORT_UNOBSERVABLE`;
- repeated calls on identical input are deterministic;
- `PlatformFreeDetector` still preserves same-stamp, status-gate,
  hold-track, raw-mismatch, and catalog-prior fail-closed behavior.

## Risks

- A pure `z` estimator assumes the support is near-horizontal, which matches
  the current platform-free v1 contract but should remain explicit.
- Wrong-height clutter inside the annulus must fail closed or be rejected by
  residual/cluster dominance; tests need to cover dense lower floor, sparse
  flyers, and side occlusion.
- Reducing support cost may reveal the next bottleneck in top rectangle refine
  or cargo publish; keep aggregated timing in the probe.

## Next Steps

1. Wait for reviews reply in
   `docs/agents/discuss/2026-09-05_1942_pf-r6-ransac-optimization-review.md`.
2. If confirmed, implement the support-only robust `z` estimator in
   `src/luggage_perception/luggage_perception/top_support_estimator.py`.
3. Add focused tests for flat support, wrong-height clutter, outliers, missing
   sides, insufficient candidates, deterministic output, and pipeline
   fail-closed semantics.
4. Re-run focused perception tests and `colcon build --packages-select
   luggage_perception`.
5. Send the updated checkpoint to `test/cursor/grok-4.6` for short
   PF-R6-VERIFY.
6. Only after short verification passes, commit bounded PF-R6 changes and run
   final PF-G6S plus the original PF-R5 30-trial Gate 4 regression before
   `agent_complete.sh`.

## Commands

Focused local checks after the next estimator change:

```bash
source install/setup.bash
PYTHONPATH=src/luggage_perception:$PYTHONPATH python3 -m pytest -q \
  src/luggage_perception/test/test_pf_r6_detector_instrumentation.py \
  src/luggage_perception/test/test_pf_r6_metrics.py \
  src/luggage_perception/test/test_semantic_point_filter.py \
  src/luggage_perception/test/test_platform_free_pipeline.py \
  src/luggage_perception/test/test_top_support_estimator.py
```

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select luggage_perception --symlink-install
```

Accepted-profile short verification command used by grok:

```bash
ROS_DOMAIN_ID=7 ros2 launch luggage_gazebo sim_world.launch.py \
  gui:=false use_rviz:=false use_semantic:=true use_motion:=true \
  use_vacuum:=true visual_kind:=mesh size_mode:=catalog \
  sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12 \
  yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe
```

```bash
python3 scripts/platform_free_height_gate4_eval.py \
  --out docs/status/evidence/platform_free_height/<run>/gate4_short6 \
  --trials 6 --settle-sec 4.0 --min-trials-per-size 2
```

```bash
python3 src/luggage_perception/scripts/stage_perf_probe.py --duration 120 \
  > docs/status/evidence/platform_free_height/<run>/stage_probe.json
```

Always run:

```bash
scripts/stop_sim.sh
```

## Acceptance Before Close

- Short PF-R6-VERIFY should pass first: `active_output_hz >= 4.0`,
  `top_surface_rate >= 0.95`, `full3d_rate >= 0.95`, false measured height 0,
  bounded buffers, no monotonic RSS growth, and residual 0.
- Final PF-R6 close still requires full PF-G6S on the accepted GPU profile and
  the original PF-R5 30-trial Gate 4 accuracy matrix on an exact committed
  revision.
- The PF-R6 thread must get a passing `## Result`, the bounded changes should
  be committed, and only then should `scripts/agent_complete.sh` close
  `Q-20260905-10`.

## Pointers

- `docs/agents/discuss/2026-09-05_1731_pf-r6-generation2-codex-takeover.md`
- `docs/agents/discuss/2026-09-05_1840_pf-r6-grok-validation.md`
- `docs/agents/discuss/2026-09-05_1942_pf-r6-ransac-optimization-review.md`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/metrics_extract.json`
- `docs/status/evidence/platform_free_height/2026-09-05_1923_pfr6-verify/stage_probe.json`
- `src/luggage_perception/luggage_perception/top_support_estimator.py`
- `src/luggage_perception/luggage_perception/platform_free_pipeline.py`
