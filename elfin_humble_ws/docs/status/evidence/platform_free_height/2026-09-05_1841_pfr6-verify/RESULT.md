# PF-R6-VERIFY 2026-09-05 18:41 — independent checkpoint validation

- checkpoint: `18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3-wt-pfr6-g2-checkpoint`
- git_commit: `18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3`
- git_dirty_files: 158 (workspace); perception dirty listed in `revision.txt`
- cargo_voxel_size intended: `0.005`
- outcome: blocked (semantic stack did not start)

## What ran

- `scripts/stop_sim.sh` before launch and after timeout
- focused pytest after `source install/setup.bash`: 56 passed in 2.10s
  (`test_pf_r6_metrics.py`, `test_pf_r6_detector_instrumentation.py`,
  `test_semantic_point_filter.py`, `test_platform_free_pipeline.py`,
  `test_top_support_estimator.py`)
- `colcon build --packages-select luggage_perception --symlink-install`: ok
- accepted semantic launch (`gui:=false use_rviz:=false use_semantic:=true
  use_motion:=true use_vacuum:=true visual_kind:=mesh size_mode:=catalog
  sequence_ids:=carryon,standard,large xy_jitter_range:=0.12,0.12
  yaw_range:=-0.6,0.6 observe_pose_name:=pickup_observe`), `ROS_DOMAIN_ID=7`

## Bottleneck

`src/luggage_perception/config/semantic_segmenter.yaml` line 110 inserts
`output.cargo_voxel_size: 0.005` as a sibling of `ros__parameters` (2-space
indent) instead of inside it (4-space indent like the other `output.*` keys).
rcl refuses the whole params file:

`Cannot have a value before ros__parameters at line 110`

`semantic_segmenter` and `semantic_point_filter` both die on that file.
`/luggage/perception/detection_frame` never published (wait ~136s). Gate 4
(6 trials) and `stage_perf_probe` were not started.

## Metrics (this run)

| metric | value |
|---|---|
| active_output_hz | n/a (no detection_frame) |
| top_surface_rate | n/a |
| full3d_rate | n/a |
| pca_reasons | n/a |
| valid geometry_ms P50/P95/max | n/a |
| filter process/publish P50/P95/max | n/a |
| first TOP_ONLY / FULL_3D latency | n/a |
| detector RSS first/last | n/a |
| bounded buffer counters | n/a |
| online GT fallback | detector source has GetCurrentBox only in a docstring; online path not exercised |
| stop_sim residual by comm | 0 |

## Recommended TODO for eng/codex

Move `output.cargo_voxel_size: 0.005` under `semantic_point_filter.ros__parameters`
(same indent as `output.cargo_points`). Confirm both semantic nodes start with
that YAML. Then re-run Gate 4 ≥6 trials plus `stage_perf_probe` on a parseable
checkpoint. Do not treat this VERIFY as a PF-R6 close.
