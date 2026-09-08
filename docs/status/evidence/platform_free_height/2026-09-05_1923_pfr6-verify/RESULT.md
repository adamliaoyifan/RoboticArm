# PF-R6-VERIFY re-run 2026-09-05 19:23

- checkpoint: `18ebc7c62d94a90f2293a78153fa7f5f5dbaa8b3` plus dirty perception
- YAML parse fix: `output.cargo_voxel_size: 0.005` moved under
  `semantic_point_filter.ros__parameters`
- outcome: blocked (`active_output_hz` 3.557 < 4.0)

## What ran

- focused pytest: 56 passed in 2.06s
- `colcon build --packages-select luggage_perception --symlink-install`
- accepted semantic launch, `ROS_DOMAIN_ID=7`
- Gate 4, 6 trials, `--min-trials-per-size 2`
- concurrent `stage_perf_probe.py --duration 120`
- `scripts/stop_sim.sh` after; residual by comm name 0

## Metrics

| metric | value | VERIFY bar |
|---|---|---|
| active_output_hz | 3.557 | >= 4.0 fail |
| top_surface_rate | 1.0 | >= 0.95 pass |
| full3d_rate | 0.961 | >= 0.95 pass |
| pca_reasons | ok 329, DETECT_NO_CLOUD 90 | report |
| valid geometry_ms | p50 210.8, p95 276.8, max 342.6 | report |
| filter process_total_ms | p50 68.6, p95 78.1, max 106.5 | report |
| filter publish_ms | p50 35.8, p95 41.4, max 62.8 | report |
| first TOP_ONLY | 192–723 ms | report |
| first FULL_3D | 1.60–2.11 s when present | report |
| detector RSS | 188068 → 279240 kB | not growing fail |
| buffers | filter occupancy <= 10; raw 10/10 | bounded pass |
| online GT | false_measured_height 0 | pass |
| stop_sim residual | 0 | pass |

Probe rates: cargo 3.68 Hz, frame 3.52 Hz, yolo 4.66 Hz, pre_rgb 4.40 Hz.
`yolo->frame` stamp delta p50 426 ms. Valid support RANSAC dominates geometry.

## Recommended TODO for eng/codex

Keep the YAML indent (voxel key inside `ros__parameters`). Raise cargo-join /
geometry throughput so Gate 4 `active_output_hz` is >= 4.0 (now 3.56 Hz;
geometry p50 ~211 ms). Recheck detector RSS over a settled window. Do not
treat this VERIFY as a PF-R6 close.
