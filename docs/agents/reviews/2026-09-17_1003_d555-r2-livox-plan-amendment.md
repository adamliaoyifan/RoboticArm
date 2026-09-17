# 2026-09-17 -- D555 R2 Livox plan amendment

- role: reviews
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Amended the D555 simulation plan R2 text to separate lifecycle certification from stream semantics, restate the simulated RGB-D flow, and add measured Livox point-density and surface-quality monitoring expectations from real MCAP bags. The amendment keeps Livox independent from the exact RGB-D acquisition: `/livox/lidar` may be present as a separate `PointCloud2` stream in `livox_frame`, but must not be synthesized from D555 depth or exact-paired with camera data.

## Acceptance

- R2 remains a lineage/certification gate and does not alter the RGB-D path.
- DSIM-INTEGRATION reports optional Livox metrics separately from the camera stream gate.
- Livox quality monitoring includes rate, point count, finite ratio, range distribution, nearest-neighbor spacing, dominant-plane inlier ratio and residual mean/std, intensity statistics when available, and deskew state.

## Pointers

- `docs/plans/d555_sim_depth_pipeline_execution.md`
- `docs/status/evidence/d555_livox_bag_audit/20260917_1002/RESULT.md`
- `docs/agents/discuss/2026-09-09_1522_d555-sim-integration.md`
