# PF-R6 current problem summary (2026-09-07, generation 3)

Status snapshot after the zmode_median production implementation. All
numbers measured on this machine (Ryzen 9 9950X3D, RTX 5090, ROS Humble,
`ROS_DOMAIN_ID=7`, accepted sim profile) unless noted.

## 1. Detector side is done and verified

- Support-plane stage: RANSAC -> `zmode_median` (adopted candidate,
  dominant 8 mm z-bin cluster + median, dominant-window count-first with
  5% tie tolerance to the higher plane).
- `support_ransac_ms` p50 126.008 -> **0.604 ms** (~208x); valid
  `geometry_ms` p50 210.8 -> **65.3 ms**; support-Z error sub-micron;
  `false_measured_height` 0 in every run; 69 focused tests pass.
- The detector no longer constrains the acceptance bars. Remaining
  failures are upstream of, or beside, the detector.

## 2. Acceptance state (gate4_short6, 6 trials each)

| run | active_hz (bar 4.0) | top_rate (bar 0.95) | full3d | failed | pass |
|---|---|---|---|---|---|
| 09-07 r1 | 3.589 | 1.000 | 0.982 | 0 | yes |
| 09-07 r2 | 3.670 | 1.000 | 0.963 | 0 | yes |
| 09-07 r3 | 3.342 | **0.820** | 0.951 | **9** | **no** |
| 09-05 fresh | 3.515 | 0.680 | 1.000 | 16 | no |
| 09-05 rerun | 3.753 | 0.804 | 1.000 | 11 | no |

Two independent, intermittent failure modes; both reproduced and
captured (`failed_case_capture/`, 155 frames + images).

## 3. Mode A — cargo-cloud join loss (kills `active_output_hz`)

Signature: `DETECT_NO_CLOUD`, cargo points = 0 (139/155 captured
frames). YOLO detected the box (max conf up to 0.77), the mask held
48k-112k cargo pixels, and RGB snapshots show the box fully visible and
unoccluded — yet the detector received zero cargo points.

Mechanism (measured, 60 s with box):

- The join in `semantic_point_filter_node` is exact (sec, nanosec)
  equality — zero tolerance by construction: both streams are supposed
  to carry the preprocessor's identical primary stamp.
- Measured mask->nearest-cloud stamp agreement: **71.8% exact, the
  remaining 28.2% all >= 10 ms apart** (p50 34 ms, p95 353 ms,
  max 1.09 s). There is no 1-10 ms "near miss" population: pairs are
  either exactly co-stamped or one side simply does not exist.
- The missing side is the depth point cloud. Preprocessor diagnostics:
  `depth_ok` true only ~50% of emitted observations; filter counters
  `cloud_waiting_for_mask` 800 / `mask_waiting_for_cloud` 880 /
  stale drops ~570 / `executor_lag_sec` 0.66.

Root cause chain:

```
Gazebo /camera/depth/points (307,200-pt cloud, ~3.7 MB/frame,
generated on the sim side) is itself irregular:
  period p50 33 ms (nominal 30 Hz) but p95 132 ms, max 265 ms
  -> when no cloud arrives within the preprocessor's 20 ms slop
     (camera_slop_sec), the observation is emitted RGB-only
  -> mask exists, cloud never will for that stamp
  -> downstream exact join fails in bursts (17 clusters / 122 s)
  -> detector emits DETECT_NO_CLOUD for whole stretches
```

GPU/render-rate hypothesis tested and rejected: setting the Gazebo
camera `update_rate` 30 -> 10 Hz (temporary experiment, reverted) made
the downstream join **worse** (mask<->cloud exact-join 71.8% -> 15.2%,
`pre_cloud` 3.7 -> 1.5 Hz) while `depth_ok` went to 100%. The depth
*image* stream is healthy; the gz-generated *point cloud topic* is the
starved artifact (generation/serialization of 307k points, CPU-side,
plus bursty executor lag). GPU pre-partitioning is not applicable
(single RTX 5090; MPS does not cover Gazebo's OpenGL path; YOLO uses
only 3.5 ms/frame).

## 4. Mode B — FOV edge clipping (kills `top_surface_rate`)

Signature: `DETECT_TOP_UNOBSERVABLE`, cargo points = 3605 (16/155
frames, single spawn pose). All YOLO bboxes clamp at x2 = 640 (image
right edge): only a 30-230 px sliver of the suitcase is inside the FOV.
Mask cargo 8,582 px -> 3,605 points, all box side wall, no top plane ->
correct fail-closed rejection.

Root cause: spawn jitter (`xy_jitter_range 0.12`, yaw ±0.6 rad) versus
the `pickup_observe` FOV margin. The pose yaml already documents the
same family of failure ("0.80 m box clipped at image left" for the
previous pose). Fix belongs to pose tuning or spawn constraints, not to
perception.

## 5. Ruled-out hypotheses (with evidence)

| Hypothesis | Verdict |
|---|---|
| "mask is black -> box painted as background (floor->0 mapping)" | Display artifact: mask topic is raw mono8 label ids (cargo=2 renders ~black). Live sampling: cargo 57k px present while viewer looked empty. The floor->0 mapping risk remains theoretical but was not the observed failure. |
| instance_mask missing breaks the pipeline | No consumer (`semantic_point_filter_node` discards it explicitly). Expected for the bbox_fill backend. |
| zmode support estimator regression | No: false_measured_height 0 everywhere; support error sub-micron; failures are top-side/upstream. |
| GPU render contention as primary bottleneck | Rejected by the 10 Hz experiment (see Mode A). Depth image stream is fine at 100% `depth_ok`; the point-cloud topic is the artifact. |
| Join policy too strict (needs nearest+tolerance) | The preprocessor already does nearest-with-20 ms-slop, RGB-primary. The problem is data absence, not pairing arithmetic. |

## 6. Fix directions (owner and cost)

| # | Direction | Fixes | Owner/scope | Cost |
|---|---|---|---|---|
| 1 | Generate the world/cloud from the depth image inside the preprocessor (it already receives a healthy depth stream); stop relying on gz `/camera/depth/points` | Mode A (frame rate) | perception pipeline subtask | Medium (architecture) |
| 2 | Relax `camera_slop_sec` 20 -> ~50 ms (+ larger join buffers) as a conservative interim | Mode A partially | perception pipeline subtask | Small |
| 3 | `pickup_observe` pose margin or spawn-jitter constraint | Mode B (top rate) | sim/deployment config | Small |
| 4 | Preprocessor/segmenter executor load-shedding (0.66 s lag) | Mode A bursts | perception pipeline subtask | Medium |
| 5 | Pre-partition GPU / lower render quality | neither | rejected by experiment | - |

Hardware outlook: a real D435 has hardware-synced RGB+depth at a stable
30 Hz with no gz cloud-generation stage, so Mode A should largely
disappear; the fail-closed semantics (no cloud -> no cargo -> no height)
carry over unchanged.

## 7. Pointers

- Implementation + verification: `RESULT.md` (this directory)
- Failure captures: `failed_case_capture/ANALYSIS.md` (155 frames,
  20 RGB snapshots, per-mode analysis)
- Gate4 runs: `gate4_short6*`, `gate4_cap_r1..r3/`
- Eng note: `docs/agents/eng/2026-09-07_1830_pf-r6-gen3-zmode-implementation.md`
- Research evidence: `docs/status/evidence/platform_free_height/pf-r6-ransac-research/14c23038d0bdc0e211588d65cfb40c1cce7869a2/`
- Raw captures (outside Git):
  `/home/adamliao/work/pf_r6_ransac_data/accept_capture_2026-09-07_run2/`
