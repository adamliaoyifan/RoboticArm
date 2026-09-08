# 2026-09-07 -- PF-R6 gen3 CURRENT_ISSUES review: Mode B misdiagnosed, Mode A cause is preprocessor throughput

- role: reviews
- agent: cursor
- model: opus-5
- cli: cursor
- status: done

## Summary

Re-derived both failure modes in
`docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/CURRENT_ISSUES.md`
from the captured evidence. The detector conclusion (section 1) holds. Two of
the five fix directions target the wrong cause:

1. **Mode B is not FOV clipping.** All 16 `DETECT_TOP_UNOBSERVABLE` frames plus
   26 of the 139 `DETECT_NO_CLOUD` frames have **no detection on the suitcase
   at all**. The only cargo detection is a static false positive on a fixed
   structure at the image right edge, whose bbox is byte-identical
   (`[611,112,640,295]`, conf 0.25) at `t=54 s` and `t=164-168 s` across
   different spawns. A spawn-jitter artefact cannot repeat an identical bbox
   110 s apart. Fix direction 3 (`pickup_observe` pose margin / spawn-jitter
   constraint) addresses a non-problem.
2. **Mode A's dominant loss is the preprocessor, not gz cloud jitter.**
   `stage_probe.json` measures `raw_img` 20.97 Hz -> `pre_rgb` 3.63 Hz (83 %
   of frames never emitted) with `raw_img->pre_rgb` p50 **248.7 ms** / p95
   428 ms. `sensor_preprocessor_node` runs `rclpy.spin` (single-threaded, one
   default callback group) over ~113 callbacks/s and performs ~5 full
   float64 passes over 307 k points per cloud. At the filter topic boundary
   the exact join already succeeds for 397/477 clouds, so fix direction 2
   (`camera_slop_sec` 20 -> 50 ms) cannot recover the missing 17 Hz.

## Findings

### F1 -- persistent border false positive disables the temporal hold

`confidence_threshold: 0.04` admits the right-edge structure at conf
0.22-0.25; the `bbox_fill` backend paints it as ~8.5 k cargo pixels. Because
`DetectionTemporalGate.apply` computes `had = largest_cargo_bbox(...) is not
None` over all cargo detections, the ever-present false positive makes every
frame "positive", so the `if had: return` early exit means the consensus hold
**can never fire** (`held: false` in all 155 captured frames). Worse, the
largest-area bbox alternates between the suitcase and the disjoint false
positive, giving `bbox_iou == 0 < bbox_iou_reset` and clearing the window on
every alternation. The one mechanism built for YOLO flicker is defeated by the
false positive.

Real-suitcase detection confidence is at the noise floor: n=120 non-border
cargo detections, min 0.042, p25 0.124, p50 0.240, max 0.863; 55/120 below
0.20; the suitcase drops out entirely in 42/155 frames.

### F2 -- `camera_slop_sec` conflates pairing tolerance with a wait deadline

In `sensor_preprocessor._window_ready`, `cloud_ready` includes
`(now_hint - rgb_stamp) >= self.camera_slop_sec`, and `now_hint` is the max
stamp over **all** buffers including `/joint_states` at 50 Hz. One joint
message 20 ms after the RGB stamp declares the cloud dead. `self._emitted`
then makes the decision permanent, so a correctly co-stamped but late cloud is
discarded even though `camera_horizon_sec` 0.35 still holds it. The pairing
tolerance and the wait deadline need to be separate parameters: gz
`rgbd_camera` co-stamps colour/depth/points from one sensor, so the correct
pairing tolerance is ~exact while the deadline should exceed the observed
p95 arrival lag.

### F3 -- the pipeline destroys and then reconstructs pixel identity

gz packs a 307 k-point cloud (3.7 MB/frame) -> bridge -> preprocessor
deserialises, `isfinite`, TF-transforms and copies it in float64 -> republishes
3.7 MB -> `semantic_point_filter` deserialises and **re-projects every point to
`(u,v)` via `fx,fy,cx,cy`** to look up its mask label. That reprojection
inverts a projection that the organised depth image never needed. Applying the
mask in pixel space and deprojecting only the ~50 k cargo pixels removes two
3.7 MB serialisations, the 307 k-point float64 TF pass, and the reprojection.

### F4 -- cheaper items on the same path

- `float64` in `update_camera_cloud` doubles memory traffic over the float32
  wire format; float32 costs ~0.2 um at 2 m, far below the sub-micron support-Z
  claim's own margin.
- Preprocessor inputs are RELIABLE depth=5. For 3.7 MB/frame from a bursty
  producer that queues up to ~18 MB and turns producer jitter into drain
  bursts; sensor data wants BEST_EFFORT depth=1-2.
- `semantic_point_filter_node._publish_stats` runs `json.dumps(sort_keys=True)`
  on a nested dict and publishes RELIABLE + TRANSIENT_LOCAL on **every** cloud
  and mask callback (~60-70 Hz), inside the same exclusive callback group as
  the join.
- `_take_newest_join` clears both buffers on every join
  (`stale_cloud_dropped` 376 / `stale_mask_dropped` 196). That is a symptom
  mitigation for the queue backlog F2/F3 create, not an independent defect.

### F5 -- an unverified premise worth one cheap measurement

CURRENT_ISSUES states gz `/camera/depth/points` "period p50 33 ms, p95 132 ms"
without saying whether that is header-stamp or arrival-time period. With
`raw_img` already at 20.97 Hz against a nominal 30 Hz, stamp-vs-arrival period
decides whether gz genuinely drops frames or the RELIABLE backlog (F4) is
producing the bursts. Fix direction 1 is a medium architecture change; this
measurement should precede it.

### F6 -- claims in the document that the capture does not support

- "YOLO detected the box (max conf up to 0.77)" and "the mask held 48k-112k
  cargo pixels" count any label-2 detection as the suitcase. At `t=54-59 s` the
  label-2 detections are the border false positive and the mask holds 8.6 k px.
  The cargo-pixel histogram is bimodal: 50 frames under 10 k px (false positive
  only), 105 frames over 40 k px.
- "single spawn pose" for Mode B is consistent with the frames but the shared
  identical bbox shows the pose is the *camera-static* structure, not the spawn.
- ANALYSIS.md notes "No RGB was joined for these stamps" for Mode B, so the
  FOV-clipping conclusion rested on the bbox clamp alone. The Mode A RGB
  snapshots at the identical bbox show the suitcase well inside the frame
  (`[178,144,405,297]` at `t=59.467 s`) while the clamped bbox sits on a
  separate right-edge structure.

## Recommendation

Re-scope the fix table. Ordered by (blocker relieved) / (risk x cost):

| # | Action | Relieves | Risk |
|---|---|---|---|
| A | Reject cargo detections whose bbox touches the image border, or require them to project into the pickup-platform region; raise the floor for border boxes | Mode B entirely, 26 Mode A frames | Low, config plus a predicate |
| B | Make `had_cargo` in `DetectionTemporalGate` depend on an accepted suitcase-region detection so the hold can fire, and reset the window on scene change rather than largest-bbox IoU flips | the 42/155 suitcase dropouts | Low, unit-testable without sim |
| C | Split `camera_slop_sec` into `camera_pair_tolerance_sec` (~exact) and `camera_wait_deadline_sec` (> observed p95 arrival lag); do not emit before the deadline | Mode A yield | Low; costs latency, not rate, and the bar is throughput |
| D | Move the preprocessor to a `MultiThreadedExecutor` with the cloud in its own callback group, drop cloud math to float32, remove the redundant observation copy, and set sensor inputs BEST_EFFORT depth=1 | the 248 ms hold and 21 -> 3.6 Hz collapse | Low-medium, no architecture change |
| E | Decimate the cloud (stride 2 -> 320x240) in the preprocessor; the filter's `(u,v)` reprojection already indexes the full-res mask, so no downstream change | 4x cost on the whole cloud path | Low; verify top-plane point count stays sufficient |
| F | Then, if still short, mask in pixel space and deproject only cargo pixels (F3) | remaining Mode A | Medium; needs a `sensor_data_pipeline.md` amendment |

Fix direction 3 in CURRENT_ISSUES should be dropped, and fix direction 2
replaced by C. The hardware outlook paragraph stays valid for Mode A but does
**not** cover Mode B, which is a detector-side false positive that a real D435
will reproduce.

## Risks

- A, B, and E are testable offline against the existing capture and the 16-pose
  sweep; they do not need a sim run to falsify.
- C changes emission timing and may shift `active_output_hz` measurement
  windows; re-baseline gate4 rather than comparing against the 09-05 runs.
- D touches concurrency in a node whose comments record a prior
  thread-explosion incident in the sibling filter node; keep the cloud group
  exclusive.
- This note is a review finding only. It does not claim PF-R6 or dispatch
  runnable work; the owner recorded in `Q-20260907-1` still owns generation 3.

## Pointers

- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/CURRENT_ISSUES.md`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/stage_probe.json`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/failed_case_capture/failed_cases.jsonl`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/failed_case_capture/images/fail_003_54.318000000.png`
- `src/luggage_perception/luggage_perception/sensor_preprocessor.py`
- `src/luggage_perception/luggage_perception/detection_temporal_gate.py`
- `src/luggage_perception/scripts/sensor_preprocessor_node.py`
- `src/luggage_perception/scripts/semantic_point_filter_node.py`
- `docs/agents/discuss/2026-09-07_1755_2026-09-07_pf-r6-gen3-reassignment-request.md`
