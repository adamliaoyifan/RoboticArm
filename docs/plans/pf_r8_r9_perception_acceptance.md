# PF-R8 / PF-R9 — perception acceptance after PF-R6 gen3

Date: 2026-09-07 (reviews decomposition, `cursor/opus-5`)
Parent: `PFH-REMEDIATION-20260904`
Base revision: `f34d9171f8448d7bf92ce84897dc188a110216e2`

## 1. Why this plan exists

PF-R6 generation 3 removed the detector from the critical path
(`support_ransac_ms` p50 126.0 -> 0.604 ms, `false_measured_height` 0 in every
run). `gate4_short6` still fails intermittently, and
`docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/CURRENT_ISSUES.md`
attributes the two remaining modes to (a) gz point-cloud jitter against a 20 ms
slop and (b) `pickup_observe` FOV margin versus spawn jitter.

Re-deriving both from the captured evidence
(`docs/agents/reviews/2026-09-07_1956_pf-r6-gen3-current-issues-review.md`)
gives different causes. This plan is scoped to the corrected causes only.

### Correction 1 — the `top_surface_rate` failure is a detection failure

All 16 `DETECT_TOP_UNOBSERVABLE` frames and 26 of the 139 `DETECT_NO_CLOUD`
frames contain **no detection on the suitcase at all**. The only cargo
detection is a static false positive on a fixed structure at the image right
edge, with a byte-identical bbox `[611,112,640,295]` at conf 0.25 at both
`t=54 s` and `t=164-168 s`. Spawn jitter cannot reproduce an identical bbox
110 s apart across different spawns; the structure is static in the camera
view. The Mode A RGB snapshots at that identical bbox show the suitcase well
inside the frame (`[178,144,405,297]` at `t=59.467 s`), on a separate object.

This is the already-documented `base_link` false positive
(`docs/agents/eng/2026-09-04_1755_pf-r5-online-accuracy.md`: "dropped from
cargo by the workspace crop; self-body masking of the pedestal remains a
follow-up"). The deferred follow-up is now the blocking cause.

### Correction 2 — the false positive's harm is slot occupancy, not bad geometry

`_crop_workspace` in `top_support_estimator` still discards the false
positive's points, so `false_measured_height` stays 0 and fail-closed holds.
The damage is upstream of geometry:

`DetectionTemporalGate.apply` sets
`had = largest_cargo_bbox(...) is not None` over **all** cargo detections. A
permanently present false positive makes every frame a positive sample, so the
`if had: return` early exit means the consensus hold **can never fire**
(`held: false` in all 155 captured frames). The largest-area bbox also
alternates between the suitcase and the disjoint false positive, so
`bbox_iou == 0 < bbox_iou_reset` clears the window on every alternation. The
mechanism built for YOLO flicker is disabled by the false positive, and the
suitcase drops below `confidence_threshold: 0.04` entirely in 42/155 frames
(n=120 in-region detections: min 0.042, p25 0.124, p50 0.240).

**Which bar each subtask owns.** `active_output_hz` is computed by
`scoring.active_window_hz(all_stamps)` over the header stamp of **every**
collected `DetectionFrame` (`scripts/platform_free_height_gate4_eval.py:285`),
and the detector publishes a frame even when PCA is invalid. So an empty cargo
mask does not reduce `active_output_hz`; the measured 3.34-3.75 Hz simply
tracks the preprocessor's 3.63 Hz emission rate. Therefore:

- PF-R8 owns `top_surface_rate` and cannot move `active_output_hz`.
- PF-R9 primarily owns `active_output_hz`. It also *raises* `top_surface_rate`
  as a side effect, because a frame with no cloud fails closed and can never
  produce a valid top surface; restoring cloud availability restores those
  frames. PF-R9 must not be scored as if it were `top_surface_rate`-neutral.

Suppression (A1) and hold repair (A2) are nevertheless one subtask because the
accept predicate *defines* the gate's notion of a positive sample: A2 is not
specifiable, let alone testable, until A1 exists, and both live in the same two
files. Splitting them would hand one owner a predicate and another owner the
only consumer of it.

### Correction 3 — the `active_output_hz` failure is preprocessor throughput

`stage_probe.json` measures `raw_img` 20.97 Hz -> `pre_rgb` 3.63 Hz, i.e. 83 %
of frames are never emitted, with `raw_img->pre_rgb` p50 **248.7 ms** / p95
428 ms. At the filter's topic boundary the exact join already succeeds for
397/477 clouds. The loss is inside `sensor_preprocessor_node`, which runs
`rclpy.spin` (single-threaded, one default callback group) over ~113
callbacks/s while performing ~5 full float64 passes over 307 k points per
cloud (`asarray` -> `isfinite` index -> TF transform -> `_build_if_ready`
`copy=True` -> `emitted.copy()`), roughly 37 MB of allocation per cloud frame.

That CPU attribution is a code re-derivation, not a `stage_probe.json`
measurement; the probe establishes only *where* the loss is, which is why B1
below makes the discriminating measurement a first-class deliverable.

Separately, `camera_slop_sec` conflates two different quantities. In
`_window_ready`, `cloud_ready` includes
`(now_hint - rgb_stamp) >= self.camera_slop_sec`, and `now_hint` is the max
stamp over **all** buffers including `/joint_states` at 50 Hz. One joint
message 20 ms after the RGB stamp declares the cloud dead, and `self._emitted`
makes that permanent even though `camera_horizon_sec` 0.35 still holds the
data. Because gz `rgbd_camera` co-stamps colour/depth/points from one sensor,
the pairing tolerance should be ~exact while the wait deadline should exceed
the measured p95 arrival lag. Widening the single parameter to 50 ms (the
original fix direction 2) would loosen pairing to +/-50 ms of arm motion while
still losing the tail.

## 2. Constraints (normative, do not relax)

1. `docs/architecture/sensor_data_pipeline.md`: tolerance pairing lives **only**
   in the preprocessor. Consumers keep the exact `(sec, nanosec)` join. Do not
   add slop to `semantic_point_filter_node`.
2. Every fail-closed gate stays: no cloud -> no cargo -> no height; raw-only
   fails closed; catalog priors keep `height_valid=false`.
3. No online GT or spawner geometry reads in online nodes.
4. Stamped TF only, no latest-TF fallback (`.cursor/rules/sensor-frames-and-timing.mdc`).
5. Missing data is flagged, not faked (`ObservationFlags`, never an empty array).
6. `false_measured_height` must stay 0. A change that raises availability by
   admitting a wrong plane is a failure, not a trade.
7. Agent-run sims: `gui:=false use_rviz:=false`, `ROS_DOMAIN_ID=7`, PID to
   `/tmp/elfin_humble_sim.pid`, and `scripts/stop_sim.sh` when the driver exits
   (`.cursor/rules/sim-lifecycle.mdc`).

## 3. Out of scope

- **Pixel-space masking** (apply the mask to the organised depth image and
  deproject only cargo pixels, removing two 3.7 MB serialisations, the 307 k
  float64 TF pass, and the filter's `(u,v)` reprojection). This is the largest
  available win but changes the preprocessor's published contract and needs a
  `docs/architecture/sensor_data_pipeline.md` amendment. Recorded as the
  contingency if PF-R9 misses its bar; it is not authorised by this plan.
- `pickup_observe` pose retuning and spawn-jitter constraints. Correction 1
  removes the motivation; do not spend time here.
- Gate 4 bar renegotiation. `docs/plans/platform_free_height_gate4_revision.md`
  is still an unconfirmed proposal and is not a licence to lower a bar.

## 4. Subtasks

|     ID     | Owner agent/model | Depends on | Bounded scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| PF-R8 | `claude/glm-5.3` | PF-R6 | Cargo detection availability: suppress the static/border false positive and repair the temporal hold. Files: `luggage_perception/detection_temporal_gate.py`, `luggage_perception/semantic_segmenter.py`, `config/semantic_segmenter.yaml` | A1-A5 below | Focused unit tests + offline replay of the 155-frame capture + vintage-pose regression |
| PF-R9 | `claude/glm-5.3` | PF-R6 | Preprocessor throughput and cloud-wait semantics. Files: `luggage_perception/sensor_preprocessor.py`, `scripts/sensor_preprocessor_node.py`, `config/sensor_preprocessor.yaml`, stats throttle in `scripts/semantic_point_filter_node.py` | B1-B6 below | Focused preprocessor unit tests + a >=60 s live stage probe |
| PF-R10 | `claude/glm-5.3` | PF-R6,PF-R8,PF-R9 | Integration: whole-chain gate4 re-baseline on one committed revision | C1-C3 below | `gate4_short6` x3 + PF-G6S lifecycle |

### Why `depends_on: PF-R6`, and what the base revision actually is

PF-R8 and PF-R9 touch files disjoint **from each other** and may run in either
order. They are **not** disjoint from PF-R6 generation 3's uncommitted work.
Measured against `f34d917`, two files inside their declared scopes are already
dirty with PF-R6 changes:

- `config/semantic_segmenter.yaml` (PF-R8 scope), including the uncommitted
  `output.cargo_voxel_size`.
- `scripts/semantic_point_filter_node.py` (PF-R9 scope), including the PF-R6
  instrumentation and the `_publish_stats` block that PF-R9's B5 throttles.

`f34d917` is therefore the base of this plan's *analysis*, not a legal
implementation base. The implementation base is **PF-R6 generation 3's passing
commit**, which does not exist yet (`git_dirty_files` 183 in the gen-3
evidence). The owner MUST record that commit as the base in its Claim and
re-derive the B3/B4 baseline numbers on it, because the PF-R6 changes to
`semantic_point_filter_node.py` already alter the join path PF-R9 measures.

`claude/glm-5.3` owns PF-R6 generation 3 as well, so this is a commit boundary
for a single owner rather than a cross-agent wait. It is declared as a real
dependency so `agent_start.sh` enforces it instead of leaving it in prose.

### PF-R8 acceptance

- **A1** A cargo detection is rejected unless it is plausibly the payload.
  The predicate is the owner's choice inside scope. Measure it against a
  frozen labelled fixture derived from the 155-frame capture and committed
  alongside the test, in the style of `test/fixtures/` used by
  `test_vintage_pose_regression.py`:
  - expected negatives: the 184 border-touching detection instances, whose
    static identity is established by the recurring bboxes
    (`[611,112,640,295]` x42, `[620,119,640,291]` x42, `[611,111,640,296]`
    x32, `[606,116,640,365]` x27, `[606,117,640,368]` x21,
    `[606,116,640,367]` x20);
  - expected positives: the 120 non-border cargo detections;
  - required metric: **0 false accepts and 0 false rejects** on that fixture.
  The 20 frames that have RGB snapshots MUST have their labels visually
  confirmed; the remaining labels rest on bbox identity, and the note must say
  so. Candidate signals: back-projection outside the pickup workspace XY
  extent already known to `top_support_estimator`, static-across-spawns
  persistence, or an image-border test. State the chosen predicate.
- **A2** `DetectionTemporalGate` treats "this frame saw cargo" as "saw an
  **accepted** cargo detection", so a surviving false positive can no longer
  short-circuit the hold, and window reset is driven by scene change rather
  than a largest-bbox identity flip between two disjoint objects.
- **A3** Offline replay, in two parts, because the capture stores detection
  metadata rather than the RGB frames and label maps that
  `DetectionTemporalGate.apply` consumes:
  - **A3a (deterministic)** Drive the gate with the 155-frame detection
    sequence from `failed_cases.jsonl`, reconstructing each label map from the
    captured bboxes. This is faithful because the captured backend is
    `bbox_fill:yolov8s-world.pt`, for which the label map *is* the union of
    bbox rectangles. Frames without an RGB snapshot use a no-scene-change
    signal, and the note must record that assumption. Required: each of the 16
    `DETECT_TOP_UNOBSERVABLE` frames and the 26 false-positive-only
    `DETECT_NO_CLOUD` frames yields a held cargo bbox whose IoU against the
    nearest accepted in-region detection within +/-5 frames is **>= 0.5**.
    Report accepted / held / empty counts before and after.
  - **A3b (live)** One fresh capture run that records per-frame
    accepted/held/empty, showing **0** frames whose only cargo mask originates
    from an unaccepted detection.
- **A4** Detection recall does not regress. Baseline is the PF-R5 official
  run8 flat-STL frame recall **0.9606** recorded in
  `docs/status/evidence/platform_free_height/2026-09-04_2110_pfr5-g4s-run8-official/RESULT.md`;
  recall is (settled frames with an accepted cargo detection) / (settled
  frames). `src/luggage_perception/test/test_vintage_pose_regression.py` must
  still pass. `confidence_threshold` stays 0.04 unless the note argues the
  change.
- **A5** New focused tests, GPU-free, including: hold still fires while a
  persistent unaccepted detection is present; the accept predicate rejects the
  captured false-positive bbox and accepts `[178,144,405,297]`; window reset
  does not trigger on a suitcase/false-positive alternation.

### PF-R9 acceptance

- **B1** Record the missing measurement first: header-stamp period **versus**
  arrival-time period for `/camera/depth/points` and `/camera/color/image_raw`
  over >= 60 s, in evidence. This decides whether gz drops frames or the
  RELIABLE depth=5 backlog manufactures the bursts, and it is the evidence
  that either justifies or retires the original fix direction 1.
- **B2** Split `camera_slop_sec` into an exact-ish pairing tolerance and a
  separate wait deadline. A cloud that is co-stamped but late (within the
  deadline) MUST be attached; a cloud outside the pairing tolerance MUST NOT
  be attached to a different RGB stamp. Keep both bounded by
  `camera_horizon_sec`.
- **B3** Emitted-observation rate >= 0.8 x input `raw_img` rate, and
  `raw_img->pre_rgb` p50 <= 60 ms, on the same stage probe used for B1.
- **B4** `cloud_ok` true on >= 95 % of emitted observations (denominator:
  observations emitted by the preprocessor over the probe window). From the
  `semantic_point_filter` stats over the same window: exact-join success
  `joined / cloud` >= 0.95, and stale drops
  `(stale_cloud_dropped + stale_mask_dropped) / (cloud + mask)` < 0.05.
- **B5** Cost reductions are in scope and expected: multi-threaded executor
  with the cloud in its own callback group (keep it exclusive — the sibling
  filter node records a prior thread-explosion incident), float32 cloud math,
  removal of the redundant whole-observation copy, BEST_EFFORT depth=1 sensor
  inputs, optional cloud decimation, and a timer-throttled stats publish
  instead of one `json.dumps` per sensor callback. Any subset that meets
  B3/B4 is acceptable; justify what was skipped.
- **B6** Existing preprocessor unit tests pass, plus new tests for the split
  parameters: late-but-co-stamped cloud attached; out-of-tolerance cloud not
  attached; RGB-only emission still flagged (`cloud_ok=False`), never faked.

### PF-R10 acceptance

- **C1** `gate4_short6`, 6 trials, 3 consecutive runs on one committed
  revision with `dirty=0`: `active_output_hz` >= 4.0, `top_surface_rate`
  >= 0.95, FULL_3D rate >= 0.95, `false_measured_height` == 0, `failed` == 0
  in **all three** runs. Two-of-three is not a pass; the failures being fixed
  are intermittent.
- **C2** PF-G6S as defined in `docs/plans/platform_free_height_remediation.md`
  ("Focused gate PF-G6S"): accepted semantic geometry output >= 4 Hz on the
  accepted GPU profile, with per-stage P50/P95/max and active-window
  end-to-end latency reported. The parent's "buffers remain bounded" and
  "callback lag and RSS do not grow monotonically" are qualitative, so this
  plan makes them decidable for this run. Let Q1 and Q4 be the first and last
  quartiles of the run by wall time. Pass requires **all** of:
  - every bounded buffer's peak `buffer_occupancy` <= its configured maxlen
    (`camera_maxlen` 10, filter `buffer_maxlen` 10, `join_buffer_maxlen` 10,
    `geometry_status_buffer_maxlen` 16). The 0.5 x maxlen mean rule applies
    **only to pending-work buffers** — the filter's `_clouds` / `_masks` and
    the detector's join buffers — whose Q4 mean occupancy must be
    <= 0.5 x maxlen, because occupancy pinned at maxlen there is a backlog.
    It does **not** apply to the preprocessor's `camera_*` ring buffers,
    which are deliberate history: `camera_horizon_sec` 0.35 at ~21 Hz fills
    roughly 7 of 10 slots with no backlog at all;
  - `executor_lag_sec`: Q4 mean <= 0.20 s **and** <= 1.25 x Q1 mean;
  - RSS per online perception node: Q4 mean <= 1.10 x Q1 mean + 50 MiB;
  - residual process count exactly 0 after `scripts/stop_sim.sh`.

  These thresholds tighten the parent gate; they do not relax it. If a
  threshold proves wrong on measurement, report the measured value and raise
  it as an amendment rather than silently rescoring.
- **C3** Evidence under `docs/status/evidence/platform_free_height/<run>/`
  recording the exact commit and dirty-file count.

PF-R10 is the integration subtask for *this* plan. It does not replace PF-R7
(`Q-20260904-2`), which remains the parent-level independent E2E audit owned by
`cursor/grok-4.6`.

## 5. Risks

- A1's predicate is the one place this plan can silently over-reject. A
  workspace-projection predicate is safer than a pure border test, because a
  legitimately edge-clipped suitcase should still be accepted; a border test
  alone would fail closed on a real clipping case. Prefer the projection test
  and keep the border signal as a diagnostic.
- B2 changes emission timing, so `active_output_hz` measurement windows shift.
  Re-baseline against fresh runs; do not compare against the 09-05 numbers.
- B5 touches concurrency. The `semantic_point_filter_node` comments record a
  MultiThreadedExecutor + Reentrant group combination that spawned ~70 threads
  and froze `/stats_json`. Keep the cloud group exclusive.
- A3 and A5 are offline and can falsify PF-R8 without a sim run. Do not open a
  sim to test PF-R8.
- If PF-R9 meets B1/B2/B6 but misses B3/B4, that is the trigger to raise the
  out-of-scope pixel-space masking change as a new consensus item, not to
  widen scope in place.

## 6. Pointers

- `docs/agents/reviews/2026-09-07_1956_pf-r6-gen3-current-issues-review.md`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/CURRENT_ISSUES.md`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/stage_probe.json`
- `docs/status/evidence/platform_free_height/2026-09-07_pfr6_gen3/failed_case_capture/`
- `docs/plans/platform_free_height_remediation.md`
- `docs/plans/platform_free_height_gate4_revision.md`
- `docs/agents/eng/2026-09-04_1755_pf-r5-online-accuracy.md`
- `docs/architecture/sensor_data_pipeline.md`
