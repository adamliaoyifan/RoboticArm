# PF-R6 acceptance failure-case capture (2026-09-07)

Reproduction of the intermittent gate4 failures with a research capture
node (`research/pf_r6_ransac/pfr6bench/accept_capture.py`) recording, per
acquisition stamp: detection_frame outcome, same-stamp YOLO detections
(prompt/label/conf/bbox), same-stamp semantic-mask label distribution,
and an RGB snapshot. No production node modified; sim accepted profile,
`ROS_DOMAIN_ID=7`, fresh session, teardown residual 0.

## Runs

| run | active_hz | top_rate | full3d | failed | gate4_pass |
|---|---|---|---|---|---|
| r1 | 3.589 | 1.000 | 0.982 | 0 | true |
| r2 | 3.670 | 1.000 | 0.963 | 0 | true |
| r3 | 3.342 | 0.820 | 0.951 | 9 | **false** |

155 suspicious frames captured (pca_reason != ok or cargo < 10k);
122.5 s span, 17 time clusters. Two distinct failure modes.

## Mode A — DETECT_NO_CLOUD, cargo = 0 (139 frames)

- YOLO: box detected in **139/139** frames (2-4 detections, max conf
  median 0.242, up to 0.77).
- Mask: cargo pixels present in **139/139** frames (48k-112k px).
- RGB snapshot (`images/fail_022..024`): box fully visible, unoccluded,
  fully in frame (image analysis confirmed).
- Detector received **zero** cargo points for these stamps.

=> The perception results existed at the detector's stamps; the cargo
cloud never arrived. This is a **same-stamp join loss** in
`semantic_point_filter_node` (its own stats during the same session:
`cloud_waiting_for_mask` 800, `mask_waiting_for_cloud` 880,
`stale_cloud_dropped` 376, `stale_mask_dropped` 196,
`executor_lag_sec` 0.66). Bursts of stamp mismatches between the
preprocessed depth cloud and the mask kill whole stretches of frames.
This mode depresses `active_output_hz` (lost frames), not accuracy.

## Mode B — DETECT_TOP_UNOBSERVABLE, cargo = 3605 (16 frames)

All 16 frames share one spawn pose (stamps 164.3-165.5 s). YOLO bboxes:

```
[611,112,640,295] conf 0.25
[606,116,640,365] conf 0.08
[620,119,640,291] conf 0.04
```

Every bbox has **x2 clamped at 640 (the image right edge)**: only a
30-230 px sliver of the suitcase is in the FOV. Mask cargo 8582 px ->
3605 cargo points -> the visible sliver is a box side wall, no top
plane -> `DETECT_TOP_UNOBSERVABLE` (correct fail-closed behavior).

=> Placement/FOV geometry: with `xy_jitter_range 0.12` and yaw range,
some spawns place the box edge at the camera FOV boundary. This mode
depresses `top_surface_rate` (16 frames = the 0.82 run). Same family as
the yaml-documented pickup_observe history ("large 0.80 m box clipped
at image left"; now clipping right for some spawn draws). (No RGB was
joined for these stamps in the capture; the bbox clamp is the evidence.)

## Conclusions

1. Both failure modes are **outside detector geometry** — consistent
   with the gen-3 result that the detector path is no longer the
   bottleneck. `false_measured_height` stayed 0 in every run.
2. Mode A (join loss) is the `active_output_hz` blocker; fix belongs in
   `semantic_point_filter_node` stamp handling / executor scheduling
   (PF-R6 adjacent scope, currently instrumented but not fixed).
3. Mode B (FOV clipping) is the `top_surface_rate` blocker; fix belongs
   in spawn jitter vs pickup_observe FOV margin (pose tuning or spawn
   constraint), not in perception.
4. The visualizer "black mask" question is closed separately: the mask
   topic publishes raw mono8 label ids (cargo=2 renders black);
   live sampling showed cargo 57k px present while the viewer looked
   empty.

## Files

- `failed_cases.jsonl` — 155 rows (stamp, pca_reason, n_cargo, yolo
  dets, mask label dist)
- `images/` — 20 representative RGB snapshots (mode A)
- gate4 runs: `../gate4_cap_r1..r3/`
- raw capture (incl. per-run jsonl): outside Git at
  `/home/adamliao/work/pf_r6_ransac_data/accept_capture_2026-09-07_run2/`
