# 2026-09-04 15:30 — PF-R3 stamped geometry status + acquisition-stamp TF

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R3
- base_revision: 0674f84 (working tree, after PF-R2)
- started_at: 2026-09-04 15:16 local
- completed_at: 2026-09-04 15:30 local

## Summary

Replaced the `None -> geometry_ok=True` behavior with a stamped status
gate: preprocessor `flags.geometry_ok` is only evidence for the
acquisition named by `last_geometry_ok_stamp`, and missing/malformed/
stale status yields TOP_ONLY with its own machine reason — never a
MEASURED_SUPPORT height. The semantic filter's TF lookup now uses the
acquisition stamp with no latest-TF fallback.

## Changed

- `src/luggage_perception/luggage_perception/top_support_estimator.py`:
  new codes `DETECT_SUPPORT_STATUS_MISSING/MALFORMED/STALE`.
- `src/luggage_perception/luggage_perception/platform_free_pipeline.py`:
  new pure `GeometryStatusGate` (stamp tolerance default 0.5 s — absorbs
  one preprocessor frame of status lag; JSON float64 round-trip error is
  sub-nanosecond and negligible; receipt age bound default 1.0 s wall);
  `PlatformFreeDetector.update(..., geometry_gate_reason=...)` routes
  the gate label through the support gate and surfaces it as
  `BoxGeometryEstimate.reason`; `GATE_SUPPORT_REASONS` maps every gate
  label to a machine-readable reason.
- `src/luggage_perception/scripts/luggage_detector_node.py`: `_on_status`
  stores the parsed payload + wall receipt (unparseable JSON stored raw
  so the gate reports MALFORMED, not MISSING); `_pca_from_cloud_msg`
  evaluates the gate at the cloud acquisition stamp; new params
  `geometry_status_stamp_tolerance_sec` / `geometry_status_max_age_sec`.
- `src/luggage_perception/scripts/semantic_point_filter_node.py`:
  `_lookup_rt` uses `Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)`
  with a bounded 50 ms wait; a missing stamped TF is an explicit miss
  (`tf_miss` counter path) with no latest-TF retry.
- `src/luggage_perception/test/test_pf_g2a_stamped_status_tf.py` (new):
  PF-G2A suite (14 tests).

## Verification

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select luggage_perception
python3 -m pytest src/luggage_perception/test/ -q   # 401 passed
# (same result with cwd=src/luggage_perception)
```

## Requirement

- PF-G2A: matching fresh status+stamp permits measured support;
  missing/malformed/false/stale each forbid it with a distinct reason;
  one-nanosecond cargo/raw key mismatch does not fuse (ExactStampJoin
  test); semantic TF lookup receives the acquisition stamp (recorded
  nanoseconds verified) and a missing stamped TF does not retry latest
  TF; hold_track stays top-only even with fresh status; a stale-status
  frame resets the stability window so a later valid frame cannot
  relabel an earlier estimate as current measurement.

## Result

- pass: PF-G2A green (14 tests), perception regression 401 passed.

## Pointers

- `docs/plans/platform_free_height_remediation.md` (PF-R3/PF-G2A)
- `src/luggage_perception/test/test_pf_g2a_stamped_status_tf.py`

## Open

- None.

## Rework (2026-09-04 16:0x — closure review finding 1)

The 0.5 s stamp tolerance let status for acquisition N-1 authorize
support fitting for acquisition N: the robot can start moving between
frames, so that was not same-acquisition evidence. Reworked to an
exact-stamp join:

- the preprocessor status payload now carries
  `primary_stamp_sec`/`primary_stamp_nanosec` (exact integers; float64
  ulp at the ROS epoch is ~0.24 us, so a float alone cannot be compared
  exactly across producers);
- `GeometryStatusGate` buffers payloads keyed by their exact
  (sec, nanosec) acquisition stamp (bounded, default 16). Only the
  entry for the cloud's own stamp can authorize support fitting:
  N-1 (newest buffered stamp older than the cloud) -> `status_stale`,
  out-of-order/empty -> `status_missing`, unusable payload ->
  `status_malformed`, entry with `geometry_ok=false` ->
  `geometry_not_settled`. The 0.1 s minimum-tolerance clamp is gone
  (float fallback epsilon is 1 us, representation error only);
- PF-G2A grew: `test_n_minus_1_status_cannot_authorize_n`,
  `test_out_of_order_status_fails_closed`,
  `test_float_fallback_covers_representation_only`,
  `test_buffer_is_bounded`,
  `test_exact_match_beats_recent_malformed` (16 tests total).

Verification after rework: perception 432 passed (planning 221,
packing 75 unchanged).
