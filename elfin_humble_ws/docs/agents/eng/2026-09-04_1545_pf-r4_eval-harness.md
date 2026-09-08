# 2026-09-04 15:45 — PF-R4 Gate 4 evaluator semantics repaired

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude-code
- status: done
- parent: PFH-REMEDIATION-20260904
- subtask: PF-R4
- base_revision: 0674f84 (working tree, after PF-R3)
- started_at: 2026-09-04 15:31 local
- completed_at: 2026-09-04 15:45 local

## Summary

The Gate 4 evaluator could pass accuracy metrics with zero samples
(`p95 is None or p95 <= limit`), merged warmup/orchestration frames into
its rates, and did not filter stale instance frames. Scoring semantics
now live in a pure, unit-tested module; the ROS script is only a
collection harness.

## Changed

- `src/luggage_perception/luggage_perception/eval/gate4_scoring.py`
  (new, pure): `filter_instance` (instance_id+generation),
  `split_warmup` (documented `warmup_frames` rule, default 5 = the
  support-stability window), `aggregate` (TOP_ONLY/FULL_3D/prior/failed
  reported separately; full3d_rate measured among top-valid frames),
  `gate_pass` (required metrics with zero samples FAIL; adds support-Z,
  width, depth limits from the test plan), `active_window_hz` (splits
  the stamp sequence on gaps > gap_sec so spawn/delete orchestration
  does not dilute the rate).
- `scripts/platform_free_height_gate4_eval.py`: rewritten as the ROS
  harness — per-trial instance filtering, warmup/settled split,
  support/width/depth error rows, `active_output_hz` (active windows)
  vs `trial_cycle_hz` (end-to-end, separate), spawn-failure count,
  deterministic-pose coverage report (sizes/XY offsets actually
  covered), `--launch-params` recorded verbatim, git commit + dirty
  file count recorded into the evidence summary,
  `--allow-no-full-geometry` as the explicit negative-control mode
  (raw-only fail-closed check) that still gates top/xy/width/depth.
- `src/luggage_perception/test/test_pf_g4h_evaluator.py` (new):
  PF-G4H suite (16 tests).

## Verification

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select luggage_perception
python3 -m pytest src/luggage_perception/test/test_pf_g4h_evaluator.py -q  # 16 passed
python3 -m pytest src/luggage_perception/test/ -q   # 417 passed
python3 -m pytest src/luggage_planning/test/ -q     # 221 passed
python3 -m pytest src/luggage_packing/test/ -q      #  75 passed
# eval script parses and imports cleanly
```

## Requirement

- PF-G4H on synthetic records: zero FULL_3D frames fail; missing error
  samples fail required metrics; stale instance/generation frames are
  ignored; warmup TOP_ONLY frames are reported but excluded only by the
  documented settled-window rule; inserted spawn gaps do not reduce
  active-window Hz (3 windows at 5 Hz separated by 20 s gaps still
  measure ~5 Hz, naive rate would be ~0.6 Hz); bad support/width/depth
  estimates independently fail their limits; false measured height and
  empty settled sets fail; negative-control mode drops only the
  full-geometry requirement.

## Result

- pass: PF-G4H green (16 tests); perception 417, planning 221, packing
  75 all green.

## Pointers

- `docs/plans/platform_free_height_remediation.md` (PF-R4/PF-G4H)
- `src/luggage_perception/luggage_perception/eval/gate4_scoring.py`

## Open

- None. PF-R1..PF-R4 complete: PF-R5 (30-trial semantic accuracy) is
  unblocked and next; requires a running simulation stack.

## Rework (2026-09-04 16:0x — closure review findings 2-5)

1. **Expected identity is eval-side now.** `filter_expected_instance`
   keys on the spawn/current-box GT response id, never on detector
   output: if only stale frames arrive after a spawn they are counted
   (`stale_instance_frames`), not adopted as the new instance and
   scored against the new GT.
2. **Coverage gate added.** The coverage report now includes yaw and
   per-size trial counts, and `coverage_gate` fails Gate 4 when sizes,
   XY offsets, yaw values, trial counts, or per-size coverage are
   missing (defaults: 3 sizes / 3 offsets / 3 yaws / 30 trials / 10 per
   size, CLI-overridable).
3. **Hz is interval-based.** `active_window_hz` counts k-1 intervals
   per k-frame window over the first-to-last span (10 frames at 5 Hz
   now measure exactly 5.0 Hz; the previous frame-count/span gave
   5.56 Hz), and a sub-threshold stall inside a window correctly lowers
   the rate.
4. **Negative control is a fail-closed verdict.**
   `--allow-no-full-geometry` is replaced by
   `--negative-control-raw-only`: the control passes only when frames
   were collected, zero frames have a valid top, zero have measured
   geometry, and every frame failed with
   `DETECT_CARGO_SEGMENTATION_REQUIRED`.

PF-G4H grew to 29 tests (eval-side identity, coverage gate, exact Hz,
negative-control verdict suites). Verification after rework:
perception 432 passed.
