# 2026-11-13 — Geometry updates and reusable trajectories

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: done
- parent: GEOMETRY-PLAN-REUSE-20261113
- subtask: FOUR-ITEM-INTEGRATION
- base_revision: ed2c47f434497226601a0c2ea7fc712a4db52e98
- started_at: 2026-11-13T10:19:18+08:00
- completed_at: 2026-11-13T11:27:00+08:00

## Summary

Implemented all four authorized changes. Three-box functional execution passed;
separate physical payload containment failed for boxes 2/3. This is not a fully
qualified physical baseline or a claim of calibrated geometry accuracy.

## Requirement

Use docs/plans/GEOMETRY_PLAN_REUSE_20261113.md at 6c83aeda. Harmless dimension
updates retain physical goals; real penetration must reject. No blanket deadband.

## Changed

- SelectedPoseAdmission shared by search and execution admission.
- Bounded stationary geometry observation window, separate observed seal support.
- Measured source/mount/hull-bound LiDAR structure labels and native O ingestion.
- Successful-only durable trajectory proposals with mandatory current certification.
- Closest architecture contracts and behavioural regressions updated together.

## Verification

- `bash docs/status/evidence/geometry_plan_reuse/20261113/verify_affected.sh`:
  139 passed (`verification_final.log`).
- `ctest --test-dir build-production/uld_occ_observer -R 'observer_(initial_input|keyset)_checks' --output-on-failure`:
  2 passed (`native_structure_tests_enabled.log`).
- `ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 STRUCTURE_OBSERVER_EXECUTABLE=$PWD/install-production/uld_occ_observer/lib/uld_occ_observer/observer_node python3 -m pytest -q src/luggage_perception/test/test_lidar_structure_native.py --basetemp=docs/status/evidence/geometry_plan_reuse/20261113/native_wire_02`:
  1 passed; isolated installed-O subscription probe (`native_wire_02.log`).
- After sourcing ROS and install-production,
  `python3 docs/status/evidence/geometry_plan_reuse/20261113/run_diagnostic.py --name simulation_01 --build build_01 --contacts --cycle --three-box --failure-hold-s 25 --prepick-replans 1`:
  THREE_BOX_PASS, floor/floor/stack, 3 successful path promotions, normal teardown.
- Offline `replay_diagnostic53_admission.py`: exact archived source/map and unchanged
  targets, 28/32 retained versus old 0/32; four remaining rejections violate existing
  20 mm margin after growth, not physical hull penetration.
- Offline `audit_integration.py`: 6,934 retained boundary/reference records hash
  verified; 534 installed files unchanged. Physical containment FAIL: boxes2/3
  exceed envelope by at most6.630/6.143 mm in sampled references. Raw perception
  root-cause replay lacks the full synchronized boxes2/3 sensor windows.

## Result

Development/functional acceptance passes at `4e9b9622367d33020fbcd0bef4661468418e53a8`;
later owned changes affect tests/eval documentation only. Full physical payload
qualification remains NOT_PASSED. Actual LiDAR source qualification and real-world
baseline remain NOT_EVALUATED. No margin/rounding tuned to simulator truth.
Final behavioural tests and retained integration audit are committed at
`25a10b0c86500f18352fdda529dfd83564f63a23`.

## Pointers

- docs/plans/GEOMETRY_PLAN_REUSE_20261113.md
- docs/status/evidence/geometry_plan_reuse/20261113/
- docs/status/evidence/geometry_plan_reuse/20261113/REPORT.md
- docs/status/evidence/geometry_plan_reuse/20261113/simulation_01/integration_audit.json
- docs/agents/discuss/2026-11-13_1019_geometry-plan-reuse.md

### 2026-11-13T10:58:00+08:00 — Implementation freeze before integration

Selected-pose predicate uses continuous partial-cell support integral sums and
native XYZ volume. The detector retains lateral extent/bottom, keeps current
observed contact top and raw seal support, reports bounded medians/spreads and
original stamps, rejects conflicting/out-of-order acquisitions, and ends pickup
history on vacuum-on/attached. This is repeatability handling, not absolute
body-error calibration. Successful-path cache rebinds current identity/model and
always invokes full native certification; exact start/tool endpoint keys only.
LiDAR calibration is measured-patch, source/mount/hull-bound and empirical-coverage
only; outside coverage keeps hits. O skips only labelled occupied terminal hits;
its ray, terminal position and static ULD remain. Qualification on actual sensor
observations is separate from functional filter development.

Verification so far: affected Python suite130 passed, focused predicate/geometry/
cache40 passed; native keyset/initial checks2 passed. Installed O wire gate passes:
two input rays, calibrated structure gives1 occupied hit versus2 without label;
input/receipt CDR preserved in native_wire_02. Two early collection commands
incorrectly replaced ROS PYTHONPATH and one named a nonexistent test; fixed before
scoring. Broad regression revealed an accidental module-name collision with the
existing lifecycle placement_admission; restored that file byte-for-byte and put
new predicate in selected_pose_admission. Existing legacy H3 service test also
fails with baseline query source ed2c47f4 (missing required XYZ evidence); retained
control log, no product gate relaxation. Final suite/run still pending.

One all-free 1cm synthetic149x197x148 measurement: baseline enumeration4.047s,
continuous-gate query6.156s, volume prefix.031s. Extra validation increases full
proposal cost; selected-target checks eliminate full re-enumeration. No general
latency/success-rate improvement claimed. Benchmark source/inputs/result retained.

### 2026-11-13T11:32:59+08:00 — Master integration verification

User requested merge into master. The shared workspace already uses master;
implementation `4e9b9622`, validation `25a10b0c` and lifecycle `2aeae8e2`
are all ancestors of master (`git merge-base --is-ancestor` returned0 for each).
No additional code merge is necessary. Concurrent staged/unstaged work remains
owned by its original agents and is excluded from this documentation commit.
