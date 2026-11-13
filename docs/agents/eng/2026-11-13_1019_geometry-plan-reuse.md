# 2026-11-13 — Geometry updates and reusable trajectories

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: GEOMETRY-PLAN-REUSE-20261113
- subtask: FOUR-ITEM-INTEGRATION
- base_revision: ed2c47f434497226601a0c2ea7fc712a4db52e98
- started_at: 2026-11-13T10:19:18+08:00
- completed_at: n/a

## Summary

Implementing the four user-authorized changes in order, with original observation
identity, fixed attachment and mandatory native trajectory certification.

## Requirement

Use docs/plans/GEOMETRY_PLAN_REUSE_20261113.md at 6c83aeda. Harmless dimension
updates retain physical goals; real penetration must reject. No blanket deadband.

## Changed

- SelectedPoseAdmission shared by search and execution admission.
- Bounded stationary geometry observation window, separate observed seal support.

## Verification

- Initial placement matrix: 28 passed; log under evidence pointer.

## Result

In progress. Three-box integration and LiDAR qualification not yet evaluated.

## Pointers

- docs/plans/GEOMETRY_PLAN_REUSE_20261113.md
- docs/status/evidence/geometry_plan_reuse/20261113/

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
