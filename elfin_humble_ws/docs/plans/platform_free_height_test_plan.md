# Platform-free Height Estimation - Test and Acceptance Plan

Date: 2026-09-04

Companion implementation plan:
`docs/plans/platform_free_height_eng_todo.md`

Post-evaluation remediation plan:
`docs/plans/platform_free_height_remediation.md`

## Incremental Remediation Checkpoints

The initial E0-E5 implementation was evaluated together at `0674f84-wt`.
Follow-up work uses these smaller checkpoints so focused testing starts as soon
as each independent correction is available:

| ID | Eng deliverable | Focused gate | Prerequisites | Blocks |
|---|---|---|---|---|
| PF-R1 | Complete ROS adapter validity/top-surface contract | PF-G0A | E0 messages | PF-R5, PF-R7 |
| PF-R2 | Raw-only/auto input fails closed | PF-G3A | E2 pipeline | PF-R5, PF-R7 |
| PF-R3 | Stamped status and semantic TF | PF-G2A | E2 exact-stamp join | PF-R5, PF-R7 |
| PF-R4 | Correct accuracy/rate evaluator and placement matrix | PF-G4H | Eval-only GT | PF-R5, PF-R6, PF-R7 |
| PF-R5 | Semantic 30-trial online accuracy | PF-G4S | PF-R1-PF-R4 pass | PF-R6, PF-R7 |
| PF-R6 | Profile/optimize accepted semantic path | PF-G6S | PF-R4, PF-R5 pass | PF-R7 |
| PF-R7 | Full G0-G4/G6 regression | PF-E2E | PF-R1-PF-R6 pass | Engineering release |

Focused gate definitions and execution order are normative in the remediation
plan. Gate 5 remains the additional hardware-deployment gate.

## Evidence Contract

Each non-unit run stores raw outputs under:

```text
docs/status/evidence/platform_free_height/<run_id>/
```

Required artifacts:

- exact launch/profile and command;
- code revision/worktree identity;
- parameter snapshot;
- per-frame JSONL with acquisition stamp, geometry level, source, fit metrics,
  estimate, reference, and error;
- aggregate summary with count, success rate, P50/P95/max, and failure reasons;
- logs needed to prove online nodes did not consume eval truth.

Test role notes point to evidence; they do not copy raw output.

## Gate 0 - Build and Interface

Checks:

- Build `luggage_msgs`, `luggage_perception`, `luggage_planning`, and affected
  bringup/eval packages.
- Run existing package tests before new tests.
- Verify `DetectedLuggage` can represent both `TOP_ONLY` and `FULL_3D` without
  overloading a numeric height sentinel.
- Verify every world-valued result carries the source acquisition stamp and
  frame.
- Verify all consumers compile and explicitly branch on height validity.

Acceptance:

- Build and existing tests pass with zero new failures.
- No consumer creates collision/place geometry from `height_valid=false`.
- Pick waypoint Z equals `top_surface_pose.position.z + configured clearance`.

## Gate 1 - Pure Synthetic Algorithm Tests

Create deterministic numpy fixtures for a rectangular top plane and surrounding
horizontal support. Cover at least:

1. Three box sizes and heights.
2. Yaw values near 0, 45, and 90 degrees.
3. Gaussian depth noise up to 3 mm sigma.
4. At least 10% finite outliers.
5. A larger floor plane below the local support.
6. Missing support on one, two, and all sides.
7. Sparse/non-finite cargo and support inputs.
8. Platform heights including values other than 0.86 m.

Acceptance for valid full geometry:

| Metric | Limit |
|---|---:|
| top Z absolute error | <= 5 mm |
| support Z absolute error | <= 5 mm |
| height absolute error | <= 10 mm |
| width/depth absolute error | <= 20 mm per axis |
| wrong floor-plane selections | 0 |
| non-finite outputs | 0 |

Behavioral acceptance:

- Same input and seed produce identical output.
- Top valid plus missing support returns `TOP_ONLY` and
  `height_valid=false`.
- Insufficient side coverage never returns measured height.
- Catalog prior is labeled prior and never marked measured.
- Varying or omitting configured `platform_z` does not change top Z in `auto`
  mode.

## Gate 2 - Stamp, TF, and State Integration

Checks:

- Exact matching cargo/raw/mask stamps produce one geometry observation.
- A one-nanosecond mismatch does not fuse frames.
- Buffer length stays bounded under missing-stream and rate-mismatch tests.
- TF lookup uses the acquisition stamp.
- Missing stamped TF produces an explicit invalid result.
- `hold_track` cargo plus fresh raw depth cannot produce
  `MEASURED_SUPPORT` height.
- Motion/geometry-not-settled input cannot update the support estimate.
- Support temporal state returns copies and resets on task/instance epoch.

Acceptance:

- Cross-stamp or cross-epoch fusion count is zero.
- No latest-TF fallback occurs.
- No stale support estimate is relabeled as a current measurement.
- All rejection paths expose stable machine-readable reason codes.

## Gate 3 - Compatibility and Fault Injection

Run regression cases against the accepted detector baseline.

Fault cases:

- omit `platform_z` entirely;
- set `platform_z` 0.20 m too high and 0.20 m too low;
- remove all local support points;
- expose only one support side;
- add a dominant floor plane and a horizontal robot/panel distractor;
- drop raw depth while retaining cargo;
- drop cargo while retaining raw depth;
- delay one stream beyond the bounded join window.

Acceptance:

- In `auto` mode, top Z changes by <= 5 mm across omitted/wrong platform
  settings for identical sensor data.
- Missing support yields top-only, not false full geometry.
- Missing cargo cannot produce a luggage detection from the platform alone.
- A bad configured prior is used only in an explicitly selected prior mode and
  its source is visible.
- Existing configured-support compatibility tests retain their previous output
  within floating-point tolerance.

## Gate 4 - Gazebo Online/Eval Separation

Run at least 30 trials: each of the three catalog sizes at least 10 times,
covering multiple yaw and XY offsets. Online launch omits `platform_z` and uses
`support_mode=auto`. The eval driver alone reads spawned pose/size and scene
truth.

Acceptance:

| Metric | Limit |
|---|---:|
| valid top-surface rate | >= 95% |
| valid full-geometry rate when support coverage passes | >= 95% |
| top Z error P95 / max | <= 15 mm / 25 mm |
| support Z error P95 / max | <= 15 mm / 25 mm |
| height error P95 / max | <= 25 mm / 40 mm |
| XY center error P95 | <= 30 mm |
| width/depth error P95 | <= 50 mm per axis |
| false measured-height results | 0 |
| online GT/spawner geometry reads | 0 |

Record one fixed set of online sensor topics, then replay it twice while only
the detector-visible decoy `pickup_source.z` is changed. Online
top/support/full geometry must remain unchanged within the numeric tolerances.
Do not move the Gazebo platform or suitcase for this invariance test, because
that would change the sensor observation rather than isolate configuration
dependence.

The eval report must distinguish `TOP_ONLY`, `FULL_3D`, prior-derived, and
failed frames instead of merging them into one detection-success rate.

## Gate 5 - Rosbag Replay

Dataset readiness and metadata validation are implemented under PF-A2 in
`platform_free_height_remediation.md`. The normative topic/type/time/TF/
reference contract is
`docs/plans/platform_free_height_gate5_bag_contract.md`, checked by
`scripts/gate5_bag_readiness.py`. Passing the PF-A2 checker is necessary
but not sufficient for Gate 5: accuracy acceptance still requires real sensor
data and independent references. The checker must not claim Gate 5 accuracy.

When real data is available, record canonical preprocessed RGB, depth, points,
camera info, joint states/TF, masks or reproducible segmentation inputs, and
preprocessor status. Reference dimensions come from offline reconstruction,
manual measurement, or an independent calibrated measurement process.

Dataset minimum:

- at least three luggage sizes;
- at least three XY/yaw placements per size;
- at least 30 accepted settled observations total;
- at least one platform whose height is not 0.86 m;
- visible-support and intentionally support-occluded cases.

Acceptance:

| Metric | Limit |
|---|---:|
| top Z error P95 / max | <= 20 mm / 30 mm |
| measured height error P95 / max | <= 30 mm / 50 mm |
| XY center error P95 | <= 30 mm |
| width/depth error P95 | <= 50 mm per axis |
| false `height_valid=true` on occluded support | 0 |
| replay result divergence on identical bag/config | 0 |

Offline reconstructed labels must be marked as reference estimates unless their
accuracy is independently established; they must not be published into online
algorithm input topics during replay.

## Gate 6 - Performance and Lifecycle

Checks:

- Measure top fit, support crop/fit, join delay, and end-to-end detection
  latency separately.
- Run long enough to detect unbounded dictionaries, callback backlog, and RSS
  growth.
- Terminate every agent-started simulation with `scripts/stop_sim.sh` and
  verify zero residual processes.

Acceptance:

- Semantic geometry output remains at least 4 Hz on the accepted GPU profile.
- End-to-end perception P95 latency is no worse than 110% of the recorded
  pre-change baseline, unless a reviewed replacement threshold is recorded.
- Join buffers never exceed configured capacity.
- No callback queue shows monotonic lag growth.
- Residual simulation processes: zero.

## Release Decision

- Engineering handoff may complete after Gates 0-4 pass and evidence is
  recorded.
- Hardware deployment acceptance additionally requires Gate 5 rosbag replay.
- Gate 6 is mandatory for both handoff and hardware deployment.
- Any false `height_valid=true`, online GT dependency, cross-stamp fusion, or
  collision geometry created from top-only data is an automatic release block.
