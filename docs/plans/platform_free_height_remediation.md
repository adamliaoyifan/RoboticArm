# Platform-free Height Remediation Plan

Date: 2026-09-04

Source evidence:

- `docs/status/evidence/platform_free_height/2026-09-04_1412_g0g6/`
- `docs/agents/test/2026-09-04_1423_platform-free-height-g0g6.md`
- `docs/agents/reviews/2026-09-04_1111_platform-free-height-estimation.md`

## Objective

Close the G0/G4/G6 failures without weakening the platform-free validity
contract. Simulation and hardware must run the same online geometry algorithm.
Gazebo/spawner truth remains available only to eval code.

This remediation does not add platform-tilt estimation. It also does not claim
hardware acceptance without the Gate 5 rosbag dataset.

## Fixed Decisions

1. `support_mode=auto` requires segmented semantic cargo points. An
   unsegmented raw depth cloud is not a luggage observation.
2. Raw-only input must fail closed. It must not report the dominant platform
   plane as a valid luggage top.
3. Missing, malformed, or stale motion/preprocessor status may still permit a
   top-only result, but it must never permit `MEASURED_SUPPORT` or `FULL_3D`.
4. Every TF used by the semantic and detector paths must be queried at the
   acquisition stamp. Latest-TF fallback is forbidden.
5. Gate 4 scores settled semantic observations. Initial support-stability
   warmup is measured separately and is not silently counted as a geometry
   failure.
6. Gate 6 measures active-window throughput separately from spawn/delete idle
   time. Optimization follows stage profiling, not guessed threshold changes.
7. No acceptance threshold may be relaxed to make the existing evidence pass.

## Subtasks

Parent task: `PFH-REMEDIATION-20260904`

| ID | Owner agent/model | Depends on | Bounded scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| PF-R1 | `claude/glm-5.3` | none | Preserve the complete E0 contract through the planning adapter | Live adapter supplies valid pick top Z and preserves validity/source | PF-G0A plus affected package regression |
| PF-R2 | `claude/glm-5.3` | none | Fail-closed cargo input for raw-only/auto | Platform plane cannot become a valid luggage top | PF-G3A plus detector regression |
| PF-R3 | `claude/glm-5.3` | none | Stamped geometry status and semantic TF | Missing/stale status or TF cannot create FULL_3D | PF-G2A plus perception regression |
| PF-R4 | `claude/glm-5.3` | none | Correct evaluator semantics and placement coverage | Zero/missing samples fail and active Hz excludes orchestration gaps | PF-G4H evaluator tests |
| PF-R5 | `claude/glm-5.3` | PF-R1,PF-R2,PF-R3,PF-R4 | Thirty-trial semantic online accuracy | All Gate 4 limits pass with no online GT use | PF-G4S simulation regression |
| PF-R6 | `claude/glm-5.3` | PF-R4,PF-R5 | Profile and optimize accepted semantic path | At least 4 Hz with bounded queues/RSS and zero residual processes | PF-G6S performance/lifecycle |
| PF-A1 | `cursor/grok-4.6` | none | Independently audit online privileged information and hardware input availability | No unresolved privileged-geometry dependency; every task-state input has a hardware provider | Static/data-flow audit plus optional launch graph evidence |
| PF-A2 | `cursor/grok-4.6` | none | Define the Gate 5 rosbag contract and implement a readiness checker | Missing/type/time/TF/reference defects fail with stable reasons; no real-data pass is claimed | Synthetic checker tests and metadata fixtures |
| PF-R7 | `cursor/grok-4.6` | PF-R1,PF-R2,PF-R3,PF-R4,PF-R5,PF-R6,PF-A1 | Independently run the integrated acceptance chain | G0-G4/G6 pass on one reproducible revision | PF-E2E |

PF-R1 through PF-R4 are independently ready. The assigned Claude agent owns
implementation, focused testing, failure repair, evidence, and closure for
PF-R1 through PF-R6. PF-R5 must not start until all four prerequisites pass.
PF-A1 and PF-A2 are independent Cursor tasks. PF-A1 is a release-safety input
to PF-R7. PF-A2 prepares Gate 5 but does not block the current simulation
regression. PF-R7 is the final independent integration audit. Gate 5 remains a
separate hardware-release dependency.

## PF-R1 - Complete ROS Adapter Contract

### Implementation

Update `luggage_planning/ros_message_adapters.py::pick_from_detected` to
preserve at least:

- acquisition header/frame;
- `top_surface_pose`, validity, and confidence;
- `height_valid`, confidence, and source;
- existing pose, dimensions, yaw, and detection identity.

Use structured pose conversion where the planning API expects its internal
pose type. Do not pass a mutable ROS message where downstream code expects a
plain planning object.

Audit all other `DetectedLuggage` conversion boundaries. A positive numeric
height must not substitute for `height_valid=true`.

### Focused gate PF-G0A

- Construct a real generated `DetectedLuggage` ROS message.
- Convert it with `pick_from_detected`.
- Verify every E0 field survives with the same values.
- Verify `pick_contact_top_z` uses `top_surface_pose.z` through this adapter.
- Verify prior-only/top-only input cannot create full collision/place geometry.
- Run affected planning and packing package tests.

Failure blocks every integrated pick test, but does not block independent
PF-R2 through PF-R4 development.

## PF-R2 - Cargo Input Safety

### Implementation

The accepted platform-free path is:

```text
semantic cargo cloud -> top fit
same-stamp raw depth  -> local support fit
```

For `use_semantic=false` with `support_mode=auto` or
`auto_then_configured`, reject the configuration at startup or publish an
explicit invalid reason such as `DETECT_CARGO_SEGMENTATION_REQUIRED`. Do not
call `estimate_top_surface` with the unsegmented depth cloud.

If legacy raw-only operation must remain for another workflow, isolate it
behind an explicit non-default mode and do not present its output as accepted
platform-free geometry. Do not implement an implicit raw clustering heuristic
inside this remediation.

### Focused gate PF-G3A

- Reproduce the prior raw-only scene containing the 0.86 m pickup platform.
- Assert there is no valid luggage top at platform Z.
- Assert there is no `FULL_3D` and no `height_valid=true` result.
- Assert the failure reason identifies missing segmentation/configuration.
- Verify `use_semantic=true` still reaches the normal detector path.
- Verify configured `platform_z` cannot make raw platform points become cargo.

Failure blocks online accuracy and release.

## PF-R3 - Temporal and Motion Safety

### Implementation

Replace the current `None -> geometry_ok=True` behavior with stamped status
validation. Compare the cloud acquisition stamp with preprocessor
`primary_stamp`/`last_geometry_ok_stamp` using a documented tolerance suitable
for the JSON floating-point representation.

Required behavior:

- matching fresh `geometry_ok=true`: support fit may run;
- `geometry_ok=false`: top-only, support gate `geometry_not_settled`;
- missing/malformed/stale status: top-only with a distinct machine-readable
  support reason;
- no stale support estimate may be relabeled as current measurement.

Update `semantic_point_filter_node.py::_lookup_rt` to use the supplied message
stamp rather than `rclpy.time.Time()`. A missing stamped TF must produce an
explicit miss; do not retry with latest TF.

### Focused gate PF-G2A

- Matching status/cloud stamp permits measured support.
- Missing, malformed, false, and stale status all forbid measured support.
- One-nanosecond cargo/raw mismatch remains rejected.
- Semantic filter TF lookup receives the acquisition stamp.
- Missing historical TF does not call latest TF.
- `hold_track` remains top-only and cannot fuse with fresh raw depth.

Failure blocks online accuracy and release.

## PF-R4 - Repair the Eval Harness

### Implementation

Update `scripts/platform_free_height_gate4_eval.py` and its tests so absence of
samples cannot pass an accuracy metric.

The evaluator must:

- filter frames by current `instance_id` and `generation`;
- separate warmup, settled scoring, and trial orchestration intervals;
- score top Z, support Z, height, XY, width, and depth;
- enforce valid top and `FULL_3D` rates explicitly;
- reject zero-sample top/support/height metrics where that metric is required;
- report TOP_ONLY, FULL_3D, prior, and failed frames separately;
- calculate active-window output rate per trial, not across spawn/delete gaps;
- retain a separate end-to-end trial-cycle rate;
- store exact launch parameters and code revision in evidence.

The simulation matrix must deterministically cover all three luggage sizes,
multiple XY offsets, and multiple yaw values. Placement variation belongs to
the eval/spawner side and must not be published as an online algorithm input.

### Focused gate PF-G4H

Use synthetic evaluator records to prove:

- zero FULL_3D frames fail the gate;
- missing error samples fail required metrics;
- stale instance/generation frames are ignored;
- warmup TOP_ONLY frames are reported but excluded only by the documented
  settled-window rule;
- inserted spawn gaps do not reduce active-window Hz;
- bad support/width/depth estimates independently fail their limits.

Failure blocks PF-R5, PF-R6, and release.

## PF-R5 - Semantic Online Accuracy

Run the accepted profile with:

- `use_semantic:=true`;
- `support_mode:=auto`;
- `platform_z` omitted;
- no online GT/spawner geometry client.

Run at least 30 deterministic trials with each luggage size represented at
least ten times and with the PF-R4 XY/yaw matrix. Preserve raw JSONL, logs,
parameter snapshot, revision, and aggregate summary.

### Focused gate PF-G4S

Apply all Gate 4 limits from `platform_free_height_test_plan.md`. In addition:

- raw-only negative control fails closed;
- no online node reads `GetCurrentBox` or spawned geometry;
- changing only the detector-visible decoy `pickup_source.z` does not alter
  results for an identical recorded sensor stream.

Do not tune thresholds using GT inside the online process. Parameter changes
must be sensor-observation based and recorded before rerunning the held-out
matrix.

## PF-R6 - Performance and Lifecycle

### Instrumentation first

Record, at minimum:

- raw and semantic input rates;
- joined-frame and output rates;
- cloud decode/filter time;
- TF transform time;
- top fit and support fit time;
- join delay, callback drops/backlog, buffer occupancy, and RSS trend;
- time to first TOP_ONLY and first stable FULL_3D after each box change.

Use one clock domain per latency calculation. Do not subtract a ROS simulation
timestamp from a wall-clock receipt timestamp.

### Optimization order

1. Remove duplicate decode/TF work for the same topic and stamp where present.
2. Reuse bounded same-stamp transformed arrays rather than copying them.
3. Crop/downsample raw support candidates before RANSAC when frame semantics
   permit it.
4. Avoid fitting support when cargo, stamped TF, or geometry status has already
   failed its gate.
5. Only then consider algorithm parameters, without weakening accuracy gates.

### Focused gate PF-G6S

- Accepted semantic geometry output is at least 4 Hz on the accepted GPU
  profile.
- Report per-stage P50/P95/max and active-window end-to-end latency.
- Join buffers remain bounded; callback lag and RSS do not grow monotonically.
- Stop the agent-started simulation with `scripts/stop_sim.sh`; residual count
  is zero.

If no valid pre-change latency baseline exists, report that fact and establish
a reviewed baseline; do not invent a 110% comparison.

## PF-A1 - Privileged Information and Hardware Availability Audit

This is an independent read/audit task. It must not edit PF-R3/PF-R4 core files
or weaken online behavior.

Inventory every online function-node input involved in platform-free pickup
and classify it as:

- sensor observation;
- calibrated/deployment configuration;
- backend-neutral task lifecycle state;
- privileged simulation truth;
- eval-only reference.

At minimum, inspect scene/pickup configuration, Gazebo services, spawned model
state, `GetCurrentBox`, `/luggage/current_box`, instance/generation resets,
semantic tracking, detector, waypoint/planning, placement/map, and vacuum
paths. Distinguish task epoch identity from box geometry: task identity is
allowed only when a hardware-side orchestrator can provide the same contract.

Required output:

- source-to-consumer data-flow table with file/line evidence;
- simulation provider and hardware provider for every accepted online input;
- list of direct and indirect truth leaks;
- severity, runtime consequence, and proposed remediation owner/checkpoint;
- proof that eval reference topics/services cannot feed online geometry.

PF-A1 may pass only when there is no unresolved privileged-geometry input and
every required task-state input has a credible hardware provider. Otherwise it
must close as blocked and route each release blocker to reviews/engineering.

## PF-A2 - Gate 5 Rosbag Contract and Readiness Checker

Define a backend-neutral Gate 5 dataset contract without depending on a bag
that does not yet exist. The contract must cover canonical preprocessed RGB,
depth, points, camera info, joint states, stamped TF/TF-static, semantic masks
or reproducible segmentation inputs, preprocessor status, detector output, and
offline reference annotations.

The checker must validate at least:

- required topic names and ROS types;
- non-empty message counts and minimum recording duration;
- timestamp monotonicity and overlapping observation windows;
- required frame IDs and TF coverage at sensor acquisition stamps;
- availability of segmentation inputs needed for deterministic replay;
- separation of offline references from online algorithm input topics;
- dataset coverage metadata for sizes, XY/yaw placements, support visibility,
  and non-0.86 m platforms.

Use synthetic metadata/fixture tests for missing topic, wrong type, empty
stream, non-overlap, missing TF, and reference leakage. The checker must emit
stable machine-readable failure reasons and return nonzero on invalid input.
PF-A2 completion means the recording/replay input can be validated; it must not
mark Gate 5 accuracy as passed without a real bag and independently established
references.

## PF-R7 - Final Regression

After PF-R1 through PF-R6 and PF-A1 pass on exact revisions, run G0-G4 and G6
as one integrated regression. The final evidence must identify one
reproducible code revision. A label such as `0674f84-wt` is not sufficient for
release.

Engineering acceptance requires:

- complete ROS pick contract works in the live node path;
- semantic online top/support/full geometry passes Gate 4;
- raw-only configuration cannot report the pickup platform as luggage;
- missing/stale status and missing stamped TF fail closed;
- no online privileged-truth dependency;
- performance and teardown gates pass.

Hardware acceptance additionally requires Gate 5 against the future real
rosbag/reconstruction dataset. Its absence is not an engineering workaround
and must remain visible as `inconclusive`.

## Owner-Closed Execution Protocol

1. Work with the existing dirty workspace; do not revert unrelated changes.
2. Start with PF-R1. PF-R2 through PF-R4 are also ready and independent.
3. For each subtask, implement, run its focused and proportionate regression
   tests, fix failures, preserve evidence, and reply in that subtask's thread.
4. Record an exact reproducible output revision and an eng note before closing
   each subtask. Continue only work whose dependencies are complete.
5. Do not start PF-R5 until PF-G0A, PF-G3A, PF-G2A, and PF-G4H pass.
6. PF-A1 and PF-A2 may run immediately without touching Claude-owned core
   files. PF-R7 becomes runnable after PF-R6 and PF-A1 close; its assigned
   owner performs the independent integration audit. Do not claim hardware
   readiness until Gate 5 data exists and passes.
