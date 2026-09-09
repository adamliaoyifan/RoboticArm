# D555 eye-in-hand calibration

Status: plan for CAD seeding and ChArUco eye-in-hand calibration of the wrist
D555. Supersedes nothing. Does not authorise a message-contract or
acceptance-bar change.

Parent task: `D555-HANDEYE-20260909`.

## Why

HB-3 measured the mount extrinsic residual against Mid-360 at three static
poses (`docs/status/evidence/d555_bringup/20260908_2036_hb3/`): plane offsets
10.10 / 2.42 / 1.03 mm and plane angles 1.91 / 2.33 / 2.19 deg. The angle is
consistent while the offset varies with pose, which is the signature of a
**mount rotation error** of about 2 degrees.

That matters quantitatively. Every camera measurement reaches the base frame
through `FK(joints) * T_flange_camera`. A 2 deg rotation error is roughly
35 mm of lateral error at a 1 m working distance, and it tilts a fitted top
plane by 2 deg, which is about +/-9 mm of height error across a 0.5 m box
footprint. Height estimation is the product requirement, so this is material.

The current transform is not a calibration. `camera_mount_origin.xacro` is
headed "Saved by `eof_mount_stack_tune_gui.py`" and `NOTES.md` attributes it
to the "D435 mount GUI": visual tuning against a **D435** body origin. The
mount is physically correct for D555, but the D555 body origin sits elsewhere
inside it.

HB-3 explicitly did not authorise a correction, and it was right not to: three
poses all viewing the floor is a degenerate configuration that does not
constrain yaw. Writing a 2 deg tweak from that data would fix pitch and roll
while leaving yaw wrong, on top of a value whose provenance is already broken.

## What the two methods are for

**CAD seed** is not calibration. It reads the nominal
`eef_mount_adapter` to D555 body transform out of the mount geometry, which
was designed for D555, and uses it as the initial estimate. It replaces a
GUI-tuned guess with design intent, and it gives an independent sanity bound:
if the solver returns something far from CAD, the capture data is bad rather
than the design.

**ChArUco eye-in-hand** is the calibration. A ChArUco board is a chessboard
with ArUco markers in the white squares, combining sub-pixel chessboard corner
accuracy with per-corner unique IDs, so partial views and occlusion stay
usable. With the board fixed in the world and the camera on the arm, capturing
the board pose plus joint angles at many arm poses yields the classic
`AX = XB` problem whose solution is the unknown `T_flange_camera`.

## Board specification

Derived from the HB-1 measured intrinsics, not from a datasheet.

### Resolution constraint, read this first

Measured colour intrinsics at 640x360 are `fx 323.18`, `fy 322.90`,
`cx 317.75`, `cy 178.03`, giving about 89.4 x 58.3 deg FOV.

ArUco decoding needs roughly 25 px across a marker, and marker pixel size is
`fx * marker_m / Z`. At 640x360 a 37.5 mm marker is 20 px at 0.6 m and only
**12 px at 1.0 m**, which does not decode.

Therefore **capture the calibration session at 1280x720 or 1280x800 colour**,
where `fx` is about 646. This is legitimate and not a parity violation:
`T_flange_camera` is a rigid-body transform and is **resolution independent**,
so a transform solved at 1280 applies unchanged to 640x360 production.

Disable the point cloud for the session. HB-1 measured it at about 513 Mbps of
the roughly 701 Mbps total on the 1 GbE PoE link; freeing it leaves ample
headroom for 1280 colour (about 332 Mbps at 15 Hz, and 5 Hz is sufficient for
calibration).

### Primary specification

| Property | Value |
|---|---|
| Pattern | ChArUco (chessboard with ArUco markers in white squares) |
| Squares | 10 x 8 |
| Square length | 50 mm |
| Marker length | 37.5 mm (0.75 x square) |
| Dictionary | `DICT_5X5_100` |
| Markers used | 40 of 100 |
| Interior corners | 9 x 7 = 63 |
| Pattern area | 500 x 400 mm |
| Substrate with quiet zone | at least 600 x 500 mm |

Marker pixel size at 1280 width (`fx` about 646):

| Distance | Marker px | Board fills image width |
|---|---|---|
| 0.4 m | 61 | 63% |
| 0.6 m | 40 | 42% |
| 0.8 m | 30 | 32% |
| 1.0 m | 24 | 25% |

Usable calibration range is therefore **0.4 to 1.0 m**, which brackets the
cell's working distance.

### Fallback specification

If 1280 colour cannot be enabled, use larger features and a lower-bit
dictionary, accepting fewer corners:

| Property | Value |
|---|---|
| Squares | 8 x 6 |
| Square length | 55 mm |
| Marker length | 41 mm |
| Dictionary | `DICT_4X4_50` (24 of 50 used) |
| Interior corners | 7 x 5 = 35 |
| Pattern area | 440 x 330 mm |

Fewer dictionary bits decode at smaller pixel sizes. At 640x360 this gives
about 22 px at 0.6 m, which is marginal, so restrict the fallback range to
0.4 to 0.7 m.

### Physical requirements

These are as important as the pattern geometry, and are the usual source of
silent systematic error.

- **Flatness at or better than 0.5 mm across the pattern.** Mount on aluminium
  composite (Dibond-type) of 3 mm or thicker, float glass, or acrylic of 5 mm
  or thicker. Paper taped to a wall or to foam board is **not acceptable**: a
  1 mm bow over 400 mm is absorbed by the solver as extrinsic error.
- **Measure the printed pitch, do not trust the nominal.** Use callipers
  across at least 5 squares and divide. Consumer printers commonly scale by
  0.5 to 2 percent, and scale error maps one-to-one onto translation error;
  1 percent at a 0.5 m lever is 5 mm. Record the measured value and use it in
  the solver.
- **Matte surface.** Matte print or matte laminate. Gloss produces specular
  blowout that destroys corner detection.
- **Quiet zone of at least one square width** of white margin around the
  pattern, otherwise edge markers fail to segment.
- **Even diffuse lighting.** Avoid a strong one-sided source that puts a
  brightness gradient across the board.
- **Rigid and static during capture.** The board must not move between poses;
  the whole method assumes a fixed world target.

## Pose set requirements

Rotation in `AX = XB` is unobservable if all poses differ only by translation,
or if all rotations share a single axis. HB-3's three floor-viewing poses were
exactly that degenerate case.

- At least 15 poses, target 20.
- Rotation about **at least 3 distinct axes**, with at least 30 deg between
  extremes on each axis.
- Distance spread across the usable range, not clustered at one depth.
- The board must appear near the image centre and near all four image corners
  across the set, so lens distortion is covered.
- Most poses tilted relative to the optical axis. Fronto-parallel views are
  degenerate for board pose estimation.

## Procedural requirements

- **Dwell at least 1 s after motion stops and confirm zero joint velocity
  before capture.** Image and joint reading must come from the same instant.
- **Read joints from CPS `HRIF_ReadActACS`, not `/joint_states`.** HB-3
  recorded that `/joint_states` from the Sep-2 executor published all zeros.
  This is a known trap in this cell.
- Record `camera_info` from the same session at the calibration resolution.
- Safety per `ros2_humble_mvp_and_migration_plan.md`: read-only, low speed,
  small amplitude, static poses commanded one at a time, operator present. No
  packing cycle. No gripper command.

## Subtasks

| ID | Owner agent/model | Depends on | Scope | Acceptance | Required tests |
|---|---|---|---|---|---|
| HE-1 | `cursor/grok-4.6` | none | Derive the CAD seed and emit the board spec sheet and capture tooling | Seed transform with derivation, printable board, capture and solve scripts | Offline geometry checks, no hardware |
| HE-2 | `cursor/grok-4.6` | HE-1 | Capture the pose set, solve hand-eye, validate | Solver residual, holdout error, HB-3 re-measurement collapse, CAD agreement | Multi-solver comparison and independent re-measurement |

### HE-1 -- CAD seed and calibration assets

1. Derive nominal `eef_mount_adapter` to D555 body origin from
   `src/luggage_gazebo/models/arm_realsense/arm_realsense_v1.3.stl`. This is a
   mesh rather than parametric CAD, so state the method used to locate the
   D555 seat and the uncertainty it carries. If original STEP or the mount
   design source exists, prefer it and say so.
2. Cross-check the seed against the D555 datasheet body geometry
   (167 x 42 x 48 mm) and the driver's own `camera_link` to `d555_link`
   identity from HB-1.
3. Report the seed against the current GUI value
   (`0.013 0.097 -0.021`, rpy `0.03770 1.36345 1.57080`) and state the delta.
4. Generate the printable board at the primary specification, at true scale,
   with a printed legend carrying dictionary, square length, marker length,
   and grid size so the artefact is self-describing.
5. Provide the capture script (pose, settle, grab colour, read CPS joints,
   detect ChArUco, store) and the solve script, both runnable offline against
   recorded data.
6. Do **not** edit any URDF, xacro, or yaml. The seed is reported, not applied.

Pass condition: seed transform with a stated derivation and uncertainty, a
true-scale printable board, and capture plus solve scripts that run on
synthetic or recorded input.

### HE-2 -- capture, solve, validate

1. Capture at least 15 poses meeting the pose-set requirements. Record per
   pose: colour image, `camera_info`, CPS joint vector, detected ChArUco
   corner IDs and pixel coordinates, and the board pose in camera frame.
2. Solve with at least two OpenCV methods from `cv2.calibrateHandEye`
   (for example `TSAI`, `PARK`, `DANIILIDIS`) and report the spread between
   them as a stability indicator.
3. Report residuals: rotation in deg and translation in mm.
4. Holdout validation: solve on all but 5 poses, predict the held-out board
   poses, report the error.
5. **Independent check.** Re-run
   `docs/status/evidence/d555_bringup/20260908_2036_hb3/measure_mount_residual.py`
   at the same three poses with the new transform substituted. The plane angle
   near 2 deg must collapse. This criterion uses no calibration data and is the
   strongest available evidence.
6. Compare translation against the HE-1 CAD seed. Agreement within 10 mm is
   expected; a larger gap indicates a capture-data problem and must be
   reported as such rather than accepted.
7. Do **not** write the result into URDF, xacro, or yaml in this subtask.
   Applying it is a separate change that also has to resolve the
   `realsense_d435.yaml` `tune_joints` versus `fixed.rpy` contradiction.

Pass condition: solver residuals, multi-method spread, holdout error, the HB-3
re-measurement result, and the CAD comparison, all recorded as numbers.

## Out of scope

- Applying the calibrated transform to URDF, xacro, or yaml.
- Resolving the `realsense_d435.yaml` `tune_joints` versus `fixed.rpy`
  inconsistency. Report it; a separate change owns it.
- Re-qualifying `pickup_observe` against the ~26 cm Min-Z. That belongs to
  `Q-20260908-2`.
- Any message-contract or topic-remap change. `Q-20260908-3` owns that.
- Any PF-R acceptance-bar change.
- Camera intrinsic calibration. HB-1 confirmed factory intrinsics are
  published and consistent; do not re-derive them.

## Evidence

`docs/status/evidence/d555_handeye/<run>/` with the tested commit and
dirty-file count, per `docs/agents/FORMAT.md` section 12.

## Pointers

- `docs/status/evidence/d555_bringup/README.md`
- `docs/status/evidence/d555_bringup/20260908_2036_hb3/`
- `docs/plans/d555_hardware_bringup_verification.md`
- `src/luggage_gazebo/models/arm_realsense/arm_realsense_v1.3.stl`
- `src/luggage_description/config/camera_mount_origin.xacro`
- `src/luggage_description/config/realsense_d435.yaml`
- `docs/architecture/motion_compensation.md`
