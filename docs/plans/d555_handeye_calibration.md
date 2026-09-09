# D555 eye-in-hand calibration

Status: revision 2 plan for CAD seeding and ChArUco eye-in-hand calibration of
the wrist D555. HE-1 generation 2 supersedes the original STL-only derivation
because official D555 CAD and operator-confirmed assembly constraints are now
available. Does not authorise a message-contract or acceptance-bar change.

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

**CAD seed** is not calibration. It registers the adapter mesh to the official
D555 mechanical CAD through the actual screw-hole and seating-plane interface,
then maps that mechanical assembly to the explicitly defined ROS `d555_link`.
It replaces a GUI-tuned guess with design intent and gives an independent
sanity bound. The seed is not valid if the camera CAD origin is silently
treated as `d555_link`, or if the camera-side optical datum cannot be traced to
an authoritative source.

## HE-1 mechanical facts and frame contract

The following observations were confirmed by the operator on 2026-09-09 and
are inputs to HE-1 rather than features to guess from mesh appearance:

- The D555 is fastened to the mount through screw holes on the inclined,
  elongated rectangular bar. That bar contains four holes in total. HE-1 must
  determine from the official camera CAD and geometric agreement which subset
  is used by D555; it must not assume that all four, or an arbitrary pair, are
  camera fasteners.
- The D555 bottom mounting surface is parallel to the mounting surface around
  those holes. The optical windows face outward, away from the mount and robot
  body.
- The separate square opening/pocket is the Mid360 installation feature. It is
  an orientation and collision discriminator, not a D555 datum.
- Another distinct two-hole interface attaches the mount to the EEF flange.
  Those holes belong to the robot-side interface and must be excluded from the
  D555 hole set.

The transform to report is `^eef_mount_adapter T_d555_link`, meaning that a
point expressed in `d555_link` is mapped into `eef_mount_adapter`. The current
URDF loads `arm_realsense_v1.3.stl` at zero origin and zero rotation with a
`0.001` scale, so the STL export frame, after millimetres-to-metres conversion,
is the `eef_mount_adapter` frame for this task. The separate EEF-flange-to-
adapter joint is not part of this transform.

`d555_link` is the ROS camera root frame, not the enclosure centre, a lens
cover surface, the colour optical origin, or an aligned-depth output frame.
HE-1 must record the authoritative mapping from the official D555/D450
mechanical model to the left-IR/depth datum and verify it against the running
RealSense driver's static TF tree. Internal transforms from `d555_link` to
`d555_depth_optical_frame` and `d555_color_optical_frame` remain driver-owned.

Official source entry points:

- D400/D555 CAD index: `https://dev.realsenseai.com/docs/stereo-depth-camera-d400/`
- D555 CAD archive: `https://dev.realsenseai.com/download/41953`
- D555 datasheet: `https://dev.realsenseai.com/download/42013/`
- RealSense ROS frame parameters: `https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md`

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

#### A. Acquire and preserve authoritative inputs

1. Download the official D555 CAD archive and datasheet from the URLs above.
   Record source URL, retrieval time, filename, byte count, SHA-256, declared
   units, CAD format, and any licence/readme in a source manifest. Do not commit
   a large or redistribution-restricted upstream archive; preserve the
   manifest and derived evidence instead.
2. Prefer an official STEP/assembly model over a tessellated export. If the
   archive contains multiple parts, retain part names and assembly transforms.
   Record whether it exposes the enclosure, D450 optical module, mounting
   screws, mounting face, and sensor origins.
3. Load `arm_realsense_v1.3.stl` as millimetres and report mesh units, bounds,
   connected components, watertightness, and triangle count. Preserve its
   export coordinates: those coordinates define `eef_mount_adapter` in the
   current URDF.

#### B. Label the mount before solving

4. Produce an annotated feature inventory, in STL coordinates, containing:
   the inclined rectangular bar and its four hole axes/centres; its candidate
   D555 seating plane; the square Mid360 opening/pocket; and the separate two
   EEF-flange hole axes/centres. Use stable feature IDs in the report and
   derivation code. Include orthographic renders with axes and labels.
5. Fit cylinders to hole walls and a plane to each mating surface rather than
   selecting individual mesh vertices. Report fit RMS, radius, centre, axis,
   plane normal, and support count for every feature. Repeat at two CAD
   tessellation tolerances, where applicable, to expose mesh sensitivity.
6. Fail closed if the four-hole bar, square Mid360 feature, and separate
   two-hole flange interface cannot be distinguished. The current GUI pose and
   Mid360 configured pose may be used to reject impossible orientations, but
   not as observations in the fit.

#### C. Identify the D555 mechanical correspondence

7. Extract the D555 bottom mounting plane and all official mounting-hole
   centres/axes from the official CAD. Use the datasheet mounting drawing as
   an independent dimension check.
8. Enumerate geometrically compatible subsets/correspondences between the
   D555 hole pattern and the rectangular bar's four holes. Do not manually
   select the nearest-looking pair without recording alternatives.
9. For every candidate, enforce the operator-confirmed constraints: D555
   bottom parallel and seated against the bar's mounting surface, optical
   windows facing outward, EEF two-hole interface excluded, Mid360 square
   opening unobstructed, and no impossible housing/mount penetration. Record
   why each rejected symmetry or hole correspondence fails.

#### D. Compute the nominal rigid transform

10. For a two-hole correspondence, construct a right-handed datum on each
    side from the unit vector between hole centres and the oriented seating-
    plane normal. Use their cross product as the third axis and the hole-pair
    midpoint plus seating offset as the origin. For three or more used holes,
    solve the same plane-constrained rigid registration over all centres with
    SVD/Kabsch and retain residuals.
11. Compute `^eef_mount_adapter T_D555-mechanical`, then compose the
    authoritative `^D555-mechanical T_d555_link`. Emit the complete 4x4 matrix,
    translation in metres, quaternion, fixed-axis RPY with stated convention,
    inverse transform, frame names, units, and multiplication order.
12. Do not equate an arbitrary CAD assembly origin with `d555_link`. Establish
    the left-IR/depth reference point and axes from official D555/D450 material
    and verify the frame relationship against a live or recorded driver TF
    dump. If no authoritative mechanical-to-ROS datum can be established,
    report HE-1 as blocked with the solved mount-to-housing transform; do not
    invent the missing offset from enclosure dimensions or photographs.

#### E. Quantify uncertainty and reject false fits

13. Include at least: CAD unit/scale uncertainty, STL tessellation sensitivity,
    plane and cylinder fit residual, screw-hole clearance, seating gap or
    spacer thickness, fastener-pattern correspondence, and the authoritative
    optical-datum uncertainty. Propagate them to translation-mm and rotation-
    deg bounds, analytically or by a seeded Monte Carlo run.
14. Required nominal geometry checks are: used-hole centre RMS at most 0.5 mm,
    maximum centre residual at most 1.0 mm, hole-axis disagreement at most
    0.5 deg, non-contact housing penetration at most 0.5 mm, and repeated-fit
    change at most 0.2 mm and 0.1 deg across the two tessellations. A source
    drawing with larger stated tolerance supersedes a numeric threshold only
    when cited and carried into the uncertainty result.
15. Cross-check the official D555 enclosure against the datasheet
    `167 x 42 x 48 mm` envelope. Render the final assembly from at least top,
    side, camera-facing, and EEF-facing views with both coordinate frames,
    selected fasteners, contact plane, Mid360 opening, and collision result.
16. Transform at least four non-coplanar test points forward and back; maximum
    round-trip error must be below `1e-9 m`. Recompute the reported matrix from
    the saved feature table in an automated offline test.
17. Compare the seed with the current GUI value
    (`0.013 0.097 -0.021`, RPY `0.03769911 1.36345121 1.57079633`). Report
    `T_seed^-1 * T_gui` as translation norm, rotation angle, and axis. The GUI
    value is comparison-only and must not bias the registration.

#### F. Emit calibration assets

18. Generate the printable board at the primary specification and true scale,
    with a printed legend carrying dictionary, square length, marker length,
    and grid size. Add an automated PDF/page-size and rendered-pitch check.
19. Provide the capture script (pose, settle, grab colour, read CPS joints,
    detect ChArUco, store) and solve script, both runnable offline against
    recorded or synthetic input. The solver must accept the measured physical
    square pitch rather than hard-code nominal print scale.
20. Store the source manifest, feature table, derivation report, final transform,
    uncertainty result, annotated renders, and exact verification output under
    `docs/status/evidence/d555_handeye/<run>/`. Keep reusable scripts and tests
    in the appropriate source/test package discovered from repository layout.
21. Do **not** edit any URDF, xacro, or YAML. The seed is reported, not applied.

Pass condition: all authoritative inputs are traceable; the unique physical
hole/plane correspondence and rejected alternatives are evidenced; the
reported transform terminates at the verified ROS `d555_link`; checks 14-17
pass; uncertainty is numeric; the true-scale board passes its print-geometry
test; and capture plus solve scripts pass on synthetic or recorded input. A
mount-to-housing result without an authoritative housing-to-`d555_link` datum
is a useful blocked result, not a passing HE-1 seed.

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
