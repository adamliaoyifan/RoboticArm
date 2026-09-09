# D555 Gazebo depth-image pipeline execution

Status: approved requirement after distinct-Codex consensus; dispatch held
until the external readiness gates in section 3 are satisfied.

Parent task: `D555-SIM-DEPTH-20260909`.

Consensus:
`docs/agents/discuss/2026-09-09_1455_d555-sim-depth-consensus.md`.

## 1. Objective

Retarget the active ROS 2 Gazebo wrist-camera backend from a transported
camera-native point cloud to the same canonical data product selected for the
live D555:

```text
Gazebo colour image
Gazebo depth image (32FC1 metres)
Gazebo CameraInfo
        |
        v
single-stream depth adapter (16UC1 millimetres)
        |
        v
preprocessor exact acquisition: colour + aligned depth + both CameraInfo
        |
        v
consumer-local pixel selection / bounded support sampling
        |
        v
deprojection with same-acquisition K
        |
        v
optical-to-world TF lookup at the image stamp
```

The ROS graph must not bridge, publish, subscribe to, decode, transform, or
score a Gazebo camera-native `PointCloud2`. Gazebo's `rgbd_camera` may still
generate an internal `points` transport; it is outside the ROS product so long
as no bridge or ROS consumer exists.

This plan also moves the simulated canonical profile to the measured D555
colour-aligned profile: 640x360 at 15 Hz, truthful colour optical frame,
colour-grid intrinsics, `16UC1` millimetres, and a 0.26 m near clip.

## 2. Fixed facts and decisions

These are inputs, not parameters for the implementer to reinterpret.

| Item | Required value or behavior | Evidence |
|---|---|---|
| Canonical grid | colour-aligned depth | HB-1/HB-2 |
| Sim colour/depth | one Gazebo `rgbd_camera` grid | no dual-camera split in this task |
| Width x height | 640x360 | HB-1 stable profile |
| Update rate | 15 Hz | HB-1 measured ceiling |
| Colour K target | fx 323.1775, fy 322.8994, cx 317.7526, cy 178.0294 | HB-1 live `CameraInfo` |
| Sim distortion | truthful renderer output; do not publish hardware D unless rendered | frame/K correctness rule |
| Canonical depth | `16UC1` millimetres; zero means invalid | architecture |
| Gazebo source depth | `32FC1` metres; invalid or out-of-range becomes zero | adapter contract |
| Near clip | 0.26 m | D555 VGA Min-Z |
| Optical frame | configured colour optical frame, identical across four products | F3 contract |
| World transform | TF at the exact acquisition stamp | architecture |
| Camera cloud | forbidden in active ROS 2 graph | F3 and Gazebo frame defect |

Runtime `CameraInfo`, rather than xacro text, is the acceptance source of
truth. The SDF custom intrinsics block may be used when supported by the
installed sdformat/Gazebo. If the renderer cannot reproduce all four measured
K values exactly, `fx`, `fy`, `cx`, and `cy` must each be within 2
percent of HB-1 and remain self-consistent with the rendered grid.

## 3. External readiness and isolation gates

### R0 -- PF-R9 generation 2 must finish first

`docs/agents/discuss/2026-09-09_1147_pf-r9-g2-payload-depth-primary.md`
is claimed by `claude/glm-5.3` and overlaps the detector, semantic filter,
preprocessor, configuration, and probes used here.

No D555 simulation subtask may be claimed until that thread has all three:

1. `status: done`;
2. a `## Result` with `outcome: pass`;
3. an exact passing Git commit.

Reviews then binds DSIM-1 generation 1 to that exact commit or a clean
descendant. Absence from `OPEN.md` is not sufficient.

### R1 -- use an isolated worktree

The owner creates a satellite worktree from the bound revision and keeps all
mailbox writes in the primary workspace through `AGENT_COORD_ROOT`. The
satellite worktree must be clean before each checkpoint and at final Result.
Do not copy or overwrite dirty files from the shared primary workspace.

### R2 -- protect PF-R10/PF-R7 fixed revisions

The D555 profile changes alter simulation timing and geometry. They do not
retroactively certify or modify PF-R10/PF-R7. Final DSIM-INTEGRATION runs only
after PF-R7 completes, unless reviews creates explicit higher-generation
replacements for PF-R10/PF-R7 under separate authority. This plan does not
grant that authority.

## 4. Ownership and task graph

All tasks are owned end to end by `eng/cursor/grok-4.6`: implementation,
tests, repair, evidence, commits, teardown, and thread closure.

| ID | Depends on | Scope | Passing outcome |
|---|---|---|---|
| DSIM-1 | none, plus R0 | Gazebo backend and D555 canonical profile | No camera-cloud bridge; correct 640x360@15 RGBD |
| DSIM-2 | DSIM-1 | Requalify `pickup_observe` for Min-Z and FOV | Reachable pose with valid visible geometry |
| DSIM-3 | DSIM-1 | Remove active camera-cloud assumptions; prove stamped-TF failures | No active cloud consumer or latest-TF fallback |
| DSIM-INTEGRATION | DSIM-1,DSIM-2,DSIM-3, plus R2 | Full headless regression | Stream, geometry, timing, graph, teardown pass |

## 5. DSIM-1 -- Gazebo backend and profile

### Scope

- Gazebo camera xacro/SDF and ROS 2 bridge in
  `src/luggage_description/` and `src/luggage_gazebo/`.
- D555 simulation camera configuration and focused tests.
- `depth_image_republisher.py` only if its contract tests expose a defect.
- Do not edit PF-R9 perception files delivered by R0 unless a failing
  interface test proves a simulation-backend defect; record any such edit.

### Checkpoints

#### DS1-C0 -- clean intake

Record the PF-R9 passing commit, plan revision, worktree path, branch, and
`git status --porcelain` count. Re-run PF-R9's focused tests unchanged. If
they fail, report blocked; do not repair PF-R9 under this task.

#### DS1-C1 -- baseline and consumer inventory

Before changing code, preserve:

- bridge arguments and remaps;
- ROS topic/type graph;
- publishers/subscribers for `/camera/depth/points` and
  `/luggage/preprocessed/camera/depth/points`;
- 30-second colour/depth/camera-info rate, stamps, dimensions, encodings, K,
  and frames;
- every repository reference to both camera-cloud topics, classified as
  active ROS 2, eval/probe, test, ROS 1 legacy, history, or documentation.

The inventory is a comparison basis, not an allowlist that can hide new active
consumers.

#### DS1-C2 -- remove the ROS camera-cloud boundary

The active `sim_world` bridge contains image, depth image, camera info, and
Livox cloud bridges, but no camera `PointCloudPacked` bridge and no camera
points remap. Livox `PointCloud2` remains legal. Remove the obsolete camera
point-cloud key from the active D555 simulation config. Add a static regression
test that fails if the camera bridge/remap returns.

#### DS1-C3 -- bind the measured canonical profile

The active simulation publishes:

- colour and depth grids at 640x360 and nominal 15 Hz;
- a truthful common colour optical frame;
- runtime `CameraInfo` within the K tolerance in section 2;
- source depth as `32FC1` metres and adapted depth as `16UC1` millimetres;
- a 0.26 m near clip.

The simulation may model aligned depth directly with one common pinhole. Do
not add a second Gazebo camera, approximate synchronizer, native-depth product,
or fake depth-to-colour warp.

#### DS1-C4 -- adapter boundary tests

Focused tests cover at least:

- finite metre values to nearest millimetre, absolute error at most 1 mm;
- `inf`, `-inf`, `NaN`, zero, negative, below-near, and above configured
  maximum values become zero;
- height, width, stamp, and frame are preserved;
- output encoding, endianness, step, and byte length are valid `16UC1`;
- truncated or malformed payload fails closed rather than reshaping guessed
  bytes.

#### DS1-C5 -- 30-second runtime checkpoint

After at least 5 seconds warm-up, measure at least 30 seconds:

- colour, adapted depth, both camera-info products, and preprocessed products;
- colour and adapted-depth rates each between 13.5 and 16.5 Hz;
- at least 95 percent of emitted acquisitions have exact integer stamp,
  dimension, and frame equality across all four canonical products;
- depth encoding is `16UC1` and sampled conversions satisfy the 1 mm rule;
- the ROS graph has no camera-native `PointCloud2` publisher or subscriber.
  Semantic cargo/obstacle and Livox clouds are exempt and named separately.

### DSIM-1 pass condition

DS1-C0 through DS1-C5 pass on one clean commit. Static source inspection alone
cannot pass runtime camera-info, timing, or graph requirements.

## 6. DSIM-2 -- `pickup_observe` geometry requalification

### Scope

- Simulation named-pose configuration and geometry/reachability tests.
- Eval-side measurements against Gazebo model truth.
- Do not change camera extrinsics, near clip, estimator thresholds, catalog
  geometry, or a hardware pose file to make this checkpoint pass.

### Checkpoints

#### DS2-C0 -- measure before changing the pose

At the DSIM-1 profile, measure the existing `pickup_observe` pose for every
valid catalog suitcase/box, including the 0.80 m tallest case. Record:

- camera pose and optical depth extrema;
- valid-depth ratio in the cargo ROI;
- projected bounding rectangle and image-edge margin;
- IK, collision, controller-execution, and settle results.

The expected approximately 0.24 m nearest surface must appear as a failing
baseline; otherwise explain the geometry discrepancy.

#### DS2-C1 -- select a valid observation pose

The owner may change the simulation named pose after recording the baseline.
The chosen pose must satisfy all of the following for every valid catalog
model:

- every surface required by cargo/top/support estimation has optical Z at
  least `near + 0.04 m`, therefore at least 0.30 m;
- valid-depth ratio inside the eval-side cargo ROI is at least 0.95;
- the projected cargo rectangle has at least 10 pixels clearance from every
  image edge;
- arm IK/planning succeeds, the pose is collision-free, controller execution
  succeeds, and the motion-settle gate passes;
- visible pickup-platform support retains at least the configured
  `min_support_points` after bounded support sampling.

Record the minimum distance to the D555 ideal 0.6 m range separately. A result
between 0.30 and 0.60 m may pass this Min-Z gate but must not be described as
ideal-range parity.

#### DS2-C2 -- negative controls

Prove that restoring the old pose fails the near-margin assertion and that a
synthetic frame whose required surface is inside 0.26 m yields invalid/zero
depth and no measured geometry. Do not lower the near clip to pass.

### DSIM-2 pass condition

DS2-C0 through DS2-C2 pass for the full valid catalog on a clean commit. If no
reachable and collision-free pose meets every DS2-C1 gate, return blocked with
the attempted poses and limiting constraint; do not weaken the gates.

## 7. DSIM-3 -- active-consumer cleanup and stamped TF

### Scope

- Remaining active ROS 2 simulation/eval references after PF-R9 and DSIM-1.
- Focused fault probes and contract tests.
- ROS 1 reference launch files, historical evidence, and archived docs may
  retain old strings only when explicitly classified as non-active.

### Checkpoints

#### DS3-C0 -- repository closure inventory

Repeat DS1-C1's reference inventory on the current commit. Every active ROS 2
or active eval reference to either camera-cloud topic must be removed or
migrated. List every allowed legacy occurrence with file and reason; an
unclassified occurrence fails.

#### DS3-C1 -- trace every world-XYZ path

For semantic cargo, semantic obstacle, and detector support geometry, record a
code and runtime trace proving:

1. input is the accepted aligned depth image;
2. intrinsics come from the same grid's `CameraInfo`;
3. invalid depth is removed before deprojection;
4. deprojection produces points in the declared colour optical frame;
5. TF lookup uses the depth acquisition stamp;
6. output geometry inherits that stamp and a truthful frame.

No path may use `now()`, zero/latest TF, a guessed frame, approximate
consumer pairing, or an empty point set standing in for valid geometry.

#### DS3-C2 -- fail-closed fault matrix

Automated tests inject at least:

- missing aligned depth;
- mismatched integer stamp;
- mismatched width/height;
- mismatched frame;
- missing or invalid K;
- unsupported or truncated depth encoding;
- missing TF at the exact acquisition time while a newer TF exists.

Every row produces no geometry and increments a named counter or failure
reason. The newer-TF row specifically proves there is no latest-TF fallback.

#### DS3-C3 -- runtime graph/type audit

For at least 30 seconds, capture `ros2 topic list -t`,
`ros2 topic info -v`, and relevant `ros2 node info`. Classify every
`PointCloud2` in the active graph. Allowed classes are Livox input and
semantic/evaluated output; camera-native or preprocessed full-camera clouds
fail.

### DSIM-3 pass condition

DS3-C0 through DS3-C3 pass on one clean commit with no active camera-cloud
reference and no successful geometry output in any fault-matrix row.

## 8. DSIM-INTEGRATION -- complete simulation regression

### Preconditions

- DSIM-1, DSIM-2, and DSIM-3 are `done` with passing Results.
- External gate R2 is satisfied.
- Rebase or merge onto one exact clean revision containing all three results,
  then rerun their focused tests after conflict resolution.

### Checkpoints

#### DSI-C0 -- reproducible launch and safety

Run headless on a dedicated `ROS_DOMAIN_ID` with `gui:=false` and
`use_rviz:=false`. Record launch arguments, configs, environment, exact
commit, and dirty count. Follow `.cursor/rules/sim-lifecycle.mdc`; always
finish with `scripts/stop_sim.sh` and record zero residual
Gazebo/ROS/controller processes.

#### DSI-C1 -- 120-second stream and join gate

After at least 15 seconds warm-up, collect a common window of at least 120
seconds. Require:

- emitted acquisitions / unique colour stamps at least 0.80;
- paired canonical depth / emitted acquisitions at least 0.95;
- semantic exact joins / received accepted depth at least 0.95;
- detector same-stamp support-depth hits / joined cargo at least 0.95;
- stale depth-plus-mask drops / received depth-plus-mask below 0.05;
- rate and four-product identity requirements from DS1-C5;
- no camera-native cloud in the graph.

The 15 Hz profile does not lower any ratio gate.

#### DSI-C2 -- geometry and world-coordinate oracle

Run the established six-trial Gate-4 short matrix three consecutive times on
the same commit. Do not lower existing bars. At minimum require:

- failed trials: 0;
- `FULL_3D` rate at least 0.95;
- `false_measured_height` exactly 0;
- top/support error p95 at most 15 mm and max at most 25 mm;
- height error p95 at most 25 mm and max at most 40 mm;
- XY error p95 at most 30 mm;
- width/depth error p95 at most 50 mm.

Additionally sample a known Gazebo target at no fewer than three distinct arm
poses/timestamps. Compare only eval-side Gazebo truth with world points
obtained from canonical depth deprojection plus stamped TF. The samples must
satisfy the same applicable top/support/XY limits. A Gazebo camera point cloud
is not a permitted oracle input.

#### DSI-C3 -- regression suites

Run, record, and repair failures in:

```bash
python3 -m pytest src/luggage_description/test -q
python3 -m pytest src/luggage_gazebo/test -q
python3 -m pytest src/luggage_perception/test -q
colcon test --packages-select luggage_description luggage_gazebo luggage_perception --event-handlers console_direct+
colcon test-result --verbose
git diff --check
scripts/check_agent_contract.sh
```

Narrower commands are allowed at intermediate checkpoints, but DSI-C3
requires every listed package suite.

#### DSI-C4 -- evidence and closeout

Store commands, launch config, exact revision, dirty count, topic/node graph,
stream counters, K/frame/stamp samples, adapter cases, pose geometry, fault
matrix, three Gate-4 summaries, and teardown proof under
`docs/status/evidence/d555_sim_depth/<run>/`. A passing Result names the
single tested commit and evidence path.

### DSIM-INTEGRATION pass condition

DSI-C0 through DSI-C4 pass on one clean exact revision. Passing this task
certifies the simulated depth-image path; it does not certify the unapplied
hardware hand-eye transform.

## 9. Forbidden changes

- Do not use D555 native depth or `/camera/d555/depth/color/points` as the
  canonical aligned product.
- Do not repair the Gazebo cloud by relabelling its header.
- Do not introduce approximate pairing, latest-TF fallback, `now()` stamps,
  invalid-depth substitution, or an empty point set as valid geometry.
- Do not reduce ratio or geometry gates because the profile is 15 Hz.
- Do not add split colour/depth Gazebo cameras in this generation.
- Do not use D555 IMU as a substitute for FK.
- Do not edit or apply HE-2, `camera_mount_origin.xacro`, or hardware launch.
- Do not claim D555 housing collision or mass-distribution parity. The 167 mm
  enclosure placement relative to the optical datum is a separate mechanical
  task.
- Do not modify PF-R10/PF-R7 accepted scope or revisions from this task.

## 10. Required evidence layout

Each subtask writes a separate directory:

```text
docs/status/evidence/d555_sim_depth/
  dsim1_<timestamp>/
  dsim2_<timestamp>/
  dsim3_<timestamp>/
  integration_<timestamp>/
```

Each directory contains a short `RESULT.md` with exact commit, dirty count,
commands, checkpoint table, pass/fail, and pointers to raw artifacts. Raw ROS
logs, JSON counters, images, and topic dumps stay in evidence, not
`docs/agents/`.

## 11. Pointers

- `docs/architecture/sensor_data_pipeline.md`
- `docs/architecture/motion_compensation.md`
- `docs/plans/pf_r9_g2_payload_depth_primary_execution.md`
- `docs/plans/d555_replace_d435/replacement.md`
- `docs/status/evidence/d555_bringup/20260908_1951_hb1/SUMMARY.md`
- `docs/status/evidence/d555_bringup/20260908_2004_hb2/SUMMARY.md`
- `src/luggage_description/urdf/realsense_d435.urdf.xacro`
- `src/luggage_description/config/realsense_d435.yaml`
- `src/luggage_gazebo/launch/sim_world.launch.py`
- `src/luggage_gazebo/scripts/depth_image_republisher.py`
- `src/luggage_perception/luggage_perception/depth_deprojection.py`
