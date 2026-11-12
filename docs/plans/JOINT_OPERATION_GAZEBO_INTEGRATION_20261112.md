# 2026-11-12 -- Observed full-model Gazebo operation integration

- status: approved
- dispatch_ready: yes
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- generation: 1
- owner: codex/gpt-6
- base_revision: 5086d84f2873db9b36ec94d8e67efeb83481c74f
- authorization: User requests runnable latest-code observation, complete planning and execution in simulation before hardware.

## Scope and decisions

Use existing Fortress full S20/camera/suction/scene/controller engineering. Add an
explicit joint_operation pipeline/profile to sim_world; retain named legacy_segments
compatibility. New selection composes measured dynamic suction export, one O owner,
shared ComputePlacement, private native endpoint/connection/certification services,
prepare root and bound execution consumer. Reuse exactly one expanded full URDF/SRDF
across all model readers. Record resolved configuration, installed origins and hashes.
No GT/catalog geometry enters observed box/O/planner; spawn and physics adapters may
use simulator truth separately. G is not launched with O. G execution remains outside
this O integration qualification. Startup never authorizes motion; an explicit driver
start command authorizes the test cycle. No physical robot endpoints are launched.

Preprocessed depth/intrinsics yield optical rays with acquisition stamp and frame,
without semantic cropping, missing-space filling or latest-TF substitution. O owner
resolves stamped TF. Existing motion-stability gate admits stationary observations.
Source resolution is shared; no independent consumer voxel size. Sensor maps, actual
attachment/release and observed post-release adoption remain necessary for success.
Keep current 5cm collision padding and release gap pending separate evidence review.
No obstacle removal or predicted-geometry occupancy to force a passing run.

## Acceptance and procedure

Environment: ROS Humble production prefix only; deploy_update current source;
Gazebo Fortress headless, full S20 mesh collisions, domain7, one world, one /clock.
Dirty unrelated inputs are preserved and fully hashed in build/run manifests.
Freeze owned source at a commit before live testing. Any repair creates a new
revision/run directory. Failed boundary data must be durable before repair/retry.

1. Behavioural offline tests once per input: profile rejects mixed old/new owners,
missing evidence path, unsupported pipeline; old selection preserves compatibility;
new composition has exactly one mapper, prepare/native providers and execution
consumer, same model/config identity. Depth adapter exact stamps/frames/intrinsics,
invalid/missing data, motion rejection and uncropped observed optical points.
All selected cases pass (AND over deterministic cases, no reliability claim).
2. Build with scripts/production/build.sh --evidence <root>/build_N; all13 packages
pass, before/after source hashes identical; check_install.py passes all installed
checks. Record ros2 launch --show-args and resolved model/owner hashes.
3. Unscored startup rehearsal: explicit profile, isolated domain7, bounded180s per
attempt and total15min, maximum3 attempts before root-cause review. Persistent probe
requires exactly one /clock publisher, both controllers active, six measured joints,
RGB/depth/intrinsics, O receipts, prepare/native/execution endpoints. No test motion
before explicit start. Zero residual owned Gazebo/bridge/clock after teardown.
4. One diagnostic full cycle after rehearsal: start -> measured ULD observations ->
measured pickup detection/domain -> empty-slot prepare request -> original bound
six-leg execute -> confirmed attach/release -> retreat -> observed O adoption.
Every requested stage must complete successfully, same box/grasp/plan binding; otherwise
report first failing stage and exact reason, keep overall baseline OPEN. A diagnostic
failure is not a reliability measurement and not evidence of physical impossibility.
A success is one witness only, not a production reliability or timing SLA.
5. Capture per-stage timings and failure collision names, shapes, sampled q/TCP and
phase scene. Classify model collision separately from physical simulator contact.
Do not infer box-box or padding cause from generic failure text.

Canonical launch: scripts/production/run.sh ros2 launch luggage_gazebo sim_world.launch.py
profile_config:=<installed share>/config/joint_operation.profile.yaml operation_run_dir:=<absolute fresh root>.
Driver: scripts/production/run.sh ros2 run luggage_gazebo joint_operation_sim_driver.py
--start --evidence <root>/driver (explicit test authorization). Final command options
are recorded verbatim in evidence. Driver must preserve actual inputs and fail closed.

## Evidence and dump matrix

Root docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/.
No overwrite; per-attempt folders. T1 all transitions/reasons/IDs/stamps/timings.

| Boundary | Trigger | Payload | Window / format / budget | Inspection |
|---|---|---|---|---|
| Build/launch | every attempt | source+dirty hashes, commands, resolved profile/model, logs, readiness | whole startup JSON/text <=32MiB | manifest and readiness.json |
| Sensor/preprocess/detection | stale/missing/invalid | raw RGB/depth/K, relevant TF/joints, domain/detection masks and quality records | bounded10s ROS bag <=512MiB | ros2 bag info; stored CDR types |
| O/placement/prepare | every refusal | request/response, full admitted map/surface/private scenes, candidate trace/native inputs | one request <=256MiB | prepare CDR manifest and trace |
| Controller/attach/release/retreat | refusal or success | original plan, controller results, measured joints, vacuum, actual G, three phase scenes, contact pairs | one cycle <=256MiB | operation manifest + summary |

Total retained <=2GiB per attempt / 6GiB campaign; lossless compression after durable
capture, no deletion of failed cases. Capture-health before another attempt requires
nonempty timestamped inputs, explicit ROS type/identity, all hashes and missing-item
list, capture_complete/replay_possible flags. If instrumentation cannot capture a
failed boundary, mark incomplete and repair capture before spending another cycle.
