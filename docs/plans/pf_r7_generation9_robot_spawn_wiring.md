# PF-R7 generation 9 robot-spawn wiring repair and one scored live

Date: 2026-09-15

Parent: `PFH-REMEDIATION-20260904`

Subtask: `PF-R7`

Generation: `9`

Owner: `test/cursor/grok-4.6`

Base revision: `85168758f2f1bb691a959fd5f5239f9a8c4d4a3f`

Dependencies: `PF-R1,PF-R2,PF-R3,PF-R4,PF-R5,PF-R6,PF-A1,PF-R8,PF-R9,PF-R10`

## Decision and objective

G8 is `inconclusive/infrastructure_invalid`, not a product failure. Both of
its permitted startup launches completed a nonempty
`get_parameters(robot_description)` round trip (`22613` bytes), but the custom
launch matcher compared `event.action is target`; it never scheduled the S20
`ros_gz_sim create` action. Consequently the robot plugin never loaded,
`gz_ros2_control` never logged `Received URDF from param server`, and the only
valid controller manager never existed. G8 must not launch again.

G9 replaces G8. It repairs only this launch event chain and the corresponding
startup classification/evidence gate, proves the wiring offline, then runs at
most one scored campaign. The controller manager remains exclusively owned by
the in-Gazebo `gz_ros2_control` plugin bound to
`gz_ros2_control/GazeboSimSystem`. G9 must not add a ROS-side
`controller_manager` node, fabricate joint states, or redirect spawners to a
second manager.

## Immutable workload and product bars

- Production anchor: `60dafb7deee50a6f3a76d48076b743bf3e3e1bc8`.
- G9 implementation base: `85168758f2f1bb691a959fd5f5239f9a8c4d4a3f`.
- Proposal confidence floor: `0.20`.
- Conditional valid-top and `FULL_3D` rate floors: `0.95` each.
- Output rate: `>=4 Hz`.
- Support-ready starts at admitted `support_window_count == 5`.
- Score the half-open `[t_steady, t_steady + 8.000s)` interval.
- `t_first_valid_top`, `t_first_FULL_3D`, and `t_steady` are each
  `<=t_proposal+1.4s`.
- All G4 geometry P95/max thresholds remain byte-for-byte unchanged.
- Run three consecutive slots with two eligible cases per size per slot.
- Keep the frozen standard order exactly `standard_00, standard_01,
  standard_06, standard_07, standard_09, standard_10, standard_12,
  standard_13, standard_14`; carryon and large remain the exact G7 16-seed
  matrices. No rescan, reorder, replacement, wrapping, reuse, or append.
- Preserve at most 12 attempts and six exclusions per slot, three consecutive
  same-size exclusions, two stack resets, 36 campaign attempts, and 2700 s
  scored-campaign wall timeout.
- Executor-lag Q4 mean is `<=0.20s` and `<=1.25*Q1`; workload-adjusted RSS is
  `<=2 MiB/min`; teardown residuals are exactly `0`.
- One eligible failure stops the campaign. Score per case; do not average
  failures or retune any threshold.

## Revision and bounded scope

Create a clean descendant of the G9 base in a fresh isolated worktree/branch.
The final evaluator worktree and built overlay must be the same exact clean
commit; record full evaluator and overlay hashes plus tracked and untracked
dirty counts, all zero.

Allowed changes are limited to:

- `src/luggage_gazebo/launch/sim_world.launch.py`;
- a focused static launch-wiring regression under `src/luggage_gazebo/test/`
  and its `CMakeLists.txt` registration if needed;
- G8/G9 startup classification and live-driver helpers under
  `src/luggage_perception/.../eval/`, `src/luggage_perception/test/eval/`, and
  `scripts/pf_r7_*`;
- G9 evidence and the required test role note.

Do not change robot xacro, controller YAML, production perception/planning,
D555/Livox profiles, seed matrices, architecture, scheduler, or site runtime.

## Required launch chain

Delete `_on_exit_zero` and the imports used only by it (`EventHandler` and
`ProcessExited`). Use the stock handler form already proven by the scene edge:

1. RSP is running and serves a real nonempty
   `get_parameters(robot_description)` response;
2. `OnProcessExit(target_action=wait_rsp, on_exit=[...])` starts the S20
   `ros_gz_sim create` action and emits an unambiguous bounded log marker such
   as `spawn_robot: create S20` immediately before it;
3. the S20 URDF loads the `gz_ros2_control` plugin, which must log
   `Received URDF from param server`;
4. `OnProcessExit(target_action=robot, on_exit=[wait_cm])` starts the
   `/controller_manager/list_controllers` readiness wait;
5. `OnProcessExit(target_action=wait_cm, on_exit=[jsb_spawner])` starts the
   joint-state broadcaster; the existing stock edge then starts the arm
   spawner; the existing final edge starts observe hold and MoveIt.

There must be no identity-based launch-event predicate and no independently
launched `ros2_control_node` or `controller_manager` executable. The spawners
must still target the plugin-owned `/controller_manager`.

## Startup classification and fail-fast observations

The readiness manifest records monotonic and ROS timestamps plus ordered log
offsets for every transition. Classification precedence is:

1. duplicate or absent `/clock`;
2. if the real RSP round trip never succeeded,
   `robot_state_publisher_get_parameters_timeout`;
3. after RSP success, if the S20 create marker/process is absent within 2 s,
   `robot_spawn_missing`;
4. after S20 create, if `Received URDF from param server` is absent within
   10 s, `plugin_urdf_not_received`;
5. after plugin URDF receipt, if `/controller_manager/list_controllers` is
   absent by the existing 30 s stage-2 deadline,
   `controller_manager_service_absent` (or the existing spawner-died reason
   when observed);
6. after controller startup, missing active controllers or a finite six-joint
   sample is `joint_states_absent_after_controller_startup`.

Once `robot_description_ok=true`, neither old launch-log timeout text nor any
later-stage failure may be classified as
`robot_state_publisher_get_parameters_timeout`. A missing S20 create must fail
at the 2 s wiring deadline; a missing plugin receipt must fail at the 10 s
plugin deadline. Neither case may consume an otherwise empty 30 s manager wait.

`robot_spawn_missing` is a deterministic, non-whitelisted wiring failure and
must stop G9. `plugin_urdf_not_received` is eligible for the same single
pre-case infrastructure restart as the G7 plugin RPC timeout, provided every
existing G8 identity, T2 capture, zero-attempt, single-clock, teardown,
quiescence, and zero-residual condition passes. Keep the other G8 whitelist
rules, but use `robot_state_publisher_get_parameters_timeout` only when stage 1
actually failed. There is at most one restart, at most two G9 startup launches,
and at most one scored campaign; any second startup failure stops.

## Offline verification

Before acquiring Gazebo, validate the same SHA-256-locked G7 import used by G8
and refuse `scan` mode. Run the G8 focused suite plus new G9 regressions three
consecutive times with zero failure, error, skip, or retry. Run the existing
Gate4 37-test suite and PF-R10 RSS 10-test suite once each with the same zero
failure/error/skip requirement, and rerun the strict saved G6 `standard_02`
offline replay once within 30 s with finite GT bbox, confidence `<0.20`, IoU
`>=0.50`, `AUTOINSTALL=False`, no installer, and no cache/package mutation.

The new tests must prove:

- the launch source contains all four stock `OnProcessExit` edges and contains
  neither `_on_exit_zero`, `EventHandler`, nor `ProcessExited`;
- no standalone manager node/executable was introduced and both spawners still
  target `/controller_manager`;
- stage-1 success followed by missing S20 create is `robot_spawn_missing`, not
  RSP timeout, and fails at the 2 s gate;
- S20 create without plugin receipt is `plugin_urdf_not_received`, not RSP
  timeout, and fails at the 10 s gate;
- ordered logs require `wait_robot_description: ok`, then the S20 create
  marker, then `Received URDF from param server`; missing or out-of-order lines
  reject readiness;
- restart eligibility/refusal, exactly-one-restart maximum, no restart after
  case start, unchanged identity hashes, and one-scored-campaign maximum remain
  enforced.

Store exact commands, exit codes, three focused logs, launch-wiring test log,
Gate4/RSS logs, strict replay result, import manifest, and revision/dirty-state
record under `offline/`.

## Live procedure and evidence

Use exclusive Gazebo with `gui:=false`, `use_rviz:=false`,
`ROS_DOMAIN_ID=7`, and `/tmp/elfin_humble_sim.pid`. Queue behind any existing
stack. Before every permitted launch require zero owned sim/bridge/GPU
residuals; after launch require exactly one `/clock` publisher. Preserve the G8
75 s global startup deadline and all stage-4 camera/inference, semantic,
pickup-spawner, and observe-pose gates.

The launch log and `startup/attempt_<n>/ready.json` must show, in order:

- `wait_robot_description: ok` with nonempty byte count;
- `spawn_robot: create S20` and the corresponding create process start;
- `Received URDF from param server`;
- responsive `/controller_manager/list_controllers`;
- both controllers active and one finite six-joint `/joint_states` sample.

Absence of either required post-RSP log marker is a startup wiring/plugin
failure, not permission to wait blindly for the manager. A successful gate
hands the same still-running stack to the single scored campaign. Preserve the
G8 T0/T1 manifests and bounded startup T2 bundle: launch log, ordered marker
offsets, RSP value/hash/bytes, service/controller snapshots, joint-state
sample, process/clock inventory, reason code, exact replay command, missing
list, `capture_complete`, `replay_possible`, and artifact sizes. Flush it
before any teardown or restart.

After completion or failure, run `scripts/stop_sim.sh`, prove PID-file absence
and zero owned simulator/bridge/eval residuals, and record teardown evidence.
No G8 launch may be reused or resumed.

## Acceptance

Pass only when the clean G9 descendant passes all offline checks, a permitted
G9 startup exhibits the required ordered S20/plugin/controller chain with the
plugin as the sole manager owner, exactly one scored three-slot campaign passes
every unchanged product/C2/capture-health bar, and teardown residuals are zero.
Any missing marker, misclassification, second startup failure, eligible scored
failure, dirty/hash mismatch, incomplete evidence, contamination, or residual
is non-pass and must not trigger an undeclared extra launch.
