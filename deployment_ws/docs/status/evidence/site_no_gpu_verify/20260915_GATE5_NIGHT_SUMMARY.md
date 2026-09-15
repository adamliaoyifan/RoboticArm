# Gate 5 Night Summary - 2026-09-15

Scope: sealed pick only, from observe pose with `--skip-observe`. Out of scope:
place, packing, production orchestrator, and Livox in the detect chain.

## Environment

- Host: ThinkPad, ROS 2 Jazzy, no NVIDIA GPU.
- Working code under test: `/home/adamliao/work/RoboticArm-master`.
- Evidence tree: `deployment_ws/docs/status/evidence/site_no_gpu_verify`.
- Preprocessor profile: B, `preprocessor_d555_site.yaml`.
- Semantic backend: CPU `bbox_fill` / YOLO-world.
- Final recorded bag: `/home/adamliao/robotarm_bags/g5_20260915_205125`.
- Final pick graph was launched without duplicate hardware ownership:
  `start_executor:=false start_d555:=false start_scene:=false`; `record_site real`
  owned CPS, D555, scene TF, and rosbag.

## Runs

### 20:15 - Gate 5 B Trial 1

Evidence: `20260915_2014_g5_B`.

- Detect passed: `xyz=(-0.046, 1.244, 0.114)`, `top_z=0.242`.
- Motion sequence built: `pre_grasp`, `approach`, `attach`, `pick_retreat`.
- Failure: `pre_grasp`, MoveIt `CONTROL_FAILED (-4)`.
- Hardware root cause: `HRIF_WayPoint 20070`, speed below Huayan minimum.
- Fix applied: floor provided trajectory velocities as well as derived velocities.

### 20:28 - Gate 5 B Relaunch

Evidence: `20260915_2028_g5_B`.

- This did not reach the driver.
- Failure: environment overlay resolved `luggage_planning` and
  `luggage_perception` from `elfin_humble_ws`, not `RoboticArm-master`.
- Symptoms:
  - `waypoint_generator_node.py` import error on `SUPPORTED_OPENING_SIDE`.
  - `semantic_point_filter_node.py` import error on `depth_deprojection`.
- Fix applied: unset inherited colcon/ament/Python prefixes and source
  `local_setup.bash` overlays explicitly so master wins.

### 20:31 - Gate 5 C

Evidence: `20260915_2031_g5_C`.

- Detect passed: `xyz=(-0.050, 1.134, 0.111)`, `top_z=0.240`.
- Driver reported `pre_grasp` failure almost immediately.
- The arm still moved for about 8 s after the driver reported failure.
- Root causes:
  - `async def execute_callback` returned a coroutine-like action result path
    that let MoveIt see a controller result too early / `UNKNOWN`.
  - MoveIt TOTG densified OMPL into many sub-degree samples. Each sample was
    sent as a full Huayan `HRIF_WayPoint` MoveJ at the speed floor, producing
    visible forward/backward jumping.
  - `HRIF_IsBlendingDone` can be true while idle, so intermediate waypoints were
    consumed too aggressively.
- Fixes applied:
  - Make the FollowJointTrajectory execute callback synchronous.
  - Decimate dense TOTG samples before sending Huayan MoveJ commands.
  - Keep only real via points at least 2 deg from the last kept point, always
    keeping the final goal.
  - Wait for blending to become busy before treating `IsBlendingDone=true` as
    an intermediate waypoint completion.

### 20:51 - Gate 5 D With `record_site real`

Evidence: `20260915_2051_g5_D`.

- Bag: `/home/adamliao/robotarm_bags/g5_20260915_205125`.
- Observe pose:
  - radians: `[0.2397, -2.0051, -2.0952, -0.6182, 1.5752, 0.1724]`
  - degrees: `[13.73, -114.88, -120.05, -35.42, 90.25, 9.88]`
- Detect passed: `xyz=(-0.050, 1.117, 0.111)`, `top_z=0.240`.
- `pre_grasp` passed: 11 TOTG points decimated to 8 MoveJ commands.
- `approach` passed: Cartesian path 9 points, decimated to 2 MoveJ commands.
- `attach` failed before vacuum:
  - Cartesian path was valid: 26 points, fraction 1.0.
  - Huayan rejected waypoint 2 with `HRIF_WayPoint 40083`.
  - Root cause: acceleration exceeded controller maximum. The old executor used
    `accel = vel * 2`; at `vel=54 deg/s`, this sent `108 deg/s^2`.
- Fix applied after the run:
  - Add `MAX_ACCEL_DEG = 80`.
  - Compute acceleration with a lower slope and cap it.
  - If acceleration would not exceed speed, reduce speed below the acceleration
    cap instead of sending an invalid pair.

## Current Acceptance State

Gate 5 is still **not accepted**. The latest run reached `attach` and failed
before vacuum enable, so there is no sealed lift yet.

Next valid attempt must:

1. Restart `record_site.sh real` so the patched executor is loaded.
2. Return the robot to observe pose.
3. Launch pick graph with:
   `start_executor:=false start_d555:=false start_scene:=false`.
4. Run `hardware_pick_driver.py --skip-observe`.
5. Pass `pre_grasp`, `approach`, `attach`, vacuum DI0 seal, and
   `pick_retreat` while holding suction.

## Safety Notes

- Only one CPS owner is allowed. During recorded Gate 5, `record_site real`
  must own CPS; the pick graph must not start a second executor.
- Only one D555 driver and one scene TF publisher should be active.
- The vacuum contact surface must be flat and without height discontinuities;
  a leaky or stepped lid will not assert DI0 reliably.
- If the arm moves after a driver failure, wait for executor completion or use
  the e-stop according to site safety practice before relaunching.
