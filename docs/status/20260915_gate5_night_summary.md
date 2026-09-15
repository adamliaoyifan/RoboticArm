# Gate 5 Night Summary - 2026-09-15

This note records the real-cell sealed-pick Gate 5 attempts and the code
changes made during the night. Gate 5 is still not accepted; the latest run
reached `attach` and failed before vacuum enable.

## Runs

### 20:15 - pre_grasp speed floor failure

- Detect and sequence build passed.
- `pre_grasp` failed with MoveIt `CONTROL_FAILED (-4)`.
- Executor log showed `HRIF_WayPoint 20070`: speed below Huayan minimum.
- Fix: clamp both provided and derived trajectory velocities above the Huayan
  minimum before sending `HRIF_WayPoint`.

### 20:28 - wrong overlay

- The launch did not reach the driver.
- `luggage_planning` and `luggage_perception` resolved from
  `elfin_humble_ws`, not this master tree.
- Symptoms included `SUPPORTED_OPENING_SIDE` and `depth_deprojection` import
  failures.
- Fix: clear inherited colcon/ament/Python prefixes and source only
  `local_setup.bash` overlays in launch helper scripts.

### 20:31 - MoveIt returned before hardware finished

- The arm kept moving for about 8 s after the driver reported
  `CONTROL_FAILED`.
- Root causes:
  - The FollowJointTrajectory execute callback was declared `async`, which let
    MoveIt see a controller result path too early / `UNKNOWN`.
  - OMPL + TOTG generated dense sub-degree waypoints. Sending each one as a
    full-speed Huayan MoveJ caused visible forward/backward jumping.
  - `HRIF_IsBlendingDone` can be true while idle, so intermediate points were
    considered consumed too early.
- Fixes:
  - Make the action execute callback synchronous.
  - Decimate dense TOTG samples before issuing Huayan MoveJ commands.
  - Wait for blending to become busy before accepting
    `IsBlendingDone=true` for intermediate waypoints.

### 20:51 - attach acceleration failure

- `record_site.sh real` owned CPS, D555, scene TF, and the bag:
  `/home/adamliao/robotarm_bags/g5_20260915_205125`.
- Pick graph was launched without duplicate hardware ownership:
  `start_executor:=false start_d555:=false start_scene:=false`.
- `pre_grasp` passed: 11 TOTG points decimated to 8 MoveJ commands.
- `approach` passed: Cartesian path 9 points, decimated to 2 MoveJ commands.
- `attach` planned a valid Cartesian path but failed at waypoint 2:
  `HRIF_WayPoint 40083`, acceleration above Huayan limit.
- Cause: previous acceleration rule used `accel = vel * 2`; with
  `vel=54 deg/s`, this sent `108 deg/s^2`.
- Fix: cap acceleration at `MAX_ACCEL_DEG = 80` and reduce speed if needed so
  acceleration remains greater than speed without exceeding the controller cap.

## Code Changes

- `deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/huayan_interface.py`
  - adds TOTG waypoint decimation;
  - logs kept/total waypoint counts and actual Huayan MoveJ command joints;
  - waits for blending to start before consuming intermediate waypoints;
  - clamps velocity and acceleration to Huayan-safe ranges.
- `deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/trajectory_executor_node.py`
  - makes the FollowJointTrajectory execute callback synchronous.
- `deployment_ws/scripts/hardware_pick.sh` and `deployment_ws/scripts/record_site.sh`
  - clear polluted overlay variables;
  - source `local_setup.bash` overlays so this master tree wins;
  - keep Livox SDK libraries on `LD_LIBRARY_PATH`.
- `deployment_ws/src/elfin_trajectory_executor/launch/record_site.launch.py`
  - uses 30/60 deg/s for real recorded execution so `record_site real` matches
    Gate 5 speed settings.
- `src/luggage_planning/launch/hardware_pick.launch.py`
  - adds `start_scene` so a recorded run can avoid duplicate scene TF;
  - keeps the existing `start_executor` and `start_d555` switches for
    `record_site real`;
  - keeps hardware pick at 30/60 deg/s and 0.6 MoveIt scaling.

## Next Acceptance Attempt

1. Stop the old `record_site` process so the patched executor is loaded.
2. Return the robot to observe pose.
3. Start:
   `./scripts/record_site.sh real -o ~/robotarm_bags -n g5_<stamp>`.
4. Launch hardware pick with:
   `start_executor:=false start_d555:=false start_scene:=false`.
5. Run:
   `ros2 run luggage_planning hardware_pick_driver.py --skip-observe`.

Gate 5 passes only when `pre_grasp`, `approach`, `attach`, DI0 seal, and
`pick_retreat` all succeed while the box remains sealed and lifted. The vacuum
contact surface must remain flat with no height discontinuities.
