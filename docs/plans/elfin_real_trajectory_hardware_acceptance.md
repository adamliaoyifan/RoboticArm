# Elfin S20 real trajectory fix -- TODO and hardware acceptance checklist

Date: 2026-09-15
Target cell: Huayan Elfin S20 at controller `192.168.0.10:10003`
Operator workstation: `192.168.2.126:~/work/RoboticArm-master`
Source workspace: `/home/adamliao/work/elfin_humble_ws`
Baseline revision when this checklist was written:
`5822d6644554ca3b3dd621537a8c5af2bd5215f2` plus the explicitly listed dirty
files below.

## 1. Scope and current conclusion

The 2026-09-15 attach abort and the visible stop-start motion are different
failures.

- Abort root cause: attach MoveJ number 2 requested `accel=108 deg/s^2` and
  CPS rejected it with 40083. The velocity, approximately `54 deg/s`, was
  legal. The observed acceleration acceptance interval is only `[60,108)`;
  60 was accepted and 108 was rejected.
- Stop-start root cause: each retained MoveIt/TOTG point is sent as an
  independent `HRIF_WayPoint`, and the executor waits on
  `HRIF_IsBlendingDone` before sending the next point. The 5 mm blend radius,
  blocking wait, 50 ms poll interval, and CPS TCP RTT produce 0.16--0.20 s
  zero-speed intervals.
- This local patch prevents the known 40083 profile and prevents partial
  execution of a goal whose later waypoint profile is invalid. It does not
  claim that the stop-start behavior is fixed.
- The executor admits one trajectory owner at a time. A second goal is
  rejected as busy and cannot reconnect CPS or alter the owning goal's cancel
  token.
- CPS completion-poll errors now stop and abort immediately instead of being
  interpreted as motion in progress.

## 2. Local changes completed

The following changes are locally implemented and unit-tested:

- [x] Replace the unverified dynamic `accel=max(60, vel*1.5)` / 80 cap with a
  configurable `command_acceleration_deg`, defaulting to the site-proven
  `60 deg/s^2`.
- [x] Enforce `velocity <= acceleration - 1 deg/s`, while also honoring the
  configured velocity cap.
- [x] Read but never modify controller limits with
  `HRIF_ReadJointMaxVel`, `HRIF_ReadJointMaxAcc`, and
  `HRIF_ReadJointMaxJerk` after connection.
- [x] Require readable six-axis velocity and acceleration limits before real
  motion readiness, and clamp commands to at most 80 percent of their minima.
- [x] Treat jerk as a diagnostic read because `HRIF_WayPoint` does not command
  jerk.
- [x] Reject overlapping action goals and use one private cancellation token
  per accepted owner.
- [x] Treat nonzero or malformed `HRIF_IsBlendingDone` and
  `HRIF_IsMotionDone` responses as fail-closed execution errors.
- [x] Validate point count, finite positions/derivatives, joint range, and
  strictly increasing `time_from_start`.
- [x] Build and validate every retained waypoint profile before the first
  `HRIF_WayPoint`. Invalid profiles return `INVALID_GOAL` with zero motion.
- [x] Expose acceleration and `controller_limit_fraction` through
  `executor.yaml`, `jazzy_real.launch.py`, and `record_site.launch.py`.
- [x] Add unit and ROS action coverage for CPS limit parsing, 80 percent
  profile clamping, zero-waypoint-call rejection, concurrent admission,
  cancellation isolation, and CPS poll failures.

Changed source files:

- `deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/huayan_interface.py`
- `deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/trajectory_executor_node.py`
- `deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/goal_ownership.py`
- `deployment_ws/src/elfin_trajectory_executor/config/executor.yaml`
- `deployment_ws/src/elfin_trajectory_executor/launch/jazzy_real.launch.py`
- `deployment_ws/src/elfin_trajectory_executor/launch/record_site.launch.py`
- `deployment_ws/src/elfin_trajectory_executor/test/test_cps_parse.py`
- `deployment_ws/src/elfin_trajectory_executor/test/test_goal_ownership.py`
- `deployment_ws/src/elfin_trajectory_executor/test/test_hardware_safety.py`
- `deployment_ws/src/elfin_trajectory_executor/test/test_action_single_goal.py`

Local verification performed:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/elfin_hw1_ros_logs
PYTHONPATH=deployment_ws/src/elfin_trajectory_executor:${PYTHONPATH} \
  python3 -m pytest -q \
  deployment_ws/src/elfin_trajectory_executor/test
# Expected: 61 passed

python3 -m py_compile \
  deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/huayan_interface.py \
  deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/goal_ownership.py \
  deployment_ws/src/elfin_trajectory_executor/elfin_trajectory_executor/trajectory_executor_node.py \
  deployment_ws/src/elfin_trajectory_executor/test/test_cps_parse.py \
  deployment_ws/src/elfin_trajectory_executor/test/test_goal_ownership.py \
  deployment_ws/src/elfin_trajectory_executor/test/test_hardware_safety.py \
  deployment_ws/src/elfin_trajectory_executor/test/test_action_single_goal.py
```

The full package suite also runs through `colcon test`; final evidence must use
a clean worktree at the exact passing commit.

## 3. Revision and deployment gate

Do not test an unidentified dirty tree.

- [ ] Commit or archive the exact in-scope files listed above.
- [ ] Record `git rev-parse HEAD` and `git status --short` in the run manifest.
- [ ] Confirm no unreviewed local change alters `huayan_interface.py`,
  `trajectory_executor_node.py`, the launch files, or `executor.yaml`.
- [ ] Rebuild the deployment package after installing the patch:

```bash
cd ~/work/elfin_humble_ws/deployment_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select elfin_trajectory_executor --symlink-install
source install/local_setup.bash
```

- [ ] Run the complete package test in the same environment:

```bash
cd ~/work/elfin_humble_ws
source /opt/ros/jazzy/setup.bash
PYTHONPATH=deployment_ws/src/elfin_trajectory_executor:${PYTHONPATH} \
  python3 -m pytest -q deployment_ws/src/elfin_trajectory_executor/test
```

Pass criterion: collection succeeds and every test passes. Any skipped test
must be named and justified in the evidence summary.

## 4. Cell safety prerequisites

- [ ] One trained operator remains at the e-stop for every powered test.
- [ ] First tests use no payload and no bag beneath the tool.
- [ ] Verify the planned swept volume is clear and establish a conservative
  vertical clearance fixture before running attach.
- [ ] Verify pendant speed override and safety configuration; record both.
- [ ] Confirm there is exactly one CPS motion owner. Stop old `record_site`,
  `trajectory_executor`, Noetic executor, pendant program, and duplicate
  hardware-pick processes before starting.
- [ ] Confirm `/joint_states` has exactly one real-hardware publisher.
- [ ] Confirm vacuum outputs are off before motion: `DO0=0`, `DO1=0`.
- [ ] Prepare a timestamped evidence directory and bag name. Never overwrite a
  failed run.

Abort the test immediately on protective stop, unexpected direction, reverse
jump, collision risk, loss of joint telemetry, duplicate CPS owner, or any CPS
motion error. Preserve the bag and logs before restart.

## 5. H0 -- read-only controller qualification

Start the patched executor without sending a trajectory. Capture its complete
startup log.

- [ ] Record CPS SDK, CPS controller, Codesys, and robot versions using
  `HRIF_ReadVersion` or the corresponding startup diagnostic.
- [ ] Record all six returned values from `HRIF_ReadJointMaxVel`.
- [ ] Record all six returned values from `HRIF_ReadJointMaxAcc`.
- [ ] Record all six returned values from `HRIF_ReadJointMaxJerk`.
- [ ] Confirm the log contains `Controller limits vel=... acc=... jerk=...`.
- [ ] Confirm the application did not call any `HRIF_SetJointMax*` API.

Pass criteria:

- Six finite positive velocity values and six finite positive acceleration
  values are returned.
- Effective acceleration is no greater than 80 percent of the minimum returned
  acceleration limit.
- Effective velocity is no greater than 80 percent of the minimum returned
  velocity limit and remains at least 1 deg/s below effective acceleration.

If either velocity or acceleration limits are unavailable, the executor must
remain not ready and reject hardware goals. Record the API return codes and do
not continue to a motion phase. Missing jerk limits are diagnostic only.

## 6. H1 -- preflight rejection with zero motion

Purpose: verify that invalid profiles are rejected before any waypoint reaches
the robot.

- [ ] Start `jazzy_real.launch.py` with
  `max_velocity_deg:=20 command_acceleration_deg:=0.5`.
- [ ] Send a small, collision-free two-point joint trajectory while the
  operator watches the arm and joint telemetry.
- [ ] Capture executor events, CPS calls/return codes, and `/joint_states`.

Pass criteria:

- FollowJointTrajectory terminates as `INVALID_GOAL`.
- Log contains `Trajectory profile preflight failed before motion`.
- There are zero `HRIF_WayPoint` calls for this goal.
- Maximum measured joint displacement from the pre-goal pose is at most
  `0.05 deg` on every joint.

Restore `command_acceleration_deg=60` before proceeding.

## 7. H2 -- conservative empty-space motion

Run with `max_velocity_deg=20 deg/s` and
`command_acceleration_deg=60 deg/s^2`.

- [ ] Execute a small single-joint-safe target three times.
- [ ] Execute the real `pre_grasp` trajectory three times with the tool and
  entire swept volume clear.
- [ ] Execute `approach` three times above a clearance fixture.
- [ ] Execute `attach` three times with its endpoint raised sufficiently to
  prevent contact.
- [ ] Exercise cancellation once during a slow empty-space motion.

Pass criteria for all 12 segment executions:

- 12/12 action results are `SUCCEEDED`; the cancellation case is `CANCELED`.
- Zero CPS 40083 and zero other CPS errors.
- No reverse joint jump greater than `0.5 deg` between adjacent 100 Hz
  samples unless present in the desired trajectory.
- Final per-joint error is at most `0.5 deg` after settling.
- Final FK TCP translation error is at most `5 mm`.
- No `/joint_states` hole greater than `100 ms` during motion.

This phase validates safety and the 40083 fix. Stop-start gaps may still be
present and are measured separately in H3.

## 8. H3 -- Gate 5 profile and stop-start measurement

Start a fresh real-mode recording so the patched executor is the only CPS
owner:

```bash
cd ~/work/elfin_humble_ws/deployment_ws
./scripts/record_site.sh real -o ~/robotarm_bags \
  -n accel60_dry_$(date +%Y%m%d_%H%M%S)
```

In another terminal with the same `ROS_DOMAIN_ID=7`, launch the pick graph with
the executor, D555, and scene owners disabled, then run the driver. Keep the
attach endpoint above the fixture for dry validation.

```bash
ros2 run luggage_planning hardware_pick_driver.py --skip-observe
```

- [ ] Run 10 dry `pre_grasp -> approach -> attach` cycles.
- [ ] Record the exact command velocity and acceleration for every waypoint.
- [ ] Derive joint velocity from positions; do not rely only on the reported
  velocity field.
- [ ] Segment motion with a `2 deg/s` deadband and measure every internal
  zero-speed interval.

40083-fix pass criteria:

- 10/10 cycles and 30/30 phases succeed.
- Every waypoint uses acceleration and velocity at or below 80 percent of the
  reported controller minima, acceleration at or below 60 deg/s^2, and
  velocity at least 1 deg/s below acceleration.
- Zero CPS 40083, zero partial-goal abort, and zero uncommanded motion.
- Attach endpoint FK translation error is at most 5 mm.

Continuity observation, not yet a pass claim for the current MoveJ backend:

- Record the count, median, P95, and maximum internal zero-speed gaps.
- The production continuity target is zero internal gaps of `80 ms` or more.
- If any such gap remains, H4 is mandatory before declaring the trajectory
  executor production-ready.

## 9. H4 -- buffered trajectory backend TODO

Do not attempt to solve stop-start motion by merely increasing the Python loop
or ROS publisher frequency. Implement and qualify one controller-buffered
backend.

Preferred backend: Huayan `ServoEsJ`.

- [ ] Confirm the exact site firmware supports `HRIF_InitServoEsJ`,
  `HRIF_StartServoEsJ`, `HRIF_PushServoEsJ`, and
  `HRIF_ReadServoEsJState` with documented stop/cancel behavior.
- [ ] Add an explicit backend parameter such as
  `execution_backend:=servo_esj`; keep `waypoint` available as a controlled
  fallback until acceptance completes.
- [ ] Interpolate the MoveIt trajectory to a fixed `20 ms` grid. Use position,
  velocity, and acceleration fields when available; preserve the exact final
  point and zero terminal velocity.
- [ ] Start with `servoTime=0.02 s` and `lookaheadTime=0.2 s`.
- [ ] Batch at most 500 joint points per `HRIF_PushServoEsJ` call.
- [ ] Refill only when `HRIF_ReadServoEsJState` reports that pushing is allowed;
  poll no faster than the SDK's documented 20 ms interval.
- [ ] Define buffer underrun, late refill, cancel, connection loss, and final
  settling as explicit terminal states.
- [ ] Keep CPS telemetry/logging out of the fixed-period command path. Capture
  feedback asynchronously without allowing it to delay setpoints.
- [ ] Add offline tests for interpolation, batch boundaries at 499/500/501
  points, final-point preservation, preflight, cancellation, underrun, CPS
  error propagation, and no command after stop.

Fallback: `MovePathJOL` or controller-side `MovePathJ`. If used, measure the
controller-retimed swept path and prove collision clearance because it is not
timing-equivalent to the MoveIt trajectory.

Buffered-backend hardware pass criteria, 10 dry full cycles:

- 10/10 cycles and 30/30 phases succeed with zero CPS errors.
- Zero command buffer underruns or late-refill events.
- Zero internal zero-speed intervals of `80 ms` or more.
- Maximum per-joint path tracking error is at most `2.0 deg`.
- Final per-joint error is at most `0.5 deg`.
- Final FK TCP translation error is at most `5 mm`.
- Desired-to-command scheduling jitter: P99 at most `5 ms`, maximum at most
  `10 ms`, measured against the 20 ms servo grid.
- No adjacent-sample reverse jump greater than `0.5 deg` unless it exists in
  the desired trajectory.

## 10. H5 -- contact, vacuum, and loaded acceptance

Begin only after H0--H4 pass. Use a standardized rigid fixture before a bag.

- [ ] Verify the suction contact frame and expected fixture height before each
  run.
- [ ] Run 10 fixture contacts with slow attach and exact final stop.
- [ ] Require attach success before enabling DO0.
- [ ] Confirm DI0 within the approved vacuum timeout.
- [ ] Confirm DO0 stays on and DO1 stays off while holding.
- [ ] Retreat only after DI0 confirms seal.
- [ ] Run 10 representative bag pick-and-retreat cycles after fixture success.

Fixture pass criteria:

- 10/10 attach endpoints are reached without CPS error or protective stop.
- Contact TCP height error is within the fixture-specific approved tolerance
  and never exceeds 5 mm unless a smaller safety tolerance is specified.
- 10/10 vacuum confirmations occur within the configured timeout.
- Zero unintended DO0/DO1 overlap.

Loaded Gate 5 pass criteria:

- 10/10 cycles complete `pre_grasp`, `approach`, `attach`, DI0 seal, and
  `pick_retreat`.
- The load stays sealed and visibly clear of the container through retreat.
- No collision, protective stop, CPS error, buffer underrun, reverse jump, or
  internal zero-speed interval of 80 ms or more.

## 11. Required evidence for every run

Store each run under a new timestamped `docs/status/evidence/` directory or
copy the complete site bundle there without overwriting the original bag.

Minimum success bundle:

- exact Git revision and dirty-state manifest;
- controller/CPS/SDK versions and all six motion-limit values;
- launch commands and resolved parameters;
- `/joint_states` at at least 100 Hz;
- executor events/status and action result;
- CPS return codes plus commanded waypoint velocity/acceleration;
- TCP pose or sufficient TF for FK;
- vacuum DI0/DO0/DO1 for contact tests;
- summary containing repetitions, success counts, endpoint errors, tracking
  errors, telemetry gaps, zero-speed gaps, and scheduling jitter.

Additional failed-case bundle, captured before teardown or rerun:

- failed goal ID, phase, retained waypoint/TOTG index, controller buffer index;
- desired and actual joints for at least 1 s before through 2 s after failure;
- last accepted command, first rejected/late command, CPS code and error text;
- TF/FK and vacuum state for the same bounded window;
- network/RTT and command scheduling timestamps;
- operator observation and any pendant/protective-stop message.

## 12. Final sign-off table

| Gate | Required result | Status | Evidence pointer | Operator/reviewer |
|---|---|---|---|---|
| H0 limits | Valid six-axis limits recorded | TODO | | |
| H1 preflight | Invalid goal, zero motion | TODO | | |
| H2 conservative motion | 12/12 segments plus cancel pass | TODO | | |
| H3 accel=60 dry Gate 5 | 10/10, zero 40083 | TODO | | |
| H4 buffered continuity | 10/10, no gap >=80 ms | TODO | | |
| H5 fixture contact | 10/10 contact and vacuum | TODO | | |
| H5 loaded Gate 5 | 10/10 full pick-retreat | TODO | | |

Production approval requires all rows marked PASS with durable evidence
pointers. Passing H3 alone proves the acceleration-rejection fix, not smooth
continuous trajectory execution.

## 13. References

- `docs/status/evidence/site_replay/20260915_0915/RESULT.md`
- `docs/agents/reviews/2026-09-15_2158_elfin-real-trajectory-control-review.md`
- `third_party/huayan_python_sdk/CPS.py`
- `SDK_sample/CppLinux_SDK/src/Sample_Servo.cpp`
- `SDK_sample/CppLinux_SDK/include/HR_Pro.h`
