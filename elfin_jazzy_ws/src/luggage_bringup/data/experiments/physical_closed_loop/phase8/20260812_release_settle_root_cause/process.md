# Phase 8 process log — EX1 release settle

Chronological: hypothesis, measurement, what it ruled out.

## 1. Two plausible causes were dead before any experiment ran

The failure message said "release settle failed after hold 3.01s/0.0318", and
the obvious readings of that are gravity sag under the payload or contact
noise from the box touching down. Reading the configuration ruled out both
before spending a minute of simulation:

- `gazebo_pid_gains.yaml` is `pid_gains: {}`. With a `PositionJointInterface`
  that selects Gazebo's kinematic `SetPosition` path: the joint is placed at
  the commanded angle every step. There is no effort loop and no gravity term
  acting on the arm.
- `vacuum_simulator_node.gazebo_follow` moves the box with
  `set_model_state` at 50 Hz and `state.twist = Twist()`. The box is a slave of
  the suction frame; there is no mechanical path from the box back to the arm.

So whatever the residual is, the arm cannot be feeling either the payload or
the floor.

## 2. What the reported number could not tell us

`_wait_robot_settled` returned `max_v_seen`, one number for a three-second
window, with no joint attribution and no shape. Three failures clustered at
0.0313 / 0.0302 / 0.0318 -- within six percent of the tolerance -- which is
too tight for a physical decay and looks more like a floor. But "looks like"
is not evidence, and the number as reported could not settle it.

So the first change was structural: pull the rule into `settle_criterion.py`
as a pure function, and record traces with `joint_velocity_trace.py`, which
needs only `sim_world.launch`. The diagnostics gained a peak joint, a
tail-to-peak ratio and, later, the position excursion.

## 3. Experiment A: idle noise floor. H1 refuted as stated

Arm parked at observe, not commanded, 60 s, 3000 samples:

    max over joints: p50 = p95 = p100 = 0.0013316 rad/s

Twenty-three times below the tolerance, and with **zero variance to seven
digits**. So the hypothesis as posed -- "the tolerance sits under a noise
floor" -- is wrong at this pose. But the zero variance is itself the finding:
this is not noise. Every joint reports its own fixed non-zero value forever.

That reframed the question from "how noisy is it" to "what does that constant
depend on".

## 4. Experiment B: it depends on the configuration

Perturbing the shoulder by 0.3 rad in either direction took the reading from
0.0013 to 0.0117 / 0.0100, and from that point on `elfin_joint2` -- the joint
carrying the most gravity torque -- dominated every sample.

Sweeping the shoulder:

| elfin_joint2 | steady reading |
|---|---|
| -1.33 (observe) | 0.0012 |
| -1.10 | 0.0090 |
| -0.90 | 0.0145 |
| -0.70 | 0.0206 |
| -0.50 | 0.0253 |
| -0.30 | 0.0275 |
| -0.10 | 0.0345 |
| 0.00 | 0.0352 |
| 0.10 | 0.0359 |

Monotonic in extension, crossing 0.03 exactly where the failures were.

This also explains the timing: EX1 appeared only after E25 removed the
near-ROI rectangle and placements moved further out than any previously
accepted run. The gate had never been exercised in that part of the workspace.

## 5. Experiment C: it is not a transient

Recording started with zero delay after each trajectory ended. The value at
the first sample already equals the steady value and does not decay
(`tail_ratio = 1.00` everywhere). Raising `release_settle_timeout` was never
going to help, which retires the hypothesis the previous failure note had put
first.

## 6. The decisive measurement: nothing was moving

The recorder captured controller `desired` / `actual` / `error` alongside. Over
ten seconds at the pose where the reading was 0.0092 rad/s -- which would mean
0.098 rad of travel:

    elfin_joint2 actual position drift = -1e-8 rad
    tracking error = -9e-6 rad, constant

The joint was static to eight decimal places. The reported velocity is a
readback artifact of the kinematic mode and has no relationship to motion.

## 7. The part that changes how serious this is

If the signal only read high at rest, the gate would merely be over-strict.
It is worse. On a window taken from the middle of a real move:

    actual speed from positions : 0.4786 rad/s
    reported peak velocity      : 0.0349 rad/s
    under-report                : 13.7x

and the old rule called that window **settled**. The gate that was blocking
motionless arms would have released the payload while the arm was traversing
at half a radian per second. The direction of the defect is the dangerous one.

That is why the fix changes the signal rather than the estimator. An earlier
sketch made the rule tolerate a quantile of samples above tolerance; that would
have papered over the bias at rest and left the mid-motion hole wide open. The
quantile knob is kept, but it is not the fix.

## 8. The fix: measure the movement

Over the hold window, no joint may move more than `vel_tol * hold_time`. Same
two numbers, so nothing is loosened: a joint creeping at the tolerance uses
exactly the whole budget and is still rejected. Peak-to-peak rather than
start-to-end, or an oscillation returning to its start would slip through --
there is a unit test for precisely that.

Both criteria stay available and the diagnostics print both, because a large
reported velocity beside a near-zero excursion is the fingerprint of this bias
and should stay visible if it ever shows up somewhere else.

## 9. A regression I introduced and had to fix

`_wait_robot_settled` started reading `self._settle_quantile`, and three tests
in `test_replan_logic.py` build a `MotionPlanner` through `__new__` without
running `__init__`. Settling must not depend on how the object was
constructed, so both new settings are read with `getattr` and a default.
Caught because the planning suite's per-file baseline is known.

## 10. What is deliberately not claimed

No full pipeline run. This phase closes the gate, not the blocker: EX3-b is
still open and the Phase 5 matrix rerun is a separate step. What is claimed is
narrow and measured -- at the exact configurations that failed 20 out of 20
times before, the arm is motionless, and the corrected rule passes 20 out of 20
in 0.26 s.
