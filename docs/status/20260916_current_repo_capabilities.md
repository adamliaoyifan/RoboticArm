# Current Repository Capabilities and Deployment Readiness

Date: 2026-09-16

Product revision assessed: `64a57367ba15f8cc6fa7f16165ef12b559986a6f`

Review record: `docs/agents/reviews/2026-09-16_1511_current-deployment-readiness.md`

## Executive summary

The repository supports supervised commissioning and engineering trials, but
is not ready for production autonomous pick-and-place. Sensors, perception,
planning, a single-box pick workflow, vacuum control, hardware trajectory
execution, placement algorithms, and simulation/evaluation tools exist. The
complete production cycle has not been integrated or qualified on the real
cell.

Real deployment is currently appropriate only for operator-controlled tests
with a trained person at the e-stop. It is not approved for unattended use,
loaded multi-box operation, or production placement.

## Capability summary

| Capability | Current status |
|---|---|
| D555 and Livox acquisition | Available; hardware smoke evidence exists |
| RGB-D preprocessing and detection | Functional; simulation evidence and encouraging real-bag replay exist |
| Single-box pick planning | Implemented |
| Real pick execution | Commissioning only; latest Gate 5 attempt failed before vacuum enable |
| Hardware trajectory executor | Safety defects repaired and verified offline; real H0--H5 qualification is incomplete |
| Placement constraints and candidate generation | Implemented and simulator-tested |
| Real placement and verification | Not evaluated |
| Production orchestration contract | Defined and tested as a ROS-free state machine |
| Production orchestration runtime | Not integrated on current `master` |
| Persistent multi-box autonomous cycle | Not integrated or accepted |

## Picking policy

The current hardware-pick procedure is explicit for a human-assisted,
single-box workflow:

1. Move to the pickup observation pose.
2. Detect the available box.
3. Select the first detector result.
4. Require a valid top surface.
5. Execute `pre_grasp`, `approach`, `attach`, and `pick_retreat`.
6. Enable vacuum after reaching `attach`.

This is commissioning behavior, not a complete production picking policy.
There is no general multi-object ranking policy, and recovery/re-observation is
not integrated into the production state machine. The diagnostic driver also
releases vacuum after a later segment failure or process exit. That conflicts
with the production requirement to preserve vacuum during a carrying fault
until an explicit recovery decision.

## Placing policy

Placement candidate generation and hard constraints are substantially defined.
Candidates are checked for support, clearance, container hull, aperture,
overlap, insertion corridor, and reachability. Simulation has demonstrated
place-only execution using perfect geometry.

The production policy is not yet singular:

- the ROS 2 placement service uses `placement_solver.solve_placement`;
- offline replay described as using the production policy uses
  `placement_scoring.score_candidates`.

These paths do not use identical scoring behavior. Failure semantics also need
repair because invalid input and ordinary candidate exhaustion can both be
reported as `BIN_FULL`. Real sensor-to-map placement, release, verification,
and persistent multi-box operation have not been qualified.

## Orchestrator assessment

The normative orchestration design is sound. It requires:

- explicit operator Start before motion or detection;
- one-shot request-ID correlation for pickup authorization;
- persistent live cargo-map state;
- distinct pre-pick, carrying, release, verification, and recovery faults;
- vacuum preservation during carrying faults;
- idempotent placement commits;
- no online Gazebo truth or production fake-success path.

The current runtime does not implement that production design. On current
`master`, `luggage_bringup` remains a catkin/ROS 1 package, is excluded from
colcon, and contains the legacy `rospy` orchestrator. Accepted ROS 2
orchestrator and exploration-policy implementations exist on separate branches
but are not integrated into `master`. Stamped depth/map integration, the ROS 2
exploration action, backend parity, and complete end-to-end orchestration remain
unfinished.

## Blocking work

1. **Qualify real trajectory execution.** Complete H0--H5: controller-limit
   capture, zero-motion preflight rejection, conservative dry motion,
   cancellation, continuity measurement, fixture contact, vacuum, and loaded
   pick-and-retreat.
2. **Resolve stop-start motion.** The waypoint backend has produced internal
   zero-speed gaps of approximately 160--200 ms. Implement and qualify a
   buffered controller backend if the hardware measurement still exceeds the
   80 ms production limit.
3. **Integrate the ROS 2 production runtime.** Bring the accepted orchestrator
   and exploration policy onto the current product revision, then implement
   stamped map integration, exploration action wiring, and backend composition.
4. **Make carrying recovery fail-safe.** Replace commissioning-time automatic
   vacuum release with `CARRY_FAULT` and explicit operator recovery behavior.
5. **Unify placement policy.** Use one scorer online and offline, repair
   failure reasons, and propagate geometry hash and map revision through
   placement commit and verification.
6. **Qualify real placement.** Demonstrate fixture placement, release,
   verification, return-to-observe, and persistent multi-box updates before a
   loaded multi-box campaign.
7. **Create a release artifact.** Build, test, and deploy from one clean,
   published, exact revision; do not certify the current shared dirty tree.

## Deployment decision

**Approved:** supervised sensor checks, replay, detection, plan-only runs,
small conservative commissioning motions, and operator-gated hardware
qualification following the existing safety checklist.

**Not approved:** unattended operation, production picking, carrying-fault
recovery, real placement, or autonomous multi-box loading.

Production approval requires all real-trajectory H0--H5 gates, one integrated
ROS 2 production orchestrator, one authoritative placement policy, and durable
end-to-end real-cell evidence on a clean exact revision.

## Primary references

- `docs/architecture/production_orchestration.md`
- `docs/plans/elfin_real_trajectory_hardware_acceptance.md`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/status/20260915_gate5_night_summary.md`
- `docs/agents/reviews/2026-09-16_1511_current-deployment-readiness.md`
