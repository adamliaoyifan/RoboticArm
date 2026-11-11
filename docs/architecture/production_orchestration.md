# Production Orchestration

This document defines the production loading-cell orchestration contract. The
same state machine and exploration contracts apply in simulation and on
hardware. Adapters may differ by backend; authorization, state progression, and
online map inputs may not.

## Startup Boundary

`WAIT_START` is the initial state. Launch completion, lifecycle activation,
service readiness, sensor readiness, timers, and ordinary `run` / `step`
commands must not authorize robot motion, vacuum changes, or detection.

The only initial authorization event is an explicit operator `start` command.
After it is accepted, the orchestrator resets the cargo map and performs the
initial container exploration before prompting for a placed box.

## Operator Request Correlation

The orchestrator publishes each operator prompt with a generated request ID,
prompt kind, current state, and display text. Prompt messages must not contain
box geometry, pose, identity, dimensions, or simulation truth.

An operator pickup-ready event authorizes one cycle only when it carries the
exact current request ID and the expected schema version. Missing, blank,
stale, duplicate, malformed, or wrong-ID confirmations must not emit motion or
detection effects.

## Pure State Contract

The state reducer is a ROS-free contract. It may consume plain decoded events
and return declarative effects, but it must not call ROS, sleep, read YAML,
lookup TF, inspect Gazebo, or execute motion.

Effects describe work for a node layer, for example `CALL_ACTION`,
`CALL_SERVICE`, `PUBLISH_PROMPT`, and `PUBLISH_STATUS`. Effects that may move
the robot, change vacuum, or trigger detection must mark that capability
explicitly so tests can assert startup and request-correlation safety.

## Required Cycle

The required nominal progression is:

```text
WAIT_START
  -> RESET_CARGO_MAP
  -> EXPLORE_CONTAINER
  -> RETURN_PICK_OBSERVE
  -> WAIT_PICKUP_READY
  -> DETECT
  -> COMPUTE_PLACEMENT
  -> PLAN_PICK
  -> EXEC_PICK
  -> PLAN_PLACE
  -> EXEC_PLACE
  -> COMMIT_AND_VERIFY
  -> RETURN_PICK_OBSERVE
  -> WAIT_PICKUP_READY
```

Every return to `WAIT_PICKUP_READY` must happen after reaching
`pick_observe_pose`. The cargo map persists across placed boxes and is updated
only from live depth integration and explicit placement commits.

## Failure Boundaries

### Bound complete-plan consumer

The independently prepared complete-operation path uses
`JointOperationExecution` as a pure effect/state contract. It requires consumer
identity/digest admission and live start validation before issuing motion. Its
node adapter dispatches each correlated effect once. An `execute_leg` effect
consumes the original frozen timed path, without J6 edits or replanning.

Before carrying, `confirm_attachment` must produce a valid, confirmed measured
attachment from the existing attachment provider. Then `validate_attached_suffix`
checks provider freshness, actual q, actual frozen geometry, current scene,
certificate liveness and intended placement under the actual grasp. A mismatch
may pass only through that actual-geometry revalidation; refusal enters a carry
fault for bounded repair/recovery. It must not rewrite the prediction or release.

Before retreat, release confirmation must precede application of the released
box as a world obstacle and retreat validation. A failed release confirmation
retains payload responsibility. A confirmed release followed by blocked retreat
stops motion. Cancellation during controller execution waits for its terminal
result; transport errors alone cannot establish that motion has stopped.
Duplicate/stale callbacks issue no new effects.

This consumer has no ROS, global planner or vacuum dependency. Production node
adoption must implement and verify its effects against the native scene and
controller; pure consumer tests do not certify that adoption. Existing startup,
operator correlation and map ownership requirements apply to the complete-plan
path as well as the nominal cycle above.

Failure handling must distinguish pre-pick, carrying, release, verification,
abort, and recovery boundaries.

Before payload attachment, failures return to a non-moving wait or explicit
retry/recovery boundary. While carrying, failures transition to a carry fault
and preserve vacuum until an explicit recovery confirms payload release.
Placement commits are idempotent and must not double-count attempts.

## Exploration Boundary

Exploration policy contracts are immutable, serializable, ROS-free plain data.
Policies may propose or score candidate views, but they must not execute
motion, mutate the map, read TF, or receive ground truth.

Exploration integration must correlate the settled view request ID, source
acquisition stamp, expected map revision, and container geometry hash. Stale,
ambiguous, repeated, or mismatched view integration fails closed.

Simulation evaluation may read Gazebo truth only offline or in test/eval scope.
The online production path must not branch on `sim_mode`, consume GT, or use a
fake production success path. Drivers that call production services
(`DetectLuggage`, `ComputePlacement`, cargo-map commit, MoveIt) are in that
online path even when they live under `luggage_gazebo`. Catalog size,
`GetCurrentBox`, and unverified planned poses are GT for this rule. The
defect catalog is [real_scenario.md](real_scenario.md).
