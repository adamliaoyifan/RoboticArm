# SIM-R1 Production Orchestrator and Pluggable Exploration Plan

Date: 2026-09-04

Status: approved by the user and distinct Codex GPT-5.5 consensus. The Git
commit containing this approved text is the required reproducible plan
revision for runnable mailbox rows.

Parent task: `SIM-R1-20260904`

Base source revision: `7af40220c9e86feb7f56908d9fcde389aa48dd9d`

## Objective

Provide the ROS 2 production control path for a human-assisted loading cell:

1. Launching or activating the stack never moves the robot.
2. A visible operator confirmation sends an explicit `start` command.
3. The robot explores the empty container with live depth, then returns to
   `pick_observe_pose`.
4. An operator places one box and publishes a confirmation for the currently
   displayed request ID.
5. The same production state machine detects, picks, places, commits, verifies,
   and returns to `pick_observe_pose`.
6. Steps 4-5 repeat for at least three boxes without resetting the container or
   clearing previously placed boxes.

Exploration is a replaceable policy behind one stable coordinator. The default
is prior-guided, stop-and-look NBV. A later learned model may replace proposal
or scoring behavior, but it cannot bypass geometry, timing, collision,
reachability, motion, map-revision, or termination safety gates.

## Fixed Decisions

### Operator protocol

- The initial state is `WAIT_START`. Configure, activate, launch completion,
  service readiness, and sensor readiness are not start events.
- The existing `/orchestrator/step` service gains the explicit command
  `start`. `run` and `step` are rejected before `start` succeeds.
- The operator surface must visibly show `WAIT_START` and require a deliberate
  Start confirmation before calling `start`. A launch option may start the
  node, but no launch option may synthesize operator consent.
- After initial exploration, and after every completed or safely recovered
  cycle, the robot reaches `pick_observe_pose` before entering
  `WAIT_PICKUP_READY`.
- The orchestrator publishes the current prompt on
  `/luggage/operator/prompt` with RELIABLE and TRANSIENT_LOCAL QoS. The prompt
  contains a generated `request_id`, prompt kind, state, and display text.
- The operator publishes `/luggage/operator/pickup_ready` with RELIABLE and
  VOLATILE QoS. The payload contains the exact `request_id` and optional
  `operator_id`; it contains no box geometry, pose, identity, or simulation
  truth.
- Missing, stale, duplicate, malformed, or wrong-ID confirmations do not cause
  motion. One request ID authorizes at most one cycle.
- Abort and pause remain state-boundary operations. Canceling a motion must not
  silently turn vacuum off while carrying a payload.

The exact message names may be amended during consensus, but request-ID
correlation, QoS semantics, and the absence of geometry in the operator event
are binding.

### Production cycle

The required state progression is:

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

Initial exploration occurs before an operator is asked to place a box.
Subsequent placed-box state persists. A later re-observation may be requested
by a measured-confidence failure, but it is not allowed to use Gazebo truth and
must not move while an unacknowledged pickup box may be present.

Every transition is represented by a ROS-free event and produces declarative
effects such as `CALL_ACTION`, `CALL_SERVICE`, `PUBLISH_PROMPT`, or
`PUBLISH_STATUS`. The pure state machine does not call ROS, sleep, read YAML,
lookup TF, or execute motion.

### Exploration method

The default method is prior-guided discrete NBV:

1. Use the calibrated container hull as a boundary and view-generation prior,
   never as observed free-space truth.
2. Build an immutable snapshot from depth-derived occupancy, frontier and
   visibility data, acquisition stamps, geometry hash, and map revision.
3. Generate a bounded candidate set from opening/interior geometry and
   reachability priors.
4. Score expected information gain, unknown/frontier reduction, floor and
   placement-corridor coverage, occlusion, motion cost, and repeated-view cost.
5. Apply canonical hard gates before selection: finite schema, frame and
   geometry identity, current map revision, camera envelope, true inner-hull
   constraints, collision, IK, and motion feasibility.
6. Move to one accepted view, wait for measured settle, integrate only depth
   acquired for that settled view, and update the map revision.
7. Stop on task-sufficient coverage or converged gain, subject to fixed minimum
   evidence and maximum view/time budgets.

Continuous scanning is out of scope until motion-time depth deskew is validated.
The initial implementation uses the existing ROS-free view generator, scorer,
reachability atlas, and termination helpers instead of porting the legacy
`rospy` node line by line.

### Replaceable policy boundary

The stable ROS-free contract consists of immutable, serializable plain-data
types:

- `ExplorationContext`: frames, normalized geometry descriptor/hash, camera
  model, policy configuration, and hard budgets.
- `ExplorationSnapshot`: acquisition stamp range, map revision, occupancy and
  visibility summaries, current robot state, visited candidate IDs, and
  diagnostics. It contains no ROS message and no simulation/eval truth.
- `PolicyProposal`: policy/schema ID, one or more candidate sensor poses or
  scores, candidate IDs, optional done recommendation, and diagnostics.
- `ViewOutcome`: accepted/rejected/executed/integrated result, reason code,
  acquisition stamp, and resulting map revision.

One composite `ExplorationPolicy` supports `reset`, `propose`, and `observe`.
The production default, `HeuristicNbvPolicy`, composes the existing geometry
candidate and information-gain modules. A future `LearnedExplorationPolicy`
implements the same interface, either in-process or through a node-layer model
adapter. Plugin selection is explicit configuration using an allowlisted
registry or Python entry point, not `if sim` branches.

The coordinator owns validation, deterministic tie-breaking, policy lifecycle,
view/session accounting, and hard termination. A policy may recommend `done`
but cannot waive minimum map validity. A policy never executes motion, mutates
the map, reads TF, or receives GT. Invalid model output fails closed. Production
fallback is disabled unless an explicit fallback policy is configured and
reported in status/evidence; silent fallback is forbidden.

### ROS boundary

- The orchestrator calls the exploration subsystem through
  `PlanNextCargoView.action`; it does not import or branch on policy classes.
- The exploration ROS 2 node converts messages and resolves action/service
  outcomes. It contains no NBV scoring or state-machine logic.
- `IntegrateCargoView` must correlate integration to the requested settled view,
  exact source acquisition stamp, expected map revision, and geometry hash. The
  current "latest cloud" service contract is insufficient and must fail closed
  rather than integrating an ambiguous or stale frame.
- Long robot movement remains action-based and cancellable. The policy action
  cannot command a robot directly; execution is mediated by the coordinator's
  validated effect and the existing motion action surface.
- The cargo map is updated from live depth and explicit placement commits. Eval
  may compare it with GT but cannot publish corrections into the online path.

## Simulation, Hardware, and Test Separation

- Simulation and hardware instantiate the same orchestrator, coordinator,
  policy, geometry, mapper, and planning functions.
- Backend-specific code is limited to sensor/device and actuator adapters plus
  launch configuration. Production modules contain no `sim_mode`, fake-spawn,
  Gazebo model-state, or eval-GT branch.
- Simulation exploration consumes the bridged RGB-D/depth products used by the
  online perception path, not Gazebo entity state.
- Hardware exploration consumes the corresponding calibrated camera products.
- Eval alone may read Gazebo truth. Future hardware eval may reconstruct a
  reference from ROS 2 bag image and point-cloud data offline.
- Unit tests may inject input messages or plain decoded events and inspect the
  exact same state-machine/coordinator outputs. Test doubles live under test
  scope, are not installed, and cannot be selected by production launch.
- Tests must not publish to a non-isolated running robot/simulation graph. ROS
  tests use an isolated `ROS_DOMAIN_ID` or pure in-process adapters.

## Legacy Handling

- Move the current ROS 1
  `src/luggage_bringup/scripts/orchestrator_node.py` to
  `scripts/ros1_reference/` without extending it.
- Install a new thin rclpy executable under the original production node name.
- Convert `luggage_bringup` from catkin to the repository's ROS 2 ament pattern.
- Legacy exploration code is reference input only. Maintained ROS 2 code wraps
  existing ROS-free algorithms and must not reproduce the monolithic `rospy`
  structure.

## Failure Policy

- Before payload attachment, a failed operation returns to a non-moving wait or
  explicit retry state and requires a fresh operator decision where motion
  would resume.
- While carrying, failure preserves vacuum and transitions to a dedicated
  `CARRY_FAULT` state. It must not pretend the box was placed or update the map.
- Placement commits are idempotent and occur only after successful release and
  verification. A retry cannot double-count a box.
- `placed_count` is derived from committed identities, not attempts.
- Exploration rejects stale stamps, map revisions, geometry hashes, repeated
  candidate IDs, invalid poses, failed settle, or ambiguous integration.
- Max views, timeout, cancel, no feasible view, and insufficient coverage have
  distinct stable reason codes.

## Subtask Decomposition

All subtasks are assigned to one implementation identity,
`codex-sim-r1-eng/gpt-5.5`, and therefore execute serially. The distinct
consensus identity `codex-sim-r1-consensus/gpt-5.5` reached consensus in
`docs/agents/discuss/2026-09-04_1801_sim-r1-gpt55-consensus.md` and performed
no code changes. Runnable rows use the commit containing this approved plan.

| ID | Owner | Depends on | Exclusive primary scope | Observable outcome |
|---|---|---|---|---|
| SIM-R1-1 | `codex-sim-r1-eng/gpt-5.5` | none | architecture contract, `luggage_msgs` operator/exploration interface updates, ROS-free orchestration event/effect types | Versioned interfaces and pure contracts compile/import; no motion can be emitted before explicit Start |
| SIM-R1-2 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1 | new exploration coordinator/plugin modules and focused tests; existing view modules are consumed, not rewritten where TCIG owns them | Heuristic and test model policies use one contract and the same immutable hard-gate path |
| SIM-R1-3 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1, external TCIG-2 | ROS 2 cargo mapper sensor integration, adapters and tests | Settled stamped depth updates the hull-aware map exactly once and returns a new revision; stale/mismatched input is rejected |
| SIM-R1-4 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-2,SIM-R1-3, external TCIG-7 | thin rclpy exploration action node, policy loading, validated motion/integration sequencing | `PlanNextCargoView` runs stop-and-look NBV without exposing policy internals or GT |
| SIM-R1-5 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1 | `luggage_bringup` ament migration, ROS 1 reference move, pure production state machine, thin rclpy orchestrator, operator controls | Launch is motionless until Start; exact pickup request permits one cycle; all failure states are observable |
| SIM-R1-6 | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-4,SIM-R1-5 | hardware/simulation launch composition and profile parameters; no fake runtime backend | Both backends instantiate the same production nodes/functions with only adapter/config differences |
| SIM-R1-INTEGRATION | `codex-sim-r1-eng/gpt-5.5` | SIM-R1-1..SIM-R1-6 | cross-package repair, owner-run full test and Fortress E2E evidence | Explicit Start, initial exploration, and persistent three-box operator-confirmed closed loop pass end to end |

`external TCIG-2` and `external TCIG-7` are release prerequisites rather than
same-parent mailbox dependencies. Reviews must not dispatch SIM-R1-3 or
SIM-R1-4 until their accepted revisions are available, and SIM-R1 work must not
edit files exclusively owned by an active TCIG subtask.

## Checkpoint Gates

### Gate R1: interfaces and pure orchestration

- Plain `pytest` imports the state machine and event/effect contracts without a
  ROS installation.
- Startup, readiness, timer, sensor, duplicate command, `run`, and `step` events
  produce no motion effect in `WAIT_START`.
- Exactly one valid `start` transitions into initial exploration; duplicate
  Start is rejected or idempotently reported without a second execution.
- Wrong, stale, duplicate, and malformed pickup-ready messages produce no
  detect or motion effect. The exact current request ID authorizes one cycle.
- State failure tables cover pre-pick, carrying, release, verification, abort,
  and recovery boundaries.

### Gate R2: policy replacement

- `HeuristicNbvPolicy` and a test-only deterministic model policy produce the
  same proposal schema and run through the same coordinator validation.
- Policy selection changes through approved configuration without changing
  orchestrator code.
- NaN/Inf, wrong frame/hash/revision, out-of-envelope, hull-invalid,
  unreachable, colliding, repeated, and stale proposals are rejected before a
  motion effect is emitted.
- A learned policy cannot mutate its input snapshot or coordinator-owned map
  state; outputs and getter data are defensive copies or immutable values.
- Silent fallback is covered by a negative test.

### Gate R3: stamped depth integration

- One accepted settled view integrates only correlated source frames and bumps
  the expected map revision exactly once.
- N-1 frames, stale TF, unsettled status, geometry mismatch, revision mismatch,
  empty/unsupported clouds, and repeated integration requests fail closed.
- The same recorded camera-message fixture drives simulation and hardware
  adapters into equivalent algorithm input.
- No mapper or exploration production import references Gazebo state/eval APIs.

### Gate R4: ROS 2 exploration action

- Preview is read-only; reset creates a fresh session; cancel does not commit a
  candidate or integrate a frame.
- Each executed view follows validate, move, settle, integrate, observe order.
- Map revision returned by integration becomes the next snapshot revision.
- Information gain/unknown coverage improves on deterministic fixtures;
  repeated views are avoided and termination occurs on convergence or budget.
- Action feedback reports policy ID, stage and candidate ID without leaking GT.

### Gate R5: production orchestrator

- The old `rospy` orchestrator exists only under `ros1_reference` and is not
  installed. The installed node is rclpy and thin.
- Launching all nodes for at least 60 seconds without Start produces zero motion
  action goals and zero vacuum commands.
- A visible operator Start confirmation is required before initial movement.
- The robot explores before the first pickup prompt and is at
  `pick_observe_pose` whenever `WAIT_PICKUP_READY` is published.
- No spawn, clear-current-box, Gazebo model-state, or GT service appears in the
  production state machine.
- A carrying fault preserves vacuum and does not increment `placed_count`.

### Gate R6: backend parity and E2E

- Unit tests only inject messages/events and inspect effects; test doubles are
  absent from installed package files and launch parameters.
- Static checks reject production `sim_mode`, fake backend, spawn, Gazebo
  model-state, or eval-truth imports in orchestrator/exploration algorithms.
- Fortress starts with real bridged depth and stays motionless before Start.
- After Start, initial exploration terminates with accepted map evidence and
  returns to the pickup observe pose.
- Three distinct exact-ID operator confirmations produce three persistent
  placed-box commits. Previously placed boxes remain in scene/map, no box is
  cleared between cycles, and `placed_count == 3`.
- Online logs contain policy ID, geometry hash, acquisition stamps, map
  revisions, candidate outcomes, request IDs, and stable failure reasons.
- Eval obtains identity and geometry truth independently and verifies, without
  publishing into the online graph, that all three boxes are inside the true
  container hull and do not overlap.
- Teardown follows `.cursor/rules/sim-lifecycle.mdc`; residual relevant
  processes are zero.

## Non-goals

- Platform-free pickup-height remediation and its remaining gates.
- TCIG geometry-kernel, placement, corridor, metric, offline-policy, or atlas
  implementation owned by the active TCIG plan.
- Platform tilt, arbitrary non-convex container reconstruction, continuous
  motion scan deskew, automatic upstream box feeding, or autonomous replacement
  for the operator-ready event.
- Training a learned exploration model. This task provides and tests the plugin
  boundary only.

## Risks and Controls

- The current ROS 2 cargo mapper is source-geometry-only. SIM-R1-3 is blocked
  until TCIG-2 stabilizes its hull-aware map surface so sensor integration does
  not create a competing geometry convention.
- Current exploration interfaces predate exact stamp/hash correlation. SIM-R1-1
  must version or extend them before integration code relies on them.
- Existing `luggage_bringup` is catkin and contains many historical tools. The
  migration must explicitly classify installed ROS 2 tools versus ROS 1
  references; unrelated reference behavior is not ported implicitly.
- Three-box success can still be limited by placement reachability. The E2E
  gate records a planning failure as failure; it must not remove prior boxes or
  substitute GT placement to pass.

## Required Evidence

Each subtask owner records focused test commands and results in an eng role
note. The integration run stores logs and machine-readable metrics under:

`docs/status/evidence/sim_r1/<timestamp>/`

Evidence must identify the exact code revision, profile, policy ID, geometry
hash, ROS domain, seed, box identities, map revisions, and teardown result.
