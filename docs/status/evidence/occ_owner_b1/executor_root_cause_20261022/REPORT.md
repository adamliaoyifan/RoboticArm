# Phase-two executor diagnosis — 2026-10-22

## Scope and direct observations

The reported exception refers to the Python ROS `MultiThreadedExecutor.spin`
thread in `pack_eval_driver`, not the trajectory planning/execution module.
Historical attempt A is preserved locally in `attempt_A/`, including the full
100MB plugin streams, driver/launch logs and joint-state replay ring. Artifact
hashes are in `manifest.json`; these large local raw files are not in Git.

The summary at 8550f7a2 asserted callback death caused feedback blackout.
Replaying the original receive timestamps contradicts that assertion:

- 144 joint-state samples accepted during the 6.001s wall-clock wait.
- Last receive is 17.277ms before deadline; maximum interreceive gap 101.884ms.
- Last joint stamp 199.300; action result stamp 197.862. Only 1.438 sim-s
  elapsed during the six wall seconds. Controller rows end at 199.290,
  consistent with their lower publication rate.
- Last position error 0.127389rad against the 0.01rad gate; joint velocity
  also exceeds its gate. No malformed/nonfinite/order rejection occurred.
- Plugin physical read at the same stamp agrees with the joint message.
- First subsequent plugin sample within position AND velocity tolerance is
  sim 199.690, monotonic 3258167.184603, 0.906872 wall seconds AFTER deadline.
- The launch log records current-box deletion at wall 1792645688.731221,
  after the timeout log at 1792645688.145. Convergence follows failure cleanup,
  so it cannot certify carrying/loaded hold or isolate the gain fix alone.
- `Exception in thread Thread-2 (spin)` follows printed suite output. Its
  preserved stack ends in Python threading bootstrap, with no exception type,
  callback name, message or input. Its exact historical cause is unknown.

## Classification

| Candidate | Evidence and conclusion |
|---|---|
| Malformed joint input | Not observed: finite coherent position/velocity rows, zero rejected samples, plugin agreement. Other historical callback inputs were not preserved, so they cannot all be certified. |
| Handoff computation error | Exact replay of unchanged algorithm returns the same timeout and out-of-tolerance last state. The gate verdict is supported. |
| Executor crash causes gate blindness | Contradicted by receive samples up to 17ms before deadline; historical footer cannot establish an earlier failure. |
| Physical nonarrival while carrying | Directly observed during the actual gate. Plugin applies corrected K=80 velocity conversion yet physical position moves away; mechanical/contact cause is still unisolated. |
| Teardown race | Pack main destroys the node/context without stopping/joining driver and dedicated TF executors. This is a source-established lifecycle defect; it is a possible explanation for the footer, not proof of its unknown exception type. |

Do not describe the exact historical exception as proven InvalidHandle,
ExternalShutdownException, bad JSON, an IK error, or a callback arithmetic bug.

## Repair and verification

`DriverExecutor` writes full traceback, exception type, phase, monotonic/wall
time and last joint boundary state before publishing failure. Handoff aborts
with DRIVER_EXECUTOR_FAILED if the spin thread fails. Pack main uses existing
`eval_lifecycle.close_driver_workers` to stop/join both executors before node
destruction. A runtime failure forces process exit 1.

52 focused behavioural tests pass, including real ROS normal shutdown and a
deliberate ValueError callback. These establish diagnostic capture/cleanup,
not the historical exception cause. A minimal 5+5 old/new-order probe yielded
zero exceptions and therefore provides no reproduction of that cause.

The phase-two runner now refuses old domain-7 product peers, verifies installed
driver identity, writes a revision/config manifest, captures into this durable
evidence tree, and stops only its own launch process group. Prior remains off;
180s diagnostic search budget and all frozen gates remain unchanged.

## Reproduce offline

With workspace ROS overlays sourced:

```bash
python3 scripts/g3/replay_phase2_handoff.py docs/status/evidence/occ_owner_b1/executor_root_cause_20261022
python3 -m pytest -q src/luggage_gazebo/test/test_driver_executor.py src/luggage_gazebo/test/test_eval_lifecycle.py src/luggage_gazebo/test/test_handoff_settle.py src/luggage_gazebo/test/test_handoff_driver_gate.py
```

## Live continuation

Live `live_20261022_144145` at 62e3e9ca reproduced HANDOFF_SETTLE_TIMEOUT
with 1 box attempted, 0 packed, but **no executor exception**. Lifecycle is
start -> stop_requested -> spin_return(failed=false) -> nodes_destroyed.
140 samples, last receive 8.101ms before deadline, final error 0.129308rad,
only 1.395 sim-s in the gate. This is a fresh reproduction of loaded physical
nonarrival independent of a callback crash. Full suite is not passed. Prior
is disabled. Owned stack stopped and no ROS/simulator residuals remain.

Two startup failures preceded it: old attempt-E peer identities survived
(19 nodes and 3 bridges confirmed by PID in its launch log), then a plugin
URDF request stalled and controller broadcaster exited. Neither attempted a
box; neither demonstrates a product/executor failure. Raw startup logs and
cleanups are preserved under their live directories.

## Pre-cleanup physical boundary: direct box/arm contact

`live_20261022_145336` at 58827abb again failed the first handoff with a
healthy executor. The post-verdict window retained the same box for 5.002
sim-s (21.709 wall-s), without a goal, release or cleanup. All 434 observed
vacuum records remained attached. Across 500 plugin physical rows, maximum
joint error grew from 0.152440 to 0.286922rad; zero rows met 0.01rad tolerance.
This rules out the earlier interpretation of quick loaded convergence.

The actual spawned SDF includes the existing observation sensor on all five
box collisions. Raw Gazebo contacts explicitly report:

```
pickup_box_0001_inch20::suitcase_link::body_collision
S20::elfin_link2::elfin_link2_collision
```

There are 542 complete contact blocks for this pair in the preserved prefix,
covering sim 301.571..302.112. Both raw transport streams reached their 8MiB
caps, so **contact evidence covers only this 0.541s prefix**, not the entire
five-second window. No penetration-depth number was recovered; do not infer
one from the geometry. The continuous joint/plugin stream covers the full
window. Lifecycle again ends normally with spin_return(failed=false), and
the owned sim/ROS stack is stopped.

This directly locates a real simulation interaction at the failed physical
boundary. It does not establish whether nominal certified geometry already
collides, tracking error first brought the box into the arm, or planning and
simulation use differing geometry/grasp transforms. Source touch-link lists
do not exempt elfin_link2 (`SUCTION_INTERFACE_TOUCH_LINKS` contains only the
suction seal; default attach links additionally contain elfin_link5/6).
These alternatives must be compared with the exact certified trajectory and
payload before changing control, collision allowances, geometry, or planning.

Phase two is still **not passed** (both actual suites: 1 attempted, 0 packed).
The historical truncated executor exception remains unknown. The separate
ordered-teardown defect is repaired; it has not been claimed as the proven
historical exception type/cause. No input-error or arithmetic-error theory has
been established by these runs.
