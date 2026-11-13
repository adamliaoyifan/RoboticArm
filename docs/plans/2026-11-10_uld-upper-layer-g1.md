# ULD upper-layer placement G1: independent offline feasibility prototype

User authorization: create a separate action line for a ceiling-constrained
ULD. Existing downward-only pick gates, atlases and Gazebo suction shortcuts
do not constrain this new action. Production routing is future work: trigger
on a feasible upper slot that the normal action cannot serve, never infer a
slot from a generic BIN_FULL result.

## Ownership and revision

- Parent: ULD-UPPER-LAYER-20261110
- Subtask: OFFLINE-G1
- Owner: eng/codex/gpt-6
- Base: c7039905cd9c186f27904a975ba93762ec5a49fe
- Worktree: /tmp/elfin_uld_upper_layer_g1
- Coordination: primary workspace docs/agents only.
- Plan revision: the commit containing this document, recorded in dispatch.
- Scope: research/uld_upper_layer only; no production changes or live robot.

## Questions and boundary

Test whether the existing fixed tool assembly can transfer a horizontally
carried box onto a support plane under the ceiling, release, withdraw and
reach a rear pushing pose. Model a rigid box and rigid filled support prism.
The upward carry is an input state; pickup and the outside inversion itself
are not evaluated in G1. Friction, suction peeling, unknown COM, force control,
real sensor observability, full-arm collision and continuous path planning
remain explicitly not_evaluated. Stage IK is not proof of a connected route.

Use a standalone action-stage generator with full fixed end-effector collision
geometry, including panel, adapter, camera and lidar. Keep contact modes and
permitted support contact explicit. Conservative per-collision local bounding
boxes may reject valid geometry; never call their rejection physical
impossibility. Preserve meshes/transforms and witness pairs for follow-up.

## Fixed experiment

Default scene_tf.yaml from base revision, S20 full camera URDF, and the
carryon/standard/large catalog rows. Test both horizontal long-axis choices;
ceiling gaps 0.01/0.03/0.05 m; support front at opening or 0.10 m inside;
COM distance fractions 0.4/0.5/0.6 from the leading edge; centered and maximum
rearward legal contact position; four tool-up yaws 0/90/180/270 degrees.
Total 864 combinations. Box lateral center is aperture midpoint. Contact
footprint and edge margin come from suction_contact_model.yaml. Require a
0.02 m longitudinal static COM margin, 0.005 m obstacle clearance and 0.005 m
placement lift (declared research settings, not product specifications).

Compute handoff interval and stage poses. Check payload aperture/ceiling,
fixed tool versus support/ceiling/walls/payload, permitted support touch, and
fixed-tool stage translations at <=0.01 m spacing. Stage orientation changes
are not treated as validated transitions. End-effector collision clearance
uses enclosing boxes and is conservative. Minimum window width must be
positive beyond numerical tolerance 1e-6 m to count as a geometry candidate.

For at most one deterministic geometry candidate per box/long-axis pair
(up to six), run stage IK for upward approach, handoff, withdrawal and rear
push start/end with 16 fixed seeds plus the preceding successful seed.
Position residual <=0.0005 m and orientation residual <=0.005 rad; enforce
URDF joint limits. Save all attempts, target transforms and winning joints.
No solution within the bounded search is search_exhausted, not unreachable.
If no geometry candidate exists, IK is not_evaluated with reason.

## Acceptance and commands

Task success means an honest, reproducible measurement, not that the robot
must pass. One deterministic pass through all 864 cases, no reliability rate
inference. Report counts by box, grasp and first failing stage; retain every
case and stage witness. Require exactly 864 records and zero unclassified
cases. Zero candidates is a valid negative G1 finding. IK has a minimum
sample rule of one candidate; zero is explicitly not_evaluated.

Run `python3 -m unittest discover -s research/uld_upper_layer/tests -v`.
Behavioral tests must cover stable/unstable COM, centered versus rearward
handoff, boundary contact/margin, an intentionally blocked tool, box/ceiling
failure, stage contact transitions and rigid transform consistency. All
tests must pass. No source-text assertions.

Run `python3 research/uld_upper_layer/run.py --repo . --output <EVIDENCE>` in
a ROS Humble sourced shell (for local xacro/ament imports only; no ROS nodes).
Keep geometry and IK reports separate. Generate a standalone diagram of a
candidate or rejection witness. Verify `git diff --check` and run the agent
contract checker, reporting pre-existing failures without changing them.

## Evidence and revision rules

EVIDENCE is primary docs/status/evidence/uld_upper_layer_feasibility/g1_<time>.
Before running, commit experiment code in the isolated branch; record that
exact commit, dirty state, environment/package versions, all input hashes,
parameters, timestamps, commands and seed list. Save expanded URDF and a
geometry snapshot. The shared primary dirty tree is not an experiment input.

Boundary dump matrix:

| Boundary | Always saved | Failure/rejection evidence |
|---|---|---|
| Asset expansion | URDF, source hashes, scene/catalog, box bounds | exception and missing asset path |
| Handoff generation | all case inputs, interval, COM margin | reason and limiting dimensions |
| Tool stage checks | contact mode, target, obstacle IDs | first collision pair and transforms |
| IK | every target/seed/residual and accepted joints | exhausted attempts; no blind rerun |
| Aggregation | counts, limitations, artifact inventory | incomplete if any record missing |

Limit artifacts to 100 MB per run; no continuous sensor streams. Flush case
JSONL before attempting IK. No simulator is started, so no sim teardown is
needed. Future physics experiments must comply with sim exclusivity and
debug-evidence rules independently.
