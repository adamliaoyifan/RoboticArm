# 2026-11-12 -- Ordered opening waypoint in whole-operation planning

- status: approved
- authorization: User requests implementing the discussed opening intermediate point in the existing planning flow.
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-APERTURE-ROUTING
- generation: 1
- owner: codex/gpt-6
- base_revision: 6fd7338b302c8bde2135765d6e20d2e41fe88a58

## Scope

Keep ComputePlacement as geometric slot provider. Generate grasp-bound aperture
poses inside planning from the configured opening and conservative attached box.
Use exact plane FK/IK and qualified atlas seeds. Route carry through ordered
exterior/plane/interior states before the existing local place entry. Preserve
six-leg execution/barrier wire contract and complete native certification. Return
selected via poses/joints in preparation trace; callers do not supply waypoints.
Do not replace the separate existing local release/retreat policy in this change.
No claim that this adds a complete exterior retreat or resolves observation coverage.

Require configured static geometry at the preparation boundary so a full but
empty scene cannot bypass ULD collision checks. No guessed OCC or new margin.
Conservative geometric candidate exhaustion is incomplete search, not proof of
physical unreachability. Bounded work settings are explicit configuration.

## Acceptance

Environment: production ROS Humble install, current source scene/full model,
isolated native test fixtures (no Gazebo). Preserve unrelated dirty files; record
source/build hashes and commit. Deterministic test cases run once per revision;
all selected cases must pass (AND, no stochastic reliability claim). No latency
SLA; record per-stage timing/counts. Failure artifacts must be durable before retry.

1. Geometry tests: exact plane, transformed frames, four directed headings,
off-centre grasp, tilted conservative geometry, empty aperture region and invalid
input. Every returned pose fits the stated payload proposal region; no empty
region becomes UNREACHABLE. No arbitrary world waypoint or door-centre mandate.
2. Router behaviour: IK/current-scene rejection, ordered waypoint joins, no direct
carry fallback, alternate candidate recovery, deadline/cancellation, exact J6
winding, trace witnesses. Test doubles are explicitly not collision certificates.
3. Native connector tests: path constraints reject violating start/goal/path and
accept a constrained feasible connection. Existing unconstrained behavior remains.
Native endpoint/connection/certifier regressions run on their isolated full-pose
fixtures, with real ROS requests/responses preserved; any unsupported workload
reports not_evaluated, never a deployment success claim.
4. Production root wiring: enabled from the scene config, missing/incorrect ULD
geometry rejects before planning. Existing non-deployment fixtures inject explicit
fixture geometry, never silently disable production admission. Six-leg plan and
attachment/release barriers remain unchanged. New native/schema build passes all
13 packages and installed checks; source hashes stable during build.

## Evidence

Root: docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-APERTURE-ROUTING/.
T0: revision/dirty hashes, exact commands, cases/results/timings and artifact list.
T1: candidate counts, ordered targets/q, IK/connection reasons, scene/model hashes.
T2: failed native request/response CDR, model/config and bounded node log, one
failed case per boundary. Retain <=512MiB per fixture, <=2GiB total; no overwrites.
Inspect JSON summaries and replay pytest case with its frozen model/inputs.
No scored product cycle or sensor-simulation success is claimed by this subtask.
