# 2026-11-12 -- Observed full-model Gazebo operation integration

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- base_revision: 5086d84f2873db9b36ec94d8e67efeb83481c74f
- started_at: 2026-11-12T17:35:41+08:00
- completed_at: n/a

## Summary

Integrate an explicit observed O/full-model simulation pipeline. Full-cycle
baseline remains open until an actual observed controller cycle passes.

## Requirement

Follow docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md at983e80e1.
No guessed OCC, no online simulator truth, no unilateral collision margin change.

## Changed

Explicit sim_world pipeline selection; new grouped profile; one O owner and
observed optical depth adapter; same full native model for all operation ports;
explicit start diagnostic driver; installed source checks.

## Verification

- Selection/depth/client/readiness:38 cases pass; DDS Best Effort delivery regression passes.
- scripts/production/build.sh --evidence docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/build_01:13 packages pass,6.48s,zero concurrent source changes.
- Rehearsals04/05/06: full-model sensor/controller startup ready;50 installed checks pass; owned teardown has zero residuals.
- cycles01..03: measured observation motions execute, detection/domain joined; no product-box carry yet. Failures and CDR retained.
- Full-model atlas:12,098 states native/FK requalified; distribution unchanged; hardware asset untouched.
- Map-version policy:133 offline and10 actual O/native service tests pass. Content change recertifies before issuance, duplicate observation preserves original time. Release RPC completion is awaited inside original deadline.
- Known next blocker: measured O only0.919% known active volume,0 placement candidates. Old portal view central ray points outside ULD by full-model FK;14 geometry-derived IK states yield2 valid branches for one camera view in a complete static scene; live optical coverage remains unverified.

- cycle04: first observation goal refused, elfin_link6 versus ULD mesh; no physical motion/impact demonstrated. Early ready scene was empty: previous14-state empty-world validity cannot certify environmental collision. Full-scene replay rejects12/14 states.
- Added static geometry readiness and before/after-goal full-scene capture;24 cases pass, build14 and50 install checks pass. rehearsal07 queued behind a foreign Gazebo stack; no second world launched.

## Result

Open. Composition is implementation, not a baseline success witness.

## Pointers

- docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md
- docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-GAZEBO-OPERATION/

## Collision follow-up -- 2026-11-12 19:23

Read-only exact-q native replay localizes link6 contact at the ULD upper front rim,
not O voxels:8000-triangle mesh, zero link6 padding, max reported triangle contact
depth10.536mm. Wrist origin container z2.116660m is near mesh top2.117807m; camera
origin is lower at1.864800m. Live named pair is recorded; offline contact coordinates
must not be presented as captured live contacts. Uniform2cm shell thickness and
real-model dimensional error are not established. Evidence: collision_review_01/
summary.json and analyze.py. No product changes; full-cycle baseline remains open.

## Scene and route audit -- 2026-11-12 19:29

User clarified that top-down entry was a hypothesis from the named upper-rim
contact, not an observed placement trajectory. cycle04 stopped on the first
observation target before placement preparation or carrying. This cannot prove
an over-roof placement route.

Source-established: placement.container_from_box and container-frame O runs are
explicitly composed with world_from_container; static mesh publication uses the
same configured world transform. Native connector requires a world-rooted model,
loads the supplied full scene, checks endpoints, invokes OMPL with that scene,
and checks sampled output states. No demonstrated frame mismatch in this audit;
no actual full-placement trajectory was available to certify its transform.

Confirmed admission gap: prepare._scene returns GetPlanningScene without checking
configured static objects. build_private_snapshot checks is_diff but accepts an
empty static world; synthetic installed-code probe reproduces this (zero objects
before/after). Phase scenes retain existing ULD but cannot restore missing ULD.
Driver static readiness fix is not a service-wide production guarantee. Required
next repair: enforce configured static geometry/pose identity at the preparation
boundary and revalidation, before any native search. Do not substitute scene
hash stability or is_diff=false for object completeness.

Separate search restriction: candidate generation uses vertical place entry
at release+clearance (default0.15m) and retreat at clearance+0.10m; native local
paths linearly interpolate TCP entry/contact/exit. Middle connection uses OMPL.
Thus a fixed local down/up family exists, although it does not require entering
through the roof. Its constants and route-family scope require review for deep
packing. No policy, clearance, geometry or product code changed in this audit.
Evidence: scene_admission_review_01/probe.py and summary.json. This probe only
demonstrates the Python scene-composition boundary, not a successful native plan
or actual root-service execution with an empty world.

## Aperture requirement -- 2026-11-12 19:33

User requires all robot/payload entry to use the ULD opening, to avoid searching
other entry routes. Recorded in architecture/placement.md; straight interior
corridor remains ranking-only. No implementation or successful run claimed.

MoveIt Humble exposes PositionConstraint/OrientationConstraint and per-request
path_constraints, but no dedicated ULD-entry switch. PositionConstraint applies
to one offset point on a named link and must hold throughout its request;
constraining a whole pickup-to-place request to a thin aperture slab is invalid
when the start/goal lie outside it. enforce_constrained_state_space selects a
constraint-solving approach; it does not define opening semantics. Local API
proof: /opt/ros/humble/share/moveit_msgs/msg/{MotionPlanRequest,PositionConstraint}.msg.
Native current connector sets goal_constraints only; aperture constraint not wired.

Proposed integration: retain current whole-operation owner; derive aperture from
opening_inward_datum/scene corners; generate multiple outside/inside joint-branch
pairs, using qualified atlas hints and the actual predicted grasp/payload. Plan
to a selected exterior branch, certify the crossing in a bounded aperture region,
then connect interior branch to admitted placement. Retain exact common joint
states and full-scene collision certification across joins. Candidate generation
and native checks reject non-aperture entries, rather than only post-ranking.
Check every entering body, not just TCP; observation motions obey the same rule
when any of their robot geometry enters. Starts already inside do not force an
unnecessary exit/re-entry. Exit after release uses its unloaded phase scene.

Atlas reuse can preserve static opening-connected branch/edge witnesses under
exact scene/model binding. They remain ordering/seeding evidence; measured O,
actual grasp, payload and latest scene still determine runtime motion permission.
No claimed speed gain before measuring portal proposal, native search, failures
and final certification on the same full-model workload.

Official references: https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html
and https://moveit.picknik.ai/humble/doc/how_to_guides/using_ompl_constrained_planning/ompl_constrained_planning.html.

## Fast opening candidates proposal -- 2026-11-12 19:40

User proposes a collision-free intermediate point in the opening plane, with
panel down, generated quickly. Recommend this decomposition, but return typed
TCP pose + exact joint branch + short crossing witness, not just XYZ. Point
collision freedom alone does not establish a collision-free incoming/outgoing
connection. No measured latency or completed integration claimed.

Nominal source opening in container coordinates: x=-0.745m,
y=[-0.928,0.398]m,z=[0.597,2.009]m (absolute container z). Extracted via existing
scene helper with source SHA in portal_region_review_01/summary.json. Bounds are
configuration evidence, not a physically measured/mesh-qualified passage.

Geometric broad phase for an upright box and fixed yaw: let a be box half-extent
along opening lateral direction, h its height, and d be box-centre lateral offset
from suction contact. Contact y must lie in [y_min+a-d,y_max-a-d]; contact z must
lie in [z_min+h,z_max]. These bounds deliberately omit uncertainty/tool geometry
and must not authorize motion. General form for a fixed tool/box envelope K
relative to contact origin is p_i in [A_min_i-min(K_i),A_max_i-max(K_i)] for the
two opening-plane axes. Inflate geometry or inset aperture by the explicitly
owned uncertainty once; no new default5cm assumption. Full cup-on-box-face
coverage remains independently required for the chosen grasp.

Projected fixed-tool/box envelope erosion is an efficient conservative proposal
region for a fixed-orientation transit family. It cannot label the task physically
unreachable when empty: articulating links are not rigid with the cup, some fixed
parts need not traverse the full plane, and permitted rotations/curved crossings
can require exact section/sweep analysis. Preserve actual observed small tilt in
the full transform, using the upright formula only as an explanatory special case.

Proposed cheap-to-expensive order: aperture erosion per box/grasp/yaw; qualified
static atlas branch retrieval; exact FK/IK refinement at the plane; static robot
and payload collision; O collision and approach/exit crossing checks; connect
pickup and placement branches; whole-chain certification. Prefer broad-clearance
region candidates and joint branches compatible with both ends; expand candidates
on incomplete search. Never fix one centre point, winding or entrance J6.

Reuse the existing atlas artifact with an explicit aperture section (proposal):
exact plane TCP pose/yaw, branch q, static clearance, joint-limit/singularity
metrics, and static crossing path identities. Exact plane IK is required: nearby
0.15m voxel seeds cannot be treated as plane witnesses or interpolated as proof.
Payload size and eccentric grasp remain runtime inputs. Crossing extent follows
normal projection of relevant geometry and frame thickness; do not invent a
fixed +/-centimetre pair. First try a simple passage, then flexible crossing
families under the same opening requirement. Final carry/empty exit differ.

Cache validated short paths with source identities; all changed O, actual payload,
grasp or model/scene conditions require applicable revalidation. A static path
cache is not an authorization cache. Benchmark region extraction, branch lookup,
IK/collision, short crossing, middle connection and complete certification
separately before assigning production latency targets. Current code is unchanged.
