# 2026-11-12 -- gazebo-operation-integration

- status: open
- to_role: eng
- to_agent: codex
- to_model: gpt-6
- kind: subtask
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- depends_on: ST-4-PREPARATION-LATEST-MAP
- revision: 5086d84f2873db9b36ec94d8e67efeb83481c74f
- generation: 1
- plan_revision: 983e80e1
- dispatch_ready: yes

## Post -- reviews/codex -- 2026-11-12 17:35 -- codex/gpt-6

Wire explicit observed O full-model Gazebo operation pipeline and client; rehearse startup, execute diagnostic cycle with complete failure evidence; keep baseline open until actual success.

## Pointers

- `docs/plans/JOINT_OPERATION_GAZEBO_INTEGRATION_20261112.md`

## Open

- Wire explicit observed O full-model Gazebo operation pipeline and client; rehearse startup, execute diagnostic cycle with complete failure evidence; keep baseline open until actual success.

## Claim -- eng/codex -- 2026-11-12 17:35 -- codex/gpt-6

- started_at: 2026-11-12T17:35:41+08:00
- claimed_generation: 1
- claimed_plan_revision: 983e80e1
- claimed_dependencies: ST-4-PREPARATION-LATEST-MAP=1

## Post -- eng/codex -- 2026-11-12 18:07 -- codex/gpt-6

Three unscored startup attempts preserved. Review completed: (1) missing launch
argument declarations; (2) client read identity before launch materialized it;
(3) controller activation transient sampled once, plus actual Best Effort /
Reliable domain incompatibility. No product controller goal issued. Next bounded
startup attempt uses measured controller readiness and actual DDS delivery test,
with same full-model/O policy. Baseline remains open. Source/archive/CDR/log
pointers: ST-4-GAZEBO-OPERATION/rehearsal_01..03 and offline_01/qos_transport.log.

## Post -- eng/codex -- 2026-11-12 18:18 -- codex/gpt-6

Startup rehearsal_04 passed, 3,525 sensor/state messages, both controllers active,
all required endpoints present, clean owned teardown. cycle_01 executed both
MoveGroup observation motions and generated sensor O, then stopped before prepare:
detection at50.556s lacked measured height, exact domain invalid; first subsequent
valid domain51.216s. Evidence and source archive preserved, zero residual processes.
Diagnostic client now waits within the existing30s observation budget for complete
measured geometry and an exact header/domain join; no admission relaxation.
11 behavioural readiness cases pass. Latest-only prepare/context association race
remains a source concern to measure at its actual boundary. User confirms online
IK diagnosis first, but final success baseline requires requalified full-model atlas.

## Post -- eng/codex -- 2026-11-12 18:25 -- codex/gpt-6

cycle_02 passed domain admission and frozen retention, then GetSnapshot refused
REVISION_UNAVAILABLE. Client erroneously used GetValidity's transition-time
revision1 while actual O receipts were revision5. Fix binds a real current
snapshot lease (same1000ms/allow_stale=false), retains it through preparation and
releases in finally. 13 client/readiness behavioural cases pass. No movement of
product box, no collision-policy change. Evidence: cycle_02/failure_summary.json.
Separately, observed O INTEGRATED receipts with zero free/hit keys advance the
global stamp; they cannot establish regional ULD freshness. Keep this limitation
visible for the map-age design review instead of claiming fresh ULD coverage.

## Post -- eng/codex -- 2026-11-12 19:08 -- codex/gpt-6

User clarification supersedes observation-counter equality: unchanged content may
retain original certification, preserving original evidence time; changed content
requires full recertification. Same epoch, validity generation, geometry/config,
clock and map revision remain mandatory, counters/stamps cannot regress. Exact
lease-page identity remains mandatory. O currently advances map revision for
probability changes too, conservatively beyond classification changes. Global
lease age is not regional coverage or freshness; no regional renewal is claimed.

cycle_03 failed after complete67-page1cm O capture1.693s solely because observation
counter30->39 advanced with map revision5 unchanged. Its full pages/CDR are durable.
Actual map replay has0.919% known active cells and0 common-placement candidates.
No physical no-space inference. Full-model FK shows old portal observation pose
points its central optical ray outside the current ULD; geometry-derived diagnostic
view candidates have14 IK solutions, not yet collision/visibility approved.

Rehearsal_06 STARTUP_READY,50 installed checks passed, no residual processes.
Full-model atlas requalification passed all12,098 stored states with native static
scene checks and FK tolerances; original distribution/NPZ unchanged. Both common
placement and prepare now select its reviewed Gazebo configuration; operation
hull SHA and sampler bounds MD5 have separate explicit bindings.

Current-policy offline tests133pass; actual O owner plus MoveIt native tests10pass
(content_version_native_06,4.56s). Changed content performs complete recertification
before PLAN_READY; repeated saturated observations preserve original evidence.
This also measured/fixed asynchronous release acknowledgement racing a following
refresh: await it inside original deadline, no lease allocation while uncertain.
Earlier test-harness/import failures and obsolete exact-revision test expectations
remain in offline logs; these were not Gazebo product attempts. Baseline OPEN.

## Post -- eng/codex -- 2026-11-12 19:15 -- codex/gpt-6

cycle_04 first camera goal was refused by MoveIt (99999): named model contact
elfin_link6 versus airport_container_real. No observation trajectory executed,
no physical impact demonstrated. The earlier14-state camera probe used
rehearsal_06 ready_scene_response, now proven to contain ZERO world objects.
Its validity does not qualify environmental collision: review.json explicitly
withdraws that interpretation. Post-refusal full scene was missing; instrumentation
now captures complete scenes before/after each goal before another motion attempt.
Separate atlas_requalification_01 uses an actual container mesh plus pedestal;
that qualification is not based on the empty readiness scene.

Readiness now waits for configured container collision geometry and enabled
pedestal in a non-diff full scene, rather than service availability alone.
24 behavioural client/readiness cases pass; build14 all13 packages passes with
unchanged source hashes,50 installed checks pass. Diagnostic --view-poses input
is explicitly model/scene-bound, archived, and always sent through MoveGroup.
No goal is a trajectory certificate or observed free-space substitute.

Full-scene replay view_candidates_native_02:14 IK states tested,2 valid branches
for ONE distinct camera view;12 invalid. New diagnostic input selects only that
view. No live optical coverage claim. rehearsal_07 queued without starting nodes:
another agent owns Gazebo /tmp/uld_upper_layer_scene_pick_start_g12_03. Baseline
remains OPEN; next rehearsal proves full-world readiness, then measured view cycle.

## Post -- eng/codex -- 2026-11-12 19:23 -- codex/gpt-6

User asks whether link6 collision is only a2cm thin ULD layer. Read-only replay
collision_review_01/analyze.py verifies exact equality between cycle04 refused
observation q and full-scene native request, plus response identity/digests.
The qualified scene uses8000-triangle STL, no O objects, link6 padding0 and scale1.
Contact points lie near front upper rim: container x[-.771,-.754],z[2.106,2.116]m;
max reported per-triangle depth.010536m, not a global penetration-distance claim.
Full-model FK: link6 origin z2.116660m, camera optical origin z1.864800m, STL
maxz2.117807m. This observation goal raises wrist to roof height while camera
remains lower. No physical trajectory was executed for that refused goal.
The live refusal retained named collision only; coordinates are explicitly the
same-q offline replay in qualified static geometry, not an unrecorded live scene.
STL does not establish uniform2cm physical thickness. Model-versus-real thickness
accuracy remains unverified; cannot classify it as a voxel inflation false positive.
No product code, padding or geometry changed for this review.

## Reply -- eng/codex -- 2026-11-12 19:29 -- codex/gpt-6

User clarified top-down placement was a hypothesis. Current refusal is an
observation endpoint, before product placement planning. Source audit finds
explicit container-to-world composition and scene-bound OMPL collision checks;
no demonstrated frame mismatch. However prepare root does not enforce required
static geometry: installed-code synthetic probe accepts an empty baseline and
produces an empty private scene. Driver startup gate alone does not close this
service boundary. Repair static scene admission/revalidation before further
planning trials. Separately, fixed vertical entry/release/retreat local route
family (0.15m default entry,0.10m extra exit) restricts search and needs review;
do not confuse it with an observed roof-entering trajectory. Product code and
margins unchanged. Evidence: scene_admission_review_01/; details in owner note.

## Reply -- eng/codex -- 2026-11-12 19:33 -- codex/gpt-6

User adds mandatory ULD-aperture entry. Recorded normative behavior in
architecture/placement.md; straight interior corridor remains ranking-only.
MoveIt Humble provides link-point position/orientation path constraints, not a
ULD-entry toggle. Current native connector has no path constraint wiring.
Propose multiple scene-derived external/internal opening branch pairs, constrained
crossing stages, full-model/payload checks and existing whole-chain certification;
reuse qualified static atlas connectivity as hints, with runtime O/grasp/payload
revalidation. Do not fix one entrance pose or constrain the entire pickup route
to a thin doorway slab. Implementation and performance remain unverified.
Owner note has official references, local API evidence and boundary details.

## Reply -- eng/codex -- 2026-11-12 19:40 -- codex/gpt-6

Discussed user's fast plane-waypoint idea. Recommend geometric aperture erosion
then atlas joint-branch retrieval and short crossing certification; return pose,
q and passage witness rather than XYZ alone. Nominal source aperture bounds and
source SHA preserved in portal_region_review_01/summary.json. Owner note records
the offset-grasp rectangle formula, actual-tilt/full-tool generalization, search
order, existing-atlas extension proposal and conservative-region limitations.
No fixed waypoint, new clearance constant, runtime algorithm change or claimed
speed gain. Full integration baseline remains open.
