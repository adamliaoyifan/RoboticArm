# ST09-I S5 evidence summary (2026-10-03, through S5.7-6 prep)

S5 = three-box M3 on the deployment scene (scene_tf_deploy, Gazebo
Fortress sim controllers, exclusive domain 7). S5.7 = motion-generation
convergence (approved design
`docs/architecture/motion_generation_convergence.md`; two user rulings:
carry-exit named-refusal first, pick-ladder certification in scope;
2026-10-03 lock-ins: OUTSIDE_CONTAINER rename + classification-is-not-
authorization, fail-closed latest-map admission with no stale fallback,
execution-window admission closure before S6).

## Deliverables and commits

| item | commit | content |
|---|---|---|
| S5.1 acceptance | 4034111 | unified N-box judge: 3-box sessions get the SAME 11-stage completeness + physical gates as 2-box, plus session chain (unique `@ordinal` ids, handoffs joined to committed revisions, per-event input/output revisions); raw acquisition stamps + receiver monotonic + strict `event_seq` (contradictions reported `EVENT_CAUSAL_ORDER`, never rewritten); `extract_acceptance_variable` demoted to legacy triage, main() refuses unknown box_count. tools 27p, E 14p. |
| S5.2 sim poses | 3ee2768^ (S5.2 commit in the S5.3 run) | `robot_poses.sim_deploy.yaml`: pickup_observe_sim synthesized via fixed T_tcp_camera conversion (never solving the camera pose as TCP) + offline-IK refine from the J1-pi seed, principal-branch joints inside real S20 limits; offline validation FK 0.0 / camera exactly at (+1,0,1.9) / no link in the container AABB. SIMULATION ONLY; robot_poses.deploy.yaml stays all valid:false. |
| S5.3 bring-up | 3ee2768 | exclusive domain-7 Gazebo stack on the mirrored cell; one controller race retried clean; live joints hold pickup_observe_sim to 4 decimals; dsim2 gates green on REAL spawned boxes (optical (0.99995,-0.00003,1.900002), depth ratio 1.0, margin 70 px, support 29729, settle 0.0). `dsim2_pose_validation.json`. |
| S5.4 gz atlas | 70d2ae6 | 4 variants built on the LIVE Gazebo deploy stack (builder hashes live RSP bytes); urdf_hash 689838e5 verified three ways (live rclpy param client + production hash fn / cell yaml / captured robot_description.xml; never CLI print text); all four NPZ byte-identical to the mock-flavour deploy cell (empty-load 665/63 matches exactly); explicit `atlas_runtime_config:=` loads strict-clean, implicit path still refuses `hash_mismatch_strict` (live proof of the blocker). `atlas_gz_cell_record.md`, `logs/atlas_build.log`. |
| S5.5 driver+runner | 12c5eb6 | `m3_three_box_driver.py` (finalize-after-commit gated on integration success + strict revision growth + return complete, refusal stops the session; m3_evidence/m3_frozen in the unified schema rewritten atomically per box; GtReleaseStream: dedicated reader thread drains `ign topic -e pose/info` into bounded JSONL, ~53 Hz live-verified, pre-release positive-sample precondition, respawn-gap segmentation, 0.15 s central-difference velocities, stream-vs-poll 5 mm cross-check; GT feeds gt_reference only) + `s5_m3_run.sh` (refuses existing out dir, hard 20-min deadline with 90 s teardown and SESSION_DEADLINE verdict, curated bag topic set, post-run 1 GiB verification). |
| S5.7 design | aca48dd | the convergence design doc (five-layer principle, four promoted surfaces, migration table, zone taxonomy with derived corridor depth). |
| S5.7-1 | 5097904 | ad-hoc place_exit deleted (builder + both exemptions; TYPED_SEGMENT_MISMATCH now covers every executed leg); dead code removed (_lift_for_cargo_view, _place_probe_fn + PLACE_PROBE_MAX_UNAVAILABLE, STAGING_NAMES); observation positioning rides GoToRobotPose(portal_observe_sim) at the CargoMap level. Net -199 lines. |
| S5.7-2 | 32ecbb3 | four-zone taxonomy INSIDE_CONTAINER / OPENING_CORRIDOR / OUTSIDE_SAFE_REGION / UNKNOWN; pure `scene_tf_config_utils.container_zone`; corridor depth DERIVED (wall (outer-inner)/2 + |T_tcp_camera| from live static TF, 0.30 recorded fallback + 0.10 margin; the bare 0.45 is gone); UNKNOWN on TF failure and, on the typed chain, on missing scene identity (SCENE_IDENTITY_MISSING). |
| S5.7-3 | 5ef7b9a | ValidateSelectedRoute on /trajectory_selector replaces the cartesian probe: RouteProducer.verify_selected (digest -> scene identity -> 2*pi-wrapped start state at 0.01 rad -> map regression -> MapRevalidator admission -> FRESH full re-certification of the exact edges) returns per-leg trajectory digests; the driver rebuilds them from the motion_segments it holds and refuses on mismatch (certify A, execute A); verification failure refuses the placement, the excluded/re-select loop is deleted. |

## S5.6 smoke campaign (smoke01..08, all preserved with bags; logs committed)

| run | outcome | root cause / fix |
|---|---|---|
| smoke01 | driver never started | runner WS derivation off by one level + `|| true` swallowed the driver rc — fixed |
| smoke02 | EXIT_FAILED_STUCK | frame_runtime requires the FULL chain (use_cargo_map/use_packing on) — launch flags fixed |
| smoke03/04 | exit=WRONG_OVERLAY | the driver ran from src with src PYTHONPATH — session must resolve from the install overlay (installed driver copy, no src paths) — fixed |
| smoke05 | frozen inputs incomplete | driver repo derivation from the overlay path — M3_REPO env — fixed |
| smoke06 | clear=False (expected outside) | the DRIVER ran the default SIM scene: scene_tf must be passed to the driver too — fixed |
| smoke07 | YOLO_NOT_READY | semantic chain: launch default backend yolo_world -> ultralytics CLIP AutoUpdate online install stall; runner gained a fail-closed semantic pre-warm — ROOT FIX PENDING (vendored-CLIP install check + `semantic_require_backend:=bbox_fill`, S5.7-5 ops item) |
| smoke08 | semantic_prewarm timeout (correct fail-closed refusal) | same root as smoke07 |

En-route production fixes (each with its live symptom): `_home_arm`
world-X >0.2 pre-filter and the unbounded aperture prism misread the
mirrored pickup observe (~1.7 m outside the face) as inside the opening
-> replaced by the zone classification (S5.7-2); `portal_observe_sim`
synthesized for the post-place observation goto.

## Verification standing (through 7ed74c3)

- I 57 passed / 0 skipped (live mock-stack whole chain runs pick
  CERTIFICATION, certified observation positioning, and the admission
  gate end to end; one PLANNING_BUDGET_EXHAUSTED timing flake observed
  once in four runs — the recorded frozen-10s-budget OMPL risk, not an
  admission regression; single-test rerun green).
- luggage_gazebo test + E + C: 254 passed, 3 skipped, 152 subtests.
- luggage_planning: 648 passed. perception segmenter+filter: 83 passed,
  1 skipped (50/50 with the shared-vendor env set).
- Offline semantic proof: backend bbox_fill:<weights> constructs in
  ~2.2 s with `import clip` resolving from the shared vendor, clip.load
  download_root-patched, and ~/.cache/clip ABSENT (deleted before the
  run; nothing recreated it).

## S5.7 continuation (d006507..7ed74c3)

| item | commit | content |
|---|---|---|
| locks | d006507 | zone renamed outside_container (classification is NOT authorization; the v1 portal goto is S5-debug wiring to be replaced by GoToRobotPoseV2 before formal single-box acceptance); verify map admission fail-closed on the LATEST map (MAP_STATUS_UNAVAILABLE for a failed/absent stats fetch — never a fabricated 0 nor the selection-time snapshot; four-case producer semantics pinned by tests named no-stale-fallback / revision-advanced-allows / corridor-affected-refuses); design §5b execution-window closure + task S5.7-3b. |
| S5.7-4 | 13cf971 | explicit LoadState.UNLOADED admission (UNLOADED_ATTACHMENT_DIGEST canonical binding; a missing record under CARRYING stays ATTACHMENT_MISSING); pick ladder CERTIFIED (pick_leg_records returns edges; waypoint_generator builds the fresh-snapshot unloaded scene + full certifier; CERT_* refusals named at the service boundary — never degraded execution); ObservationPolicy promoted (policy-owned target, in-frame prediction, MAX_OBSERVATION_VIEWS=2, second view still post-integration); PlanObservationRouteV2 on /trajectory_selector (producer converts camera->TCP offline, gates identity + fail-closed map admission, returns a COMPLETE certified joint_path; selection bookkept with load_state for verify_selected); cargo_map aim pose_target construction deleted — candidates move through the certified route. |
| S5.7-3b | 2cfc4c5 | admission generation closure: PlanMotionV2 goal carries (route_digest, certification_id, admission_map_revision); the pure executor checks /trajectory_selector/check_execution_admission before the first controller submission and per leg (mechanical liveness; the certifier side owns the verdict; service missing fails closed); producer.execution_admission_is_live (ADMISSION_UNKNOWN covers peer restart; superseded certificate; identity change; map regression; corridor-affecting advance or unavailable latest map revoke; an advance touching no route edge stays LIVE with the new revision recorded); verify_selected promotes the FRESH certificate to the live admission id; drivers stamp every executed route leg (place route + observation legs). |
| S5.7-5 | 7ed74c3 | offline semantic chain via ONE shared CLIP vendor (main-workspace src/luggage_perception/vendor; launch semantic_clip_vendor_dir + semantic_model_name absolute path; YOLO_AUTOUPDATE=false; clip.load download_root-patched — no ~/.cache/clip read or write, verified live with the cache deleted; runner fails closed on missing vendor/weights + semantic_require_backend:=bbox_fill; illegal require names and stub fallbacks pinned to never match); legacy locks: _fixed_slot = test fixture (typed chain refuses FIXED_SLOT_TEST_ONLY, evidence slot_source=test_fixture), world-X guess only for non-frame-runtime callers, place_only_eval_driver no-import-edge pinned. |

## S5.7-6 pre-work (user locks 2026-10-03, second batch)

| item | content |
|---|---|
| semantic assets INTO the install overlay | the S5.7-5 shared-vendor-via-main-workspace-path approach was overruled: vendor (353M) + yolov8s-world.pt now live in the worktree package and colcon-install to install-st09/luggage_perception/share/{vendor,models}; launch resolves via package share with EMPTY flags (never a bare filename), named launch errors SEMANTIC_VENDOR_MISSING / SEMANTIC_WEIGHTS_MISSING; acceptance manifest semantic_assets records weights sha256 + vendor tree sha256 (LC_ALL=C pipeline); runner defaults + hash log line semantic_assets. |
| per-leg admission load-state scope | CheckExecutionAdmission/PlanMotionV2 carry (leg_digest, leg_load_state); ValidateSelectedRoute/PlanObservationRouteV2 return aligned leg_load_states; execution_admission_is_live enforces the certified load-state index per leg; a carrying scope cannot authorize a post-release leg (LOAD_STATE_MISMATCH) nor can a foreign leg (LEG_DIGEST_MISMATCH). Drivers stamp each executed leg's own scope (place route via _leg_execution_admission digest lookup; observation legs by index). |
| certificate supersede (user: ADMISSION_SUPERSEDED) | the live admission is single-slot by design: _promote_selection demotes the displaced selection into a bounded superseded_admissions map, so after route B verifies, route A's next leg is refused by NAME (CERTIFICATE_SUPERSEDED with the demoted cert id in the detail) instead of the peer-restart ADMISSION_UNKNOWN; the refusal happens at the executor gate before any controller submission. |
| stage-timing instrumentation | frozen-budget attribution for the PLANNING_BUDGET_EXHAUSTED investigation, WITHOUT moving any budget: SelectPlacePathV2 stage_timings_json (search/certify/revalidate/total; search = residue), BuildMotionSequenceV2 pick edge_build vs certification, ValidateSelectedRoute certify_ms/verify_ms, PlanObservationRouteV2 certify_ms, PlanMotionV2 admission_check_ms; drivers record them into trial extras. |

## S5.7-6 pre-work round 2 (user locks 2026-10-03, third batch)

| item | content |
|---|---|
| frozen candidate-owned python deps (587723c) | ~/.local deps are NOT pinned system dependencies (fresh HOME -> ImportError = the candidate depended on the executing user's mutable site). deploy_python_deps.sh materializes the closure (torch 2.10.0+cu128, ultralytics 8.4.6, sympy, torchvision, triton, typing_extensions + 15 nvidia cuda dists, 6.7G) into candidate-owned <ws>/pydeps-frozen via an import-error-driven bootstrap that terminates only when the real positive proof passes; per-dist RECORD-based tree digests in the manifest. The segmenter node ALONE gets the prefix on ITS PYTHONPATH + PYTHONNOUSERSITE=1 + cuda lib dirs on ITS LD_LIBRARY_PATH (scoped: frozen numpy must never leak into rclpy nodes); named SEMANTIC_PYDEPS_MISSING; runner pre-checks DEPLOYED.json. verify_semantic_offline.sh is now the FINAL positive: fresh HOME + PYTHONNOUSERSITE=1 (site.ENABLE_USER_SITE False asserted) + dead network + hidden source vendor -> construction from allowed prefixes, torch.cuda True from the prefix, all digests match (evidence semantic_offline/semantic_offline_proof.json). Bootstrap lessons en route: nvidia cuda dists copy upfront (DT_NEEDED loads never raise ImportError), single-file modules + auditwheel .libs siblings, stale-system symbol conflicts (sympy) trigger freezing the prefix version, cmake if(EXISTS) staleness, and a broken `]` af942ca had committed into the runner (bash -n now part of the loop). |
| attachment live-state + admission phase transition | certificate scope and live state are SEPARATE gates. Place routes start in the carrying phase; TransitionAdmissionPhase on /trajectory_selector is the explicit release boundary: CARRYING phase -> release side effect (driver) -> detach confirmed + evidence identity -> UNLOADED phase ACTIVATED -> retreat executes. Carrying legs additionally require the admission to still be in the carrying phase AND the measured record to remain a valid digest match (ATTACHMENT_LOST / ATTACHMENT_INCARNATION_CHANGED named) — the frozen 10 s window is the acquisition->PLANNING-use bound (already applied at select/verify), deliberately not re-applied at execution; live payload LOSS is the vacuum feed, gated by the DRIVER before each carrying-leg submission (PAYLOAD_LOST, absent feed = no-signal). Unloaded legs require the completed transition (PHASE_TRANSITION_REQUIRED). All four user negatives pinned by named tests: payload lost before a carrying leg, release backend refusal (VACUUM_DETACH abort — retreat never submitted), detach unconfirmed (DETACH_UNCONFIRMED), live attachment + no transition (PHASE_TRANSITION_REQUIRED). A stale carrying claim after the transition refuses PHASE_STATE_MISMATCH. |

## S5.7-6 main body: B/C/D/A (5a9ab4b, user fourth lock batch + review round)

| item | content |
|---|---|
| B vacuum admission window | selector subscribes /vacuum/state (TRANSIENT_LOCAL; receive-time freshness 2.0 s on the node clock — the message carries no stamp; keyed on `attached` ONLY: the mock sim backend sets fail_reason while attached stays True). Carrying-leg selector gate in execution_admission_is_live: VACUUM_STATE_UNAVAILABLE / VACUUM_STATE_STALE(age in detail) / PAYLOAD_LOST. Adapter plan_motion gains should_abort polled in BOTH wait loops: goal-pending abort = zero submission; mid-execution abort = cancel_goal() + a BOUNDED wait for the controller's terminal receipt (message carries cancel_status=CANCELED/ABORTED; no receipt in the window = cancel_status=TIMEOUT, still fail-closed — no detach, no map commit, no auto-recovery on that path). Live fault injection test_carry_fault_live: the mock controller republishes its own backend truth at 30 Hz while attached (external attached=false cannot stick — GzSetPoseClient no longer blocks 5 s per tick when gz is absent), so the honest sustained flip is the COMMANDED RELEASE; asserted end to end: CARRY_FAULT_PAYLOAD_LOST + consumed receipt + map revision unchanged + the next carrying leg refused PAYLOAD_LOST at BOTH the selector service and the executor's pre-submission gate. place_smoke passes should_abort for carrying legs. |
| C GoToRobotPoseV2 | srv (request_id/pose_name/current_joint_state/scene_revision/latest_map_revision -> certified MotionSegmentV2 legs + route_digest/certification_id/leg scopes + already_there). SIM site-pose schema luggage.site_poses.sim/1 (SIM_POSE_KINDS pickup/portal_observe_sim; calibration_status synthetic allowed ONLY under the sim schema; observe_pose may name a pose — deploy strictness byte-identical). robot_poses.sim_deploy.yaml rewritten: identity pins md5(scene_tf_deploy)=98313fb8… + srdf_sha256, urdf EMPTY with the named-residual comment (launch urdf hash embeds spawn joints). Producer plan_named_pose_route gate order POSE_FILE_MISSING -> CURRENT_POSE_NOT_EXECUTABLE -> SCENE_STALE -> JOINT_ORDER_MISMATCH -> POSE_NOT_VALID -> START_STATE_MISMATCH -> already-there (<=0.02 rad wrap, BEFORE the map gates: pure joints-vs-file, zero legs, nothing promoted) -> fail-closed latest-map gates -> one MoveIt edge (FILE joint values, FK fills the goal node — no IK) -> full certification under explicit UNLOADED -> promote born-unloaded. Selector service + robot_poses_config lazy loader; sim_world launch wiring; I-suite conftest assert+replace patch + fixture site file pinned to md5(scene_tf.yaml.example). Driver migration: shared _goto_named_pose_v2 (observation-route pattern), goto_observe rewritten (current name = caller-side stay-put; no typed surface = GOTO_V2_UNAVAILABLE — v1 detour deleted), cargo _exit_to_portal rides V2 and freezes a STRUCTURED exit proof (review point 1): request_id/route_digest/certification_id/scene_revision/final_joint_values/unloaded/proof_ns — voided at every new attach (cycle start) and adjudicated field-by-field. |
| D EnsureCycleStartV2 | pure adjudication, no route planning: producer ensure_cycle_start_state (identity binding -> joints -> world FK -> xyz_world_to_base_link -> zone taxonomy; corridor depth = wall((outer-inner)/2) + |camera_to_tcp(urdf)[0]| + 0.10; order: attachment unknown -> carrying(any zone)=CARRY_EXIT_UNSUPPORTED -> unloaded+outside=ALREADY_READY -> inside/corridor=EXIT_ROUTE_UNAVAILABLE -> ZONE_UNKNOWN). Selector service reads its OWN latched joints/vacuum. Driver _home_arm: ensure verdict first, named stops pass through home_arm_code verbatim, the ONE exception is EXIT_ROUTE_UNAVAILABLE + a still-valid exit proof (scene match, joints within 0.02 rad wrap of the proof's final values, vacuum not re-attached, cycle not restarted); failure keeps the gz contact capture. |
| A frozen-prefix artifact | deploy recorder top_level NameError removed (on-disk records were already 4-key). freeze_python_prefix.sh: whole-prefix root digest (LC_ALL=C find|sort|xargs sha256sum|sha256sum, DEPLOYED.json included), double-compute stability self-check, anti-self-reference rule (the freeze record lives ONLY in the runtime manifest outside the prefix; nothing is written back into any prefix file), tar archive under docs/status/evidence/freeze/python_prefix/<ts>/ (gitignored large asset), then the GATE: VERIFY_ROOT_DIGEST=1 verify_semantic_offline.sh must recompute the whole prefix against the record just written (round-trip proof; root=d2846e0a…, 18891 files, cuda 12.8, driver floor 525.60.13 observed 580.178.04). verify_semantic_offline.sh final positive strengthened: device=cuda construction + one minimal inference segment(np.zeros((96,96,3))) with label_map/detections/backend asserts; VERIFY_ROOT_DIGEST=1 recomputes the root digest against manifest.semantic_assets.python_deps.freeze (missing record = named FREEZE_RECORD_MISSING; a second python block rc now propagates). Manifest freeze record carries named_residuals.urdf_hash_separation (S6: never claim URDF full match until kinematic_urdf_sha256 / initial_joint_state_digest are separated) and the R runtime rule (exact-prefix three-layer recompute: archive sha -> extracted root digest -> per-dist RECORD; never a bootstrap re-run). |
| dead-gate repairs en route | (1) PlanningSnapshot had NO scene_revision field — planning_snapshot_from_surface accepted the parameter but never wired it, so every select snapshot read "" and the identity gates were dead (a real request scene always failed IDENTITY_CHANGED; my fault-injection test surfaced it). Field added (default "", excluded from digest()) and wired at both construction sites. (2) camera_to_tcp_transform's literal tcp_path fallback ended in the LINK name — offline_ik urdf raised KeyError('suction_contact_frame') on first use; now derived via urdf.path_to. |
| verification @5a9ab4b | I live suite 97/0 (includes the new live tests: carry_fault mid-release injection, goto_v2 direct-service certified legs + already_there + named refusal, ensure_cycle_start live verdict). Offline planning/gazebo/description/PD 1202 green; PD 47+2s. Known non-regressions: frame_contracts inventory 18 pending is base-existing (stash-verified; refreshed by the frame verify runner), frame_migration_st09_verify offline requires the frame campaign's runtime sample envelopes (not in this tree). |

## G0 named close-out (2026-10-04, plan docs/plans/st09_i_finish_execution_20261004.md)

Base af71a41 (99948e4 evidence confirmed upstream). Four defects, each
one repro -> fix -> verify chain, clean commits; untouched surfaces reuse
the 5a9ab4b-bound results (installed-overlay live runs included — the
offline/live split below is deliberate).

| item | commit | content | tests |
|---|---|---|---|
| G0-1 exit-proof binding | 29af367 | adjudication requires non-empty request/route/cert (`uncertified_exit`), PRESENT vacuum feed (`vacuum_state_missing` — absent is not "not attached"), receive-time freshness 2.0 s mirroring route_producer (`vacuum_state_stale`); void reason recorded in _home_arm notes (`exit_proof_void(...)`); freeze site mints only from an actually-certified route (already-there INHERITS the original proof, never mints/clobbers; empty-cert route freezes nothing); cross-cycle voiding extracted to `_start_new_cycle` (attach site) | 12 new (uncertified/scene/joints/vacuum matrix, new-cycle void, already-there never-mints/inherits, zero-goto through the real _home_arm); focused 48 green |
| G0-2 goto live tests standalone | ca4ae55 | v1 `_goto_named_pose` bootstrap deleted; every test self-preps via V2 (`_prep_at_pickup_observe` drives or confirms pickup_observe_sim through certified legs / already_there); ONE case carries move -> arrival -> identical-goto already_there (zero legs, nothing promoted) -> certified return | file 3 passed (55 s); each test standalone green (25 s / 46 s) |
| G0-3 scene gate independence | b1c8c93 | PlanningSnapshot.scene_revision stays out of the digest BY DESIGN; select-boundary negatives added (same geometry_hash+map_revision+grid, other scene on surface or slot -> CALIBRATION_REVISION_MISMATCH inside request_dict, before plan_place_route, zero controller commits) + matching-scene positive; verify (IDENTITY_CHANGED) and admission (GENERATION_CHANGED) negatives cited from test_verify_selected.py | 3 new; test_verify_selected 26 green |
| G0-4 pending-abort late acceptance | d05c574 | goal-pending abort no longer equates "future not done" with "server did not accept": resolves the handle within the deadline and CANCELS through the shared `_cancel_and_close` (late rejection named `server_rejected`; unresolved goal named `cancel_status=TIMEOUT goal_unresolved`); mid-execution messages byte-identical | 3 new pending-gap negatives; file 7 green (src-prepended import) |

Touched-module runs (once): luggage_gazebo/test 250 passed / 3 skipped /
152 subtests (skip count matches the 5a9ab4b baseline; E/C untouched).
Offline unit tests for luggage_planning run with src/luggage_planning
PREPENDED (the I-suite appends, so the stale install-st09 overlay wins
otherwise); live stack runs must stay pure-overlay (WRONG_OVERLAY
otherwise — the smoke03/04 lesson; live carry-fault revalidation moves
to G1's fresh build, where the new adapter reaches the stack).

Audit 2026-10-04_0008 disposition: finding 1 (v1 bootstrap/order) closed
by G0-2; finding 2 (empty/already-there exit record) closed by G0-1;
finding 3 (relocatable freeze, clip resolution) is S6 scope; finding 4
(fresh non-symlink build, zero skips) is G1 scope.

G0 close state: HEAD d05c574, `git status --porcelain` empty.

## G1 fresh-install isolation re-verification (2026-10-04, 182180a)

Plan step 3 of docs/plans/st09_i_finish_execution_20261004.md. Old
build/install preserved as build-st09-g0base-99948e4 /
install-st09-g0base-99948e4 (failure-evidence reuse; nothing deleted).
Fresh colcon build in a clean env (only /opt/ros/humble sourced, no
inherited overlay/pydeps PYTHONPATH): 14 packages, 28 s, RC 0, real
copies (no symlink-install). Core production modules hash-match source
to install (fresh_install_g1/module_hashes.txt).

I suite from the fresh overlay: run3 **101 passed / 0 skipped /
0 xfail** (255.6 s). Runs 1-2 each surfaced one face of a latent
latch-lag family the G0-2 goto reordering exposed (shorter gap between
the previous file's last integrate and the whole-chain tests):
(1) route-legs revision check sampled the raw lagging latch (fixed
8c6b1fc — _fresh_surface discipline, as the loop test already did);
(2) the graph probe's exercise_compute CAS'd the lagging latch ->
STALE_MAP_REVISION (fixed 182180a — bounded catch-up vs authoritative
stats before the CAS; the probe is shared by the A-E graph scenarios).
Both fixes test/probe-side; install sources unchanged since the build.

Dependencies: A/B/C/C0/D/E/PD 363 passed / 2 skipped with itemized
skip/fail sets IDENTICAL to the preserved 99948e4 overlay baseline
(both skips PD test_install_resolution ELFIN_PD_INSTALL_ROOT-conditioned).
VERIFY_ROOT_DIGEST=1 verify_semantic_offline.sh PASSED (13 s):
root digest recomputed over 18891 files, cuda from the frozen prefix,
manifest_match, clip resolved from the isolated install share with the
source vendor hidden. Evidence: fresh_install_g1/ (logs, hashes,
proof json).

## G2 single-box campaign (2026-10-04, in progress @ ed93014 + uncommitted detector instrumentation)

Plan step 4. Three runs, each a distinct named blocker; every rerun
carried new evidence (no blind reruns). Evidence dirs under
`m3/g2_single_box_*` (each with failure_manifest*.json).

| run | outcome | root cause | fix |
|---|---|---|---|
| run1 | GOTO_FAILED @ t+2.6s, goto=MAP_STALE | goto helper read map stats ONCE; producer's fail-closed gate refused | 0ddbe7f: bounded MAP_STALE retry (3x) after an OBSERVABLE revision-quiescence wait (_wait_map_quiescence, two consecutive equal stats reads, 10 s cap, fresh request id per attempt); producer gate untouched |
| run2 | same signature | deeper: Gazebo /joint_states publishes in controller-registration order [j1,j2,j5,j3,j4,j6]; the driver stored raw positions -> the boot pose (physically AT pickup_observe_sim) compared ~2.77 rad off, already-there (adjudicated BEFORE the map gates) never fired, and the empty boot map (surface_v2 count=1, acquisition stamp 0) refused deterministically; run1's churn theory was wrong (revision was frozen) | ed93014: normalize positions to canonical elfin_joint1..6 AT INGESTION (_on_joint_state), same discipline as the selector's _on_joints; nameless messages keep raw order; 3 unit tests pin the verbatim bag shape |
| run3 | goto PASSED (already_there); DETECT_FULL_GEOMETRY_REQUIRED at pick authorization | DetectLuggage answers conf=1.00 height_valid=false every attempt: the MEASURED_SUPPORT gate fails. Camera pose CONFIRMED correct (FK replay: camera (1.0,0,1.9), 88.4 deg depression, box centered; S5.2 calibration vindicated). AABB-starvation and backend-downgrade theories REJECTED (support input is the PF-R9 g2 full-view raw-depth deprojection; yolo_world is an alias OF bbox_fill). Remaining unknown: exact-stamp raw join miss vs side-coverage geometry — no telemetry distinguishes them | instrumentation landed in the working tree (UNCOMMITTED/UNTESTED): DetectLuggage response message now carries support=<reason> sides=<coverage> raw=<lookup status> -> next run names the cause |
| run6 | DETECT_FULL_GEOMETRY_REQUIRED at pick authorization | render buffer frozen after the suitcase appears, instance mask nonempty on about 2/5 of those frames, authorization sampled `hold_track` | evidence `m3/g2_single_box_run6_20261004`. Not TF (`tf_miss=0`), not a raw-depth join miss (nonempty mask stamp equals depth stamp, 57/57), not support geometry (39/43 `measure` frames `support=ok`), not a YOLO publish-rate drop (mask topic stays at the configured 15 Hz). The 5-attempt / 0.5 s cap closed the wait at ~2 s, inside the first `hold_track` burst (~2.0 s to the first `measure`) |

Run3 world-geometry replay (bag FK): box top 27,855 pts z~1.107
(0.518x0.375 m footprint), platform 2,625 pts z~0.860, GT box yaw=0.
Support config for reference: band dz [0.15,0.60], annulus inner 0.03 /
outer 0.18 m, min 80 pts, min 2 sides x 15 inliers, stride 4.

Run6 bag
`m3/g2_single_box_run6_20261004/bag` (preprocessed color/depth, not raw
`/camera/*/image_raw`). After the first nonempty mask, color has 2
distinct images in the whole bag (one change) and depth the same: stamps
continue at 15 Hz, pixels do not. Post-freeze instance masks are nonempty
57/141, always 22016 px. `hold_track` keeps that earlier 6188-point cloud
and the detector maps the gate to `DETECT_SUPPORT_STAMP_MISMATCH`. The
service returns those frames as successful estimates with
`height_valid=false`, and the outer 5-attempt loop treats them as the
budget. Fresh `measure` frames in the same bag already satisfy support.

Known tooling gap before the G2 verdict can be produced:
frame_migration_st09_extract.py + integration_acceptance.extract_acceptance
both pin box_count to (2,3); the judge itself is N-box parameterized —
minimal change is allowing 1 plus focused E-suite tests (no second set
of physical thresholds).

## G2 run13 (2026-10-04) reachability identity

Place atlas admission now uses `reachability_scene_hash` (arm-to-container
TF, inner/opening/chamfer, collision mesh, URDF still separate). The
whole-file `scene_tf_hash` stays a diagnostic. Run13 loaded the sim atlas
(`atlas load: ok`) even though the file hash is still `35f15c4c` against
the build-time `fb0dfdc1` (pickup platform only). Placement returned
`768/768` slots, `reachable:267`, depth `0.200..1.300`. Select then
stopped at `PLANNING_BUDGET_EXHAUSTED` after one seeded route hit
`MAP_STALE` and the next cartesian traverse stayed under fraction 0.4.
G2 is not passed. G3 was not started.

## G2 run14 map identity (2026-10-04)

Slot 656 built 7 legs and stopped at `MAP_STALE_REPEATED` because
revision 1 had `last_acquisition_stamp_ns=0`. That grid is the mapper's
initial empty container, not a sensor view. First-box placement may use
`map_source=static_empty` when `placed_count=0` and `cargo_observed` is
false; stamp 0 is legal only in that case. A placed box becomes
`release_integrated` and a settled view becomes `measured`; both require
an acquisition stamp inside the existing 10 s window. `static_empty`
after a place or an observation is refused.

## G2 run15b (2026-10-04) static empty admission

`g2_single_box_run15_20261004` died at pick `BUILD_FAILED` because the
installed `st09_contracts.PlanningSnapshot` did not yet accept
`map_source`. After installing that package, `run15b` admitted slot 656
`route-b-948680a7de2c` (7 legs, `decision=continue`) with
`map_source=static_empty`, `placed_count=0`,
`last_acquisition_stamp_ns=0`, `freshness_policy=static_empty`,
`why_allowed=static_empty_unplaced_unobserved`. No `MAP_STALE_REPEATED`.
The route executed through release and retreat, then the trial failed
`PLACE_LOST_PAYLOAD`. G2 is not passed. G3 was not started.

## G2 single-box pass (2026-10-04, Run18)

`m3/g2_single_box_run18_20261004` is the G2 single-box pass. One
`carryon`, bag off, domain 7. G3 was not started.

| check | result |
|---|---|
| legs | 7/7, including retreat and return |
| retreat start | `ready`; descend final equals retreat first; post-detach joint stamp 67.24 s after detach response 66.82 s; max delta 0.005451 rad, tolerance 0.01 rad |
| handoff | `revision_handoff_count=0` |
| verdict | `m3_verdict.status=pass`, `failures=[]` |

Run17c and Run17d had refused retreat with `QP_START_STATE_MISMATCH`.
Descend had already reported success while joint 2 was still about
0.011 rad short of the certified final knot, just over the 0.01 rad
gate. The joint sample was fresh and in canonical order, and retreat's
first knot matched descend's last knot. The fix waits up to 3 s for a
`/joint_states` sample stamped after the detach response and within
0.01 rad of both knots. The tolerance is unchanged. Run18's sample was
already inside the gate (0.005451 rad), so the wait returned
immediately.

Before any G3 three-box run, check and do not mix that run into this
commit:

- boxes 2 and 3 use `map_source` `release_integrated` or `measured`, never `static_empty`
- handoffs are exactly `(1, 2)` and `(2, 3)`
- each box has its own `request_id` and `box_id`
- the retreat-start wait runs for every box
- the bag stays off or capped; do not keep a ~1.8 GB capture

## G3 three-box Run01 (2026-10-04, blocked on box 2)

`m3/g3_three_box_run01_20261004` at `15cb485`:
`carryon,standard,carryon`, bag off, domain 7, one `/clock`
publisher, 3.7 MB, zero residual processes after teardown.
`15cb485` adds evidence wiring only. Selection-time `map_source` and
the per-box retreat-start record now go into each case. For three boxes,
the judge refuses `STATIC_EMPTY_MAP` on ordinal 2 and later, and
`RETREAT_START_UNVERIFIED` on any box.

| box | outcome |
|---|---|
| carryon@01 | full chain; finalize ok; map revision 1 -> 2; `map_source=static_empty`, which is legal for box 1; retreat start `ready`, max delta 0.009924 rad, waited 0.0 s |
| standard@02 | `PLANNING_BUDGET_EXHAUSTED` in place selection; no motion submitted |
| carryon@03 | not run (session stopped) |

Verdict: `not_evaluated`. Missing: the 3 cases and the trailer.
`revision_handoff_count=1`, and the `(1,2)` handoff is joined at
revision 2. G3 is not passed.

First divergent stage for box 2 is candidate/waypoint generation into
route certification (`box_01_paths/candidates.json`):

- `ComputePlacement` returned 218 slots in reachability bands
  `reachable:67, unknown:151`.
- 24 slots were `ATLAS_UNREACHABLE`.
- Only 2 seeded slots (66, 65) were tried. Both were `NO_FEASIBLE_ROUTE`,
  `no IK goals refined`: all 6 atlas seeds missed by 0.24 m / 0.10 rad
  and 0.27 m / 0.11 rad.
- 65 seeded slots were never tried before the budget ran out.

Box 1 had a similar profile: 1 OK out of 57 rows. The atlas reported
`scene_tf file hash differs (diagnostic, atlas=fb0dfdc1,
current=35f15c4c)`. The replay manifest for trial 01 is
`capture_complete=false` (`missing: last_boundary`), because no motion
leg ran.

This is a chain defect, not wiring, so the box was not patched and the
session was not rerun.

Open questions:

- Why do atlas seeds land about 0.25 m from the `standard` place targets?
  Possible causes are a payload-variant or TCP offset, or atlas staleness
  against the current scene_tf.
- Is a 2-slot attempt budget correct when 65 seeded slots remain?

Box 1's retreat delta (0.009924 rad) also passed with almost no margin.

### G3 fix generation 2: offline T3 (2026-10-04)

Bundle: `g3_three_box_run01_20261004/t3/` (`manifest.json`,
`ik_target_t3.json`). Script: `scripts/st09_i/g3_ik_target_t3.py`, which
replays the logged targets against the live `atlas_runtime_deploy_sim`
atlas.

- **The 0.24–0.27 m residual was a logging artifact.**
  `RouteProducer._refine_to` always retries with `TOOL_DOWN_SEED_Q` and
  returns the last attempt's info. So every seed label logged the
  tool-down residual.
- **Real residuals.** Refining from the atlas seeds leaves 8 mm on slot 66
  and 34 mm on slot 65, against a 2 mm tolerance. Seed FK lands 5–7 cm from
  the target, inside the same 0.15 m atlas cell. The target math is
  consistent: not H1, H2, or H3.
- **H4: the targets are genuinely unreachable.** 300 random seeds per
  target solve 0 times, with a best residual equal to the atlas-seed
  residual. The control slot solves about 1 time in 3. Sweeping y puts the
  tool-down reach boundary at about 1.69 m from the base, and slot 66 is at
  1.70 m.
- **Why the selector chose them.** The 0.15 m cell around the boundary is
  labeled REACHABLE, and opening-distance-first ordering puts these
  boundary slots first.
- **Yaw is not the cause.** Placement uses the node's default 4 yaws; the
  catalog is not installed.

Per the stop rule, no behavior was changed. The decision is posted to discuss
`2026-10-04_1710_st09-i-g3-fix-g2.md`.

### G3 Run02 (2026-10-04, at `eade32e`; blocked on box 2 again, new stage)

This run applies the user's choice of A+B:

- A: `_refine_to` reports the requested seed's residual, with the fallback
  residual alongside.
- B: seeded slots whose atlas seeds miss the exact target become
  `ATLAS_SEED_MISS` and are deferred, seedless, to the back of the order.

Tests: `test_route_producer` passes 26/26. In the wider set, 204 pass; the 2
observation-route failures occur identically at baseline `45756d7`. On the
real Run01 targets, the control slot passes in 17 ms and slots 66/65 are
flagged in about 90 ms.

Run02 conditions: one `/clock` publisher, bag off, 4.0 MB, zero residual
processes.

- **Box 1:** passes the full chain again. Revision 1 -> 2; retreat
  `ready` at 0.00885 rad; `map_source=static_empty`, which is legal for
  box 1.
- **Box 2, the fix worked as intended.** Slots 66 and 65 became
  `ATLAS_SEED_MISS` (8 mm / 34 mm, about 350 ms each).
- **Box 2, new failure.** Slot 64 (1.662 m from the base, IK-reachable,
  atlas seeds refine to 0.0) spent 8.25 s of the 9.27 s route budget. All
  68 edge failures are `DESCEND` `fraction_below_min` (0.1 of the required
  0.95) on the 9.7 cm drop from `slot_upper` (z 0.955) to the slot
  (z 0.858). The result was `PLANNING_BUDGET_EXHAUSTED`, and box 3 did not
  run. Verdict: `not_evaluated`, with one handoff `(1,2)`.

First divergent stage: route-graph pairing on the `DESCEND` edge
(`t3_slot64_descend.json`). Every failed pair either joins a `slot_upper`
anchor to the slot goal of the other heading (joint 6 differs by exactly
π, so the straight descend would need a 180° tool rotation) or crosses
elbow branches. Offline, refining the slot target from that same anchor
gives a same-heading goal (joint-6 delta 0), and the straight descend
succeeds in 20/20 steps. The live graph never attempted that pair. This
is a route-graph wiring defect, not reach. It lives in planner code, so
it was stopped and raised for discussion.


- (landed 5a9ab4b) EnsureCycleStartV2 + GoToRobotPoseV2 + vacuum window + frozen-prefix artifact;
- G2 single-box passed at Run18 (`g2_single_box_run18_20261004`, verdict pass, zero handoffs). G3 Run01 was blocked on box 2 at place selection (`PLANNING_BUDGET_EXHAUSTED`); a joint decision is pending in discuss `2026-10-04_1712_st09-i-g3-box2-place-selection.md`.
- remaining: three-box M3 only, under the checks in the Run18 section, after that decision.
