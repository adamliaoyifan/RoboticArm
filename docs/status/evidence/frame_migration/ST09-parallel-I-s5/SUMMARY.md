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
| semantic assets INTO the install overlay | the S5.7-5 shared-vendor-via-main-workspace-path approach was overruled: vendor (353M) + yolov8s-world.pt now live in the worktree package and colcon-install to install-st09/luggage_perception/share/{vendor,models}; launch resolves via package share with EMPTY flags (never a bare filename), named launch errors SEMANTIC_VENDOR_MISSING / SEMANTIC_WEIGHTS_MISSING; acceptance manifest semantic_assets records weights sha256 + vendor tree sha256 (LC_ALL=C pipeline) + pinned user-site deps; runner defaults + hash log line semantic_assets. Proof `semantic_offline/semantic_offline_proof.json`: source vendor HIDDEN + ~/.cache/clip hidden + dead proxies (no network) -> backend constructs from the overlay, load paths under install-st09, manifest hashes match, pinned deps recorded (ultralytics 8.4.6, torch 2.10.0+cu128, cv2 4.13.0, numpy 2.2.6 — all user-site). The proof EXPOSED two real defects, both fixed: (1) ultralytics+torch are user-site, so a fresh HOME killed the import and build_segmenter silently stub-fell-back — require_backend now turns construction failure into the named SEMANTIC_BACKEND_CONSTRUCTION_FAILED (no silent stub; node passes require into the build); (2) cmake's stale if(EXISTS) cache skipped the weights install (models/ empty) — clean reconfigure required. |
| per-leg admission load-state scope | CheckExecutionAdmission/PlanMotionV2 carry (leg_digest, leg_load_state); ValidateSelectedRoute/PlanObservationRouteV2 return aligned leg_load_states; execution_admission_is_live enforces the certified load-state index per leg (carrying -> release transition -> unloaded): the vacuum-off digest change at release never reads as an identity change (liveness consults graph/map generations, never the live attachment), and a carrying scope cannot authorize a post-release leg (LOAD_STATE_MISMATCH) nor can a foreign leg (LEG_DIGEST_MISMATCH). Drivers stamp each executed leg's own scope (place route via _leg_execution_admission digest lookup; observation legs by index). |
| certificate supersede (user: ADMISSION_SUPERSEDED) | the live admission is single-slot by design: _promote_selection demotes the displaced selection into a bounded superseded_admissions map, so after route B verifies, route A's next leg is refused by NAME (CERTIFICATE_SUPERSEDED with the demoted cert id in the detail) instead of the peer-restart ADMISSION_UNKNOWN; the refusal happens at the executor gate before any controller submission (the user's "no extra submissions" — the gate position is what the whole-chain tests already exercise). |
| stage-timing instrumentation | frozen-budget attribution for the PLANNING_BUDGET_EXHAUSTED investigation, WITHOUT moving any budget: SelectPlacePathV2 response stage_timings_json (search/certify/revalidate/total ms; search = residue), BuildMotionSequenceV2 stage_timings_json (pick edge_build vs certification), ValidateSelectedRoute certify_ms/verify_ms, PlanObservationRouteV2 certify_ms, PlanMotionV2 result admission_check_ms (executor-side RPC timing); drivers record probe certify/verify ms + selector stage timings into trial extras for the M3 evidence. |

## Remaining (S5.7-6 + S5.6 close)

- S5.7-6: EnsureCycleStartV2 unloaded-exit service (inside+released ->
  certified exit; carrying -> CARRY_EXIT_UNSUPPORTED named refusal; full
  recovery search does NOT block M3 — ROUTE_INCOMPLETE named stop is
  acceptable, general recovery is a follow-up task); GoToRobotPoseV2
  (scene_revision_at_capture binding + certified motion wiring; replaces
  the v1 portal_observe_sim debug wiring before formal single-box
  acceptance); single-box full chain pick->attach->placement->route->
  release->exit->observe->integrate->return; then three-box M3.
