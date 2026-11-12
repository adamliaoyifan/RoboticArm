# 2026-11-12 -- Minimal observed three-box loading

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: MOVEIT-ATLAS-JOINT-PLAN-20261111
- subtask: ST-4-GAZEBO-OPERATION
- generation: 2
- base_revision: 4909614c70b79e90fea8c725ee5e760b59be2ea2
- plan_revision: 50119ea1

## Summary

Active 10h goal: actual new-route three-box observed loading, two floor then
one stack. User authorizes recommended choices without waiting for replies
during the budget; record observations, cause and recommendation rationale.
Full-model success remains unproven. No subagents used.

## Requirement

Follow the acceptance plan and persistent O; no prior placement or simulator
geometry entering online placement. Minimum code changes; post-place explore
is the only added product capability. Keep exact static model collisions.

## Changed

- e95180db: new-route O collision expansion .05 -> .02m; explicit grouped
  shared-resolution override .02m. Both are user-authorized qualification
  settings. Existing release height and payload margins are unchanged.
- 976b3c37: pickup observation no longer requires unrelated ULD integration;
  dedicated ULD views retain that gate. Add actual depth/LiDAR status, raw
  latest clouds and O receipt failure capture to the diagnostic driver.

## Verification

- build_01 all13 packages, unchanged input hashes; installed191 checks pass.
- launch/readiness33 behavioral cases pass; driver12 cases pass after repair.
- diagnostic_01: actual full model hash51df00b4..., reviewed atlas identity
  matches, 2cm owner resolution. Readiness passes8.64s. Initial empty succeeds.
  Pickup MoveGroup completes but driver refuses NO_INTEGRATED_SENSOR_VIEW;
  no product box spawned, no placement planning or product carry. Owned stack
  stopped with exit0. Shutdown-only Gazebo signal/segfault is in launch log;
  it was not the cycle's first failure.
- build_02 all13 packages, unchanged input hashes; installed checks pass.
- diagnostic_02 running from the frozen repair; not yet a success claim.

## Decisions and causes

| Observation | Evidence / cause status | Recommended action and reason |
|---|---|---|
| User permits proceeding on recommendations for10h | Explicit user instruction in this task | Continue bounded evidence-led repairs; record choices here instead of pausing routine decisions. |
| Source O margin still5cm despite smaller voxels | Direct operation_launch/snapshot source | Set margin2cm and source grid2cm for this profile, as requested; retain occupied/unknown and static mesh collision checks because sensor/model coverage is not yet qualified. |
| First live failure NO_INTEGRATED_SENSOR_VIEW at pickup | diagnostic_01 typed view_validity and completed MoveGroup result; source requires ULD integration at every view | Remove ULD-hit requirement only for pickup. Pickup is outside ULD and has independent measured detector/domain gates; initial ULD authority is the explicit source declaration. No claim that sensor delivery itself is repaired. |
| Existing diagnostics lacked adapter reasons at failure | Source capture included images/q but not LiDAR/status/receipt | Capture these bounded latest inputs before another attempt, so future coverage failures can be localized without guessing. |
| Typed surface rejects initial_empty | diagnostic_02 actual prepare response; converter's confidence inverse omitted the source label | Preserve initial_empty explicitly, free cells only. Add actual operation-enumerator regression, rather than test only the legacy dict solver. 32 focused checks pass. |
| Contact search exhausts256 checks | diagnostic_03 frozen map/domain/q replay: existing code needs1417/1322 checks; observed grid center passes complete1764-cell footprint | Skip centres whose physical footprint cannot fit even the grid bounds before bounded exact containment. Same data passes in27 checks in the reviewed branch. No cell filling or contact relaxation. 24 candidate checks pass. |
| Native local branches collide | diagnostic_04 eight actual queries; isolated exact-query replay retains phase/q/contact positions | Add bounded12 collision witnesses to existing diagnostics. Baseline static ULD/payload conflict; one-cell inward shift still collides with low O voxels. Do not infer physical impact from a native refusal. |
| Extra vertical O dilation blocks floor approach | local_replay_01 vs local_replay_vertical_control: same inputs, all O retained, remove only .02m extra Z expansion; inward cases become LOCAL_PATHS_VALID in .07-.08s. Full O deletion is a separate diagnostic negative control, never production evidence. | Keep source voxel height and2cm lateral padding, set explicit extra Z padding0 in the joint profile, identically prepare/recertify. Attachment height uncertainty and static ULD remain. This is a documented recommended policy refinement under delegated decisions; release gap stays5cm. |
| Nominal wall candidates dominate native budget | local request envelope .593x.475x.307m vs nominal measured .496x.367x.247m; nominal fit does not imply envelope fit | Rank using the existing attachment limits at a centred-contact proposal, release gap, source O and actual hull. Retain fallback and native authority; no inferred actual grasp or new safety margin. |
| Current full-model sensor map exists | diagnostic_02 complete pages:75x99x74 at2cm,147 occupied,533553 free,15750 inactive;39822 sensor-evidenced cells,493878 declaration-only | Render and preserve actual native-resolution data separately from declaration. Sensor evidence7.46% of active volume, so no complete-coverage claim. |

## Progress through diagnostic 05

0d6de2ca fixes the typed source conversion; e9453eb0 fixes bounded contact search;
d3fd1152 adds native collision witnesses. f3415f4b aligns ranking/vertical policy
and records the previously untracked runtime_ports baseline plus its small policy
changes (the large added-file diff is not a newly invented execution framework).
The other pre-existing effective runtime/action inputs remain in immutable source
archives and build manifests. a0a80f51 updates a constructor-bypassing test fixture
for the new explicit policy field;48 refresh cases then pass.36 focused
ranking/source/snapshot/launch cases pass. build_07 passes all13 packages with
unchanged source hashes. diagnostic_05 is running; no full-cycle success yet.

## Result

OPEN. Three-box acceptance, actual floor-box completion, post-release coverage
and actual stacking are not yet demonstrated. Existing interface tests do not
satisfy these product gates.

## Pointers

- docs/agents/discuss/2026-11-12_2345_gazebo-operation-g2.md
- docs/plans/THREE_BOX_OBSERVED_G2_20261112.md
- docs/status/evidence/three_box_observed/g2/
- docs/status/evidence/three_box_observed/g2/diagnostic_01/run_manifest.json
- src/luggage_gazebo/scripts/joint_operation_sim_driver.py

## Diagnostic 05 follow-up -- 00:27 CST

- Observation: prepare stopped at40.7s with OPERATION_EVIDENCE_BUDGET_EXHAUSTED;
  302 aperture IK replies include185 successful exact full-scene targets. Raw
  repeated scene CDRs were698828bytes each. gzip level1 retains117666bytes
  losslessly in3.25ms for the measured sample. Recommendation: gzip stage CDRs
  at rest, keep raw/stored hashes and256MiB stored cap. This preserves replay
  inputs and avoids increasing the cap. Two behavioral archive tests pass.
- Connection failure mechanism: private standalone native pipeline omitted
  external OMPL group configuration; logs confirm fallback defaults. It accepts
  coarse geometric edges that then fail existing0.01rad dense checks. Four
  exact saved queries x3 runs all12 were STATE_COLLISION under old default.
  A diagnostic .0001 fraction produces checked paths plus deadline misses,
  without STATE_COLLISION. Production repair derives fraction from the loaded
  group's extent and existing0.01rad check step, registers the requested
  RRTConnect config, and leaves full-scene checks unchanged. Derived-config
  replay is preserved in diagnostic_05/connection_derived; native fixtures and
  connection/archive checks29 pass. These are isolated diagnostic comparisons,
  not a production success-rate claim. No physical box motion occurred.
- Preserved request/response/scene inputs, executable hashes and native logs
  for all three comparisons. Evidence readers now accept old plain/new gzip
  boundaries. Full model execution and three-box acceptance remain pending.

## Diagnostic 06 -- 00:34 CST

- Full-model preparation SEARCH_INCOMPLETE at295s (cycle303.57s); no vacuum or
  product carry. Owned sim teardown0. Exact1229 stage records all gzip/size/hash
  checks pass:436436041 raw bytes retained in79531871 stored bytes. Three
  connection calls lack replies when caller budget expires; requests and final
  cancellation/search trace remain preserved. Do not claim every backend call
  returned a terminal reply.
- Connections:133 checked successes,63 deadline replies, zero STATE_COLLISION
  replies. Opening outside/plane/inside edges have actual successes; final
  inside-to-place edge repeatedly reaches2s search deadline. No evidence of
  release/retreat physical collision in this run.
- Exact three failed edge3 requests x2 at5s produce one checked success and five
  deadlines. This proves at least one2s miss is search budget, not physical
  impossibility, but increasing all edge budgets alone is not a justified fix.
- Source/trace establishes seed starvation: max3 with3 atlas branches drops
  current-carry/place-entry seeds. Regression fails before repair (16 other
  passes), then43 aperture/planner cases pass after preserving both endpoints
  and ranking remaining atlas proposals by raw joint travel. Commit e295cd68;
  no winding wrapping or geometric margin change. Saved full-router replay is
  the next comparison before another live run.

## Opening/timing isolation -- 01:04 CST

- Native IK winding selection now chooses only exact revolute2pi equivalents
  inside loaded bounds nearest its input seed, before collision/path retention.
  It does not rewrite issued paths. Example old placement J1=-5.542rad has a
  +.742rad equivalent versus actual pickup-side J1=1.832rad. Local replay and
  full-model router are preserved. With nearest IK and pure OMPL, the exact
  saved full-model case completes its ordered crossing in9.14s. No physics.
- Rejected prototype: existing MoveIt Cartesian seed could accelerate opening
  subedges, but failed whole-chain timing qualification. Full prototype sources,
  binaries and typed boundaries were archived before withdrawal. No Cartesian
  connector change remains in product; this is not claimed as a delivered win.
- Further isolation corrects the first hypothesis: excessive timing exists with
  pure OMPL too. Exact saved pure-OMPL carry:2314 checks,10.9746rad path length,
 35.9s base timing uniformly stretched12.69x to455.70s by controller quintic
  acceleration extrema around short corners. Native fixture refuses explicitly
  CERTIFICATION_SAMPLE_CAP. Increasing sample cap/deadline is not the remedy.
- Offline alternatives, same original polyline and all3 opening rows retained:
  collinear removal alone2314->14 knots still430.66s; corner rest-to-rest
  quintic timing computes46.43s under the same v/a limits. Recommendation:
  apply this standard monotone timing only to global align/carry polylines;
  retain existing local Cartesian timing. Exact opening joints are explicit
  required rows. No smoothing, collision margin, controller bound or sample cap
  is relaxed.50 pure timing/aperture/planner behavioral cases pass; native
  whole-chain certification is the next gate, not yet a live success claim.
- Add timed-leg counts/durations and specific certification refusal reason to
  the existing lightweight trace; generic refusal had hidden the first cause.

## Timing qualification -- 01:15 CST

- Nearest legal IK winding plus corner-stop polyline timing passed four native
  full-model aperture tests, including an issued complete six-leg plan and
  independent native collision certification (PLAN_READY, carry46.220s,
  whole-chain planning0.241s in the small fixture). Native endpoint suite7/7;
  runtime/execution/connection/certification71/71; timing/router/planner50/50.
  This certifies fixture behavior, not a Gazebo execution or three-box result.
- Recommendation is to keep the exact checked geometry and opening witnesses,
  stop at actual OMPL corners, and avoid raising certification caps or globally
  stretching the entire path because of one short segment. Native nearest-turn
  selection changes only newly solved IK candidates within actual bounds.
- Next live diagnostic07 will test one real full-model floor cycle before any
  three-cycle orchestration. No prototype Cartesian connector remains.

## Diagnostic07 search qualification -- 01:20 CST

- Full simulation readiness9.59s, pickup observation/spawn passed. Prepare54.19s
  refused SEARCH_INCOMPLETE before any grasp/carry. Exactly six connections hit
  deadlines; no physical release, no claimed box collision. Owned teardown0.
- Existing global3-branch cap allowed only one geometric slot with two grasps.
  Do not interpret this as unreachable placement or a failed physical exit.
- Exact saved full-router2s failed in20.05s; same model/O/targets with5s edge
  budget produced ORDERED_APERTURE_CONNECTION in10.02s. Its polyline times to
 135.78s, so successful routing is not yet an executable operation result.
- Three exact failed final-edge requests x2 at5s:3 checked successes3 deadlines.
  No reliability improvement claim. This justifies a bounded diagnostic search
  opportunity and further alternatives, not collision relaxation.
- Recommendation: expose search budgets instead of hidden literals; explicit
  diagnostic profile5s/edge,8 total branches,2 per coupled candidate, unchanged
 300s watchdog. Baseline generic defaults retained. Native final checks remain.
- Source launch test initially used stale installed gazebo module because the
  source-test wrapper omits that package. Preserve the failure; rebuild before
  certifying composition. Not a product runtime search failure.

## Diagnostic08 scene transition -- 01:24 CST

- Prepare19.82s now returns complete PLAN_READY with native whole-chain receipt.
  Execute returns STOPPED:OWNED_PAYLOAD_SCENE_REFUSED in2.31s before first leg.
  This is a runtime scene-effect refusal, not a physical collision. Teardown0.
- Preserved exact ApplyPlanningScene request/response under run/execution; live
  scene lacked pickup_box. Old diff unconditionally removed world pickup_box and
  detached it, then added the desired world box. MoveIt logs missing-world-object
  removal and response.success=false. No motion/grasp/cargo contact occurred.
- Recommend state-derived owned diff: complete ADD replaces existing world box;
  remove only a present world box when attaching, detach only a present attached
  payload when releasing. No unrelated scene object or collision allowance changes.
  Five transition cases plus runtime suite pass; next full simulation verifies
  actual MoveIt service behavior, not only message construction.

## Diagnostic09 target/scene equivalence -- 01:36 CST

- Actual MoveIt owned-scene application now succeeds. Preflight then refuses
  SETTLED_BOX_ADMISSION_REFUSED before motion. Cleanup also refuses
  OWNED_WORLD_OBJECT_CHANGED; all exact scene/map/plan boundaries preserved.
- Offline runtime replay: all three exact O captures revision23 still contain
  the original target in the256-pool. Release FK-derived centre differs by
 [1.19e-7,6.64e-8,3.07e-8]m; old slot identity threshold1e-8 incorrectly treats
  it as a different requested placement. This is not a2cm map inflation effect.
- Recommend keeping the requested slot as layout identity after the existing
  native FK-accuracy check. Native collision phases retain the exact FK-derived
  release obstacle; runtime retreat uses the same helper. No physical tolerance,
  sampled path or collision gate is relaxed. New native test verifies both
  requested-slot admission and actual-G collision geometry;43 native cases pass.
- MoveIt round trip moves absolute primitive pose into object.pose, primitive
  becomes identity and quaternion sign changes. Exact messages differ but box
  dimensions/frame/full composed geometry match. Compare canonical composed
  box geometry, not storage encoding;31 scene/runtime tests pass including
  millimetre displacement/size/frame/extra-shape negative controls. Fixed
  cleanup uses the same comparison. Native/live verification is next.

## Diagnostic10 retained map scope -- 01:34 CST

- Both repaired preflight interfaces now pass actual full MoveIt native suffix
  checks. Original align trajectory submitted and controller succeeded. Next
  pick-contact leg was not submitted: GetSnapshot returns STALE (no handle),
  while validity stays VALID and content remains revision19/154 occupied cells.
  Last ULD acquisition48.4s versus next execution barrier51.574s;1s hard age
  cannot work while the wrist camera turns toward pickup. Teardown0.
- Recommendation uses the user's explicit stationary-other-cargo premise for
  this one bounded action. Retained O age allowance derives from original age
  plus the actual action budget, never a new acquisition timestamp. It cannot
  outlive that budget. Source epoch/config/validity/content and full native suffix
  checks remain; prepare freshness and post-release fresh acquisition unchanged.
  Record the original time and premise, explicitly no physical collision detector.
- Observer/runtime61 behavioral cases pass;46 native-motion cases skipped because
  that command did not configure native endpoints (not counted as passes).
- Additional actual-owner/native synthetic-actuator cycle reaches all first five
  legs, real fixture vacuum receipts and release, then refuses retreat. Preserved
  exact contacts: coarse0.1m O voxels + default0.05m padding produce0.2m obstacles
  touching suction_contact_frame. This fixture is not deploy_update/profile2cm;
  no Gazebo success claim. The native cycle remains a known failing legacy
  fixture, not passed validation of the complete production workload.
- Role section labels above were approximate local working timestamps; exact
  times, revisions, input hashes and stage stamps in run/build manifests govern.

## Diagnostic11 admitted frozen-contact scope -- 01:45 CST

- Original align controller succeeds again. O revision25/149 occupied changes
  to61/418 at its terminal; source_current correctly refuses the old collision
  view and requests refresh. Retry then refuses before a map verdict.
- Exact check_prepared replies still OWNER_ISSUANCE_CURRENT. Original contact
  acquisition14.191s and measured align terminal73.942s imply59.751s age;
  refresh pushes it past generic60s observation_current limit. Not a measured
  collision or proof of unreachable carry. Preserve new O for further diagnosis.
- Recommendation: keep default60s admission for a NEW action (so old completed
  plans cannot regain replay eligibility). Only after that admission, retain
  this immutable observation/provider through initial age + actual action budget.
  User-confirmed stationary-before-seal/fixed-after-seal premise and active
  perception-owner binding checks remain; no acquisition restamp or renewed G.
  Generic provider and journal windows are unchanged.37 focused cases pass.
- Actual align alone captured53.1MiB raw. Remaining5 legs would risk the128MiB
  raw evidence cap. Apply the already qualified prepare convention to execution:
  lossless gzip, both raw/stored hashes, same cap on stored bytes. Behavioral
  round-trip/cap test passes. This is evidence retention, not planning relaxation.
- For reviewable ownership, commit20752df0 adopts the five EXISTING action/host/
  journal/test files from the exact diagnostic01 source archive. It does not add
  a new framework. Manifest runtime_baseline_adoption.json records each source
  hash. This exposes the subsequent13-line compression change separately and
  preserves all unrelated shared dirty source inputs.

## Diagnostic12–13 startup and ranking/admission boundary — 2026-11-13T01:58:00+08:00

- Diagnostic12 stopped at readiness GetPlanningScene timeout before any box
  attempt; request, model and server failed-response-send log preserved.
  Same-build bounded rehearsal diagnostic13 started normally. DDS cause remains
  unproven; no seed or product success-rate claim is made.
- Diagnostic13 PLAN_READY after47.47s, execution refuses before motion at
  SETTLED_BOX_ADMISSION_REFUSED. Exact nominal target is excluded from current
  top256 after actual eccentric attachment changes clearance ordering. Same O
  revision31, 147 occupied cells. This is ranking/cap misuse, not collision.
- Recommendation applied: shared geometric target admission retains the finite
  full directed grid set (upper bound XY cell count times four headings). Explicit
  prepare slots and runtime use it. Existing support/hull/known-volume gates are
  unchanged; only search uses ranked truncation. No fabricated placement or
  tolerance enlargement. Exact archived case now admits; all-occupied
  counterfactual refuses. Unit unknown/occupied and shifted-target cases refuse.
- Seven focused cases pass. First run exposed a stale test stub missing existing
  envelope policy; stub now supplies an explicit zero-uncertainty ranking fixture.
  The failed test log and one mistaken pytest-vs-script invocation are retained.
- Build25/install191 checks passed. Next diagnostic14 exercises actual execution;
  still no full physical box cycle or three-box success at this checkpoint.

## Diagnostic14 physical-instance identity — 2026-11-13T02:05:00+08:00

- PLAN_READY17.80s; actual align and approach controller motions complete.
  Refusal occurs at vacuum before any carry. Exact response is
  PHYSICAL_ATTACHMENT_UNAVAILABLE: physical model generation mismatch, not a
  motion collision. Task box_id=detected_box was incorrectly used as Gazebo
  model name; observed suction instance is pickup_box_0001_carryon, generation2.
- Recommendation: use the measured candidates' unique instance/generation for
  actuator model identity, keep task box ID and all measured planning geometry.
  Existing registry generation check still must agree before effect. Ambiguous,
  wrong-generation and unregistered instances are behavioral negatives. Legacy
  callers with no candidates retain their existing box_id instance contract.
- Attachment qualifier now reports backend refusal before decoding a nonexistent
  measured-grasp pose; backend diagnostics no longer claim accepted=true on
  failed attachment. Conservative CARRY_FAULT/possible-payload handling remains.
-43 focused cases pass, including real node/backend mapping and no-effect
  refusals. Eval-only Gazebo phase pose captures are durable, never fed online.
  Only owned adapter lines are staged; preexisting postseal measurement work in
  that shared dirty file remains outside this commit.

## Diagnostic15 startup callback starvation — 2026-11-13T02:07:00+08:00

- No box attempted: static ULD missing during45s readiness gate, first scene
  apply response timeout. Source shows reentrant1s sync timer, blocking sync
  mutex and four-worker executor. Four concurrent sync callbacks can consume
  every worker while the first awaits a reply callback on that executor.
- Offline four-worker behavioral reproduction fails on old implementation
  (completion callback cannot run), passes after single-flight nonblocking
  acquire; competing callers immediately return SYNC_STATIC_BUSY. Static
  readiness still requires the actual full ULD and pedestal objects.
- Recommendation removes the deterministic starvation amplifier. Initial DDS
  response-send loss cause remains unproven, and no live success claimed yet.
  Failed/successful behavioral logs and original startup CDRs remain durable.

## Diagnostic16 composed observation/replay window — 2026-11-13T02:13:34+08:00

- Startup passes with static world after8.18s. PLAN_READY194.88s. No motion:
  action rejects original observation age above generic60s despite current
  issuance/frozen-owner approval. Saved connector stats:51 checked connections,
  21 deadlines, zero collision refusal across72 replies,158.82s native search.
- Recommendation applied: NEW preparation explicitly admits acquisition age
  <=the existing60s physical-observation window. NEW execution permits that
  window plus the configured preparation watchdog (300s in this profile).
  The same original-acquisition360s bound now governs consumed-plan replay
  tombstones; only an admitted action then receives its bounded execution scope.
  Original timestamps, owner/generation checks and native recertification remain.
  This is budget composition, not renewed evidence or an arbitrary fresh stamp.
- Root/host receive the same max_preparation_budget_s launch value. Default
  journal API stays60s for legacy callers.66 focused cases pass including
  original-vs-retained observation and exact replay-expiry boundary. Native
  fixture is now explicitly clocked at its synthetic12s acquisitions instead
  of host wall time, to test real new-admission behavior.

## Diagnostic17 measured-normal/local-sweep ranking — 2026-11-13T02:26:24+08:00

- All8 locally checked proposals refuse NO_PLACE_LOCAL_BRANCH before motion.
  Saved native replay: original/1-cell inward fail,2/4/8/16-cell inward pass.
  Original witness is O voxel75 vs prior-local-box,36.154mm penetration; the
  obstacle is retained in every positive replay. No physical collision occurred.
- Competing explanation tested: retaining full finite solver set before ranking
  changes no first4 targets (0.59s vs0.78s). Do NOT make that speculative edit.
- Measured mechanism: cheap ranking assumed exactly vertical centered contact
  and evaluated release pose only. Actual observed normal needs the previously
  approved tilted conservative envelope; descent's vertical sweep crosses an O
  voxel above the otherwise clear release envelope.
- Recommendation applied: centered ranking uses the OBSERVED normal and
  geometric box center (proposal only, no suction permission); include the
  existing DEFAULT_PLACE_CLEARANCE_Z descent sweep. No new target offset,
  margin, map removal or hard rejection; blocked hints remain fallback.
- Re-ranking same saved O/domain changes first4 targets. Independently compose
  actual geometric pick contacts for both orientations:8/8 native local checks
  pass,0.07–0.39s, each with2 pick and2 place branches. This is LOCAL scope, no
  full route/execution or success-rate improvement claim.36 focused tests pass.

## Diagnostic18 source convergence evidence — 2026-11-13T02:39:06.416774+08:00

- Preparation16.86s; actual align completes, next approach never submitted. Both bounded source refresh attempts refuse changed O; no collision reported. Exact captures: revision26→63 adds297/removes14 occupied;63→74 adds21/removes27;74→89 removes4 only. This is measured classification change, not observation-number-only mismatch.
- Cause remains unresolved between controller settling and fusion convergence. Do not raise retries or remove O speculatively. Added bounded diagnostic CDR pre-trigger10s/96MiB RAM, post-refusal stationary25s,256MiB stored cap; retain mapper input clouds, TF/static TF, joints and UpdateReceipt. Independent gzip-block index permits replay; missing records explicit.14 focused capture/driver tests and191 installed-origin checks pass, build31.
- Recommendation: measure whether map converges after terminal motion before altering execution policy. Diagnostic19 uses exact same product behavior with capture only. This diagnostic overhead is not a clean latency measurement.

## Diagnostic19 actual grasp/lift; capture correction — 2026-11-13T02:40:37.801664+08:00

- Prepare10.14s; actual align, local approach, confirmed physical vacuum attachment and lift all succeed. Carry index3 is never submitted: O changes on both checks, revision61→65 and69→79. No collision verdict or actual collision measured; no place/release/retreat success.
- Failure-window capture did not fire: it was incorrectly inserted in inner stage exception handler; returned unsuccessful action raises only in outer run. Added capture_health_review.json marking fusion-root-cause evidence incomplete, preserving complete original RPC/map/native/action artifacts. No blind claim of complete fusion replay.
- Repair moves trigger to outer operation boundary; a behavior test injects CURRENT_SOURCE_REFUSED vs controller timeout and asserts trigger/25s hold/close only for known terminal source refusal.16 focused cases pass, build32/install191checks pass. Diagnostic20 only adds this corrected capture, no behavior workaround.

## Diagnostic20 observed map convergence; bounded retry recommendation — 2026-11-13T02:44:42.166787+08:00

- Prepare82.63s; actual align succeeds, next approach not submitted. Both existing source checks refuse changed O; no native collision reported.
- Correct failure trigger captures35.16s of nonstatic input:3506 joint messages,703 dynamic TF,2 static TF batches,289 admitted Livox clouds,145 depth clouds,434 O receipts.34.97MB stored,0 dropped records; frames livox_frame and camera_depth_optical_frame. Last occupancy change at window8.602s; failure-trigger starts~10s; remaining~26s zero classification changes with both sensors continuously integrated. Joint displacement is negligible after window10s. Evidence directly DISPROVES continuous dual-sensor classification oscillation for this parked pose; cannot claim all poses/real sensor stability.
- Source-established cause: runtime checks immediately after terminal movement and exhausts two complete recertifications while fresh stationary rays still converge. Recommendation under user delegation: expose EXISTING operation_source_refresh_attempts in launch/profile, explicit profile3/default2, within unchanged action deadline. No O removal, timestamp renewal, ignored update or collision/unknown relaxation. Validate one extra full recheck against actual runtime; do not blindly increase further.
- Build33/install191checks pass;34 focused behavior cases pass. Diagnostic21 measures actual effect; this single witness cannot establish reliability or timing SLA.

## Diagnostic21 stable-map target rejection; bounded pre-seal reprepare — 2026-11-13T02:50:59.954527+08:00

- Third source attempt obtains equal map content despite revisions92→104 and111→123, confirming counter-only updates no longer starve this boundary. Native suffix then refuses SETTLED_BOX_ADMISSION_REFUSED; approach is not submitted. Source retry configuration is effective for this witnessed transition, not a reliability improvement claim.
- Exact target voxel replay:25 occupied intersections, all at source z-index0, container[.53,.55]m. New floor-hit voxels invalidate original initial-empty floor target; initialmap admits it, every postalignment map refuses. Latest shared solver has alternatives including targets elevated by exactly one source voxel. This is conservative map/target mismatch; no simulated physical collision established.
- Recommended integration response: before any seal, stopped terminal empty-tool action may be discarded and one NEW complete operation prepared from fresh detection/domain/O and actual current joints. Same observed instance mandatory; all trajectory/placement checks rerun; no target shift/voxel erosion/timestamp reuse. Optional diagnostic driver --prepick-replans1, default0, max2; cannot retry carrying, uncertain or controller-timeout results. Each trace/result preserved.
- Added behavior coverage of new-plan loop and no-retry cases.24 capture/driver tests pass and191 installed checks pass, build35. Source/native terminal failures both retain bounded observer input windows. Diagnostic22 tests this integration route.

## Diagnostic22 re-observation boundary — 2026-11-13T02:57:18.094803+08:00

- Initial prepare22.21s, actual align, target rejected on latest floor voxels as21; one fresh complete-plan retry starts but fresh matching observed top unavailable30s. No second plan or grasp issued. Latest domain invalid generic acquisition mismatch. Preserved RGB directly shows cropped partial box at align endpoint; this does not alone distinguish every PCA rejection mechanism, so add DetectionFrame to failure diagnostics.
- Recommendation: return to existing pickup observation pose before fresh detection, rather than reuse old contact evidence or weaken support gates. This view motion temporarily includes exact latest O occupied/unknown volume in MoveIt, uses launch-recorded2cm/0vertical collision policy, and only removes owned generated O IDs on acknowledged successful terminal. Uncertain motion retains obstacles. No full-world overwrite, no unobserved target offsets. Snapshot RPC reuses existing OperationRpc/ObserverSnapshotPort and durable typed captures; diagnostic recorder now serializes concurrent RPC-release callbacks.
-44 focused tests pass (including occupied+unknown in actual view-scene composition, retained obstacles on motion timeout, scoped removal and complete-plan retry); build36/install191checks pass. Diagnostic23 verifies this wiring live.

## Diagnostic23 source semantics and view-scene completeness — 2026-11-13T03:05:09.525386+08:00

- Actual O-backed return to pickup view succeeds7.78s and reacquires measured detection. New prepare refuses during map capture: OBSERVER_SOURCE_CHANGED_DURING_CAPTURE. Native O source establishes map_revision increments for any CellValue/log-odds change, independently of occupancy classification. Prepare still used strict source-revision freshness while runtime used exact planning content.
- Recommended correction unifies exact volume/active/surface classification comparison in luggage_mapping.observer.same_planning_content, adds planning_current fresh full evidence plus fixed-identity/generation/progress checks, and uses it for prepare finalization while retaining strict current() for callers requiring exact version. Immutable map capture no longer insists future revision stay unchanged during transfer; full latest-map certification still required for changed content. Original timestamps/observed evidence never renewed.
- View baseline replay also proves pre-seal operation cleanup restores original world without the pickup box. Added the bound measured pickup collision box to temporary return-view scene (no physics/catalog geometry).44 previous view/source tests plus new exact-content positive/negative tests;143 focused cases pass. Updated a stale finalizer fixture to supply measured-normal ranking limits/keyword introduced earlier; native actual-O full prepare positive passes1/1 in1.86s with real native certification (synthetic sensor fixture, not deploy/full Gazebo success). Build37/install191checks pass.
- Diagnostic24 verifies actual recovery/carry chain with these fixes. No full cycle or3-box completion claimed.

## Diagnostic 24 -- 2026-11-13T03:50+08:00

- Full model and persistent O: actual grasp, lift and103.57s carry complete.
  Descent is NOT submitted; three source refresh attempts see classification
  changes. Failure window35.30s/51.25MB/0 dropped records; last class change
 14.95s from window start, then approximately20s stable. No physical collision,
  release, retreat or full box success has been demonstrated.
- Offline latest-O native FCL of retained descent/retreat fails at descent
  sample0: `elfin_link2` vs O objects644/670/699/793, max depth~.0316m.
  The actual attached envelope has zero occupied voxel centres in/near it:
  payload-self-map hypothesis is disproved for this case, not a general claim.
- Saved acquisition-TF + full URDF MoveIt ShapeMask replay: optical clouds
  retain all13331 points. Each LiDAR cloud removes610 finite robot endpoints
  plus1101 clipped/nonfinite endpoints;9809 remain. Robot endpoints fall
  directly inside all four native collision O objects (8/5/1/1 hits). This is
  direct evidence of robot returns contributing to the current map collision.
- Recommendation: use standard MoveIt body masking on BOTH sources BEFORE ray
  integration. Rationale: O must represent external obstacles, not measured own
  arm; a model/stamp-based input repair preserves all other obstacles and does
  not delete collision voxels after a refusal. Zero extra scale/padding; no
  hardcoded link2 region or GT cargo. All exact-stamp body TF required, whole
  frame refused on any missing TF; removed hits drop whole rays.
- Existing C++ URDF self-mask is unbuilt ROS1; Python filter uses hand-configured
  bodies and optional latest TF. Neither is the current ROS2 O input path.
  New thin ROS2 adapter delegates classification to existing MoveIt ShapeMask.
- build38 fails CMake due Humble perception export requiring GLUT imported
  target. Adding explicit find_package(GLUT) repairs deterministic build;
  build39/build40 succeed with unchanged inputs. Installed192 checks pass.
  First native behavioral fixture used a private rclpy context with implicit
  default-context spinning; fixture repaired to use its explicit executor.
- Evidence: diagnostic_24/{latest_suffix37b,self_mask39,
  self_mask_contact_review.json,carried_voxel_review.json,
  observer_window_review.json}; source replay scripts retain exact inputs.

## Diagnostic 25 -- 2026-11-13T03:45+08:00

- Actual grasp/lift/carry completed again; carry retained duration141.68s,
  preparation27.52s, execution215.05s. Descent not submitted. Three source
  checks converge, then SETTLED_BOX_ADMISSION_REFUSED. This run is incomplete.
- Latest target volume contains121 occupied voxels, all first Z layer
  [.53,.55]m. Nominal target bottom is.53m. Native suffix countercheck ALSO
  fails: link3 vs O at descent start and link2 vs O at retreat start. Thus
  settled admission is not the only remaining gate.
- Exact collision-convex halfspace replay finds33 remaining LiDAR own-surface
  endpoints with positive distance <1mm, while optical has none within5cm.
  Actual MoveIt replay with5mm mask removes all33; other10845 finite LiDAR
  points and all6091 optical points retain original frame/stamp/fields.
- Recommended simulation error bound: half the declared GPU LiDAR .01m range
  bin, .005m. Derived from current URDF sensor declaration, not O margin,
  hand-fitted arm regions or a real-sensor calibration. Default adapter bound
  remains0; explicit Gazebo composition derives and records its value/source.
  Bad nonfinite/nonpositive range metadata refuses composition.
- Floor cause remains unproven. Mesh vertical intersections on225 target XYs
  show highest floor visual/collision/physics Z~.529007m, below configured.53m.
  There are no target-floor in-hull hits anywhere in the saved35.7s window,
  so a floor-erasing rule or another planner tolerance change is not justified.
- Before the next live case, extend bounded evidence capture from carry-start
  feedback (768MiB cap, original prewindow10s), preserving both raw/filtered
  input streams, TF, joints and receipts through terminal refusal/hold. This
  captures the earlier views that may have written those121 floor cells.
  No source behavior or admission relaxation follows from that hypothesis.
- Also retain terminal driver outcome across late lease-release evidence
  flushes, and forbid late feedback reopening a closed capture. These repair
  diagnostic integrity; they do not change motion authority.
- Evidence: diagnostic_25/{target_voxel_review.json,latest_suffix41,
  self_surface_residual.json,self_mask42_5mm,floor_mesh_review.json,
  floor_hit_window_review.json}. All stacks/native processes stopped cleanly.

## Diagnostics 26--27 -- 2026-11-13T04:00+08:00

- diagnostic26 fails startup readiness120s: world services and create success
  replies exist, but no controller manager/joints/sensors. No box task starts.
  ROS node/transport service lists and complete launch log preserved; teardown0.
  Root cause remains unknown (world not advancing is a hypothesis).
- diagnostic27 uses identical product/build43 with a unique IGN_PARTITION in
  the external runner; readiness8.94s. This is discovery isolation, not evidence
  of a startup reliability improvement. Only one stack ran throughout.
- First plan stops before seal because new O makes its target inadmissible.
  Existing bounded pre-pick recovery returns to O-backed pickup view13.39s,
  reobserves the same instance, prepares a new whole plan70.76s, seals/lifts.
- The second retained carry is253.56s; six legs total~273s before RPC/native
  overhead. The driver's literal300s action expires exactly at300s during
  carry, returns CANCELLING/DEADLINE with motion_terminal=false. Actual carry
  endpoint/release/retreat/adoption are NOT demonstrated. Original controller
  requests, cancel, joint/TF/source streams and reference CANCELLING pose
  persist before teardown; this is not a collision refusal.
- Recommendation: expose execution watchdog in the explicit profile,600s
  matching the existing runtime maximum, and share it between action server
  and client. Reject before execution if retained trajectory time alone meets
  or exceeds that budget. Remaining wall time is not a latency guarantee.
  Rationale: user chose complete functional qualification before setting SLA;
  spending a known-too-short budget on sealed motion is avoidable.
- Extend source capture from the first action feedback, including align.
  Before-seal map changes can also invalidate floor targets, so carry-only
  evidence misses that cause. Existing768MiB bounded source capture is reused,
  and opted-in EXECUTE failures retain the requested stationary hold even on
  deadline, allowing actual cancel/observer tail evidence. Add latest immutable
  O pages and full scene at failed boundary; capture gaps mark incomplete
  evidence without overwriting the original product failure.
- Eval-only physical dimensions for this selected catalog mesh are
  .515886x.390038x.237463m, not the nominal .55x.40x.25 shorthand in the plan.
  Online geometry remains measured (.495564x.373311x.247161m in retry1).
  Three saved physical model/link6 poses (lift start,carry start,cancelling)
  keep all8 true box corners inside the retained measured envelope, with
  >=21mm per-axis reserve. This rules out gross envelope exclusion only at
  those3 samples; no whole-path contact/success claim. GT was evaluation only.
- Evidence: diagnostic_26/{startup_ros_nodes,startup_ign_services,launch.log};
  diagnostic_27/{cycle/observer_failure,reference,
  payload_containment_eval_v2.json,run/execution}. No floor tolerance, nominal
  target, release gap, map obstacle or static model was changed.

## Startup28--30 and earlier source capture -- 2026-11-13T04:20+08:00

- diagnostic28 repeats startup failure123.53s; unique IGN_PARTITION is not a
  demonstrated repair. Both26/28 log an actual RSP get_parameters response
  send timeout, then gz_ros2_control never logs URDF receipt. This localizes
  the observed failure to parameter response delivery; DDS root cause remains
  unproven. startup29/30 readiness-only rehearsals pass8.99/8.75s, teardown0.
  No box task or motion acceptance is inferred from those passes.
- Recommendation: retain working transport, distinguish this named startup
  refusal from task failure, preserve next delayed-start live graph/backtrace
  before retry. Do not claim unique partition improved reliability.
- Source capture now starts before initialize_empty in opted-in diagnostics,
  retaining earliest pickup/prepare writes rather than only action feedback.
  Earlier-only occupied floor voxels otherwise cannot be traced. Same768MiB
  bounded archive and explicit drop reporting; no map policy changed.
- Behavioral driver checks33 pass; build46 all13 packages passes7.09s with
  no changed source inputs. Actual full cycle and3box baseline remain open.

## Diagnostic31 and simulator sensor isolation -- 2026-11-13T04:30+08:00

- build46 actual cycle364.37s incomplete, retry action234.46s reaches
  endpoint but descent is never submitted: CURRENT_SOURCE_REFUSED, all three
  fresh captures change. Pre-seal first attempt stops on settled admission;
  bounded recovery/new full plan succeeds in actual seal/lift/carry. No release.
- Capture complete472188929 stored bytes, zero drops; immutable final O and
  scene retained. Final O4649 occupied,1022 bottom-layer; only3 occupied voxel
  centres inside measured carried envelope, so bulk map pollution is not
  explained by carried-box interior returns alone.
- Exact native suffix replay on latest O fails descent/retreat at sample0:
  link3 vs O, and payload vs O. This is a planning refusal, not observed impact.
  No real collision detector is claimed. Target full-admission replay retained.
- Full source capture now finds1290 target-floor LiDAR returns in98 frames
  before seal, Z .5300003-.5369219; optical contributes none to that region in
  this window. Same target's225 XY mesh intersections top at~.529007m.
- Eval-only physical link6 versus exact-time TF22 samples: max translation
  .044mm; fixed sensor SDF vs TF translation .00033mm, angle~1e-5rad. These
  comparisons do not explain the millimetre floor bias.
- Isolated ONE-world-at-a-time known-plane calibration reuses actual60.1s
  acquisition pose and unchanged360x32 scan,10 frames per renderer. OGRE2
  target area40 points,17 above configured .53 floor, max plane error+7.918mm;
  OGRE41 points,zero above .53, max+0.413mm. Broader plane errors also decrease.
  This reproduces false floor occupancy without O/planner/robot geometry.
- Recommendation under user delegation: explicit joint profile selects OGRE
  server sensor backend via standard Gazebo flag; bare launch remains OGRE2.
  Record resolved selection; do not erase floor voxels, alter collision gates,
  shrink full model, change release or inject known plane into online map.
  Actual full-model RGB-D/detector compatibility and route still need testing.
  Model URDF/SRDF and atlas identity are unchanged by renderer selection.
- Primary mechanism corroboration (not a substitute for our measurements):
  https://github.com/gazebosim/gz-sim/issues/2743 contributor explains OGRE2
  sample-dependent approximation and recommends OGRE alternative; Fortress
  Sensors.cc confirms --render-engine-server overrides the world SDF setting.
- Evidence: diagnostic31/{floor_hit_window_review.json,floor_mesh_review.json,
  reference_tf_review.json,sensor_extrinsics_review.json,carried_voxel_review.json,
  latest_suffix46}; floor_sensor_{ogre2,ogre}_360/{world.sdf,result.json,
  target_area.json,*.cdr.gz}. Known plane and simulator poses are eval-only.

## Diagnostic32 -- 2026-11-13T04:36+08:00

- OGRE full-model startup8.14s, measured detection/prepare15.03s pass; actual
  seal/lift/carry pass in123.27s execution. No descent submission or release.
  Actual initial O35 then49 occupied, compared with hundreds in earlier runs.
  Same URDF/SRDF and reviewed atlas identity; RGB-D detector compatibility is
  actually exercised, not inferred from the isolated LiDAR calibration.
- At carried endpoint the latest O5010 occupied,48 floor-layer, no occupied
  centres inside/near actual carried envelope. Filtered actual LiDAR endpoints
  have zero own-body halfspace residuals within1cm in the saved last cloud.
- IMPORTANT: latest full O native replay accepts BOTH retained descent and
  retreat. No source deletion, extra margin change or replanning was needed
  for that offline test. It is diagnostic evidence, not physical execution.
- Actual runtime stops CURRENT_SOURCE_REFUSED while the new view is still
  converging. Receipt history shows last occupancy class change150.46s after
  capture begins, about7s after three attempts expire; next18.7s unchanged.
  Latest transitions go206,101,1,2 cells then stop; the last2 changed voxels
  are side-wall/floor boundary near x=.725,y~-.035m, not carried payload.
- Recommended minimal repair: existing profile retry count3->8, within existing
  validated1..8 range and600s action budget. Every retry still captures fresh O,
  checks exact classified map equality, target admission and complete native
  suffix. No synthetic stable map, sleep-until-guessed-success, erased cells,
  hysteresis or spatially weakened certification is introduced. Reason: the
  measured fusion convergence window exceeds the former count; full latest
  native paths already pass. Not a success-rate claim; next live verification
  must actually reach release/retreat/adoption.
- Evidence: diagnostic32/{observer_window_review.json,observer_receipts.json,
  map_changes.json,self_surface_residual.json,carried_voxel_review.json,
  latest_suffix47}. Raw source archive complete/no drops and failed-boundary
  O/full scene preserved; owned teardown0. Three-box gate remains open.

## Diagnostic33 -- 2026-11-13T04:49+08:00

- Explicit OGRE/8 refresh attempts startup8.04s; prepare111.02s;
  actual seal/lift/carry pass, execution211.73s, but descent not submitted.
  Boundary retain response refuses CURRENT_MEASURED_ACQUISITION_MISMATCH.
- Direct cause: same frozen acquisition retained at preparation start expired
  at the perception owner's300s storage lifetime before the post-carry check.
  This occurred before reading latest O; eight refresh attempts remain untested.
  Slow native search (27 checked connections/12 five-second deadlines) and
 148.4s carry consumed the formerly inconsistent lifetime, not proven collision.
- Recommended repair: explicit joint composition derives storage lifetime from
  its existing observation admission window plus execution budget:360+600=960s.
  Original acquisition timestamp, physical freshness checks, identity/model
  revocation and hard expiry remain; lookup/duplicate retain never renew time.
  Legacy composition retains its existing default. Launch manifest records value.
  Missing expected binding now names FROZEN_BINDING_UNAVAILABLE_OR_EXPIRED.
- Focused behavioural checks:84 passed,16 environment-dependent skips, including
  owner service lookup at310s for old/new lifetimes and strict final expiry.
  Build49 passed; manifest-only change requires build50 before next live run.
- Evidence: diagnostic33/{run/prepare/boundaries.jsonl,run/execution,reference,
  run_manifest.json}; actual failure capture durable before owned teardown0.
  Release, retreat, adoption and three-box gate still unachieved.

## Diagnostic34 -- 2026-11-13T04:57+08:00

- Build50/install192 checks pass. Startup9.20s, prepare16.36s, execute105.58s.
  FIRST actual descent and vacuum release both succeed. Retreat not submitted:
  released barrier checks once while occupied O changes1198->1188 (rev37->43),
  then returns CERTIFICATE_OR_SOURCE_NOT_CURRENT. No collision verdict occurred.
- Preserved final boundary O and full native replay accept retained descent AND
  retreat (source revision43, replay0.505s). This distinguishes stale evidence
  from blocked motion. Physical post-release observations are saved separately.
- Source mechanism: execute_leg already has bounded valid-source refresh, while
  validate_* and apply_release_and_validate_retreat each called validate once.
  Recommended consistency repair: reuse the same configured attempt bound for
  unexecuted suffix barrier validation, only when ports marks a valid source
  change. Vacuum/scene event effects stay outside the loop; collision, identity
  refusal, cancellation and deadline stop retries. Full native checks remain.
- Behavioural verification40 pass/46 environment-dependent skips covers source
  convergence, collision no-retry, exhaustion, cancellation/deadline. First
  invocation used nonexistent test filename and ran zero checks; corrected
  invocation above is the actual evidence, not the typo result.
- Evidence diagnostic34/{cycle,run/execution,reference,final_suffix50}; raw source
  archive complete, owned teardown0. No retreat/adoption/three-box pass yet.

## Diagnostic35 and physical handover audit -- 2026-11-13T05:12+08:00

- Startup7.89s/prepare34.36s; actual long carry succeeds but descending refusal
  at execution309.64s precedes any new O read. Exact CheckPrepared response is
  current=false, PLAN_NOT_CURRENTLY_ISSUED. Owner registry still defaults300s.
  This is distinct from frozen acquisition storage fixed in build50.
- Recommended composed policy: same bounded960s storage lifetime for issued
  provenance and frozen observation. Derive from existing admission+execution
  budgets; original timestamps, expiry/no-lookup-renewal and live checks remain.
  Manifest and behavioural composition/expiry checks expose both values.
- IMPORTANT correction to prior intended physics description: diagnostic34
  reference19/20/21 shows same released physical centre z=.70502148 throughout
  ~20s, about5cm above intended settled centre. Source establishes carried SDF
  static=true; SimVacuumBackend.detach only stops SetPose follow and removes
  scene attachment. Dynamic replacement existed ONLY in legacy smoke driver,
  absent from the new route. Thus34 vacuum-off is not physical settled success.
- Minimal wiring repair: simulation spawner now owns ReleaseSimBox, bound to
  current model/generation. Reuse its actual original spawn SDF, change only
  static flag/carry collision mask, observe actual current pose, require observed
  deletion before one dynamic recreate. Same geometry/visuals; no target pose,
  measured dimensions, planner or O write is accepted by this service. Duplicate
  unresolved release cannot issue another destructive command. Actuator presence
  acknowledgement is not settled support. Save full report/SDF on separate
  simulation_release topic; driver archives it under reference only.
- Only explicit joint_operation opts into physical_release_on_detach. Legacy
  smoke driver keeps its original handover ownership and cannot double-recreate.
  Sim vacuum stops future follow, drains in-flight follow lock, then requests
  handover; ambiguous release remains a failure on duplicate detach.
- Focused tests123 pass before final follow-lock/composition assertion updates.
  Build52 compiles typed interface; build53 covers first wiring; final build54
  required before live verification. No physical settle/retreat/adoption claim.

## Diagnostic36 and measured post-place explore -- 2026-11-13T05:35+08:00

- Build55/install192 pass. Actual prepare56.76s; all SIX controller legs pass,
  including descent, dynamic physical release, retreat. Descending succeeds on
  source refresh attempt4; release barrier and retreat pass. This exercises the
  previously unverified source retry/issued lifetime fixes rather than claiming
  them from offline replay. Physical handover READY1.47s, same mesh/SDF geometry;
  captured physical released centre settles at z=.65400652 and remains stable.
- Observation remains incomplete:475 queried top columns,4 matched,471 missing
  (456 free/19 occupied; occupied height errors[-.06709,-.00709]m). Passive view
  receives new source data for~100s without adoption. We explicitly cancel only
  this owned, stationary observation wait after preserving>=60s false evidence;
  cancel ack is not itself a terminal-motion claim. Driver then captures final
  full O/scene and exits incomplete; owned teardown0. No full-cycle pass yet.
- This is the measured missing capability authorized by the user: bounded
  post-place explore. Add only that capability, reusing existing exact IK,
  snapshot OMPL connection, polyline timing, native full-model route checker,
  current O admission and the existing shared controller transport/ownership.
  Extra observation paths are separately certified/archived, never substituted
  into the original six-leg certificate. No live scene or O write is added.
- Camera proposals reuse9 existing geometry-derived interior positions and aim
  optical+Z at the CURRENT missing footprint centre. Rank projected calibrated
  FOV cell coverage, then camera travel; actual observed O remains sole adoption
  authority. Max3 actual extra views, each at most3 fresh snapshot integrations;
  all work shares the existing600s action budget. Unknown/occupied/map/reset and
  support gates are unchanged; released/unloaded state is mandatory.
- Why aim: first archived straight-down review has only2 positive-FOV proposals;
  one lacks IK, one covers320/471. Aimed review admits8 native paths,7 of those
  project all471 missing columns. This is a ranking proxy without occlusion
  guarantee, not measured coverage or functional success. Runtime must measure
  actual gain. No camera pose, nominal placement or simulator dimensions fill O.
- Coverage instrumentation now evaluates the entire footprint rather than
  returning at its first failed cell. Same all-columns/one-source-voxel height
  predicate; no relaxation. Container-absolute coordinates stay explicit.
- Checks47 geometry/runtime cases pass;94 focused cases including launch/driver
  and fresh-vs-pre-settle observation boundary pass. Actual native/controller
  tests50 pass (46 original+4 exact unloaded view admission cases). Archived
  full-model view replay36c uses actual final O, no controller; all3 native
  backends tear down0. First replay36 had a diagnostic configuration type error;
  preserved as failed evidence, then corrected in36b/c. It is not a live failure.
- Evidence: diagnostic36/{cycle,run/execution,reference,observation_wait_cancel,
  release_coverage_review.json,release_coverage_arrays.npz};
  offline_explore36{,b,c}/; native_motion56/ and native_motion57/.

## Release observation review and persistent cycle wiring -- 2026-11-13T05:50+08:00

User's10h delegation supersedes the earlier wait-for-discussion step for this
measured admission review; recommendation and reason are recorded in
`docs/status/evidence/three_box_observed/g2/wiring59/decision.json`.
Diagnostic37 completed all6 original motions and3 actually checked camera views,
with actual dynamic release/settle. The action still refused at298/475 observed
columns. Eval-only physical mesh raycast proves38 demanded columns cannot match
at nominal target height;140 attainable columns remain unobserved. The old100%
rectangle gate therefore conflates missing observation with a false geometry
assumption. No reference mesh/pose is supplied to online consumers.

For the explicit joint profile only, release observation uses the existing
`min_support_ratio` as its minimum directly occupied, height-consistent column
fraction (currently0.6); unconfigured runtime defaults1.0. This is an explicit
coarse observation completion witness, not complete box reconstruction, physical
identity proof or stacking permission. Capture acquisition must still postdate
actual settle/release. Missing cells remain unchanged. No support/tilt/volume,
collision, inflation, release-height or motion certificate threshold is changed.
Future placements consume actual complete O and retain every original hard gate.

The same diagnostic37 O has0 admissible stack candidates;16 near-passing windows
have support>=0.6 but0.04m height span, exceeding the unchanged0.05 tilt ratio.
This remains an actual blocker, not reported as successful stacking. Full3box
functional gate remains AND over3 observed successful terminal empty-tool cycles,
same persistent epoch and separate physical support reference. Added driver flag
`--three-box` uses floor/floor/stack intent, exact pinned actual O,4 directions,
existing query/ranking and native complete route. Finalize preserves each physical
box before next spawn; return view retains full O but adds no phantom pickup.
Raw capture rotates per box and triggers on post-place exploration/failure;
all typed plan/map/certificate boundaries persist independently. No prior
placement, simulator geometry as online input, new planner or cache is added.
104 focused behavioral cases pass; build/install/live evidence follows.

## Diagnostic38 initial observation race -- 2026-11-13T06:00+08:00

Build59/install192 pass. Three-cycle attempt stops before any product motion:
first explicit floor enumeration then prepare refuses independent detection/domain
mismatch. `diagnostic_38/observation_pair_review.json` proves submitted detection
and saved domain share exact stamp11.287s/digest32c4a0f1 and match all geometry;
later domain differs. Source establishes prepare reads latest domain, while the
new client full candidate enumeration took2.85s. Exact service-side domain was
not captured, so this race mechanism is supported but the original service
acquisition is not directly recoverable. Added typed `domain_admission_input`
capture before validation so the next failure is complete evidence.

Recommended minimum wiring fix: after observed candidate enumeration, reacquire
fresh matched detection/domain immediately before submission. Preserve candidate
support height from O while adjusting center by half the newly measured height
difference. These remain proposals: main shared admission rechecks actual new
size/volume, contact and full native trajectory. No pose or dimension approval
is inherited from the older acquisition. This avoids adding a second domain
history/cache and its raw-context lifetime problem. No gate is loosened and no
failed full planning run is automatically retried.39 focused wiring cases pass.
Diagnostic38 failure raw/map/scene captured; teardown0;3box remains incomplete.

## Diagnostic39 startup lost response -- 2026-11-13T06:07+08:00

Build60/install192 pass. Startup only, no box attempt: controller manager loads
joint_state_broadcaster but logs failed DDS service response. Standard Humble
spawner times out10s, retries LoadController, receives already-loaded failure,
exits1. Old unconditional OnProcessExit still starts arm/MoveIt, but no joint
state publisher; readiness fails SENSOR_OR_JOINTS_NOT_READY after120s. Full launch
log/ready capture/owned teardown0 preserved. Later controller-state CLI was too
late (after teardown), so no actual state listing is claimed.

Recommended minimum wiring repair for explicit joint route: standard serial
spawners advance only on exit0. One fresh standard spawner on failure queries
actual existing controller state via built-in ListControllers and skips an
already-loaded controller; second failure emits named shutdown. No force-active,
duplicate load implementation or manual controller state is introduced. Legacy
startup path unchanged. Four actual launch-event-handler cases cover normal
advance, lost-reply recovery branch and repeated failure at both stages; all
selected launch/driver cases pass. Live healthy startup rehearsal follows;
transport failure-rate improvement is not inferred unless that branch occurs.

## Diagnostic41 first observed pass; second texture refusal -- 2026-11-13T06:22+08:00

Build61/install192 and startup-only diagnostic40 readiness7.94s/teardown0 pass.
Diagnostic41 first floor cycle returns success, terminal empty tool, epoch2
revision1115 after actual six legs, dynamic release and one checked camera view.
Second box spawn succeeds but no matching measured top in30s; RGB shows same
shape rendered white and DetectionFrame YOLO empty/DETECT_NO_CLOUD. Failed case
captures complete actual image/K/depth/raw/source/scene; teardown0. No second
box motion. First target support/actual payload still need final physical audit.

Two short frozen-build61 renderer controls establish cause boundary without
robot product motion: delete/reuse first mesh gives detect1 then0; retaining and
moving first physical instance before same mesh second gives detect1 then1.
The error follows last-instance removal rather than multi-instance coexistence.
Official OGRE MeshFactory skips an existing mesh URI on reload; material destroy
can clear associated resources. Exact installed binary lifetime is not traced,
so this remains source-supported mechanism plus direct controlled observations.

Recommend per-incarnation file URIs containing IDENTICAL copied original visual
assets, all hashes recorded in run/render_assets; only URI text changes, no
physics/collision/texture content changes. Prepare visual assets before clearing
old instance; resource failure preserves it and RNG/sequence. Enable only the
explicit joint route. New helper is a simulation resource repair, not perception
fallback or another planner. Bare legacy path unchanged. Rehearse deletion and
actual dynamic-replacement controls before another full3box run.

Separate same-image density test:640x360, same measured K and stationary actual
FK (nearest joint288ms earlier, max velocity2.1e-13); stride4 reaches270 target
cells, stride2 432, stride1 438. These are projections, not integrated-O claims.
Select explicit profile stride2:4x direct pixels vs4, almost stride1 coverage at
one quarter of stride1 rays. O resolution2cm unchanged, no interpolation/fill.
Copied visual assets and sensor-density choice recorded at wiring62/decision.json.
Per-box raw capture384MiB,3boxes<=1.125GiB raw plus bounded typed artifacts;
trigger also on successful adoption so passive completion keeps a raw window.
78 focused cases pass. Support ratio/tilt/gap policy remains unchanged: offline
.01/.02/.03/.05 support-contact band sweep recorded, NOT applied merely because
some values yield passing stack candidates. Full3box functional gate unproven.

## Diagnostic42 frozen-acquisition handoff -- 2026-11-13T06:37+08:00

Frozen62 renderer controls pass for delete/reuse and dynamic release replacement.
Diagnostic42 readiness9.09s, box1prepare43.18s and complete execution351.90s
pass; post-place view measures432/475 target columns (90.95%) at2cm O. Actual
settled contact capture identifies box1 body versus ULD collision mesh, at floor
height. Return pickup succeeds; box2 rendering/detection is restored. Box2
prepare refuses before map/native planning on frozen acquisition mismatch.
Captured server admission detection/domain stamps and requested digest agree;
retention roundtrip1.87ms. The old response does not distinguish unavailable
context from a newer digest, so that exact live subcause is not established.
Failed case raw/map/scene complete and teardown0.

Source clears current samples BEFORE constructing the next context. Reentrant
retain during that construction reproduces refusal in2/2 offline behavioral
cases, including a valid next frame. Recommend complete-then-swap publication:
keep last published immutable acquisition while building; valid next frame swaps
matching samples, invalid next frame explicitly clears. Identity/model revocation
and original acquisition freshness unchanged; no history or permissive fallback.
New refusal diagnostics distinguish unavailable from differing digest for future
evidence.76 focused cases pass,17 opt-in ROS transport cases skipped; no measured
live success-rate improvement claimed. Evidence wiring63 and diagnostic42/
frozen_handoff_review.json. Three-box gate remains incomplete.

## Diagnostic43 second-box candidate starvation -- 2026-11-13T06:55+08:00

Build63/install192. First observed floor cycle passes: prepare39.15s, execute
209.14s, return pickup55.52s. Box2 freezes exact measured observation successfully
then fails SEARCH_INCOMPLETE34.15s; no second product motion. All8 local requests
cover only4 placement proposals (2 directed contacts each), all NO_PLACE_LOCAL_BRANCH.
Typed collision samples name carried conservative box versus O rear-wall voxel
481/529, not box1. Original nearby-deep/shallow unused proposals fail5/5 saved
native local replays; five existing shared-admitted interior proposals pass5/5
with2 place branches each, unchanged grasp/scene/model. No full-route claim.
Evidence diagnostic43/second_{local_rejection_review,payload_ranking_review}.json
and second_candidate_native_replay,second_central_native_replay. Actual contact
ledger only reports ULD floor for released box1; static carry means this is not
physical collision proof for the whole carry. All failed raw/scene/O captured;
teardown0. First boundary actual O gives52 legal stack proposals at unchanged
shared gates; previous stride4 snapshot gave0. This is evidence of availability
for that measured workload, not a generalized rate improvement.

Recommend minimal proposal ordering and budget-unit repair (plan append): keep
packing preference, reserve an existing interior fallback per heading before
old shallow fallback, retain8 placement groups with2 contacts each rather than
spending8 groups on4 placements. No new target or planner, no relaxed geometry.
Extra maximum16 contact/native candidate attempts and potential packing-order
tradeoff explicitly recorded. Unit tests exercise exact resource bound and
legacy retention. Source of observed normal mismatch in diagnostic ranking is
not yet changed; ranking remains only a proposal, native actual-envelope checks
remain final authority.
