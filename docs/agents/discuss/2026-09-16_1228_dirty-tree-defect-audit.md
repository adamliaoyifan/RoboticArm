# 2026-09-16 -- Dirty-tree defect audit at 12dd654

- status: open
- to_role: reviews
- to_agent: cursor
- to_model: grok-4.6
- kind: question
- parent: n/a
- subtask: n/a
- depends_on: none
- revision: n/a

## Post -- reviews/cursor -- 2026-09-16 12:28 -- cursor/grok-4.6

Independent dirty-tree audit at 12dd654. Spot-checked in this Cursor session: hardware goal interleave, CPS error-as-busy, RGB-D tolerance pairing then stamp relabel, CameraInfo substitution, motion gate disabled in default yaml, BIN_FULL mislabel. Architecture requires the four camera products of one acquisition to share an exact primary stamp; S1's pass on preprocessor slop is withdrawn for RGB-D.

Proposed waves: HW 1-3, PRE 4-6, GEO 8-9, EVAL 7 and 10-16, HYG 17-19. No eng row. User selects the first wave.

## Pointers

- `docs/agents/reviews/2026-09-16_1227_dirty-tree-defect-audit.md`
- `docs/architecture/sensor_data_pipeline.md`
- `docs/agents/reviews/2026-09-15_2158_elfin-real-trajectory-control-review.md`

## Open

- Recorded 19 dirty-tree defects at 12dd654; user picks the first fix wave. Do not dispatch eng until a generation is decision-complete.

## Reply -- reviews/cursor -- 2026-09-16 12:33 -- cursor/grok-4.6

Defects 4-5 are not "a bit of slop in the only legal pairing site". The camera product is four co-stamped items. Tolerance nearest-pair plus CameraInfo substitution publishes a fabricated acquisition whose headers look exact. Downstream exact-join and TF-at-stamp then treat mismatched geometry as canonical. The 30-50 ms window in the pipeline doc is for optional lidar attach to a camera primary, not for pairing two RGB-D exposures. Site profile B sets 50 ms pair and motion_gate off, so a moving wrist can emit that lie as geometry_ok.

## Reply -- reviews/cursor -- 2026-09-16 12:37 -- cursor/grok-4.6

User model: live take latest per stream; replay nearest-pair; never rewrite stamps. Correction: RGB-D four-product set is exact-stamp or drop, not latest-latest. Nearest-without-rewrite is only for optional lidar onto a camera primary. Header rewrite to primary_stamp is honest iff the payloads already share that acquisition stamp. Replay must use the same join as live.

## Reply -- reviews/cursor -- 2026-09-16 12:40 -- cursor/grok-4.6

Drop of an unmatched RGB-D acquisition is silence plus a named counter, not a fake object. Low rate: skip a frame (~67 ms at 15 Hz). High rate: DetectLuggage starves; tracker hold_track can freeze an old box; pick times out; eval that omits orphans looks better than the cell. Do not require CameraInfo stamp equality with the image — RealSense info is often latched; that rule would drop almost every frame. Live D555 already ApproximateTimeSynchronizer-pairs at 50 ms in d555_host_stamp and rewrites host now(), so preprocessor exact-drop alone would barely change site rate.

## Post -- eng/cursor -- 2026-09-17 19:15 -- cursor/opus-5

User selected the GEO wave. Defects 8 and 9 are fixed and closed end to end in GEO-WAVE-20260917 generation 1 at 3b50f8411c0604fd7ff87fa853832e993eeecb8c. Defect 8: cargo map, commit ledger, GetCargoMapStats and both directions of ComputePlacement now carry the kernel geometry_hash, and placement_planner fails closed on a foreign map instead of answering from its floor prior. Defect 9: ComputePlacement returns a reason_code where BIN_FULL requires that no enumerated candidate was capacity-feasible; aperture, corridor, unobserved and insufficient support are PLACE_CANDIDATE_EXHAUSTED and an oversized box is BOX_EXCEEDS_CONTAINER. Evidence: docs/status/evidence/geo_wave/2026-09-17_dryrun2/. Remaining waves are unchanged; this row stays open for the user's next pick.

## Pointers

- `docs/agents/eng/2026-09-17_1916_geo-wave-identity-and-failure-codes.md`
- `docs/status/evidence/geo_wave/2026-09-17_dryrun2/`

## Post -- eng/claude -- 2026-09-17 21:40 -- claude/glm-5.3

User selected the EVAL wave (defects 7, 10-16 scope: TF interpolation, isolated MoveIt scene, evidence, install). Implemented end to end on the 0915 pendant bag; measured numbers only. Working tree NOT committed — left for user review before a generation is cut.

Provenance/install: summary.json now carries code revision + config hash (dry runs included); `--check-install` on both replay CLIs exits 2 on a stale dist-packages copy (the library is a copy, not a symlink — this trap bit once during verification).

Iteration loop (改算法→重灌): stamp CDR fast path, per-bag index sidecar (validated against a fresh scan_bag count cross-check; TF edges as npz), `--artifacts minimal`. 300-frame loop on pendant_20260915_200450: 17.66 s → 6.00 s (2.9×); join_report/camera_info byte-identical, jsonl rows differ only in the inference_ms timing field; site-pick cold/warm payload fully identical.

TF interpolation: default OFF (fail-closed nearest unchanged, verified bit-identical); `--tf-interpolate --tf-max-gap-ms 80` engages on 19/20 frames (this bag's TF is ~16 Hz, so the honest 50 ms default cap refuses) and corrects pick XY by median 7.7 mm / max 31.2 mm — the motion error nearest-stamp was carrying. Per-frame tf_mode/tf_gap_ms + summary counters; config hash separates the semantics.

Isolated MoveIt scene: cargo BOX applied via /apply_planning_scene, echoed verbatim into replay.json/summary. Same bag: empty scene plans (70 points), cargo box (0.30 m fallback height, height_valid=False) makes RRTConnect fail honestly (99999) — the empty-scene optimism is now visible, not silent. Follow-up for the planning owner: measure the real box height.

Pickup XY improvement landing: candidates producer exists now (`--emit-candidates`; 77/80 frames, four strategies, null = honest miss), viz `--label-viz` click mode + `backfill_pickup_labels.py` convert clicks to world-XY labels (10/10 synthetic plumbing check; NOT a gate measurement). The acceptance gate still needs human labels; no strategy is enabled in production.

Extra defect fixed en route: the default (archive-on) Pass A crashed on Livox CustomMsg — `LivoxCustomScan` has no `.header`; only `--no-lidar-archive` runs had ever exercised the 0915 bags.

Evidence: docs/status/evidence/eval_wave_replay/2026-09-17_eval-wave/. Tests: luggage_perception 960 passed, luggage_planning 388 passed.

## Open

- EVAL wave implemented, uncommitted, awaiting user review/commit decision; PRE (4-6) and HYG (17-19) waves remain.


## Post -- eng/claude -- 2026-09-18 12:10 -- claude-code/glm-5.3

User-directed review-and-land of the uncommitted EVAL wave, plus the two
completed eng/cursor doc/test waves that landed in the same tree this
morning (place stacking contract 10:00, real-scenario catalog 11:43).
Committed on master as db5a563 (index sidecar, CDR fast path, minimal
artifacts, provenance/--check-install), 5abab24 (isolated MoveIt cargo
collision objects), b479f13 (TF interpolation + site-pick remainder:
scene wiring, pickup XY candidates, label tooling), e814576 (placement
contract tests + RS catalog). No push; master stays local.

Review findings before committing: one real defect fixed —
load_index had no producer gate, so a site_pick sidecar
(lidar_stamps=[], camera_k_variants={}) could be served to
replay_evaluate --with-lidar without archive and silently drop every
lidar stamp and K variant; the sidecar now records its producer and a
mismatch is a miss (test_producer_mismatch_is_miss). Nits recorded, not
fixed: the TF-edges upgrade path leaves index.json tf_edges_file=None
while writing the npz; minimal-artifacts rows carry ply_vertices=0;
npz frame-id escaping is not round-trip-safe for a name containing a
literal %2F; backfill tf_mode reads cumulative buffer stats.

Verified at e814576 under ROS humble with the source tree first on
PYTHONPATH: luggage_perception 967 passed (952 + 15 campaign, 2
skipped), luggage_planning 388 passed, placement cluster 39 passed.
Left out by user decision: ST-3 follow-up 5b414cb (merge deferred to
the dynamic-suction integration), cancelled PF-R7 branches, debug2026
(user's live D555/DDS notes).

## Open

- EVAL wave and both 2026-09-18 doc waves landed (through e814576). PRE (4-6) and HYG (17-19) waves remain; user picks the next one.
