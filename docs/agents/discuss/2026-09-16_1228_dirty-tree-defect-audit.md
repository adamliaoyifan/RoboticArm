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

## Open

- GEO wave (defects 8-9) closed at 3b50f84; user picks the next wave.

