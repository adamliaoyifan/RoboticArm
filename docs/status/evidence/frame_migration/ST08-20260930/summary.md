# ST08 execution evidence

- Base: `8debda0c49438c7d8688e39fe5684dcf7fea546f`
- Plan: `db62e460da66fffb3f342e070c7e01bcc9068043`
- Branch: `integration/frame-migration-st08-20260930`

## Gates

- `pytest src/luggage_planning/test/test_frame_migration_execution.py`: 12 passed. The three matrix methods each walk 64 geometry cases (execute, collision, attach).
- `pytest src/luggage_description/test/test_frame_contracts.py`: 15 passed, 57 subtests.
- `check_frame_contracts.py --mode baseline`: exit 0, 50/50.
- `check_frame_contracts.py --mode migration`: exit 2, 5 pending (`F01,F21,F22,F23,F24`).
- `pytest src/luggage_planning/test`: 690 passed, 171 subtests, after `luggage_description` is installed. That is the accepted 678 plus 12 new tests. The eight atlas FK certification tests pass in that install and error only when the package is absent from the overlay.
- Packing, same install `/tmp/elfin-frame-migration-st08/install/st08`, `ROS_DOMAIN_ID=87`, and the same pytest command on base `8debda0` and candidate `0dadfe0`: both collected 128 node ids (lists equal) and both finished `1 failed, 127 passed, 81 subtests`, exit 1. The only failure on both is `test_placement_geometry_identity_live.py::TestGeometryIdentityLive::test_matching_geometry_answers_with_its_hull_hash` with `ATLAS_IDENTITY_UNVERIFIED: urdf hash missing`. Logs, node ids, and the comparison are under `run/packing/` and `run/comparison.json`.
- Planning on that same install: base `678 passed, 171 subtests`, exit 0; candidate `690 passed, 171 subtests`, exit 0. The 12 extra candidate node ids are `test_frame_migration_execution.py`. Node-id lists are under `run/planning/`.
- Execution rerun with `FRAME_MIGRATION_CASE_LOG` wrote 192 JSONL rows (`run/execution/cases.jsonl`). One negative bundle, empty `request_id` -> `MISSING_IDENTITY` with effect counters all zero, was reloaded and joined (`run/negative/`). Checker stdout is under `run/checker/`. `run/manifest.json` records revisions, commands, exits, and `capture_complete`.

F08, F11, and F13 proofs were rehashed because `SelectPlacePathV2.srv`, `BuildMotionSequenceV2.srv`, and `place_service.py` changed. Their migration state stays verified. F15–F18 are verified by the runtime proofs in this directory.
