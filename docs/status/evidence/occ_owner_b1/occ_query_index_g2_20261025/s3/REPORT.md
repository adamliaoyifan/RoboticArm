# S3 dirty-column surface implementation checkpoint

Baseline: user-adopted b33dda91; parent implementation ae9281f0. Plan remains b9f4b2dc. No overall G2 PASS.

Ordinary transactions recompute only unique XY columns whose net state/source/label changes. Immutable 16x16 tiles retain unchanged data; instance/shadow/log-only changes do not rebuild projected columns. Reset/init uses the unchanged full projection oracle. Legacy mapper/store exports, confirmation gates and digest inputs retain their values. The Store refreshes derived columns before its existing positive-free removal confirmation read, then publishes after confirmation transitions. This refresh has no external publication side effect.

New schema-1 surface() returns frozen detached column rows plus existing v2 datum fields. ROI selects whole columns by positive XY intersection (an optional finite XYZ ROI validates but does not clip Z). surface_changes(since_identity, region, page) requires an explicitly retained same-epoch predecessor; it emits actual net changed columns in stable x-fastest order, <=4096 per page, and HMAC-binds cursor to both identities and ROI. Missing predecessor expires explicitly; never substitutes CURRENT. COW tile bytes are included in the 512MiB budget estimate.

## Verification

- Focused first run: 25 passed (three resolutions, random inactive/state/source/label columns, transactional overlap/shadow/reset, immutability, bounded 4225-column patch, ROI/cursor/expiry).
- Extended full focused/regression command in regression_fixed.log: 127 passed + 2 subtests. Includes 20 seeds x500 existing summary/frontier changes, S2 query/concurrency tests, existing Store/confirmation/projection/anchor/commit and b33 vectorized fallback tests.
- Initial extended run: 126 passed +2 subtests, one new test fixture failed before mapper creation: width=.2 with chamfer.wall_y=.2 is invalid. Corrected fixture width/length=.4. Raw logs, inputs and standalone replay are retained in failures/invalid_chamfer_fixture/. Product behavior was not weakened.
- The replay worker now optionally emits exact old mapper/v1/v2 serialized structures with only random reset epoch normalized. Baseline 10,000 rows are captured; clean post-commit candidate comparison is pending at this checkpoint.

## Limits

These are offline correctness tests, not formal timing/heartbeat/RSS or full shape/resolution/box-count matrix acceptance. Typed ROS, S4 distance, S5 installation/load/placement gates remain pending. No simulation, motion, shared install, planner/lease/fusion modifications or production merge.

## Frozen post-commit replay result

Frozen 7a77256f and b33 separate worker processes produce exactly identical 10,000 rows including full mapper/v1/v2 exports, primary fact hash, revision, old digest, acceptance and stats. Shared SHA256 ef2cf7d769e3bb3662933955b99a8f14371e45dd4cdfb8b1a9786520f0d1f39e. Product source hashes stayed unchanged. See surface_replay_manifest.json; this is the small .12m/.02m fixture, not the full formal matrix.

The first export comparison exposed a generator clock issue: 1,679 v1 acquisition_stamp_ns fields used independent wall times; all other fields were identical. Original rows/manifests/first difference are retained in failures/serialized_export_difference/. Replay worker now injects the same Store clock (1000000000+transaction) into both processes; the timestamp field remains compared. This does not normalize away a product field or modify Store timing semantics.
