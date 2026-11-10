# S1 incremental status/frontier implementation checkpoint

Implementation baseline b33dda91 adopted by user instruction; original 352191b3 oracle and plan b9f4b2dc remain recorded. No repeated vectorized frontier/lease fix: b33 fallback, diagnostics and tests are preserved.

Store owns the net evidence collector and derived counters/frontier bitmap. Stats adapts the published immutable summary. Node reads snapshot+summary from one atomic bundle. A changed state re-evaluates only that cell and its six neighbors; label/instance/shadow changes are captured even without state changes. Primary lists, fusion expressions, ray ordering, clipping, revisions and commit acceptance are unchanged. Legacy frontier enumeration order remains ix/iy/iz. Index faults preserve committed facts, reject dependent queries explicitly, and recover only on explicit reset.

## Demonstrated

- 76 focused/regression tests and 2 subtests pass (unit_tests.log).
- 20 seeds x 500 mapper mutations match retained full count/frontier oracles after each transaction, including geometry shadow/removal/reset-preserve and label changes.
- Separate b33 baseline/candidate processes execute another 20 x 500 Store transactions. All 10,000 rows are identical: fact hash includes live/logodds/lock/shadow/source/identity, status/reason/revision/old content_digest/stats match. 4,517 distinct fact states; 6,396 accepted, 653 idempotent replays. See transaction_comparison.json and raw JSONL.
- Single state change evaluates exactly seven frontier positions. Hot stats use cached counts (zero voxel reads); mutation of exported label dictionaries cannot change the published summary.
- Injected index failure retains the box commit and ordinary snapshot, returns QUERY_INDEX_UNAVAILABLE, reset recovers.

## Not yet evaluated

This is an implementation checkpoint, not S1 or G2 overall PASS. Formal ROS stats, heartbeat concurrency, query/write performance and memory gates await independent installed acceptance. Tiny cuboid transaction replay is not the complete three-resolution/chamfer/0-5-20-box matrix or concurrent oracle. S2-S5 remain active. No motion/stack started, shared installation and integration branch untouched.

## Replay

```bash
python3 docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025/s1/replay/transaction_worker.py --root /tmp/occ-query-g2-baseline --out /tmp/baseline.jsonl
python3 docs/status/evidence/occ_owner_b1/occ_query_index_g2_20261025/s1/replay/transaction_worker.py --root /tmp/elfin-occ-query-index-g2 --out /tmp/candidate.jsonl
source /opt/ros/humble/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:$PYTHONPATH python3 -m pytest -q src/luggage_perception/test/test_cargo_voxel_delta.py src/luggage_perception/test/test_cargo_query_indexes.py src/luggage_perception/test/test_frontier_count_vectorized.py src/luggage_perception/test/test_occupancy_map_store.py src/luggage_perception/test/test_occupancy_map_store_projection.py src/luggage_perception/test/test_voxel_log_odds.py src/luggage_perception/test/test_cargo_grid_projection.py
```

Initial stub-adapter failure and schema-mismatch generator rehearsal are preserved; they were not used to claim gate coverage.
