# OCC structure oracle commands

Run from the implementation worktree. Do not point `PYTHONPATH` at the main workspace or another install.

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=.:src/luggage_perception/test:src/luggage_perception:src/luggage_description \
python3 scripts/occ_structure/freeze_baseline.py \
  --out docs/status/evidence/occ_structure_optimize_20261102/t_freeze \
  --yaw /home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_structure_review_20261102/yaw_scalar_vectorized.json
```

`freeze_baseline.py` exits non-zero if product files differ from `a34472d0682f35172d1fc80a0cfb654a0b8ddc16`.

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=.:src/luggage_perception/test:src/luggage_perception:src/luggage_description \
python3 scripts/occ_structure/compare_candidate.py \
  --expected docs/status/evidence/occ_structure_optimize_20261102/t_freeze \
  --yaw /home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_structure_review_20261102/yaw_scalar_vectorized.json
```

Regression, including this replay:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=src/luggage_perception:src/luggage_description \
python3 -m pytest -q -p no:cacheprovider --tb=line \
  src/luggage_perception/test/test_fusion_vectorization_oracle.py \
  src/luggage_perception/test/test_depth_view_rays.py \
  src/luggage_perception/test/test_cargo_grid_projection.py \
  src/luggage_perception/test/test_cargo_surface_projection.py \
  src/luggage_perception/test/test_cargo_map_commit_contract.py \
  src/luggage_perception/test/test_cargo_map_confidence.py \
  src/luggage_perception/test/test_frame_migration_map.py \
  src/luggage_perception/test/test_cargo_map_query_contracts.py \
  src/luggage_perception/test/test_cargo_query_indexes.py \
  src/luggage_perception/test/test_occupancy_map_store.py \
  src/luggage_perception/test/test_occupancy_map_store_anchor.py \
  src/luggage_perception/test/test_occupancy_map_store_confirmation.py \
  src/luggage_perception/test/test_occupancy_map_store_projection.py \
  src/luggage_perception/test/test_occupancy_confirmation.py \
  src/luggage_perception/test/test_occ_structure_oracle.py
```

The oracle test adds `src/luggage_perception/test` itself. The 14 existing files do not need that path.
