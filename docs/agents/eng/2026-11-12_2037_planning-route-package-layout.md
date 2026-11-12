# 2026-11-12 -- Planning route package layout

- role: eng
- agent: codex
- model: gpt-6
- cli: codex
- status: open
- parent: PLANNING-ROUTE-LAYOUT-20261112
- subtask: PACKAGE-LAYOUT
- base_revision: dfbcc39fa8c5e58defc46d7fdb33f3f30bb38e35
- started_at: 2026-11-12T20:37:45+08:00
- completed_at: n/a

## Summary

Group 72 flat Python modules into joint_operation, placement_prior, routing and
atlas packages. Update active callers without duplicate implementations/import
hooks. Preserve ROS names and route defaults; document actual entry ownership.

## Requirement

Execute plan `bd46a9f03f6aaba0e5cc746134856ee8611e175b`. Verify frozen source
preservation and before/after behavioral matrix. Do not absorb pre-existing dirty
algorithms or untracked runtime implementation into the refactor commit.

## Changed

- 29 whole-operation, 19 prior, 14 routing and 10 atlas modules moved; 256 active
  source callers/import strings updated. Module paths shorten redundant prefixes.
- Package README indexes ROS entries, Python entries, shared backends and actual
  `planning_pipeline` selection. Architecture links this index.
- Explicit old/new map in `config/python_module_layout.json`. `placement_prior`
  is now a package; the former module's functions are in `placement_prior.core`.
- Build validates every replacement before pruning only mapped obsolete installed
  Python files. Real install removed 144 stale source/bytecode files; repeat build
  is harmless. Checker verifies all 72 modules and absence of obsolete flat files.
- Atlas source resource anchor adjusted for one deeper directory, including
  source-only config/data/mesh discovery. Explicit module_file injection retains
  its original flat-layout meaning. No planning algorithm/default/margin changed.

## Verification

- Exact commands and frozen inputs: evidence `baseline.json`, `migrate.py`, native
  `native_runs.json`, build receipts and JUnit XML. `REPLAY.md` gives procedures.
- Frozen 32-file baseline:536 pass,11 skip in12.01s. Same547 case IDs and outcomes
  after mechanical move:536 pass,11 skip in12.29s. Extended path/cleanup regression:
 544 pass,11 skip in12.38s. Existing skip reasons unchanged.
- `python3 docs/status/evidence/planning_route_layout/20261112/migrate.py check`:
 256 files match the explicit mechanical transform; installation checker has its
  separately tested extension, atlas resource-anchor adjustment is explicitly
  encoded. Zero remaining active old qualified/unqualified imports.
- `scripts/production/build.sh --evidence docs/status/evidence/planning_route_layout/20261112/build_03`:
 13 packages pass,6.666s, zero concurrent source changes.
- `scripts/production/run.sh /usr/bin/python3 scripts/production/check_install.py --output docs/status/evidence/planning_route_layout/20261112/installed_03.json`:
 188 checks pass, including imports/source bytes of all 72 relocated modules.
- `scripts/production/run.sh /usr/bin/python3 docs/status/evidence/planning_route_layout/20261112/verify_native.py`:
 real native MoveIt aperture3, G prepare16, O observer10, connector10, endpoints7,
 certification10 pass. Each fixture uses fresh paths and owned teardown; no Gazebo.
- `scripts/production/run.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q src/luggage_planning/test/test_production_install.py scripts/production/test_module_layout.py src/luggage_planning/test/test_module_layout_paths.py`:
 13 pass, including incomplete migration refusing cleanup and preserving old files.
- `ros2 launch luggage_planning joint_operation_prepare.launch.py --show-args`
 through production runner passes. Additional hardware launch inspection cannot
 complete: installed prefix lacks `elfin_trajectory_executor`; original HEAD
 already requires that package at launch line442. No hardware validation claimed.

## Result

Verification passes; commit pending. Working-tree validation includes pre-existing
dirty changes and untracked runtime files, frozen in evidence; it does not certify
an isolated clean checkout or whole Gazebo/hardware baseline. Stage transformed
HEAD only for previously tracked files; leave concurrent content dirty at the new
paths and leave existing untracked implementation untracked.

## Pointers

- `src/luggage_planning/README.md`
- `docs/plans/PLANNING_ROUTE_LAYOUT_20261112.md`
- `docs/agents/discuss/2026-11-12_2037_planning-route-package-layout.md`
- `docs/status/evidence/planning_route_layout/20261112/summary.json`
