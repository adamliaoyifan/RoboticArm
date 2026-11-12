# Planning package migration verification

Use the frozen `baseline.json` mapping and test list. `snapshot/working` and
`snapshot/head` retain the exact pre-migration inputs, including already dirty
changes. `additional_inputs.json` separately freezes the machine frame inventory.
`preservation.json` proves expected working-byte transformations;
`staged_preservation.json` proves only transformed HEAD content was staged for
previously tracked callers. The production checker has a separate, tested
installation extension. Old and new resource anchors are explicitly audited.

The migration script is a one-shot operation against frozen paths, not a command
to re-run on the already migrated tree. Do not run snapshot/migrate/stage again.
Inspect its transform and receipts instead. Frozen inputs are committed losslessly
in `snapshot.zip`; extract it in this evidence directory to restore `snapshot/`
on a fresh checkout. Expanded local copies are retained. Historical evidence imports keep
historical names; archive replay must pin its original revision or explicitly
apply the recorded module map. It must not silently load a stale installed module.

Baseline and post-migration unit workload (fresh output names required):

```bash
scripts/production/run.sh env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -c 'import sys,json,pytest;from pathlib import Path;sys.path.insert(0,"src/luggage_perception/test");p=Path("docs/status/evidence/planning_route_layout/20261112");files=json.loads((p/"baseline.json").read_text())["tests"];raise SystemExit(pytest.main(["-q",*files,"src/luggage_planning/test/test_module_layout_paths.py","scripts/production/test_module_layout.py"]))'
```

Build/installation:

```bash
scripts/production/build.sh --evidence docs/status/evidence/planning_route_layout/20261112/build_replay
scripts/production/run.sh /usr/bin/python3 scripts/production/check_install.py --output docs/status/evidence/planning_route_layout/20261112/installed_replay.json
```

Native commands, executable paths, opt-in environment, expected case counts and
JUnit/log outputs are frozen in `native_runs.json`. `verify_native.py` generated
these commands sequentially in isolated domains173/174/175. Change its evidence
root to a fresh sibling before replay; fixture paths refuse overwrite. All model,
scene, CDR boundaries, parameters and executable hashes live in native manifests.
No Gazebo, measured sensor campaign or actual controller motion was launched.
The additional hardware `--show-args` failure is retained; it is not a pass.
