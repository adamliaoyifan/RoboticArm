# Production build and runtime boundary

Build from ROS Humble with `scripts/production/build.sh`. It uses only
`build-production/`, `install-production/` and `log-production/`; old prefixes are
not modified. An existing evidence directory refuses overwrite; later rebuilds
use `--evidence docs/status/evidence/<task>/<new-run>`.

Use `scripts/production/run.sh COMMAND [ARGS...]` for all production commands.
It clears inherited Python/library/workspace overlays, sources ROS Humble and
only the named production local setup. It does not authorize robot movement.

Verify installation with:

```bash
scripts/production/run.sh python3 scripts/production/check_install.py \
  --output docs/status/evidence/<task>/<new-result>/install.json
scripts/production/run.sh ros2 launch luggage_planning hardware_pick.launch.py --show-args
scripts/production/run.sh ros2 launch luggage_planning joint_operation_prepare.launch.py --show-args
```

The checker imports actual modules, verifies resolved origins and source bytes,
checks executable installation and package resources. A passing installation
check is not a map/model/trajectory or whole-cycle qualification. Current default
flow convergence is tracked in
`docs/plans/PRODUCTION_PLACEMENT_CONVERGENCE_G3_20261112.md`.

Planning module ownership and actual route entries are indexed in
`src/luggage_planning/README.md`. After a successful stable-source build, the
builder removes only the obsolete Python modules explicitly mapped in
`src/luggage_planning/config/python_module_layout.json`. It first requires every
replacement source and installed file to exist. The checker imports all relocated
modules and requires their old paths to be unimportable; stale installed files
cannot silently preserve an obsolete route implementation.
