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

For a simulation diagnostic that needs exact suction-input replay, add
`--capture-suction-inputs` to `simulation.py`. This enables the detector's existing
bounded input capture under that run's `suction_inputs/`; it records the actual
consumed acquisition transform instead of reconstructing it from a truncated TF
window. It adds no observation or motion. At teardown the runner audits selected
instance/generation/stamp and original depth bytes. Missing or mismatched selected
inputs make the diagnostic capture incomplete; an operation's own Action result
remains separately retained. Startup without a selected pickup is
`NOT_EVALUATED`. This optional capture has overhead and is not a timing benchmark.
Raw selected windows are recorded for every explicitly started observed cycle,
including single-box runs. The audit also reads actual selected-domain receipts:
a missing window is incomplete evidence even if no window directory exists.

For an explicit simulated sensor-loss experiment, add `--depth-dropout-fixture
CONFIG.json` together with `--start --capture-suction-inputs`. The runner freezes
the exact configuration, checks its model hash and scene revision against the
actual launch, and binds only the Gazebo depth adapter. Configuration schema
`simulation_depth_dropout_v1` requires model/scene identities, reference
observation and input SHA256, world/optical frames, unit `reference_axis_world`,
`pitch_pixels`, `square_side_pixels` and `active_axis_cone_rad`. These are experiment
parameters, with no production defaults or hardware calibration claim.

Within that optical-axis cone the adapter changes only periodic pixel squares
to unknown depth0. Outside it, actual rendered depth passes unchanged; optical
roll alone cannot clear the cone. Exact stamped TF is required. Its bounded
nonblocking TF wait forwards the original frame on timeout/overflow and records
why no fault was injected. `depth_dropout/` owns a bounded original/output ring,
pins exact frames selected by suction capture, and flushes its summary on owned
teardown. Missing/truncated selected evidence makes the diagnostic incomplete;
it does not rewrite operation results. With the flag absent the normal callback
has no fixture TF subscription, timer or queue. This capture is not a latency
benchmark. The diagnostic caller includes any explicitly enabled recovery time
budget once per box; producer planning and recovery deadlines remain unchanged.
