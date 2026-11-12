# Complete-operation simulation entry

Use Gazebo Fortress with the existing full S20, camera, suction panel, ULD meshes
and controller. This is the integration baseline environment; sim success does
not establish hardware contact/uncertainty performance. Sim vacuum uses measured
planning attachment and a separate Gazebo kinematic-follow physics adapter.

Build with `scripts/production/build.sh --evidence <fresh-build-folder>` and use
only `scripts/production/run.sh`; `install/` and the legacy rehearsal shell wrapper
are not this entry. Check installed hashes with `check_install.py`.

Select explicitly:

```bash
scripts/production/run.sh ros2 launch luggage_gazebo sim_world.launch.py \
  profile_config:=/home/adamliao/work/elfin_humble_ws/install-production/luggage_gazebo/share/luggage_gazebo/config/joint_operation.profile.yaml \
  operation_run_dir:=/absolute/fresh/run \
  semantic_pydeps_dir:=/absolute/frozen/semantic/dependencies
```

Set ROS_DOMAIN_ID=7 before launching. Check exclusive sim ownership and follow
`.cursor/rules/sim-lifecycle.mdc`; own the launch process group and tear it down
with `scripts/stop_sim.sh --owned-pgid <pgid> --pidfile <pidfile>` after each run.
The frozen dependency prefix requires a DEPLOYED.json inventory; the legacy
self-referential pydeps-frozen symlink is not a valid dependency installation.

The grouped profile selects `planning_pipeline=joint_operation`. It starts one
O mapper and no G mapper, observed depth-to-optical-rays adapter, dynamic suction
support export, common ComputePlacement, prepare, native endpoint/connection/
certification services, full-model MoveIt and bound six-leg execution. Old
SelectPlacePath and the automatic startup FJT hold are disabled. All model readers
consume identical expanded URDF/SRDF bytes; launch.json records model and map-owner
identity. The old frame_runtime switch belongs to the legacy peer graph and must
remain false here; complete-operation typed identity is bound independently.

Launch performs no authorized cycle. Readiness-only client:

```bash
scripts/production/run.sh ros2 run luggage_gazebo joint_operation_sim_driver.py \
  --run-dir /absolute/fresh/run --evidence /absolute/fresh/run/readiness
```

Explicit simulation operator start:

```bash
scripts/production/run.sh ros2 run luggage_gazebo joint_operation_sim_driver.py \
  --start --run-dir /absolute/fresh/run --evidence /absolute/fresh/run/driver
```

The client observes ULD at configured named views through collision-checked MoveIt,
returns to pickup view, spawns physics cargo, observes measured detection/domain,
requests an empty-slot complete plan, and executes that exact issued plan. Spawned
box truth is not used by perception or placement. Default portal view is a
synthetic site pose, not proven sufficient O coverage; partial coverage must refuse
and requires measured view redesign. Driver records the first failure and original
RPC/action messages; prepare/runtime retain full native scene/collision inputs.
Capture bounded raw sensor/TF/joint data during integration for replay.

`planning_pipeline=legacy_segments` retains the old segmented services/profile.
`start_placement_prior`, `volume_evidence_source` or `frame_runtime` alone never
select complete-operation planning. New mode is the explicit integration entry;
whole-cycle success, default production promotion and real baseline remain open.

The owned runner captures a bounded sensor snapshot and tears down its process
group automatically. Readiness rehearsal (add `--start` for one explicit cycle):

```bash
scripts/production/run.sh /usr/bin/python3 scripts/production/simulation.py \
  --evidence /absolute/fresh/rehearsal \
  --semantic-deps /absolute/frozen/semantic/dependencies
```

`runtime/launch.json` reports whether the existing reviewed atlas matches the
actual full simulation model. A mismatch remains an explicit online-IK fallback;
the runtime never reissues an atlas receipt to hide a model change.
