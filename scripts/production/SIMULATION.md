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
group automatically.

The controller-manager startup watchdog requires an actual typed
`ListControllers` response. Global service names also include clients, so names
alone cannot qualify readiness. Discovery and the single read-only request share
the existing total timeout. The runner saves its request/response and outcome
under `startup/controller_manager`; missing replies remain named startup failures.
This gate does not certify active controllers or repair a lost vendor-plugin
URDF response. The driver separately requires active controllers and sensors.

Readiness rehearsal (add `--start` for one explicit cycle):

```bash
scripts/production/run.sh /usr/bin/python3 scripts/production/simulation.py \
  --evidence /absolute/fresh/rehearsal \
  --semantic-deps /absolute/frozen/semantic/dependencies \
  --build-receipt /absolute/passing/build/manifest.json
```

The complete-operation entry defaults to `atlas_runtime_gazebo_operation.yaml`,
whose unchanged empty-load seed arrays passed all12,098 FK/native state checks
against the exact full Gazebo model. The hardware model retains its separate
receipt. `atlas_runtime_config` explicitly overrides this selection.
`runtime/launch.json` reports the actual model/scene/artifact binding. Any mismatch
remains an explicit online-IK fallback; runtime never reissues a receipt. Atlas
qualification alone does not establish live consumption or a successful cycle.

The client binds preparation to a real current O snapshot lease. GetValidity
metadata is transition-time evidence, not the latest map revision. Settled view
RGB/depth/intrinsics/joints and the initial full PlanningScene are saved at their
stage boundaries so later motion cannot evict them from the failure sensor ring.

The same owned runner can execute the observed floor/floor/stack workload:

```bash
scripts/production/run.sh /usr/bin/python3 scripts/production/simulation.py \
  --evidence /absolute/fresh/three-box \
  --semantic-deps /absolute/frozen/semantic/dependencies \
  --build-receipt /absolute/passing/build/manifest.json \
  --start --three-box --capture-physics
```

Three-box uses the installed production driver, one initially-empty O epoch,
the existing task budgets and a bounded 25s failure capture. It does not select
another planner. `--prepick-replans` explicitly enables up to two existing
pre-pick retries; default is zero. Readiness never forwards motion/cycle flags.
`--capture-physics` enables a separate eval-only Gazebo pose/contact recorder;
its examples retain original acquisition stamps and execution phases. The
recorder starts after world readiness and before the first spawn, so an early
missing world topic cannot consume its first measurement timeout. The original
failed startup capture remains evidence; its summary is never rewritten.
The reference is never a sensor/geometry input. Report functional execution and
sampled physical model coverage separately; capture absence is not proof of no
physical contact. Recorder failures stay visible in its reference summary.

Sensor-window flush uses a typed Snapshot RPC from the runner's existing ROS
discovery node. A missing or negative response stays an evidence failure even
if a partial bag exists. Startup and snapshot teardown retain separate outcomes.
Unavailable acquisition handoffs dump the requested key, live-joint availability,
bounded history keys and the last explicit invalidation cause before refusal.
