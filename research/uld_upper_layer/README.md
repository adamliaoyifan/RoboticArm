# Independent upper-layer placement study

This is an offline prototype for upward suction-panel carry, transfer onto a
filled support surface under a ULD ceiling, withdrawal, and rear-face push.
It has no production imports of downward-pick gates or atlas queries and
keeps planning offline; the explicit wiring rehearsal sends a robot hold command. Its geometry comes from the source worktree.

The current design is [ULD upper-layer V1](../../docs/plans/ULD_UPPER_LAYER_V1.md).
Historical results remain in Git history and the primary workspace; no evidence
is retained in this worktree. This prototype is not the V1 implementation.

Run in a shell with ROS Humble Python dependencies available:

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s research/uld_upper_layer/tests -v
python3 research/uld_upper_layer/run.py --repo . --output /absolute/new/evidence-directory
```

`assets.py` expands source xacro and maps source meshes explicitly.
`geometry.py` owns conservative oriented-box collision primitives.
`action.py` generates contact-mode states and checks fixed-tool geometry.
`run.py` captures the fixed 864-case matrix, bounded stage IK and diagrams.

The historical G1 scope and thresholds are recorded in commit `45a50252`. A geometry candidate means only
that the modeled fixed end-effector passes sampled geometric checks. Stage
IK omits full-arm collision checks; a set of IK solutions is not a connected
path. Pickup/inversion outside the container, transition rotation during
reposition, unknown actual COM, suction peeling, support compliance, friction,
force control, sensor occlusion and live performance remain not_evaluated.

The support is a declared synthetic prism, not measured occupancy. Catalog
dimensions are legal experiment inputs only and must never replace live
perception in a later production action. The eventual caller must establish
a measured upper slot and action-specific feasibility; generic BIN_FULL is
not evidence that such a slot exists.

## Aligned dense lower-box scene

The fixture uses the original ceiling, opening, chamfered ULD mesh and scene
transforms. Twenty-four **dynamic** cuboids form four aligned layers. The top
is at container Z=1.73 m; 0.28 m remains to the ceiling. A blue 0.55 x 0.40 x
0.25 m carryon is on the normal pickup platform. Lower boxes are synthetic
fixture geometry, not detected real luggage. They fill about 91% of the
canonical hull volume below the support plane. There is no elevated support shelf or joint locking them together.
A declared planar contact at the canonical ULD floor (Z=0.53 m) sits above the
slightly uneven decimated floor mesh; the original ULD mesh remains intact.
Source mesh rails/jambs require positive Y <=0.920 m and X between
-0.730 and 0.720 m. An exact initial box/triangle check rejects mesh
intersections; semantic hull containment alone misses those protrusions. All contacts retain collision geometry.
The declared research physics step is 0.001 s; contact tests at other step
sizes need separate validation.

Only 0.03 m remains above the inserted test box. The source panel alone extends
about 0.286 m behind its contact face (0.326 m for the whole fixed assembly),
so the usual downward-panel placement cannot enter that slot. Upward carry,
handoff, withdrawal and rear push still require a new complete motion plan;
this scene does not execute those motions or prove full-arm reachability.
The normal observe/pick posture outside the ULD may still point downward.
This aligned fixture is the requested first experiment; irregular luggage
support remains part of the wider V1 goal.

Build the small isolated overlay once from this worktree (do not build into
or overwrite the shared primary install):

```bash
source /opt/ros/humble/setup.bash
source /home/adamliao/work/elfin_humble_ws/install/setup.bash
colcon --log-base /tmp/uld-upper-build/log build \
  --base-paths src/luggage_description src/luggage_gazebo \
  --build-base /tmp/uld-upper-build/build \
  --install-base /tmp/elfin_uld_upper_layer_g1/install --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF
```

Start the scene, S20 controllers and MoveIt from a terminal:

```bash
cd /tmp/elfin_uld_upper_layer_g1
bash research/uld_upper_layer/launch_scene.sh
# Or: bash research/uld_upper_layer/launch_scene.sh --headless
```

The script checks sim exclusivity, generates `/tmp/uld_upper_layer_scene_v1_aligned` if
absent, and stops only its own process group on Ctrl+C. It sends no automatic
pick/place commands. `ULD_UPPER_SCENE` overrides the generated scene directory;
use a new/empty directory when regenerating. `ULD_UPPER_INSTALL` overrides
the isolated install. Camera publishing comes from the existing simulator;
perception algorithms and production packing/motion executors stay disabled
for this scene-only launch.

Geometry and gravity checks (evidence must be outside this worktree):

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s research/uld_upper_layer/tests -v
python3 research/uld_upper_layer/scene.py --output /tmp/another_new_upper_scene
python3 research/uld_upper_layer/check_scene.py \
  --scene-dir /tmp/another_new_upper_scene \
  --evidence /home/adamliao/work/elfin_humble_ws/docs/status/evidence/uld_upper_layer_scene/NEW_RUN
```

`overlay.sdf` contains the fixture and payload; `sim_world.launch.py` supplies
the original static environment and controlled robot. `standalone.sdf` also
contains that static environment, for a bounded gravity check without a robot.
`fixture.json` describes inputs and source hashes; it is not online perception.
The standalone check preserves the full short pose trace and stops its own
Gazebo before returning. Its tolerances measure setup settling, not the V1
requirement that supports remain stationary during actual pushing.

Use the standard `<worktree>/install` layout: the existing GPU gate locates
its helper in `<worktree>/scripts`. Lower-box collision geometry is present
in Gazebo; scene-only startup does not inject fixture truth into MoveIt.
Future placement planning must obtain those obstacles from the measured map.


## V1 ideal-input geometry contracts

`upper_config.yaml` is the single default configuration. `planning_types.py`
contains immutable boundary records, `ideal_snapshot.py` is explicitly fixture
sourced, and `support_geometry.py` separates actual overlap area from the static
rigid-box support polygon. `candidates.py` jointly enumerates final placements,
offset grasps, handoff and panel pushing poses. Candidate records explicitly
state that full-arm collision/path/physics have not yet been evaluated.

Run the source-bound geometry experiment (new evidence directory required):

```bash
source /opt/ros/humble/setup.bash
python3 research/uld_upper_layer/feasibility.py \
  --scene-dir /tmp/uld_upper_layer_scene_v1_aligned \
  --evidence /home/adamliao/work/elfin_humble_ws/docs/status/evidence/uld_upper_layer_v1/NEW_GEOMETRY_RUN
```

This does not drive ROS or change simulation objects. The sparse ideal snapshot
represents dynamic lower-cargo occupancy; static hull/platform/robot collision
models remain mandatory additional inputs for whole-arm planning. A reported
geometry candidate is not an executable plan.

## Whole-arm and physical integration (in progress)

The single plan remains `docs/plans/ULD_UPPER_LAYER_V1.md`. All new evidence goes
into the primary workspace, never this worktree. Install native dependencies in
an isolated environment, retaining the ROS Python packages:

```bash
python3 -m venv --system-site-packages /tmp/uld-upper-python
/tmp/uld-upper-python/bin/pip install -r research/uld_upper_layer/research_requirements.txt
source /opt/ros/humble/setup.bash
/tmp/uld-upper-python/bin/python -m unittest discover -s research/uld_upper_layer/tests -q
/tmp/uld-upper-python/bin/python research/uld_upper_layer/motion_feasibility.py \
  --scene-dir /tmp/uld_upper_layer_scene_v1_aligned --capture-only \
  --evidence /home/adamliao/work/elfin_humble_ws/docs/status/evidence/uld_upper_layer_v1/NEW_CAPTURE
```

Capture launches the existing headless controlled robot, observes settled cargo
and actual joints, and tears down its own launch. `--observation <observation.json>`
replays that ideal input offline; add `--plan-paths` for connected stage search.
`--config <yaml>` reads runtime configuration. Geometry proposals may be reused
with `--candidates <candidates.json>` only when snapshot, configuration and
geometry source hashes match; full-arm paths are always recomputed.

The private FCL scene uses the original source robot/tool and physical ULD
meshes, actual platform collision geometry and observed lower cargo. Historical
SRDF `Never` pairs do not exempt sensors from moving-arm checks. Only adjacent
links and fixed assemblies are excluded. Mesh queries use conservative AABB
bounds for distant pairs, then exact FCL checks near the margin. Recorded lower
bounds are not exact minimum-clearance scores. Free joint intervals can be
certified by conservative Lipschitz bounds; Cartesian controller interpolation
is checked at the configured point-travel bound. Watchdog failures preserve the
native query, input, seed and stage trajectory, and never count as solutions.

`controller_io.py` provides the actual FollowJointTrajectory boundary but is not
yet invoked by the feasibility entry. Connected nominal plans, contact/release
calibration and physical pushing are separate evidence gates. No current stage
result establishes complete physical feasibility or real-camera performance.

For the native contact diagnostics, build a separate copy of the official 6.18
Physics system, with only its contact-message conversion augmented:

```bash
python3 research/uld_upper_layer/native/build_observed_physics.py \
  --output /tmp/NEW_observed_physics_build
python3 research/uld_upper_layer/scene.py --output /tmp/NEW_contact_scene \
  --physics-plugin /tmp/NEW_observed_physics_build/build/libuld_upper_observed_physics.so
```

The builder pins the upstream release commit and verifies all source hashes. It
writes a patch/build manifest and never installs into the system. Default Gazebo
contact messages lack normal/depth here; an absent field must remain unverified.
Both observed poses and contacts are compressed JSONL. The source and fixture
are explicit ideal-input diagnostics, not real camera output. Reusing geometry
while changing only search/path time budgets explicitly rebinds configuration
and candidate IDs; every other configuration mismatch rejects cache reuse.


The ideal-vacuum adapter is a separate research plugin. While attached it follows
an observed link-to-box relation at physics steps, matching the declared ideal
vacuum model. Attachment requires the caller to verify actual panel contact.
Detachment sends no box pose commands, changes no collision masks or gravity,
and acknowledges a processed command counter rather than only a queued request.
It does not validate real suction holding force. Build and rehearse explicitly:

```bash
cmake -S research/uld_upper_layer/native -B /tmp/NEW_vacuum_build
cmake --build /tmp/NEW_vacuum_build -j2
python3 research/uld_upper_layer/scene.py --output /tmp/NEW_vacuum_scene \
  --physics-plugin /tmp/NEW_observed_physics_build/build/libuld_upper_observed_physics.so \
  --vacuum-plugin /tmp/NEW_vacuum_build/libuld_upper_vacuum.so
/tmp/uld-upper-python/bin/python research/uld_upper_layer/wiring_rehearsal.py \
  --scene-dir /tmp/NEW_vacuum_scene \
  --evidence /home/adamliao/work/elfin_humble_ws/docs/status/evidence/uld_upper_layer_v1/NEW_WIRING
```

The rehearsal verifies two distinct processed detach acknowledgements, zero
payload pose commands and a real controller hold within 1mrad. This is unscored
wiring evidence, not a pick, release stability or physical push result. The
fixture verifies source/binary hashes before launch and owns stack teardown.
`ompl_range_rad` controls free-motion RRT expansion in runtime YAML. Sampled
push endpoint screening precedes the expensive pick/carry prefix; only identical
snapshot, box, push poses, configuration and ordered IK seeds reuse a negative
screen. A failed sampled screen is not a proof of unreachable geometry.


Whole-action search backtracks sampled pick joint branches after downstream
failure, instead of freezing the first reachable pick posture. Each attempt
starts from the same observed home state and shares the action deadline;
accepted pickup endpoints alone never count as complete plans. Geometry search
flushes candidate/subsearch inputs incrementally and checks its deadline inside
subsearches. The occupancy broadphase proves distant AABB separation before
running the unchanged OBB SAT on near objects.


`motion_bounds.py` derives each body's joint point-travel radii from downstream
source origin translations and actual collision vertices. Attached payload
radii include the TCP-to-box offset. Relative pair bounds cancel shared ancestors;
unknown bodies reject instead of silently receiving static zero motion. These
bounds certify whole intervals without changing the required collision margin.
