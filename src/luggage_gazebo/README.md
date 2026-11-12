# Gazebo scene geometry

`sim_world.launch.py` spawns the pickup platform using the selected
`scene_tf_config` dimensions. `luggage_gazebo.platform_model.platform_sdf`
copies the stock SDF and sets both visual and collision box sizes and their
bottom-centred poses before passing XML to `ros_gz_sim create -string`.
The stock model is a template, not an independent source of platform height.
Platform-top TF and pickup box spawning use the same scene configuration.

This matters because pickup boxes are static until release: a box spawned
above a shorter stock platform will not settle onto it. Measured top-minus-
support height then includes the air gap. Do not compensate by feeding scene
height or reference box dimensions into perception.

Behavioural regression: `test/test_platform_model.py` checks visual/collision
top agreement with platform TF at two sizes, invalid dimensions, and the actual
launch action's generated model arguments. Live depth acquisition is still
required to verify measured geometry after changes to simulation assets.

The prior rehearsal starts its launch in a private session and tears down with
`scripts/stop_sim.sh --owned-pgid <launch-pid>`. This mode never searches for a
replacement launch or uses a foreign pidfile. Keep the one-simulation rule;
private teardown scope does not authorize concurrent worlds.

The explicit `joint_operation.profile.yaml` selects observed, retained whole-chain
planning. `operation_source_refresh_attempts` bounds full suffix recertifications
after a source-change refusal before controller submission; every retry retains
the original action deadline and all O/static-model collision checks. The profile
uses three attempts after diagnostic20 measured both sensors converging just
after the second capture. Other launches retain the two-attempt default.

For source-change diagnosis, `joint_operation_sim_driver.py --failure-hold-s 25`
retains ten seconds of mapper input clouds, TF, joints and O receipts, then records
a stationary post-refusal window. It only triggers on terminal
source or native-admission refusal, performs no motion or map writes, and bounds RAM to
96MiB and stored CDR blocks to256MiB. `observer_failure/index.jsonl` locates each
independently gzip-compressed message in `cdr.gzblocks`; `summary.json` reports
capture gaps. This instrumented timing is diagnostic, not a latency benchmark.

`--prepick-replans 1` explicitly permits one new complete plan after a terminal,
empty-tool source/native refusal. The driver obtains fresh detection/contact and
O, uses the actual current joints, and requires the same observed instance. It
never retries a carrying or uncertain action and never changes a sealed grasp.
Every original plan, result and preparation trace remains preserved.
