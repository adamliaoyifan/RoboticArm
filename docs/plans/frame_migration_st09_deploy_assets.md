# ST09 deploy-profile asset preparation

## Scope and dependency

This is the parallel `ST09-PD` input to `ST09-I` and `ST09-R`.  It starts at
`5f56ba0d76acff2a6255030a30c5fab2e5b13097` and owns only deploy-profile
configuration, generated reachability assets, their validators, focused launch
selection tests, and evidence.  It must not edit the production route,
certifier, executor, caller adapters, or QP branch.  No Gazebo or hardware
motion is authorized.

The deploy cell is distinct from simulation.  The authoritative scene is
`scene_tf_deploy.yaml`; a hardware launch must never obtain
`scene_tf.yaml[.example]`, simulation robot poses, or the simulation atlas by
an empty/default argument.

## Deliverables

1. **Fail-closed profile selection.** Hardware launch defaults resolve the
   installed deploy scene and `atlas_runtime_deploy.yaml`, use
   `use_sim_time=false`, and expose the resolved absolute paths in the runtime
   manifest.  An explicit path still overrides the named profile.  Missing,
   ambiguous, sim-named, or hash-mismatched assets reject readiness.
2. **Versioned identity manifest.** Add a machine-readable deploy asset
   manifest containing schema version, scene and seven-face geometry hashes,
   expanded hardware URDF hash, SRDF hash, canonical ordered joints,
   `elfin_arm`, `suction_contact_frame`, tool transform identity, atlas and
   graph hashes, payload envelope, units/datum/orientation semantics, and the
   generating command/environment.  Hashes are computed from installed or
   explicitly selected files, never copied from expected launch arguments.
3. **Deploy atlases.** Regenerate the empty-payload atlas and the three frozen
   payload envelopes `0.55x0.40x0.25`, `0.70x0.45x0.28`, and
   `0.80x0.50x0.32` against the exact expanded deploy robot model and scene.
   Metadata uses the concrete-tool v2 contract and `hash_policy: strict`.
4. **Directed reachable graphs.** Persist one graph per atlas with distinct
   q-state nodes, directed edges, branch/family identity, edge kind and cost,
   start/portal/release/exit semantics, and all source artifact hashes.  The
   graph is an offline generated asset consumed by B's graph route planner;
   it must not create a second planning method.
5. **Site poses and observation policy.** Keep `current` as the only executable
   fallback until measured deploy poses are supplied.  Provide a deploy site
   file with explicit calibration status, scene/model identity, joint order,
   acquisition stamp and provenance fields for pickup, return, portal
   observation and second-region observation.  Unmeasured entries are
   `valid:false` and cannot execute.  Validators reject simulation pose files.
6. **Install completeness.** Fresh `colcon` install contains the deploy scene,
   runtime settings, manifest, all atlas/graph pairs and site-pose schema, and
   resolves the same hashes from the install prefix.

## Acceptance matrix

- Default and explicit hardware launch resolution are tested from both source
  and fresh install.  Four cases are required: default deploy success,
  explicit deploy success, sim asset rejection, and missing asset rejection.
- Every one of the four atlas/graph pairs strict-loads with the exact scene,
  expanded URDF, SRDF, tool link and ordered joints.  Flip one byte independently
  in scene, URDF, SRDF, atlas and graph: each of the five mutations must return
  its named identity rejection and expose zero usable seeds/edges.
- Graph validation checks all edge endpoints, preserved direction, finite
  costs, q-state uniqueness, release twins, and at least one certified route
  from each declared start class to every healthy fixture target.  Missing
  coverage is `not_evaluated`, not success.
- Deploy pose validation covers a valid synthetic measured fixture plus stale,
  wrong-scene, wrong-joint-order, sim-provenance and `valid:false` cases.  Only
  the synthetic fixture is allowed to enter offline MoveIt state validity;
  invalid real placeholders remain non-executable.
- Run focused description/planning/bringup tests and a plain fresh build.  Save
  commands, expanded model hashes, asset inventory, mutation results and install
  resolution in `docs/status/evidence/frame_migration/ST09-parallel-PD-<rev>/`.
  The component passes only from a clean commit with all computable checks
  green; absence of measured site poses is reported as a hardware-calibration
  prerequisite and cannot be relabeled as a validated pose.

## Integration handoff

`ST09-I` may merge this component before its final immutable candidate and use
the manifest for deploy mock-graph coverage.  It must not wait for real site
pose measurement to finish the non-scored simulation M3.  `ST09-R` requires
the exact merged deploy assets for deploy-profile startup and identity faults.
Actual hardware motion remains a separate authorization after scored
INTEGRATION; before that motion, all required site-pose entries must be
measured, marked valid, state-validated against the frozen deploy scene, and
refrozen in a new candidate.
