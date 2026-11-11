# Diagnose a failed OCC place route

Use the copied Humble overlay with system Python 3.10. Leave the original
evaluation directories and install intact. For archived evidence:

```bash
python3 scripts/occ_motion_diagnostics.py audit RUN --out NEW_AUDIT_DIR
python3 scripts/occ_motion_diagnostics.py replay RUN --out NEW_REPLAY_DIR \
  --scene-tf SCENE_YAML --robot-poses POSES_YAML --observe-pose POSE_NAME
python3 scripts/occ_motion_diagnostics.py compare PAIRED_SAMPLES.json --out NEW_COMPARISON_DIR
```

Audit hashes the exact Select wire pair and identifies observed candidate
failures. It reports missing evidence separately from planner outcomes.
The native replay uses the existing isolated MoveIt runner, requires idle
domain 8 and supports the single-Select, atlas-unavailable profile. Do not
feed it a failure-time scene summary or a scene from another request. Native
replay does not execute trajectories. A diagnosed collision rejects that
state, and budget exhaustion leaves reachability unresolved.

For one non-scored capture, set `ELFIN_SELECT_REPLAY_DIR` to
`RUN/dumps/select_replay` in the motion OCC driver's environment. Its typed
recorder saves the wire request and response under
`RUN/dumps/typed_service_boundaries`. The opt-in scene read saves the
lossless PlanningScene and binds it to the exact request bytes. The snapshot
is from the client before transmission; it is not server acknowledgement.
Set `ST09_PORTAL_CAPTURE_DIR` in the selector's environment to save the
first four carrying joint calls and first four insert calls, including raw
responses and missing-artifact flags. Use a fresh output directory each run.

For paired comparisons, supply a JSON array in acquisition order with
`case`, `arm` (baseline or candidate), `input_identity`,
`capture_complete`, `success`, `certified` and `wall_s`. The input
identity must bind the scene, robot/model configuration, workload and
execution environment. Set `certified` only after independent validation.
The gate requires ten alternating repetitions per arm and at least five
certified successes in each arm for a latency comparison.

Adapter timing includes `joint_retime` and
`joint_interpolation_gate` separately, including rejected joint edges. The
interpolation gate duration includes sampling and collision service time.
Do not sum cumulative per-candidate rows or treat the sum of the two fields
plus their combined total as three independent costs.
