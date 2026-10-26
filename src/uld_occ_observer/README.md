# Independent ULD occupancy observer

This package adds an isolated OctoMap observation backend. It does not publish a
production cargo map, attach payloads, choose placements, run lease logic or infer
box instances. Its implementation base is b33dda91. Acceptance is separate from
implementation; see the delegated Cursor task and frozen plan for O0--O9.

## Ownership and data flow

`OpticalDepthPreprocessor` filters read-only optical XYZ in meters using inclusive
`depth_z_min_m` / `depth_z_max_m`. `UldFrameAdapter` applies the acquisition-stamp
rigid transform once to surviving endpoints and the true camera origin.
`UldDomain` restricts evidence to the authoritative container geometry.
`ObservationStore` is the only tree/version writer. Rays use real endpoints;
there is no Euclidean maxrange truncation. Frame hit/free sets are deduplicated,
with hit winning. Rejected points generate neither hit nor free evidence.

Normal updates retain one OcTree. The probability profile is hit=.7, miss=.4,
threshold=.5, clamps=.1192/.971. Unknown is distinct from free. No timed decay or
unseen-interior fill occurs. Only actual accepted free rays lower occupancy.
`observed_revision` advances on accepted nonduplicate observations;
`map_revision` advances only on probability/known-state changes. All-filtered
frames return NO_VALID_POINTS without advancing either revision or freshness.

Grid resolution defaults to 0.01 m. The origin is (-length/2,-width/2,floor_z).
Cells use the legacy nominal-center active rule, including chamfer boundaries.
`configuration.cpp` reads the Python authority's exported descriptor and checks
its artifact digest; it does not invent another geometry hash. Export is offline;
the running node has no mapper, planner, lease or Python geometry dependency.

Snapshots share immutable 8x8x8 blocks. An acquisition copies the block registry,
not the tree. Repeated acquisitions reuse the current snapshot. Surface is lazy
and cached: highest occupied top plus known/active counts; it is not a support
certificate. `SnapshotRepository` owns at most 8 handles across 2 pinned map
revisions. TTL=5 steady-clock seconds, idempotent release, explicit pressure
failure, and ongoing reads retain ownership. TTL and max_age never delete cells.

Checkpoint/restore is explicit, full-probability OcTree serialization plus a
checksummed metadata bundle. Restore validates before replacing state, creates a
new epoch and invalidates old handles. Restored timestamps reject old frames;
old-epoch duplicate tokens are not revived. No .bt maximum-likelihood export is
used. No ordinary observation invokes full-tree serialization or reconstruction.

## Build

Use a dedicated build/install and source only the ROS underlay:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select uld_occ_observer --build-base build_observer --install-base install_observer --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

The CMake contract pins `/opt/ros/humble/share/octomap`, exactly 1.9.8; verify
`ldd install_observer/uld_occ_observer/lib/uld_occ_observer/observer_node` as well.
ROS-free build: `cmake -S src/uld_occ_observer -B /tmp/observer-core -DOBSERVER_WITH_ROS=OFF`
then `cmake --build /tmp/observer-core -j2`.

Export the geometry with the frozen workspace authority:

```bash
PYTHONPATH=src/luggage_description python3 src/uld_occ_observer/scripts/export_geometry.py --scene-config src/luggage_description/config/scene_tf.yaml --out /tmp/observer-geometry.yaml
```

`config/observer.yaml` is the single configuration input for resolution, optical
Z limits and clock domain. It is frozen on startup; changing it starts a new
owner. ROS launch parameters configure transport only. No live parameter callback
mutates dimensions or filter bounds in another layer.

## Standalone ROS

```bash
source install_observer/setup.bash
ROS_DOMAIN_ID=77 ros2 launch uld_occ_observer standalone.launch.py geometry_file:=/tmp/observer-geometry.yaml input_mode:=service
```

The launch starts only the observer. Subscription mode additionally requires
`input_mode:=subscription optical_frame:=<actual-optical-frame> cloud_topic:=<raw-optical-pointcloud>`.
For Gazebo use `use_sim_time:=true`; do not use a wall-time stamp for a simulation
frame. TF lookup uses the acquisition timestamp, with a 100ms bound and no latest
fallback. Raw depth image deprojection belongs upstream. The cloud adapter handles
organized row padding and FLOAT32/FLOAT64 XYZ; unsupported endianness/layout is
explicitly rejected. Configure the exact optical frame; relabeling a non-optical
cloud is invalid.

All endpoints are under `/uld_occ_observer`:

| Endpoint | Contract |
|---|---|
| integrate_observation | Service mode only; raw optical PointCloud2 + same-stamp T + sensor/id/epoch/geometry identity; returns receipt. Subscription and service writes are mutually exclusive. |
| get_snapshot | Acquisition time comes from node ROS clock; caller supplies max_age_ms, allow_stale and optional exact revision/epoch. Returns pinned identity, age and stale flag. |
| read_cells | Handle, half-open integer ROI, cursor, limit (1..65536); x-fastest state/logodds page with original snapshot identity. |
| read_surface | Handle, cursor, limit (1..65536); x-fastest XY columns; has_occupied=false means top_z has no geometric meaning. |
| release_snapshot | Idempotent handle release. |
| reset_observation | expected_epoch and nonempty reset_id; resets only on matching epoch. |
| checkpoint_observation | operation checkpoint/restore, path and expected_epoch; unknown operations return UNSUPPORTED_OPERATION. Existing checkpoint files are not overwritten. |
| receipts | Each submitted observation's result and stage/queue timing. |
| health | 2Hz metadata, revisions, queue/resource counts, capabilities; no full map and no freshness refresh. |

The worker queue holds at most two waiting observations, one processing. Queue
pressure returns QUEUE_FULL instead of replacing frames. Transport receipt time,
TF/adaptation and DDS accounting must be measured by the acceptance harness as
well as core receipt timings. Requests serialize mutations inside the store;
queries operate on immutable snapshots after acquiring ownership.

## Offline replay

`observer_cli --help` describes the executable. It takes `--geometry`, `--config`,
`--commands` and `--out`. Commands are a YAML sequence. Integration fields are
sensor_id, observation_id, optical_frame, clock_domain, geometry_hash, epoch,
stamp_ns, points_m (XYZ rows), and transform (stamp_ns, row-major rotation[9],
translation[3]). The runtime never fills missing identity/TF from guessed values.

Other operations: diagnostics; acquire(now_ns,max_age_ns,allow_stale,
expected_epoch,expected_revision); cells(begin,end,cursor,limit,handle);
surface(handle); release(handle); reset(expected_epoch);
checkpoint(path); restore(path,expected_epoch). Omitted query handle uses the
last acquired handle. Responses retain stage counts/timings and status. Optional
expect_status declares an expected negative case. Exit 0 means all command statuses
matched; exit 3 means mismatch/invalid input. It is a replay primitive, not the
O0--O9 acceptance runner. Cursor supplies independent oracles and the test driver.

## Verification boundary

A build and a minimal command replay demonstrate wiring only. Formal probability,
ray/grid boundary, snapshot concurrency, long-running resources, performance,
failure capture and sensor simulation are delegated for independent validation.
Neither the source code nor a successful replay establishes D555 accuracy,
production packing success or a performance advantage over the existing grid.
