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
`ObservationStore` publishes blocks and identity. `OccupancyGrid` is the only
probability writer. Rays use real endpoints; there is no Euclidean maxrange
truncation. Frame hit/free sets are deduplicated, with hit winning. Rejected
points generate neither hit nor free evidence.

`discretize_endpoints` defaults to false: every accepted endpoint casts a free
ray. Setting it true keeps the first original endpoint in each OcTree cell for
the free ray, so that evidence can change with input order. Occupied cells are
still collected from every original endpoint. The default
`config/observer.yaml` bytes stay unchanged. Writing the field into a loaded
file changes the configuration digest.

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

## Opt-in update profiling (performance diagnosis)

`ObservationStore::integrate(frame, ProfileMode::Stages)` records disjoint lock,
validation, fingerprint, dedup, Z filter, transform, ray preparation, fusion and
publication spans. `Detailed` additionally times per-ray generation/filtering and
per-cell tree/block work. Default `Off` has no diagnostic clock calls/counters;
legacy receipt timing fields remain. Per-key domain, hash-insert, hit-erase, and
release clocks are not provided: `split_key_timers` is `not_provided` and those
fields are omitted, not written as zero. Set release still happens before the
receipt total is closed. These modes are per-call measurement inputs, not mutable
map settings. No new sensor filtering, ray sampling or clipping is introduced.
ROS messages and production node retain default-off behavior; this first
profiling interface is native/offline, not a claim to measure ROS TF lookup or
middleware costs.

Receipts expose `profile` via `receipt_to_yaml`. `stage_visited` means a span was
entered, not that an exceptional path completed. Nested spans must not be added
to their parents. `rays_with_active_keys` is based on visited grid cells, not an
analytic segment/hull test. `copied_cells` counts entries actually copied from
old immutable blocks. Array/map object overhead is not a byte-memory measurement.

`Snapshot::surface(&profile)` reports actual cache-hit state, block-entry scan
count, lock/build/total time. `scanned_cells` counts stored block entries visited
by that rebuild. `domain_init_*` records whether this call executed the one-time
column active-count walk, how many geometry cells that walk visited, and its
time. A null profile is Off: domain init is not collected, and serialization
says `not_collected` instead of emitting zeros. The first surface total includes
domain initialization when that call performs it. A cache hit reports both scans
as zero. Statistics belong to that query and do not change the snapshot or any
older snapshot. A surface query after a changed map revision may rebuild; the
second call on the same snapshot must hit the cache. Column active counts are
immutable after publication and are computed once per domain.

Build `observer_profile_cli` with the normal package or the ROS-free CMake build.
It reads one lossless frame file at a time. Manifest paths are relative to the
manifest file; frame schema is the same as observer_cli integration input:

```yaml
case_id: unique-case-window-id
mode: detailed  # off / stages / detailed; frozen for this process
surface: true
state_digest: false  # explicit checkpoint hash; only for semantic diagnostics
frames:
  - path: frame_000.yaml
    sha256: actual-sha256-of-exact-frame-file-bytes
    warmup: true
    expect_status: INTEGRATED
```

```bash
observer_profile_cli --geometry geometry.yaml --config observer.yaml --manifest window.yaml --out window_samples.yaml
python3 scripts/g3/occ_observer_profile_report.py --samples window_samples.yaml --out fresh_report_directory
```

The runner appends/flushed YAML documents and refuses to overwrite existing
output. Rows contain numeric timings/counts, original input SHA, output identity,
mode, status, process CPU and peak RSS (not current RSS). Load/parse, checkpoint
hash and external query timings are separate; JSON/YAML serialization and file
IO are outside the integrate stopwatch. Invalid inputs are preserved as named
failure rows and exit3. Exit0 means collection completed, not product acceptance.
The report writes raw_samples.jsonl, WORKLOAD_COUNTS.csv, STAGE_TIMINGS.csv and
summary.json; it returns4 for <30 scored rows per window and3 for invalid records.
It does not pool independent windows. Cache-hit and build samples are classified
by actual cache_hit, not by request position. Unprofiled queries have unknown cache
classification. Detailed timings are perturbed diagnostics until overhead is
measured; use matched off-mode runs for latency decisions.

Frame-local bitmap deduplication:

`OccupancyGrid` owns two-bit classification scratch (unseen/inactive/active).
Each `collect` clears it, including after a failed frame or domain change. Keys
outside the integer grid bounds are rejected before indexing; inactive chamfer
cells inside those bounds are remembered. All original DDA rays and endpoint
admission rules remain unchanged. First active visits append to a unique free
vector; every admitted endpoint still enters the occupied KeySet. Global hit
precedence filters that vector before each key receives one probability update.
Diagnostic active/inactive/duplicate counts retain their old meanings.

Scratch defaults to a 64 MiB limit, configurable through the `OccupancyGrid`
constructor's `bitmap_limit_bytes` argument (not a new ROS/YAML setting).
Checked dimension multiplication and allocation report
`FRAME_BITMAP_SIZE_OVERFLOW`, `FRAME_BITMAP_LIMIT_EXCEEDED` or
`FRAME_BITMAP_ALLOCATION_FAILED`; there is no automatic resolution reduction.
At 149x197x148 the two-bit scratch is about 1.09 MB. The unique free vector,
occupied set and tree are additional memory, not covered by this scratch cap.
The vector stores OcTreeKey values, not uint32 linear IDs.

The caller owns `OccupancyScan`. Failure may leave a partial prefix, but
`apply` rejects it with `INCOMPLETE_OCCUPANCY_SCAN`; the next `collect` overwrites
both output and scratch. Store applies only after collect succeeds, under its
existing single writer lock. This change adds no threads, endpoint merging or
custom DDA and makes no performance-gate claim.

Profiling implementation checks:

```bash
ctest --test-dir /tmp/elfin-observer-perf-build --output-on-failure
OBSERVER_PROFILE_CLI=/tmp/elfin-observer-perf-build/observer_profile_cli python3 -m pytest -q src/uld_occ_observer/test/test_profile_cli.py src/uld_occ_observer/test/test_profile_report.py
```

Formal workload matrices, overhead qualification and evidence obligations are in
`docs/plans/OCC_OCTOMAP_PERF_DIAG_G1_20261026.md`; Cursor Grok executes them after
receiving the exact implementation revision. No clipping optimization is included.
