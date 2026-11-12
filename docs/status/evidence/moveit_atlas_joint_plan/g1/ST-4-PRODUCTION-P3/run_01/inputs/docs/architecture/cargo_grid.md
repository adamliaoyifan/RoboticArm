# Cargo grid: one resolution and one map owner

`luggage_description/config/cargo_grid.yaml:resolution_m` is the sole deployment
default (0.01 m). Shared configuration belongs to description because mapper,
packing and planning all depend on it; putting the shared bound in perception
would reverse the planning dependency. `cargo_grid_parameters` is the node/launch
adapter; `cargo_grid_contract` is pure geometry/validation.

OCC uses this resolution on XYZ; `surface_map_2d` projects those exact cells onto
XY. Its occupied height is the upper face of the highest active occupied voxel,
clamped to the ceiling, so quantization may overestimate the top by one cell.
A smaller grid is not improved sensor accuracy. No interpolation fills holes.
Geometry commits use positive footprint-cell area and positive vertical overlap;
unknown columns remain unknown. Unoccupied gaps are not necessarily observed free.

Sim and hardware launch expose `cargo_map_resolution_m` (empty = config). One
resolved value is fed to mapper `resolution` and planner `floor_prior_resolution`.
Both parameters are read-only after declaration. Direct node launches may set
these at startup; an integrated launch must feed the same value. Actual received
surface maps carry and use their owner's resolution, not the floor-prior value.
Changing it requires a fresh map instance/epoch; no live regrid is provided.

The integrated hardware placement launch selects one owner using
`occupancy_backend=observer|grid_g` (default observer). It never creates both.
O receives immutable effective geometry/config bytes from
`luggage_mapping.configuration`; ComputePlacement and complete-operation prepare
use the shared `luggage_mapping.observer` reader and that owner's grid. A legacy
consumer resolution is only a consistency assertion, never a regrid. O input
readiness requires real optical-ray integration; launch alone does not certify
free space. G's separate snapshot adapter remains behind the same placement
surface/volume evidence boundary. This selection does not by itself qualify the
legacy selector or a complete-operation execution client for both backends.

Deployment accepts 0.005 through 0.05 m, finite. Allocation and volume transport
share MAX_VOLUME_CELLS=8,000,000. Counts are validated before buffers are allocated;
an excessive grid is refused rather than silently coarsened. 1cm on the current
1.49x1.97x1.48m hull AABB is 4,344,244 voxels. The prior's separate private-obstacle
count budget is unchanged: admitting a map does not guarantee admission of its
expanded private scene.

OccupancyMapStore.apply_update remains the sole production mutation entry point;
node adapters retain runtime admission, expected snapshot and verification gates.
The mapper owns compact buffers. Projection helpers only read them and return
fresh detached data; snapshot construction reuses one projection for its two
representations. Resolution, dimensions and geometry participate in the existing
content digest; they cannot be changed while a snapshot/route is in use.

The scalar hull kernel remains authoritative. The optimized mask evaluates its
YZ slice and independent closed X faces; any additional caller predicate still
runs on every eligible cell. Surface/source/semantic label projections preserve
legacy ties and inactive-cell handling, and must match scalar behavioural oracles.

Examples (restart to apply):

```bash
ros2 launch luggage_gazebo sim_world.launch.py cargo_map_resolution_m:=0.01
ros2 launch luggage_gazebo sim_world.launch.py cargo_map_resolution_m:=0.05
ros2 launch luggage_planning hardware_pick.launch.py cargo_map_resolution_m:=0.02
```

`ros2 param set /cargo_volume_mapper resolution 0.02` is deliberately refused;
startup `--ros-args -p resolution:=0.02` remains available for a standalone node.
The geometry API can instantiate small coarse grids for offline legacy tests;
its finite-positive and allocation-budget validation is independent of deployment
range validation. No box clearance or Attachment uncertainty changes follow from
a grid-resolution change.

OCC sensor input preparation is private to the Store integration path, after
acquisition-stamped TF into container_link. `cargo_occ_input` owns finite/height
filtering and exact hull hit cropping before downsampling. The public cargo
cloud and pick top estimator are never cropped. The existing floor+5cm policy
is passed as `MapUpdate.minimum_z` and participates in operation identity; the
node does not apply a second filter. Batched geometry uses the normalized hull,
not an independent ROI configuration.

Hit and ray streams are separate: ray downsampling and budget retain the old
uncropped order, whereas only interior hit points become occupied/confirm cargo.
Ray loops skip exterior AABB intervals while retaining original t=k/steps and
active-mask checks. Closed-face/terminal-band depth rules remain unchanged.
`preprocess_counts` and `preprocess_timings` are returned in Store details.
Reading a revision must use `map_revision`; calling full `stats()` also scans
frontiers and is not appropriate in the hot integration path.

OCC query indexes are derived state owned by OccupancyMapStore. Mapper mutation
boundaries only record first-old/final-new evidence; primary buffers, fusion,
ray ordering, shadow restoration, commits and revisions retain their semantics.
The store applies net deltas before publication. Published status_bundle pairs
the immutable map snapshot with counts/frontier membership of that exact identity.
Hot summary reads visit zero voxels and never acquire the update transaction lock.
The b33dda91 vectorized frontier count remains the unindexed mapper fallback;
legacy enumeration retains ix/iy/iz order. Scalar enumeration is retained as an
offline oracle. Index failure preserves the primary fact and publishes
QUERY_INDEX_UNAVAILABLE; an explicit reset can rebuild the derived state.

Read-only block queries (index schema 1)
---------------------------------------

`store.acquire_query_snapshot(expected_identity=None)` opens a five-second
monotonic-wall-time handle to the current published bundle. Expected identity
may be the existing three legs, or all five `(epoch, revision, content_digest,
voxel_digest, index_schema_version)`; CURRENT/None selects the current bundle.
Only two query generations are retained. An old handle expires explicitly when
its epoch/generation is retired, its token expires, or it is closed. Query handle
lifecycle uses a short registry lock independent of the write transaction lock.
The registry publishes its data and the paired legacy snapshot/summary together.

Python errors are `QueryError` with a stable `reason`; successful results are
frozen DTOs with identity, layout, completeness, work counts and elapsed time.
`summary`, `get_cell`, `query_cells`, `query_frontiers`, `count_frontiers`, and
`query_instance` never reference the mutable mapper. Regions are finite
container_link AABBs, half-open, selecting active cells by strictly positive
volume intersection. Pages default to 1024, cap at 4096, and sort by flat index.
Cursors bind query parameters and full identity using an HMAC; retrying the same
cursor returns the same page while its generation remains pinned. Incomplete
pages carry a cursor; empty successful complete pages mean no match.

Sensor instance keys require an explicit `legacy:<epoch>` namespace because the
existing observation protocol supplies no reliable producer session. Geometry
instance membership uses only a real box_id supplied on an actually added
commit, retaining overlapping memberships independently. Missing or ambiguous
identity returns INSTANCE_BINDING_MISSING; ledger positions never become box IDs.
Removed known box IDs remain queryable as empty for that epoch. No binding is
written back into the old ledger or shadow. Reset invalidates prior-epoch keys.

The index owner keeps canonical evidence blocks for incremental hashes. Published
query generations hold compact immutable cell records, block counts, and reverse
membership indexes, sharing unchanged blocks. Canonical evidence includes live
log-odds, locks and sensor shadow without migrating their primary storage. The
new voxel digest also binds publication provenance omitted by the old digest,
so a pinned handle cannot silently switch acquisition/confirmation metadata.
The legacy content_digest remains unchanged. Arrays exposed from index blocks
are backed by immutable bytes. Registry sessions are bounded to 128 and retained
index/history data to a conservatively charged 512MiB. Resource refusal rejects
new query sessions and preserves already admitted old results; physical facts
still commit, and an explicit reset rebuilds failed current derived indexes.
RSS and retained-size estimates must still be measured in acceptance workloads.

The existing legacy known_ratio row exports have a mutable-row alias. New query
generations detach and tuple-freeze those rows in their metadata record; they do
not inherit that legacy alias or change existing public read values/types.
