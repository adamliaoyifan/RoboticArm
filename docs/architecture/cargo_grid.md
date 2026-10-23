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
