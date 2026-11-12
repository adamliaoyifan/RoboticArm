# Planning routes and module ownership

The Python algorithms live under `luggage_planning/`, grouped by responsibility.
These folders are not four interchangeable execution modes: prior and atlas are
inputs to planning; whole-operation and segmented planning have separate entries.

```text
luggage_planning/
├── joint_operation/    complete pick/carry/place preparation and bound execution
├── placement_prior/    read-only candidate screening and snapshot evidence
├── routing/            segmented route production and connection strategies
├── atlas/              sampling, qualified proposals, graph and seed helpers
└── *.py                shared attachment, geometry, observation and motion ports
```

| Responsibility | ROS / launch entry | Python entry | Main modules |
|---|---|---|---|
| Whole operation | `joint_operation_prepare.launch.py`, `/joint_operation/prepare` | [joint_operation/prepare_node.py](luggage_planning/joint_operation/prepare_node.py) | `preparation`, `candidates`, `planner`, `aperture`, `certification`, `revalidation` |
| Bound execution | `motion_planner_node.py`, prepared-operation consumer | [joint_operation/runtime.py](luggage_planning/joint_operation/runtime.py) | `runtime_ports`, `execution`, `attachment`, `actual_contact` |
| Placement prior | `place_prior_node.py`, `/placement_prior/prepare` | [placement_prior/worker.py](luggage_planning/placement_prior/worker.py) | `core`, `snapshot`, `pick`, `local_paths`, `ranking`, `adoption` |
| Segmented route | `trajectory_selector_node.py`, `/trajectory_selector/select_place_path_v2` | [routing/factory.py](luggage_planning/routing/factory.py) | `producer`, `direct_first`, `lattice`, `corridor_supergraph` |
| Atlas | builder tools and planning consumers; no independent execution service | [atlas/proposals.py](luggage_planning/atlas/proposals.py), [atlas/runtime.py](luggage_planning/atlas/runtime.py) | `model`, `wavefront`, `graph`, `seed_chain`, `frame_contracts` |

## Whole-operation route

```text
joint_operation_prepare_node.py
  → joint_operation.prepare_node
  → preparation / candidates
  → planner → aperture / ik / connection
  → certification / issuance
  → retained complete plan
motion_planner_node.py → joint_operation.runtime → execution / runtime_ports
```

The aperture generator belongs to this route and retains its ordered waypoints
inside the carry leg. ComputePlacement provides geometric slots. Actual attachment
geometry, current occupancy, configured ULD and native complete-model collision
checks decide whether a candidate has a valid complete trajectory.

The standalone native connector derives OMPL's segment fraction from its own
joint-check step and the loaded group's extent. Search and returned-path checks
therefore use compatible resolution even when no external OMPL YAML is supplied.
Prepare evidence stores lossless gzip CDR files; `boundaries.jsonl` records raw
and stored sizes/hashes. Decompress before deserialization; the budget counts
stored bytes and still refuses when full.

## Selecting the route

`luggage_gazebo` selects the route with `planning_pipeline`: `joint_operation`
uses the complete-operation flow; `legacy_segments` uses the segmented flow.
The grouped [joint-operation profile](../luggage_gazebo/config/joint_operation.profile.yaml)
selects `joint_operation`. The bare simulation launch still defaults to
`legacy_segments`; this directory migration does not change that default.
`frame_runtime` is a separate compatibility setting, not the route selector.
See [production orchestration](../../docs/architecture/production_orchestration.md)
for authorization and map-owner rules.

## Shared interfaces and installation

Modules used across routes stay outside these folders until their ownership can
be established independently. In particular, placement-prior `contracts` also
contains existing shared geometry types; this move does not redefine that type
ownership. Native C++ backends in `src/` serve multiple routes. ROS wrappers stay
under `scripts/`; launch, config and test paths retain their existing names.

Repository imports use explicit new paths, for example:

```python
from luggage_planning.joint_operation.planner import JointOperationPlanner
from luggage_planning.routing.factory import build_production_route_producer
```

The complete old-to-new Python path map is
[python_module_layout.json](config/python_module_layout.json). Old internal
Python paths are retired; there is no duplicate implementation or import hook.
The former `placement_prior.py` name now denotes the `placement_prior/` package;
its former functions live explicitly in `placement_prior.core`.
ROS service types/names, executable names and launch arguments are preserved.
Build with `scripts/production/build.sh`: it removes only mapped obsolete Python
modules after the replacement install succeeds. The installed checker imports
every moved module, compares its source bytes and rejects old import paths.
