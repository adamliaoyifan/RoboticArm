# Container Geometry

Container usable space is defined by one ROS-free geometry authority:
`luggage_description.container_geometry`.

The current non-box geometry is the convex seven-face inner hull from
`scene_tf.yaml`: an axis-aligned cuboid with the lower `positive_y` triangular
prism removed. A missing chamfer describes a cuboid fallback. Arbitrary meshes,
non-convex hulls, and platform tilt are out of scope for this contract.

Hard rules:

- Online algorithms, offline replay, reachability, utilization metrics, and
  evidence denominators must use the normalized descriptor and
  `geometry_hash` from the kernel.
- AABB storage is allowed only for indexing, broad phase, and array allocation.
  Any reported usable, free, reachable, blocked, covered, or placeable volume
  must be clipped to the hull first.
- Placement and motion feasibility use the complete oriented payload box or
  swept box. Center-point checks are insufficient.
- Hull filtering happens before ranking, retention, `top_n`, or
  first-feasible selection.
- Missing, invalid, unsupported, or mismatched geometry fails closed. Silent
  fallback from seven-face geometry to legacy `inner_size` is forbidden.
- Eval or Gazebo truth may establish reference numerators, but it must not feed
  online geometry or usable-space denominators through a second API.

The kernel stays importable under plain Python with no ROS, TF, Gazebo, or
message imports. ROS nodes and adapters translate descriptors at the boundary.

## Identity propagation

Every artifact that describes the interior of the container names the hull it
was built for, so two independently-loaded scene configs cannot be joined
silently:

- `cargo_volume_mapper` stamps `geometry_hash` on `surface_map_2d`, on the
  commit ledger, and on `GetCargoMapStats`.
- `placement_planner` compares an incoming cargo map against its own kernel
  hash and rejects the map on mismatch or absence. It does not answer from the
  floor prior while a map it refused exists; that would be the silent fallback
  this document forbids.
- `ComputePlacement` carries `geometry_hash` in both directions. A non-empty
  request hash that differs from the planner hull fails closed.
- The floor prior is legal when no map has been published, but it is stamped
  with the planner hash and flagged as `floor_prior` in the result dump.

## Placement failure taxonomy

`ComputePlacement` returns a `reason_code` from a fixed set. Capacity claims
and planning infeasibility are different answers and must stay separable in
evidence:

| Code | Meaning |
|---|---|
| `INVALID_BOX_SIZE` | A requested dimension is not finite and positive. |
| `BOX_EXCEEDS_CONTAINER` | No candidate could be enumerated; the box exceeds the container at every allowed yaw. |
| `BIN_FULL` | Candidates were enumerated and none was capacity-feasible. |
| `PLACE_CANDIDATE_EXHAUSTED` | At least one candidate was capacity-feasible and was rejected by a policy or observation gate. |
| `CARGO_MAP_GEOMETRY_MISMATCH` | Geometry identity failed closed. |
| `DETECT_FULL_GEOMETRY_REQUIRED` | The detection carries no measured height. |

A candidate is capacity-feasible when no capacity gate rejected it. Capacity
gates are overlap with a placed box, top clearance, and hull containment.
Aperture, insertion corridor, unobserved support, and insufficient support
ratio are policy or observation gates: they reject a slot the container may
still have room for. Capacity gates are evaluated independently of the gate
that happens to fire first, and the capacity count is taken over every
enumerated candidate, not the retained `top_n` / `keep_rejected` subset.

Every tried candidate keeps its reason, so `BIN_FULL` remains falsifiable from
the dump alone.

How candidates are enumerated, stacked on known support, and scored is
specified in [placement.md](placement.md). What may fill occupancy and box
size is [real_scenario.md](real_scenario.md): live perception plus hull/floor
priors, not catalog, Gazebo, or an unverified planned slot.
