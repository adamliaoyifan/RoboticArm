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
