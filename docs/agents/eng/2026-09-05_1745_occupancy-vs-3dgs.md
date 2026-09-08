# 2026-09-05 -- Interior occupancy vs 3DGS for planning

- role: eng
- agent: cursor-grok-b
- model: grok-4.6
- cli: cursor
- status: done

## Summary

Learning-based occupancy and 3D Gaussian Splatting exist, but they do not
replace this cell's planning map. The online cargo map is still a
container-frame voxel grid with FREE / OCCUPIED / UNKNOWN. The Humble node
commits planned box AABBs only and does not integrate depth. Placement reads
the 2.5D surface, treats unknown columns as unstackable, and uses the
calibrated hull as a prior rather than observed free space.

3DGS is a photometric scene, not a solid volume. Published robot uses
(Splat-Nav, SAFER-Splat, PolyMerge) convert Gaussians into ellipsoids,
voxels, or conservative polytopes before collision checks. Driving occupancy
nets (GaussianOcc, VoxelSplat) target surround-camera streets, not a ULD
interior. Neither representation supplies UNKNOWN, hull clipping, or corridor
queries that packing requires.

Trustworthy interior occupancy for this robot is multi-view depth fusion into
the existing voxel contract: stop-and-look NBV, stamped TF, log-odds or TSDF
raycasts clipped to the hull, post-place scan versus the committed box, and
fail-closed unknown. Learning may propose views, complete depth, or instance
masks. It must not silently fill occluded volume as free or occupied.

## Pointers

- `src/luggage_perception/luggage_perception/cargo_volume_mapper.py`
- `src/luggage_perception/scripts/cargo_volume_mapper_node.py`
- `docs/plans/sim_r1_production_orchestrator_exploration.md`
- `docs/plans/sim_real_parity_pickup_support.md`
- `docs/architecture/container_geometry.md`
- `docs/architecture/production_orchestration.md`
