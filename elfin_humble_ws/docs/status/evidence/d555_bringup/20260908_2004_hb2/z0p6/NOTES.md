# HB-2 snapshot at operator ~0.6 m

- operator_labeled_range_m: 0.6
- median_aligned_depth_m: 0.614
- p10_p90_m: 0.563 .. 0.673
- scene: nadir tiled floor; grout is colour-only; left caster sits in colour
  columns 0-20 where aligned depth is invalid
- usable 3D edge in overlap: none (all valid depth 0.535-0.721 m, floor plane)
- pointcloud rgb: populated (std ~20, not constant)

Phase-correlation on the whole frame is not the plan test (no shared
colour-and-depth discontinuity). Native depth warped with identity extrinsics
shifts the valid-depth mask by about du=-32 px, vs 31 px expected from the
59 mm colour lever at 0.614 m. Warping with the HB-1 `depth_to_color` leaves
that mask at du=0. That supports "native product is not colour-aligned" but
does not replace a target-edge measurement.

Need a high-contrast object with a depth jump inside the overlap (roughly
columns 30-610) before quoting signed u/v for the 0.6 m station.
