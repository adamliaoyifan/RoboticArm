# HB-2 snapshot at operator ~1.0 m

- operator_labeled_range_m: 1.0
- median_aligned_depth_m (whole frame): 1.027
- floor cluster median_m: 1.062
- near object (stool) median_m: 0.715, bbox roughly u 26-374
- contour vs colour edge: n=41, signed median du=+1 px, dv=+2 px
  (~2 mm / 4 mm at 0.715 m)
- 59 mm lever would be ~27 px u at the stool; 95 mm baseline ~43 px
- identity warp of native depth (FOV mask): du=-23 px vs ~19 px expected
  from 59 mm at floor Z
- pointcloud rgb still populated (std ~43)
