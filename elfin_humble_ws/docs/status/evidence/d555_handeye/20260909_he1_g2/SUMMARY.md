# HE-1 generation 2 — D555 mechanical registration

- outcome: blocked
- CAD M4 spacing: 125.400 mm (datasheet 125.40 ± 0.20)
- unique hole pair: `STL_BAR_H0` + `STL_BAR_H3` (outer 125.00 mm vs 125.40 mm)
- inner bar pairs rejected: 15 mm, 95 mm, 110 mm vs datasheet 125.40 mm
- windows-inward `nsign=-1` rejected: anti-aligned seating, Mid360 occupancy
- 180 deg yaw of the 2x M4 pattern remains; canonical seed maps CAD +X along `STL_BAR_H0` -> `H3`
- housing-to-d555_link: not authoritative; seed is `^eef_mount_adapter T_D555-mechanical`
- reported transform is NOT `^eef_mount_adapter T_d555_link`
- xyz_m: 0.0225, 0.1023, -0.0565
- rpy_urdf_xyz: -2.8798, ~0, ~0
- geometry 14: centre RMS 0.200 mm, max 0.200 mm, axis 7e-6 deg, tower penetration 0
- round-trip max error: 2.81e-17 m; feature-table recompute: 0
- CAD envelope vs datasheet (mm): +0.010, -0.289, +0.000
- tessellation centre change: 0.039 mm (XY 0.003 mm)
- Monte Carlo p95 (sigma 0.25 mm): 0.50 mm, 0.29 deg
- GUI comparison (mechanical vs `camera_link`, not `d555_link`): 37.1 mm, 119.45 deg
- board: `charuco_10x8_50mm_DICT_5X5_100.pdf`; pitch relative error 0
- git HEAD at run: `42f97af0b5329326ae399ab0809a905e33d28906`
- workspace dirty-file count at run: 33 (includes unrelated PF-R9/progress files)

Do not apply this transform to URDF/xacro/yaml. Official CAD/STEP/PDF were not committed.
