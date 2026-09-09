# HB-3 mount residual — three static poses

Parent: `D555-BRINGUP-20260908`. Arm jogged by operator. No URDF/yaml edits.
No corrected mount transform. Joints from CPS `HRIF_ReadActACS`.
`/joint_states` remains zeros and is unused. `elfin_base` ← `elfin_end_link`
is URDF FK at CPS joints. EE ← sensor is live TF (fixed joints).
FK(q=0) matches live EE TF to 1e-16 m.

Livox PointCloud2 is on ROS domain 0 (operator `pts/7` driver). D555 and robot
TF are on domain 7. Clouds captured back-to-back at Δq = 0.

GUI-tuned value still in the chain: `eef_mount_adapter` → `camera_link`
xyz `0.013 0.097 -0.021`, rpy `0.03770 1.36345 1.57080`
(`camera_mount_origin.xacro` / yaml `fixed.rpy`).

## Nine numbers

| pose id | offset mm | angle deg |
|---|---|---|
| pose1 | 10.10 | 1.91 |
| pose2 | 2.42 | 2.33 |
| pose3 | 1.03 | 2.19 |

| pose | joints deg (J1..J6) | EE xyz m | D555 inliers | Livox inliers |
|---|---|---|---|---|
| pose1 | 6.196, -94.144, -81.569, -100.346, 94.137, -81.504 | 0.909, -0.072, 0.933 | 85108 | 1264 |
| pose2 | -6.380, -110.976, -95.576, -64.670, 97.229, -35.340 | 1.052, -0.282, 0.519 | 168172 | 2041 |
| pose3 | -7.853, -133.047, -112.309, -27.269, 105.572, -37.126 | 0.980, -0.282, 0.018 | 173154 | 1686 |

Offset is the distance of the D555 floor centroid to the Livox plane in
`elfin_base`. Angle is between the two fitted normals. Floor centroid z is
stable at about −0.83 m in `elfin_base` (same tiled floor).

## Translation versus rotation

Offset is **not** constant: 10.10 → 2.42 → 1.03 mm as EE height drops
0.93 → 0.52 → 0.02 m. Plane-to-plane angle stays near **2.1°**.
That pattern is a mount **rotation** error. A later CAD-seeded camera or
Livox replacement should be judged against this ~2° / pose-varying 1–10 mm
residual of the current GUI value, not treated as a pure lever translation.

## Mid-360 edge doubling

Two consecutive **static** Livox frames (3 cm voxels):

| pose | Jaccard | NN median mm |
|---|---|---|
| pose1 | 0.15 | 77 |
| pose2 | 0.14 | 87 |
| pose3 | 0.08 | 97 |

Arm Δq = 0 at every capture. Low Jaccard is the Mid-360 non-repetitive scan,
not intra-frame motion deskew. RViz Decay Time overlays those successive
frames and produces the doubled/thick rail edges. Do not implement deskew here.

## Reported, not edited

`realsense_d435.yaml` `tune_joints` rpy is `rx=-1.363, ry=0.038, rz=1.571`
while `fixed.rpy` / `cam_mount_rpy` is `0.038, 1.363, 1.571`. Live URDF uses
the latter. Left as-is.

Revision `40ab61ca0f6d5237f26091d946c05227bb6fc468`. Script:
`measure_mount_residual.py`.
