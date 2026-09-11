# LRF-P1 method audit

This spike compares the production model-based estimator with one runnable
learning candidate. It does not replace production modules.

## Baseline

`luggage_perception.luggage_box_estimator.estimate_box` on identical partial
clouds. Height uses the cloud's own Z range (`platform_z=None`), matching the
platform-free online contract. No catalog snap.

## Audited completion methods (not executed)

| Method | Paper / impl | License | Why not the executed candidate |
|---|---|---|---|
| PoinTr / AdaPoinTr | Yu et al., ICCV 2021 / TPAMI 2023; `github.com/yuxumin/PoinTr` | MIT | Official CUDA kernels and ShapeNet-55/PCN weights. Domain is unitless ShapeNet chairs/tables, 2048-point canonical frames, not metric suitcase RGB-D. Fetching weights would violate "do not commit large models" and still need a suitcase fine-tune the six-mesh split cannot support. |
| ConvONet / Occupancy Networks | Niemeyer/Peng et al. | MIT | Implicit occupancy completion hallucinates unknown volume as occupied. The plan forbids inferred FREE/OCCUPIED without ray evidence. |
| 3DGS occupancy (Splat-Nav, GaussianOcc) | 2024-2026 robotics papers | mixed | Photometric Gaussians are not UNKNOWN-aware boxes. Conversion to voxels is a second research program, not this spike. |
| Multi-view TSDF (nvblox / KinectFusion) | model-based fusion | BSD/Apache | Kept as the **measured multi-view ablation** (concatenate visible clouds, then the same estimator). Not learned. |

## Executed candidate

**Box residual ensemble** (`research.lrf_p1.candidate.ResidualEnsemble`):

- Trainable ridge regressors on observation-only cloud moments plus the
  baseline box. Target is residual to the PF observable-box GT.
- Five bootstrap models; mean is the inferred box, std is uncertainty.
- Conservative envelope expands each extent by `1.64485 * std` (nominal 90%).
- Observed / inferred / unknown stay separate: baseline box is `observed`,
  residual output is `inferred`, high-std axes are listed as `unknown`.
- sklearn Ridge (BSD-3-Clause). No pretrained suitcase weights.

This matches the production contract (oriented observable box, not a dense
shape) and can fail closed when the ensemble is missing.

## Split

Train: `suitcase_loafbrr` {small, medium, large}.
Val: `suitcase_vintage` small.
Test: `suitcase_vintage` {medium, large}.

Mesh identity never appears in both train and test. Six checked-in meshes are
too few for a strong generalization claim; the report treats that as a limit.

## Inputs

Inference: `points`, observation-derived `roi_center_xy` (cloud XY median),
`frame_id`, `stamp`. Generation may use STL identity and visibility ratio
only in eval records.
