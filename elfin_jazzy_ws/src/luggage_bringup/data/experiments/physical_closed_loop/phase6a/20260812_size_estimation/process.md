# Phase 6a process log

What was tried, what the numbers said, and what was ruled out. Sweep tables are
in `ablations.yaml` (regenerate with `continuous_size_matrix.py`).

---

## 1. Hypothesis

Boxes are now sampled continuously inside the catalog envelope, so recognising
one of three shapes is no longer possible. Perception has to measure: the
top-face rectangle gives width/depth, and the known platform elevation turns
the top plane into a height. The question is whether that is accurate enough
that a modest commit margin still covers the true box.

## 2. What was already there

`estimate_box` already implemented the whole pipeline -- top-face RANSAC,
rectangle refinement, and `height = plane_z - platform_z`. Nothing new was
needed in the algorithm. Two downstream steps were discarding the result:

- the orchestrator overwrote the measured size with the spawner manifest,
  keeping only the perceived pose and yaw;
- `match_catalog` snapped the estimate back to one of the three catalog sizes.

Both are removed. The manifest is now evaluation-only, on a separate parameter
so nothing in the control chain can read it by accident.

## 3. First measurement: a systematic underestimate

50 samples with up to 10% of the top face occluded:

| metric | value |
|---|---|
| height error p95 | 0.29 mm |
| major-axis error p95 | 66.3 mm |
| major-axis error **mean** | **-42.4 mm** |
| commit covers true size | 40% |

Height was excellent immediately, which confirms the platform-prior approach.
The footprint was not just noisy but *biased low*, and low is the dangerous
direction: the map believes the box is smaller than it is and the next box gets
planned into the overlap.

## 4. Separating estimator bias from occlusion

A margin sized against a biased estimator is both wasteful and fragile, so the
bias had to be attributed before it was covered. Sweeping the occlusion level:

| max occlusion | major mean | major p95 | cover @ 20 mm |
|---:|---:|---:|---:|
| 0.00 | -9.7 mm | 16.2 mm | 1.00 |
| 0.05 | -26.1 mm | 38.7 mm | 0.98 |
| 0.10 | -42.4 mm | 66.3 mm | 0.40 |

Two separate effects: about 10 mm is intrinsic to the estimator and present
even with a fully visible face; the rest scales with how much of the face is
hidden.

## 5. Fixing the intrinsic 10 mm

`_refine_rectangle` bounds the face with the [0.5, 99.5] percentiles to reject
depth outliers. Those percentiles discard precisely the points that define each
edge, so the extent comes back short by roughly `extent * 2 * trim` -- about
7 mm on a 0.7 m face, matching the observed 10 mm.

Scaling the trimmed extent back by `1 / (1 - 2*trim)` keeps the outlier
rejection and removes the bias:

| | before | after |
|---|---:|---:|
| major mean, no occlusion | -9.7 mm | **-3.0 mm** |
| major p95, no occlusion | 16.2 mm | **9.5 mm** |

All 127 existing perception tests still pass, and two new tests pin both halves
of the behaviour: a clean face must measure to within 4 mm, and stray outliers
must still not inflate the extent.

## 6. What was deliberately not done

The residual error under occlusion was **not** engineered away. An edge that is
not observed cannot be measured; pretending otherwise would mean extrapolating
a box boundary from nothing. It belongs to the commit margin, not the
estimator. The sweep records what each occlusion level costs so the margin can
be re-derived once the real occlusion at the Gazebo pickup view is known.

Catalog snapping was also not re-enabled. It would have made the reported size
error look far smaller while actually throwing the measurement away.

## 7. Result

With a fully visible top face -- which is what a top-down view of an isolated
box on an empty platform should give -- all eight gates pass:

- height p95 0.29 mm, footprint p95 9.5 mm, center p95 4.9 mm, yaw p95 1.91 deg
- commit margin covers the true size in 100% of samples
- zero catalog snaps, every estimate inside the plausibility envelope

## 8. Next

Measure the actual occlusion at the Gazebo pickup view. If it is non-zero, set
`size_uncertainty/xy_margin` to roughly half the measured major-axis p95 rather
than leaving it at 20 mm. Then Phase 6b (grasp centring and retention on
continuously sized boxes), which does not depend on the P0-EX execution
failures; Phase 6c still does.
