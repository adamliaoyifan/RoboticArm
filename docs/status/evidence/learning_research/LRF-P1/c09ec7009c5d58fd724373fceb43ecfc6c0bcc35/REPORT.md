# LRF-P1 engineering report

Frozen offline comparison of the production `estimate_box` baseline, measured
multi-view fusion, and a ridge residual ensemble. Production packages, launch
defaults, topics, and hard gates were not modified.

Reproduction:

```bash
cd /home/adamliao/work/elfin_humble_ws_eng_lrfp1
PYTHONPATH=$PWD:$PWD/src/luggage_description:$PWD/src/luggage_perception \
  python3 -m unittest discover -s research/lrf_p1/tests -p 'test_*.py'
PYTHONPATH=$PWD:$PWD/src/luggage_description:$PWD/src/luggage_perception \
  python3 research/lrf_p1/run_spike.py --workspace "$PWD" --tests-result pass \
  --out docs/status/evidence/learning_research/LRF-P1/<revision>
```

## Recommendation

`continue-research`

The harness is reproducible (L0). The learning candidate is **not** L1 / shadow
eligible. Measured multi-view fusion improved heavy-occlusion geometry without
the unoccluded regression that the residual model introduced. That ablation
says active sensing is more valuable than single-view hallucinated completion
on this six-mesh split.

## Split and leakage

| Split | Meshes | N |
|---|---|---|
| train | loafbrr small/medium/large | 162 |
| val | vintage small | 54 |
| test | vintage medium/large | 108 |

Train/test mesh identity overlap is empty. Observations contain only `points`,
median-XY ROI, `frame_id`, and `stamp`. Visible-surface ratio and mesh id stay
in eval records. Gate 5 bag fixtures exist (`bag_path=null`); no hardware bag
was available.

Seeds 1, 2, 3. Test buckets 36 light / 36 medium / 36 heavy. Unoccluded control
n=12.

## Ablation (test, seed-1 rows; seed medians match)

| Method | Heavy valid | Heavy composite p95 | Unoccluded composite p95 | Heavy width p95 (m) |
|---|---|---|---|---|
| baseline `estimate_box` | 0.972 | 1.868 | 0.090 | 0.340 |
| measured multi-view fusion | 1.000 | 1.721 | 0.090 | 0.295 |
| learned residual ensemble | 1.000 | 1.423 | 0.470 | 0.125 |

Learned heavy composite p95 is 23.8% better than baseline (L1 asks 20%). Heavy
valid-rate gain is +2.8 pp, not +10 pp. Unoccluded composite error grows from
0.090 to 0.470 (far beyond a 10% regression). Height/Z/top-Z p95 also worsen
under heavy occlusion. Under-bound rate remains 0.42 overall (not zero).
Nominal 90% interval coverage is 0.00 (uncalibrated). Conservative envelope
containment is 0.028, not 0.99. Learned p95 latency 11 ms on this CPU (below
200 ms). Peak RSS 180.5 MB. GPU unused.

## L1 gate map

| Gate | Result |
|---|---|
| no privileged inference fields | pass |
| leakage audit (mesh split + keys) | pass |
| OOD / malformed fail closed | pass (13 unittest) |
| import isolation | pass (13 unittest) |
| hard gates unchanged | pass (no production edits) |
| heavy valid rate +10 pp | fail (+2.8 pp) |
| heavy composite p95 +20% | pass (-23.8%) |
| no critical p95 +10% worse | fail (height/Z/top-Z) |
| conservative containment 99% | fail (0.028) |
| zero 30 mm face under-bounds | fail |
| 90% interval coverage 85-95% | fail (0.00) |
| nominal no 10% regression | fail |
| per-frame p95 <= 200 ms | pass (11 ms) |
| uncertainty 85-95% on sim | fail |

## Method

See `research/lrf_p1/METHOD.md`. PoinTr/AdaPoinTr (MIT, official
`yuxumin/PoinTr`) was audited and not executed: ShapeNet weights, CUDA
kernels, and a domain gap that six suitcase meshes cannot fine-tune without
leakage. Occupancy networks were rejected because they paint UNKNOWN as solid.
The runnable candidate is sklearn Ridge residual ensemble (BSD-3-Clause).

## Limits

Six checked-in meshes. Train and test are different visual families, so this
is a harsh transfer test, not a same-mesh pose split. Synthetic viewpoint
dropout is not a wrist-camera ROS bag. Results must not be read as hardware
L2 evidence.
