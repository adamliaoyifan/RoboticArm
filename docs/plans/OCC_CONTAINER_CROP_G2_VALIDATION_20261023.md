# OCC crop G2 validation refinement after measured baseline profiling

- parent_plan: e6a5b19f
- base_revision: e6a5b19f
- generation: 2
- scope: Same product implementation and correctness thresholds; explicit causal performance comparison.

Baseline profiling preserved under occ_container_crop_20261023/baseline_profile.txt established that original apply_integrate calls stats twice just to read revision. Full frontier scans dominate 1cm time. A two-sample non-scored rehearsal measured ~9.89s original vs ~0.23s candidate, but this is not a crop-only speed claim.

Keep original frozen 86565812 vs candidate target workload (100k,90% nonintersecting exterior,1cm) at 5 warmups+30 measurements and original 20% target. For the full 30-workload matrix use an explicitly labelled revision-only control: exact old source with only `int(mapper.stats()["map_revision"])` replaced by `int(mapper.map_revision)`. This control keeps old hit/downsample/ray algorithms and avoids claiming crop benefit from incidental statistics overhead. Keep all old thresholds, including 20% improvement target and per-group 110%+1ms regression bound. Record original and control separately. This supersedes the full-matrix original-runtime baseline requirement, not its behavior oracle; no passing data may be renamed as original runtime. Control is frozen test code, not deployed behavior. Required sample counts and ALL aggregation stay unchanged.

All original correctness, 20 synthetic branch frames, original72 grid cases, build and installed-node requirements remain. Real measured-frame replay remains required for a full deployment gate; if unavailable, mark not_evaluated and overall deployment verification incomplete, not pass. The independent functional/offline performance gate can report its own outcome. ROS transport latency is similarly separate from Store in-process latency. Do not silently substitute synthetic inputs or in-process latency for these missing samples.
