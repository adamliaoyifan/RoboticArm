# ApplyBatch: eliminate redundant lookup, then batch tree maintenance

- owner: eng/codex/gpt-6
- base: 7007b352c09608df210291273d2b80b468ec7ddd
- worktree: /tmp/elfin-occ-octomap-perf-cursor-g1 (existing; no new worktree)
- authorization: user approved the conversational plan and requested implementation.

## Changes and invariants

A: keep OccupancyGrid::apply interface. One search captures old existence, classification and log odds. A stack OcTreeNode uses official integrateHit/integrateMiss to compute the new probability. Skip only existing unchanged values. setNodeValue(key,new,false) avoids updateNode's redundant search. Never mutate/carry a borrowed tree pointer across updates. Keep unknown distinct from known zero, free/hit unique and hit precedence. Preserve mutation semantics.

B, separate commit after A correctness: setNodeValue(...,true), then once per changed frame updateInnerOccupancy followed by prune before apply returns. Both costs remain in ApplyBatch/integrate. No periodic/background pruning, concurrent writes, OctoMap fork, ray/discretize/config/block/surface change or rollback guarantee expansion. Off adds no optional counters/clocks. Add batch timing write_loop/inner_update/prune and keys considered/unchanged/written to OccupancyDiagnostic, not per-key clocks.

## Tests and interfaces

Extend existing FusionProfileChecks and FusionBenchmarkSuite, not one-off workers. Compare original search+updateNode(false) reference for new free/hit, saturation/reversal, probability-only changes, zero-known, pruned sibling expansion, ten-frame sequences, empty/no-change, normal restore continuation and existing malformed checkpoint/incomplete scan/out-of-key rejection. Check full known probabilities, mutation, classifications, counts, identity, immutable snapshots and parent consistency at 1/2/5 cm. All specified finite regression cases require zero mismatch, not a statistical reliability assertion.

Shared test-only canonical_probability_digest(tree_bytes): read a copy; validate finite leaf probabilities and parent=max(children) first; losslessly prune copy; hash serialization, retaining raw SHA and node count separately. Test differing structures with same probabilities, altered single probability, unknown versus known. Identical shared test adapter on baseline/A/B; label adapter SHA separately from frozen core. Never repair inconsistent parents before comparison. No production query API/schema change beyond additive diagnostics.

Release -O3 -DNDEBUG, OctoMap 1.9.8 exact /opt/ros/humble. Run full CTest and related Python CLI/runner tests after frozen commits; ASan+UBSan profile/keyset checks; ROS node links without launching. Record YAML SHA, build command, binaries, ldd, revision and package dirty state.

## Paired performance procedure

Reuse test/benchmark/fusion_benchmark.py FusionBenchmarkSuite and existing lossless fixtures in docs/status/evidence/occ_owner_b1/occ_octomap_perf_diag_g1_20261026/run01/fixtures. Baseline 7007 core -> A 12 Off pairs, A -> B 12 Off pairs, B Off -> Stages 4 pairs. Four cases: 0/20 boxes x 30k/100k, 1cm, offset .5/cross0, discretize=false, surface=true, acquire+ROI+state/content digests. Each fresh-store window has 3 warmup +30 scored, alternating AB/BA; 3 repeats for comparisons, 1 for Stages. --campaign baseline --repetitions 3 --require-content; stages --repetitions 1. Lock /tmp/elfin-occ-backend-perf.lock. Preflight 30min; scored measurement 3h excluding lock wait; timeout 900s per process; complete current window, insufficient samples not_evaluated. Do not mutate frozen product while measuring.

Per-window median=(15th+16th)/2, P95=29th of30, max; no pooling hides failing windows. Report integrate, lexical Fusion, ApplyBatch/write_loop/inner_update/prune, BlockBatch, queries, CPU time, node/tree memory, mutation size/capacity and count; separately show indices3/4 and31/32. Stage attribution valid only P95 ratio<=1.10 AND max<=1.20; otherwise mechanism-only. Additional four candidate Off no-digest 3+30 replays record process RSS without checkpoint/validation allocations; not Orin measurements. Preserve same inputs, compare every paired frame's canonical probability and ROI/surface content, raw SHA differences separately. Missing digest not_comparable.

No new product performance gate or arbitrary speedup target. Report old references separately: 30k integrate P95/max500/1000ms;100k2000/5000;queries100/250. Finite windows do not establish long-term reliability. Product O5/O6/O7 remains not_evaluated. If B slows, recommend A based on per-window evidence; do not shift timing, defer pruning, increase voxel size or automatically promote production.

## Evidence and failure capture

New primary evidence docs/status/evidence/occ_owner_b1/occ_apply_batch_20261029/run01/. Never overwrite old reports. Save commits/builds/environment/commands/config and input SHA, flushed frame YAML/JSONL, per-window and aggregate reports, exact replay manifest, and role note/thread result.

Boundary | T1 | Trigger | T2 and window | format/budget | replay
---|---|---|---|---|---
ApplyBatch/tree | frame identity, probabilities/count summaries, times | mismatch/error/crash | lossless failing input plus complete cumulative manifest prefix, pre/post checkpoints when available, mutation diff/logs | YAML/JSON/binary gzip,512MiB per case | same frozen CLI with manifest prefix
Store/queries | identity, query digests, counts | content mismatch/error | frame prefix and ROI/surface result, tree checkpoint, stdout/stderr | same | same profile CLI and config
Timing | all30 values and stage counts | reference exceeded/timeout | failing index and lossless cumulative prefix, process log | same | original Off window; diagnostics separately labeled

Retained total cap8GiB; stop rather than truncate replay-critical data. Existing lossless fixtures may be referenced by stable path+SHA. Capture readiness checked in preflight; missing artifacts label incomplete evidence and stop dependent comparisons. No Gazebo. Failures preserved before rerun; a repair changes candidate SHA and invalidates affected scoring.
