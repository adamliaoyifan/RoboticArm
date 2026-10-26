# OctoMap slow-update diagnosis: instrument first, Cursor measures

- date: 2026-10-26
- parent: OCC-QUERY-INDEX-20261025
- subtask: OCTOMAP-PERF-INSTRUMENTATION
- generation: 1
- base_revision: d3cb53ea (resolve full SHA at dispatch; contains product9f1547f3 plus Cursor harness)
- owner: eng/codex/gpt-6
- evaluation_owner: test/cursor/grok-4.6
- status: implementation-and-measurement-contract

## Scope and hypotheses

The observed defect is slow updates, not incorrect occupancy. Preserve original
run01 FAIL evidence. No ray cap, downsampling, geometric clipping, probability,
input validity, frame identity or map semantics change in this task. Add opt-in
measurement only. Core owns update statistics, snapshot owns surface statistics,
serialization owns reporting, and the diagnostic harness owns workload/aggregation.
No shared production files, planning, Gazebo or migration are needed for this
performance diagnosis. Existing OCTOMAP-OBSERVATION remains open; comparison gate
stays blocked. The user delegates actual performance measurement to Cursor Grok.

Distinguish hypotheses with measurements, not P95 subtraction: H1 fingerprint and
preprocessing, H2 excessive full-ray exterior traversal, H3 repeated key/domain/set
work across rays, H4 tree mutation, H5 immutable block copy/maintenance, H6 surface
rebuild being mislabeled a cached query. Old b33 crops hits, downsamples and budgets
rays, clips the ray sample interval to AABB, and then enforces active hull cells.
Current OctoMap traverses each surviving endpoint's complete ray before filtering
keys. This establishes differing work, not the fractional latency contribution.

## Profiling contract

Three explicit per-call modes: off (default), stages, detailed. Diagnostics do not
enter the map config hash and never alter generated keys or probability updates.
Off emits profile_enabled=false and zero diagnostic values, not valid zero-cost
samples. Stage mode has constant-count clocks around lock/validation/fingerprint/
dedup/Z-filter/transform/ray preparation/update-publication. Detailed adds per-ray
and per-updated-key clocks; this is T3, potentially perturbed, never certified O5
timing. Every record carries mode, observation id/stamp, output identity/status.

Top-level spans are disjoint: lock_wait, validation, fingerprint, dedup,
depth_filter, transform, ray_preparation, fusion, finalization; residual is
total minus the sum. Nested spans must not be added again: ray_generation and
ray_filter_dedup lie inside ray_preparation; tree_update and block_maintenance lie
inside fusion. Tree time includes search/update/ancestor maintenance; block time
includes locating/copying/updating mutable replacement blocks. Never call block
copies once per changed cell in reports: count actual copied blocks/cells.

Counts: raw/valid/rejected points; actual rays attempted; generated full-ray keys;
active/inactive key visits; rays with/without any active traversed key; free-set
insertion attempts/duplicates; unique hit/free keys; changed probabilities and
classifications; rebuilt blocks and copied cell entries. `rays_with_active_keys`
is a discrete traversal classification, not a geometric segment-intersection
oracle. Exterior fraction=inactive/generated only when generated>0.

Surface record: lock wait, actual cache hit, scanned cells, output columns,
construction time and total. Call surface twice on the same snapshot: first may
build, second must hit cache; repeat after an accepted changed-map observation.
A surface request after each changed revision is not a cache-hit benchmark.
Record external acquire/get/ROI/surface round trips separately from core spans.

Native diagnostic runner must preserve each sample as JSONL/YAML with numeric
fields, status and exact request SHA. Input parse, file IO, state digest and YAML
encoding are outside the integrate stopwatch and timed separately where relevant.
Store raw samples, not only aggregates. Every failed/rejected call still produces
an identifiable record; early paths must not claim completed stages.

## Implementation acceptance (Codex, small checks only)

Build isolated core/CLI. Behavioral checks on small clouds in off/stages/detailed:
identical receipt semantics, revisions, full probability checkpoint bytes, key
states and immutable outputs. Include valid, all-filtered, duplicate/conflict,
out-of-order, invalid TF, exterior/crossing rays, new and saturated cells. Count
identities must close: generated=active+inactive; insertion attempts=active visits;
attempts=unique-before-hit-removal+duplicate attempts; copied/rebuilt counters
match actual work. Timing spans nonnegative, nested<=parent within clock tolerance,
exclusive sum<=total within 0.1ms. On rejection incomplete subspans are identified.
Surface first-build then true cache hit: first scanned>0, second scanned=0 and
same output. No heavy performance score or simulation in this implementation task.

## Cursor execution procedure

1. Claim separate PERF-MEASURE task at delivered exact implementation/plan SHA;
   create dedicated worktree/build/install. Read original a0121527 acceptance
   thresholds and this measurement contract. Record compiler flags, source tree,
   ldd/header OctoMap1.9.8, CPU/RAM/OS/affinity and install hashes. Sources clean;
   generated build/evidence listed separately. Acquire
   `/tmp/elfin-occ-backend-perf.lock` for measured workloads; no simultaneous heavy
   backend/Gazebo work. Do not change code/config inside a campaign.
2. Validate CLI, lossless fixture replay and sample association in a non-scored
   rehearsal (<=30min per attempt). Rehearsal failure consumes no scored sample.
   Preserve inputs before fixes. The instrumented branch includes old harness but
   its aggregate-only bench is not an adequate profiler; use delivered raw-sample
   CLI and extend the orchestration harness as necessary.
3. Freeze O5 reproduction workload:1cm chamfer,0/20 boxes x30k/100k, outside=.5,
   cross=0, seeds fixed to original run01 generator;3warm+30 scored per group.
   Preserve exact raw clouds, origin/TF, counts, hash and checkpoint/start-state.
   Run old product9f1547f3 with d3cb53ea test-only additions and instrumented-off
   against the same ordered observations on separate stores; alternate AB/BA
   in15 pairs, each pair replays3warm+30. This produces450 samples/arm/group;
   each30-sample window scored separately. If wall budget below allows only a
   subset, report remaining groups not_evaluated, never silently lower samples.
4. Profile overhead using identical-state paired windows: off vs stages, then
   off vs detailed, each3warm+30, 3 AB/BA repetitions per group (90/arm/group).
   Equal input/output hashes required. Report per-window P50/P95/P99/max and
   ratios; stage P95 ratio<=1.10 and max ratio<=1.20 qualifies stage timing as
   low-overhead. Detailed ratios are mandatory but no pass exemption: detailed
   figures explain mechanisms only; off remains latency verdict. If stages fail,
   use off for scores and label diagnostic shares perturbed.
5. Diagnostics matrix (detailed,3warm+30 each):1cm,0/20 boxes,30k/100k;
   scenarios A all endpoints inside, B endpoints outside with rays crossing ULD,
   C endpoints outside with no active ray keys. For each scenario hold ULD-frame
   endpoints fixed and use camera offsets0,.5,2m outward from aperture, recomputing
   optical coordinates/TF consistently. All points must satisfy .52<=opticalZ<=6;
   count each dropped point, and if valid cardinality differs, report unmatched
   workload rather than a causal comparison. Fixture generator must assert
   scenario classifications with the independent oracle and preserve its inputs.
   Record all ray/counter and stage fields; compare per-frame work and time.
6. Redundancy diagnostic: take a fixed1000-endpoint inside-ULD cloud and repeat
   its points1/10/30 times (1k/10k/30k). For0/20 scene context,3warm+30 each.
   Expected unique key sets and probability changes per frame identical. Higher
   generated visits/duplicates with unchanged unique cells isolates repeated-ray
   work. This is not a substitute for full-cloud accuracy or authorization to cap
   production rays. Include saturated repeated-ray windows: same geometry with
   new stamps,32 warmups then30 samples; report actual changed_keys, do not assume
   saturation if it did not occur.
7. Surface series on the four O5 groups: fresh-store first build30; updated-map
   build after each distinct revision30; same-snapshot second query30; saturated
   same-map-revision queries30. Assert actual cache_hit and scanned_cells; report
   each separately. Retain full output digests to prove output equivalence.
8. Optional old-grid context: use b33 complete-update receipts on exact same ULD
   observations with its original downsampling and ray budget. Report actual rays,
   cells, bytes and semantic differences alongside latency. It remains a workload
   comparison, not formal OCC-BACKEND-COMPARISON and not a speed ratio claim for
   equal algorithms. Do not disable its policies to force a passing comparison.
9. Aggregate and write bottleneck report, proposed next optimization with measured
   mechanism and semantic risks, exact commands to reproduce every reported case.
   Do not implement clipping/downsampling optimizations in this measurement task.

Total wall budget8h across measured campaigns, excluding lock wait; stop at a
window boundary and report unfinished groups not_evaluated if exhausted. Never
restart a slow failed run just to obtain a favorable result. Required count per
window is30; P95=sorted index28, P99/max=index29. Ratios compare matched windows;
P95 differences are not additive stage costs. Frame-level stage fraction uses
same-frame stage/total; aggregate those fractions separately. Hotspot ranking is
by measured median contribution plus tail-case evidence, not correlation alone.
The AND gate requires zero semantic mismatches and all measured windows satisfying
their stated bounds; it reports finite-campaign outcomes, no long-term reliability.

## Outputs and acceptance

Required: MANIFEST.json, raw_samples.jsonl, WORKLOAD_COUNTS.csv, STAGE_TIMINGS.csv,
PROFILE_OVERHEAD.md, SURFACE_CACHE.md, BOTTLENECK_REPORT.md, REPLAY.md. Keep
unmeasured targets explicit. Instrumentation acceptance: zero semantic changes,
100% sample association, nonnegative consistent clocks/counters. Measurement task
completion means all requested measurements/report or explicit budget/missing-data
outcomes, not product performance PASS. Product O5 remains30k P95<=500ms/max1s,
100k<=2s/5s; acquire/ROI/surface P95<=100ms/max250ms. Failures remain failures.

| Boundary | T1 | Trigger | T2/T3 payload | Window / replay |
|---|---|---|---|---|
| Raw input/Z/TF | id,stamp,hash,mode,counts | mismatch/reject/unexpected status | lossless optical XYZ, config,TF,geometry,starting checkpoint | failed frame plus previous2/next1, CLI replay |
| Ray preparation | generated/active/inactive/duplicates, spans | count identity/oracle mismatch, slowest3 frames/group | original endpoints, independent expected sets; detailed profile | same frames, no unbounded per-key log |
| Tree/block update | before/after identity,change/copy counts, spans | semantic mismatch/latency bound | full probability before/after snapshots with checksums | explicit checkpoint outside timing |
| Surface/query | identity,cache_hit,scans,external/internal spans | false cache label/output drift/latency | query and output digest, frozen view identity | first/second request pair |

Evidence `docs/status/evidence/occ_owner_b1/occ_octomap_perf_diag_g1_20261026/`;
8GiB/run32GiB total, lossless compression/dedup, never delete failures. Capture
complete/replay_possible fields mandatory; stop when durable capture capacity is
exhausted. On timing failure retain slowest3 frames plus generating input manifest
and all raw timing rows. Causal claims require the measured stage/counters;
absence of native library profiling cannot be replaced by speculation.
