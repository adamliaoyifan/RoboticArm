# Placement prior paired benchmark input contract

`benchmark.py` validates frozen artifact hashes, emits a 30-pair schedule and
analyzes captured samples. It does not collect timings, launch the planner or
certify functional gates. Instrumented process-tree collection and the six
measured workload acquisition remain required before the performance campaign.
Run only after F0-F2 acceptance, and retain F3 as a final acceptance requirement.

```bash
python3 scripts/placement_prior/benchmark.py --manifest workloads.json --output schedule.json
python3 scripts/placement_prior/benchmark.py --manifest workloads.json --samples samples.jsonl --output report.json
```

The output must be new. Artifact paths resolve relative to the manifest's
directory; absolute paths also work. Every artifact is checked against SHA256.

## Frozen manifest

Required fields:

- `schema`: `placement-prior-benchmark/1`.
- `revision`, `hardware_id`, `profile`, `installed_hash`: exact runtime commit,
  machine identity, isolated runtime configuration identity and installed artifact
  digest. The instrumented runner must measure/check these, not copy labels from
  the manifest without verification.
- `repetitions`: 5.
- `budgets`: `select_s=60`, `certification_s=2`, `client_watchdog_s=5`,
  `prior_s=10`, `prior_candidates=32`.
- `cases`: six records, each with unique `id`, `snapshot_id`,
  `pickup_orientation_id`, `measured=true`, `situation`, and `artifacts` mapping
  paths to SHA256. Include exact requests, scene, OCC, measured suction candidates
  and model/calibration identity. Each of two snapshots has exactly one
  `central_feasible`, `offset_required` and `infeasible` case. At least two
  captured pickup orientations are required. Classifications need preserved
  native collision/feasibility evidence, not merely labels.

## Captured arm samples

One JSON object per arm per repetition, including all errors and timeouts.
There are 60 rows, not 30 rows. Required fields:

- `case_id`, `repetition` (0 through 4), `arm` (`baseline` or `prior`), `order`
  (0 or 1). Even repetitions execute baseline then prior; odd ones reverse it.
- Same runtime identity fields and `budgets` as the manifest, `dirty=false`,
  `input_artifacts` equal to the selected case's frozen artifact map.
- `artifacts`: outcome logs, exact requests/responses and failure evidence as a
  path-to-SHA256 map. Every terminal outcome must retain evidence.
- `status`: `success`, `infeasible`, `timeout` or `error`; booleans
  `route_certified`, `endpoint_ok`, `cache_hit`, `fallback`, `prior_exhausted`;
  nonnegative integer `safety_violations`. False certification cannot authorize
  a successful outcome.
- Nonnegative finite measurements: `exposed_s`, `full_request_s`, `prior_wall_s`,
  `cpu_s`, `peak_rss_bytes`, `prior_cpu_s`, `prior_peak_rss_bytes`,
  `certification_s`, `collision_calls`, `ik_calls`, `service_calls`,
  `overlap_available_s`.
- `resource_scope`: `planner_and_prior_process_trees`. CPU is cumulative process
  CPU including descendants; peak RSS covers the sampled aggregate, not merely
  the thin client. State collector sample period and measured limitations in
  outcome artifacts. Prior-specific CPU/RSS are also recorded separately.

`full_request_s` is the foreground placement request duration, excluding earlier
prior preparation; `exposed_s` is the observed post-pick wait. `prior_wall_s`
records preparation even when overlapped. `overlap_available_s` comes from
measured readiness-to-pick timing. The report computes zero-overlap cost as
foreground duration plus preparation, and simulated-overlap cost as foreground
duration plus the preparation portion exceeding the available overlap. Baseline
prior-only metrics are measured zero; missing measurements are not zeros.

## Reporting and limits

All raw samples and per-case outcomes remain in the report. Missing samples or
invalid evidence give `not_evaluated`. Duplicate/unknown samples or invalid
manifest matrices are refused. Completed warm feasible workloads must each
improve median exposed latency by at least 10%, with no more than 10% p95
regression. Cache misses/fallbacks do not count as warm success. Failures cannot
be omitted to reach five successful samples. Infeasible cases returning only
transport errors do not establish their classification.

Report per-case metric medians/p95, paired exposed-time differences, and pooled
warm exposed-time statistics. p95 uses linear interpolation at `0.95*(n-1)`;
five repetitions are a regression comparison, not a fleet reliability estimate.
Resource and timing coverage must still be audited against raw artifacts: a
well-formed report cannot prove the collector measured the intended processes.
