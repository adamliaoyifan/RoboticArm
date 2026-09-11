# PF-R10 C2 RSS amendment — measured report

- Decision date: 2026-09-11
- Evidence revision: `bd70df563c5567bcf36298c0aa083abeeed44af3`
- Scope: replace raw mixed-workload RSS slope scoring; do not rescore old data

## Amendment rationale

Carry-on, standard, and large luggage legitimately require different bounded
working sets. A line fitted directly to a short sequence ordered from smaller
to larger work, or to time-window minima affected by `malloc_trim`, aliases
workload size and allocator state into elapsed time. It is not a valid test of
time-dependent accumulation.

The `2 MiB/min` guard remains, but is applied to the elapsed-time coefficient
after controlling for luggage size. Raw RSS statistics remain mandatory
reporting. Bounded arena retention, native-library caches, and freed pages that
remain resident are not defects unless the workload-adjusted series continues
to grow.

## Existing measured values

| run | node | first MiB | last MiB | min MiB | max MiB | Q1 mean MiB | Q4 mean MiB | raw slope MiB/min |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | semantic_point_filter | 98.69 | 99.73 | 97.81 | 99.73 | 98.67 | 98.97 | 0.33 |
| 2 | semantic_point_filter | 95.34 | 96.29 | 95.34 | 97.55 | 95.93 | 96.56 | 0.79 |
| 3 | semantic_point_filter | 98.53 | 99.88 | 98.53 | 101.98 | 99.12 | 99.55 | 0.32 |
| 1 | luggage_detector | 97.04 | 103.32 | 92.59 | 107.65 | 99.76 | 100.57 | 0.75 |
| 2 | luggage_detector | 99.90 | 105.02 | 87.29 | 112.27 | 95.36 | 103.63 | 8.47 |
| 3 | luggage_detector | 102.72 | 105.08 | 89.52 | 108.93 | 100.83 | 100.78 | -0.22 |
| 1 | sensor_preprocessor | 96.64 | 98.87 | 96.64 | 98.87 | 97.48 | 98.30 | 1.19 |
| 2 | sensor_preprocessor | 98.65 | 97.44 | 95.01 | 99.75 | 98.48 | 97.67 | -1.20 |
| 3 | sensor_preprocessor | 99.35 | 99.75 | 97.59 | 100.49 | 98.80 | 99.90 | 0.99 |
| 1 | semantic_segmenter | 1900.44 | 1900.84 | 1900.44 | 1900.84 | 1900.61 | 1900.80 | 0.23 |
| 2 | semantic_segmenter | 1906.46 | 1906.79 | 1906.46 | 1906.79 | 1906.54 | 1906.73 | 0.23 |
| 3 | semantic_segmenter | 1922.29 | 1922.82 | 1922.29 | 1922.82 | 1922.38 | 1922.79 | 0.50 |

Values are copied from each `g6s_summary.json`; no value was recomputed to
manufacture a pass. The run2 detector raw slope remains visible.

## Scoring consequence

The old probe recorded timestamps and RSS but not current-box size and scored
occurrence on each RSS sample. Therefore `beta` in the amended model cannot be
identified reliably from these artifacts. The three runs are **not scorable**
under the amended memory-growth gate and do not establish PF-R10 completion.

A conforming rerun must record the required labels, preserve the raw report,
fit `RSS = intercept + size fixed effect + beta * elapsed_minutes` per node,
and store the design rows and fitted coefficient so the result is reproducible.
