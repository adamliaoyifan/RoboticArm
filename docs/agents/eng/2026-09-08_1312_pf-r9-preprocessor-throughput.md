# 2026-09-08 -- PF-R9 preprocessor throughput closed blocked-on-B3

- role: eng
- agent: claude
- model: glm-5.3
- cli: claude
- status: done

## Summary

PF-R9 implemented and measured to `bebaa7c`; closed **blocked** on B3 only
per the plan's own trigger. B1 discriminating measurement recorded first
(matched-header receipt lag p50 2.8 / p95 22.9 / max 39.3 ms; 15.24% of RGB
stamps have no cloud; BEST_EFFORT loses 31-37% -> inputs stay RELIABLE).
B2 split `camera_slop_sec` into `camera_pair_tolerance_sec` 5 ms +
`camera_wait_deadline_sec` 60 ms on the RGB-stream clock (joints can no
longer kill a co-stamped cloud); skip-with-reason policy instead of
RGB-only emission (B4 arithmetic). B4 passes on the final config
(cloud_ok 0.966, exact join 0.989, stale 0.012). B5 measured root causes
and fixed the big ones: BLAS gemm fan-out pinned the node at 435% CPU
(ufunc transform + thread caps -> 110%); in-callback 3.7 MB publish p95
233 ms (daemon publisher thread + bounded queue); float32 + structured
fast decode; decimation stride 2 (cargo 29.5k -> 7.4k points); filter
stats 1 Hz throttle; MultiThreadedExecutor variant measured and abandoned
(status timer/params never serviced). B3 misses by measurement (0.462x /
256 ms vs 0.8x / 60 ms) — DDS transport and Python cloud-path costs; the
out-of-scope pixel-space masking change is the only path to both bars,
raised to reviews as Q-20260908-1. Whole-chain sanity at the final
config: **active_output_hz 12.30** (baseline 3.6-3.8), top_surface_rate
1.000, support z sub-micron, false_measured_height 0. Suite 510 passed;
teardown residual 0 after every run.

## Pointers

- `docs/agents/discuss/2026-09-07_2039_pf-r9-preprocessor-throughput.md`
- `docs/agents/discuss/2026-09-08_1310_pixel-space-masking-consensus-trigger.md`
- `docs/status/evidence/platform_free_height/2026-09-08_pfr9_throughput/RESULT.md`
- commits `bebaa7c` (implementation), `7b0b41a` (PF-R8 base)

## Open

- reviews decision on the pixel-space consensus (Q-20260908-1); PF-R10
  stays dependency-blocked on PF-R9 passing until then.
