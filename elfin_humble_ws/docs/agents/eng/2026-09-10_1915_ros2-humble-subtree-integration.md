# 2026-09-10 -- ros2_humble subtree integration

- role: eng
- agent: codex
- model: gpt-5
- cli: codex
- status: done

## Summary

Confirmed that the local workspace root maps to
`origin/ros2_humble:elfin_humble_ws/`, not to the monorepo root. Integrated
the six remote commits after `0301ae2`, resolved their D555 preprocessor
changes against the newer PF-R9 g2 depth-primary architecture, and merged
four completed agent branches with passing evidence. In-progress and
uncommitted PF-R10, HE, and DSIM work was deliberately excluded.

The remote-only `deployment_ws/` and other monorepo siblings remain outside
the replacement boundary. Existing remote-only historical evidence under
`elfin_humble_ws/docs/status/evidence/` is preserved.

## Integrated commits

- Remote workspace updates: `3ea1ce8`, `d56b914`, `7a697a2`, `b0ae5f7`,
  `5bdb6a7`, `66d705a`; imported as `e013e5a` after subtree path stripping.
- SIM-R1-5: `98db923`, `2df9d09`; merge `a087885`.
- TCIG-2 generation 2: `0a0a7d5`, `a7df9f9`; merge `f7964f8`.
- TCIG-5: `fcdc3e7`, `a5edabe`; merge `cece619`.
- TCIG-7: `8edc404`, `ee6980f`, `101e1c1`, `c499e81`; merge `77b2e35`.

## Verification

- Remote/local mapping: `40ab61c` explicitly snapshots the local workspace
  into `elfin_humble_ws/` without replacing `deployment_ws/`.
- `python3 -m py_compile` on the merged preprocessor, segmenter, and hardware
  pick launch: pass.
- Focused perception integration suite: 97 passed.
- Planning suite with the TCIG-7 generated message overlay: 263 passed and
  14 subtests passed.
- Packing suite: 87 passed and 8 subtests passed.
- Bringup plus TCIG-5 Gazebo-independent smoke suite: 14 passed, including
  the 60-second no-Start graph.
- `git diff --check`: pass.
- No Gazebo stack and no real robot motion were started.

## Pointers

- `docs/status/evidence/sim_r1/98db923/r5/summary.md`
- `docs/status/evidence/true_container_inner_geometry/0a0a7d5/g2/summary.md`
- `docs/status/evidence/true_container_inner_geometry/8edc404/g7/summary.md`
- `docs/status/evidence/tcig5/2026-09-10_tcig-5-g1/README.md`
