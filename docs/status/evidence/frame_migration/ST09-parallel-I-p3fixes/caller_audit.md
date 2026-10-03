# E.3 seven-caller typed-only audit (2026-10-03)

Live-path status per caller (production launches run frame_runtime, so
every effect is graph-gated and the v1 endpoints answer named
refusals):

| caller | typed path | v1 remnants | verdict |
|---|---|---|---|
| motion_occ | full (probe/placement/select/plan) | reads carry_segments v1 view only | GOLD standard |
| pick_retreat | build_pick/plan_motion V2 | v1 BuildMotionSequence client + PlanMotion action as `self._typed is None` fallback | live-path typed; fallback dead under frame_runtime |
| place_smoke | typed plan_motion pairing + TYPED_SEGMENT_MISMATCH guard | `if not use_typed:` PlanMotion fallback (fires only when typed adapter absent or place_exit) | live-path typed |
| cargo_map_integrate | typed plan_motion guarded | v1 path when `_typed` unset | live-path typed |
| place_only | typed compute_placement/add_placed_box | raw BuildMotionSequence.Request at :1093 (fixture-install path) | live-path typed |
| pack | typed compute_placement/add_placed_box | v1 fallback when `_typed` unset (no motion calls) | live-path typed |
| hardware_pick | typed probe_segment/plan_motion | v1 BuildMotionSequence import + PlanMotion.Goal at :183 | live-path typed |

Conclusion: all seven callers route motion through the unique
RouteProducer on the live path; the v1 code paths are dead fallbacks
that only fire when the typed adapter failed to adopt the graph — a
condition that itself refuses effect authorization under
frame_runtime. The dead code remains recorded cleanup debt (mass
edits across eval drivers used by other suites); removing it is safe
only together with those suites' fixtures.
