# Agent contract (all CLIs)

This workspace is used by several CLI agents at once (Cursor, Claude Code,
Codex, others). Follow these files; do not invent a parallel log tree.

1. **Content index** — `docs/agents/INDEX.md` (read this first; follow only
   links needed for the assigned task)
2. **Role notes and workflow** — `docs/agents/README.md`
3. **Exact document format** — `docs/agents/FORMAT.md`
4. **Mailbox** — `docs/agents/discuss/OPEN.md` (query only rows matching the
   current agent/model; do not print or read the whole file)
5. **Sim exclusivity and teardown** — `.cursor/rules/sim-lifecycle.mdc` and
   `scripts/stop_sim.sh`. Only one Gazebo stack at a time; queue behind a
   running sim instead of launching a second world.
6. **Architecture** — `docs/architecture/README.md` (use its index to open only
   relevant normative documents; violations are defects)
7. **Debug evidence and dump granularity** —
   `.cursor/rules/debug-evidence.mdc` (applies to perception, planning, and
   end-to-end evals)

Start every independent task in a fresh CLI/chat thread so unrelated long
context is not carried forward. Reuse a discuss file only for replies or a
higher-generation continuation of the same work-item lineage.

Keep command/tool output bounded to 2,000–5,000 tokens. Read targeted line
ranges, headings, matching mailbox rows, and evidence summaries; never dump a
complete branch history, evidence tree, large document, or `OPEN.md`. Prefer
one discovery pass, one coherent edit, and one proportionate verification pass.

## Maintainability principles

1. **Codex readability** — Optimize code and configuration for fast, reliable
   comprehension by Codex and human maintainers. Use descriptive names,
   explicit control flow, nearby contracts and invariants, and focused files;
   avoid hidden coupling, unexplained magic values, and clever abstractions
   whose behavior cannot be inferred locally. When behavior changes, update
   the closest documentation and focused tests in the same change.
2. **Functional modularity** — Organize behavior into cohesive,
   single-responsibility modules with narrow, stable interfaces and explicit
   dependency direction. Do not keep extending god nodes, managers, scripts,
   or shared utility buckets as the project grows. Split modules at ownership
   and failure boundaries, and make inputs, outputs, errors, state transitions,
   and boundary-level diagnostics observable so complex bugs can be isolated
   to one module before end-to-end debugging.
3. **Measurement before inference** — Reproduce and instrument a failure before
   changing behavior around a suspected cause. Record revision, environment,
   workload, sample count, stage boundaries, timings, and preserved failure
   artifacts; label direct observations, source-established mechanisms, and
   remaining hypotheses separately. Test competing explanations and do not
   spend scarce live or scored runs on an unmeasured ordering or timing
   assumption. If a failure path did not occur during verification, claim only
   the coverage actually demonstrated—never a success-rate improvement.

Cross-agent discussion is **file-based and asynchronous**: post or reply in
`docs/agents/discuss/<thread>.md`, list unanswered asks and runnable handoffs
in `OPEN.md`. The other CLI only acts the next time it runs. There is no live
RPC unless a separate scheduler controls a registered CLI session.

The scheduler entry point is `scripts/agent_scheduler.py`. It watches
`OPEN.md` and matches rows to registered sessions in `docs/agents/RUNTIME.md`.
Already-running CLIs are only directly schedulable when that registry row has
a fresh heartbeat, `state=idle`, and a supported dispatch capability such as
Codex `queue`; otherwise the task remains file-based until the agent next
reads `OPEN.md`. File-only sessions can poll matching rows with
`scripts/agent_poll_self.py` and keep a local queue. Register or refresh a
running CLI with `scripts/agent_register.sh`.

Roles are responsibility pools, not unique workers. `eng`, `reviews`, and
`test` may each be served by multiple concrete agents such as `codex`,
`claude-code`, or `cursor`; use `agent` / `to_agent` / `from_agent` fields to
identify the concrete worker when it matters. Also record `model` /
`to_model` / `from_model` when the model choice matters; for example, Cursor
running Opus5 may serve both `reviews` and `eng`.

When using satellite git worktrees for code isolation, keep cross-agent
coordination in the primary workspace's `docs/agents/` tree. Do not create a
second mailbox per worktree.

`reviews` clarifies and decomposes unified requirements, then dispatches
directly to the concrete owner/model. Consensus is optional and must not gate
dispatch unless the user explicitly requests it. Each resulting subtask has a
concrete agent/model owner, generation, base revision, exact plan revision,
acceptance criteria, required tests, commit evidence, and explicit
`dispatch_ready` state. That owner reads the indexed requirement, implements,
tests, fixes failures, commits, records evidence, and closes the subtask end to
end.

Plans must make acceptance decision-complete before dispatch: name every
metric and threshold, workload/input matrix, environment/profile, repetition
count and aggregation rule, exact command or procedure, required artifacts,
and the revision/dirty-state rule. "Works", "looks good", or a test name
without its expected observations is not an acceptance criterion.

An aggregation rule must also state the per-unit reliability it encodes. A
campaign that requires N consecutive passes is an AND over N units; say what
per-unit success rate is acceptable and check that the gate can distinguish it
from failure. A gate that returns the same verdict for a healthy system and a
broken one is not measuring the requirement. Every bar must be computable from
the evidence the run actually produces: if a threshold has never yielded a
sample across repeated runs, give it a minimum-sample rule and an explicit
`not_evaluated` outcome instead of letting it read as unscorable.

An eval task must not modify the artifact it is certifying under a
single-scored-run budget. Launch wiring, startup ordering, and readiness gates
are deterministic behaviour: fix them first in a non-scored rehearsal loop that
may repeat freely, then enter scoring. A failure occurring before the first
scored unit consumes no seed, moves no cursor, and changes no hash, so it must
not be charged against a scarce product budget; bound it by wall clock and
report its rate as evidence.

Offline gates for startup, readiness, and wiring repairs must be behavioural.
Asserting that a source file contains or omits particular text verifies the
spelling of a fix, not its behaviour, and will pass while the same live failure
repeats.

For experiments, evals, integration, and regression, a failed run MUST preserve
the intermediate data needed to diagnose or replay the failed case under
`docs/status/evidence/` using `.cursor/rules/debug-evidence.mdc`. The default
dump unit is a failed case at pipeline stage boundaries with a bounded trigger
window, not every internal value from every module. Do not tear down,
overwrite, or blindly rerun until the required artifacts are durable. If the
current instrumentation cannot capture them, the failure is incomplete
evidence: add capture instrumentation before spending another run on the same
gate.

Claim runnable work with `scripts/agent_start.sh`; close it with
`scripts/agent_complete.sh`. Routine subtasks do not hand testing to a separate
test role. The plan includes a final integration subtask when the unified task
needs whole-chain regression.

The discuss thread is authoritative for task lifecycle; `OPEN.md` is only the
current queue projection and `RUNTIME.md` is only transient scheduler presence.
Do not rewrite claimed requirements in place. Supersede them with a higher
generation in the same parent/subtask lineage.

Start a child agent only when the work has genuinely independent parallel
units. Pass the child only the task, exact pointers, revision, and acceptance
criteria needed for its unit; never use a full-history fork by default.

If you produce a decision, code change, plan, or eval, write a role note.
Skip ack-only turns. Test evidence stays under `docs/status/evidence/`.


## ULD interior frames and height consumers (frozen map, 2026-11-10)

Frozen scene values (deployment geometry, `scene_tf` config): inner
`floor_z=0.53`, `ceiling_z=2.01`, `1.49 x 1.97` — so the interior HEIGHT
is `ceiling - floor = 1.48`. `container_link` origin sits at the
container's OUTER datum, NOT the inner floor; in `elfin_base_link` the
container in the active `scene_tf_deploy_update.yaml` carries translation
approximately `[0, -1.059, -0.86]` and yaw `-90 deg` in `elfin_base_link`.
User reconfirmed the current deploy-update source geometry on 2026-11-11; the
older `[0, -1.5, -0.86]` entry does not describe this cell. The exact configured
translation is `[-0.0000116698, -1.0589999999, -0.86]` (rounded RPY constants
introduce the small X term). World/base transforms must come from the active
scene; world and base do not coincide in this cell. `0.53` is container-frame ONLY.

| Consumer | Frame | Height semantics | Range |
|---|---|---|---|
| solver `candidate_aabb` / constraint chain | container, FLOOR-relative | z=0 at inner floor | [0, 1.48] |
| planner `_inner_dimensions` / `_placed_aabbs` output | container, floor-relative | inner_size[2] = HEIGHT 1.48 | [0, 1.48] |
| planner `_placed_aabbs` INPUT (v1 wire) | elfin_base_link poses | converts via frozen scene | — |
| v1 `surface_map_2d` (topic / bridge / view) | container, floor-relative (height arrays) | z=0 at inner floor | [0, 1.48] |
| v2 surface contract `surface_z_container` | container, ABSOLUTE | floor .. ceiling | [0.53, 2.01] |
| G `query_cells_v2` grid.origin | container, ABSOLUTE | cell iz spans [origin.z+iz*r, ...) | [0.53, 2.01] |
| oracle `verify_candidate` hull | container, ABSOLUTE | reads `hull.ceiling_z_container` | [0.53, 2.01] |
| oracle `placed_clearance_leg` | container, floor-relative | scene-derived floor/height | [0, 1.48] |
| K01 oracle payload hull (`_k01_hull`, schema v4) | BOTH, named | `inner_size[2]`=HEIGHT 1.48 + `ceiling_z_container`=2.01 | both |

**NAMING RULE (user ruling 2026-11-10)**: a coordinate/bound field
carries a frame qualifier (`floor_z_container`, `ceiling_z_container` —
container-ABSOLUTE); a size field NEVER carries one (`inner_size`,
z = interior HEIGHT 1.48). A coordinate never rides inside a
size-named field. Historical offenders, all retired or documented:
payload `inner_size[2]` carried the absolute ceiling until schema v4
(oracle keeps a legacy read shim so v3 archives re-judge
byte-identically); scene config `inner.height` is HISTORICALLY the
absolute upper-Z (see `container_inner_dimensions` docstring — use
`container_inner_ceiling_z` / `container_usable_dimensions` instead);
the map descriptor's `ceiling_z`/`floor_z` are absolute and correctly
named. Any new consumer must state its frame here before reading a
height datum.
