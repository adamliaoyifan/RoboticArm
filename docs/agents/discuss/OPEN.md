# Open cross-agent work

Read this at session start. Claim a row only when `to_agent` and `to_model`
match your concrete identity and every `depends_on` item is complete. The
assigned owner reads the requirement, implements, tests, repairs failures, and
closes runnable work end to end. Add a row when another agent must act.

| id | kind | parent | subtask | depends_on | revision | to_role | to_agent | to_model | from_role | from_agent | from_model | cli | thread | request | generation | plan_revision |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Q-20260915-4 | subtask | DYNAMIC-SUCTION-20260915 | ST-1 | none | 8f4d0a0dc8c449197caf146bd3a6ec6cf2fba627 | eng | claude | glm-5.3 | reviews | codex | gpt-5 | codex | 2026-09-15_2157_dynamic-suction-st1.md | Implement ST-1 and close it end to end against plan gates A0-A5. | 1 | 5822d6644554ca3b3dd621537a8c5af2bd5215f2 |
| Q-20260915-7 | integration | DYNAMIC-SUCTION-20260915 | INTEGRATION | ST-1,ST-2,ST-3 | 8f4d0a0dc8c449197caf146bd3a6ec6cf2fba627 | eng | claude | glm-5.3 | reviews | codex | gpt-5 | codex | 2026-09-15_2158_dynamic-suction-integration.md | After ST-1 through ST-3 pass, integrate, repair and close all D0-D8 and evidence gates. | 1 | 5822d6644554ca3b3dd621537a8c5af2bd5215f2 |
| Q-20260916-3 | question | n/a | n/a | none | n/a | reviews | cursor | grok-4.6 | eng | cursor | opus-5 | cursor | 2026-09-16_1228_dirty-tree-defect-audit.md | GEO wave closed at 3b50f84; EVAL wave (7, 10-16) reviewed and landed through e814576 by eng/claude (evidence in docs/status/evidence/eval_wave_replay/2026-09-17_eval-wave/); user picks the next wave (PRE 4-6, HYG 17-19 remain). |  |  |
| Q-20260918-1 | question | n/a | n/a | none | 09fb4843f586b46ed8fae0efd71caa93f58a5dd7 | reviews | cursor | grok-4.6 | reviews | cursor | grok-4.6 | cursor | 2026-09-18_1140_real-scenario-packing-defects.md | Catalog RS-1..RS-26 recorded at 09fb484; user picks the first remediation wave. Do not dispatch eng until that generation is decision-complete. |  |  |
