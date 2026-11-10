"""occ_compat — support package for scripts/occ_consumer_compat.py (T1).

Layout:
  cli.py       subcommand dispatch (thin)
  inventory.py freeze consumers/interfaces/policy/fixtures + EXPECTED
  ledger.py    event ledger (schema, join keys, atomic append)
  chains.py    real-node chain orchestration (G mapper / O observer+bridge
               / C1 planner) via sourced installs, one backend per graph
  v1_states.py K01-K10 executors
  v2_races.py  12 deterministic race cases (barriers + stage hooks)
  oracle.py    independent candidate oracle (full-orientation volumes)
  judge.py     A0-A5 gates recomputed from disk + 12 mutation families
  replaycmd.py archived-case replay

Clock discipline: /clock (sim time) drives only the fixture scenario
time; every deadline, rehearsal budget and matrix wall clock here uses
CLOCK_MONOTONIC in real time (contract §7).
"""
__all__ = ["cli", "inventory", "ledger", "chains", "v1_states", "v2_races",
           "oracle", "judge", "replaycmd"]
