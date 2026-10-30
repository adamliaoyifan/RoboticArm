#!/bin/sh
set -eu
cd /home/adamliao/work/elfin_humble_ws
python3 -m pytest -q --tb=line /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_repair_s0.py
# At bf388e31 this intentionally exits 1: five invalid cases are incorrectly accepted.
python3 docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_review/reproduce.py --worktree /tmp/elfin-occ-octomap-repair-2cm-g1 --output /tmp/octomap-s0-review-replay.json
