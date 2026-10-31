#!/usr/bin/env bash
set -u
cd /home/adamliao/work/elfin_humble_ws
python3 -m pytest -q /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_repair_s0.py
python3 docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_review/reproduce.py --worktree /tmp/elfin-occ-octomap-repair-2cm-g1 --output /tmp/octomap-s0-g2-previous-five.json
# Expected exit 1 on e7b8b7a7: three false passes and three exceptions.
python3 docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_g2_review/reproduce.py --worktree /tmp/elfin-occ-octomap-repair-2cm-g1 --output /tmp/octomap-s0-g2-numeric-review.json
