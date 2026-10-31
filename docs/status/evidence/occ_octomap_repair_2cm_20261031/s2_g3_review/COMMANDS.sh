#!/usr/bin/env bash
set -euo pipefail
cd /home/adamliao/work/elfin_humble_ws
python3 -m pytest -q --tb=line /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_s2_g3_offline.py
# Expected exit 1 on d653c9c8: four false passes.
python3 docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g3_review/audit.py
