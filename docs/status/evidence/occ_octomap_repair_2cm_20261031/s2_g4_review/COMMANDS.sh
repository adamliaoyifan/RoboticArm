#!/usr/bin/env bash
set -euo pipefail
cd /home/adamliao/work/elfin_humble_ws
python3 -m pytest -q --tb=line /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_s2_g3_offline.py /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_s2_g4_offline.py -k 'not materialize_and_live'
# Expected exit 1 on 7537b033: rejected and unknown windows become healthy.
python3 docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_g4_review/audit.py
