#!/usr/bin/env bash
set -euo pipefail
cd /home/adamliao/work/elfin_humble_ws
# Expected exit 1 on 16dda822 / evidence HEAD 2d42aa6a.
python3 docs/status/evidence/occ_octomap_repair_2cm_20261031/s2_review/audit.py
