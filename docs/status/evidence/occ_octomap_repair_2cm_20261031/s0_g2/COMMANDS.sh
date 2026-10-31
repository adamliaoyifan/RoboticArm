#!/bin/sh
set -eu
python3 -m pytest -q --tb=line /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_repair_s0.py
python3 /tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark/repair_s0.py write --fixtures /home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures --out /home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_octomap_repair_2cm_20261031/s0_g2
