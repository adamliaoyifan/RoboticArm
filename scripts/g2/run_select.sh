#!/usr/bin/env bash
# usage: run_select.sh <rel_out_dir> <label>   (relative to $G2_EVID)
source /tmp/elfin-master-pack-baseline-g2/scripts/g2/g2_env.sh
export ROS_LOCALHOST_ONLY=1
rm -rf "$G2_EVID/$1"
cd /tmp
python3 $G2_ROOT/scripts/g2/g2_frozen_select.py "$G2_EVID/$1" "$2" > "/tmp/g2_$2.out" 2>&1
echo rc=$?
