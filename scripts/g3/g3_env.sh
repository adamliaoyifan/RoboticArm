# Source me: clean G3 environment (no foreign PYTHONPATH).
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /home/adamliao/work/elfin_humble_ws/install/setup.bash
source ${G3_INSTALL_ROOT:-/tmp/elfin-master-pack-baseline-g3}/install/setup.bash
export G3_ROOT=/tmp/elfin-master-pack-baseline-g3
export G3_EVID=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/master_pack_recovery/2026-10-14_g3
export G1_CAPTURE=/home/adamliao/work/elfin_humble_ws/docs/status/evidence/master_pack_recovery/2026-10-13_g1/capture
