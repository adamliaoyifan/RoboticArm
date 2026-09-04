#!/usr/bin/env bash
# Build payload-aware reachability atlases for each catalog box size, for the
# per-box-size reachability sampling study (§5.6 / L2-1).
#
# Produces, alongside the empty-load atlas:
#   s20_container_collision_aware_payload_0.55x0.40x0.25.{npz,yaml}  (carryon)
#   s20_container_collision_aware_payload_0.70x0.45x0.28.{npz,yaml}  (standard)
#   s20_container_collision_aware_payload_0.80x0.50x0.32.{npz,yaml}  (large)
#
# Compare stats.reachability_rate across the yaml files to see how the carried
# box size changes the reachable set (esp. the floor / first layer).
#
# Usage:
#   ./build_payload_atlas_sweep.sh [output_dir] [scene_tf_config]
# Requires roscore + MoveIt + /compute_ik running (run inside the noetic
# container with Gazebo/MoveIt up).
set -euo pipefail

PKG="luggage_planning"
OUTPUT_DIR="${1:-$(rospack find luggage_planning)/data/reachability_atlas}"
SCENE_TF="${2:-$(rospack find luggage_description)/config/scene_tf.yaml.example}"

# Catalog box sizes [l, w, h] (m) -- match box_catalog.yaml.example.
SIZES=(
  "0.55,0.40,0.25"   # carryon
  "0.70,0.45,0.28"   # standard
  "0.80,0.50,0.32"   # large
)

echo "==> Building empty-load atlas (baseline)"
rosrun "$PKG" reachability_atlas_builder.py \
  _scene_tf_config:="$SCENE_TF" _output_dir:="$OUTPUT_DIR" \
  _avoid_collisions:=true _resolution_xyz:=0.15 \
  _payload_enabled:=false

for SIZE in "${SIZES[@]}"; do
  IFS=',' read -r L W H <<< "$SIZE"
  OFFSET="[0.0,0.0,$(python3 -c "print(round($H/2, 4))")]"
  echo "==> Building payload atlas: size=[$L,$W,$H] offset=$OFFSET"
  rosrun "$PKG" reachability_atlas_builder.py \
    _scene_tf_config:="$SCENE_TF" _output_dir:="$OUTPUT_DIR" \
    _avoid_collisions:=true _resolution_xyz:=0.15 \
    _payload_enabled:=true \
    _payload_size:="[$L,$W,$H]" \
    _payload_offset:="$OFFSET" \
    _payload_attach_link:=suction_contact_frame
done

echo ""
echo "==> Done. reachability_rate by atlas:"
for f in "$OUTPUT_DIR"/*_container_collision_aware*.yaml; do
  [ -e "$f" ] || continue
  rate=$(grep -m1 "reachability_rate:" "$f" | awk '{print $2}')
  printf "  %-60s rate=%s\n" "$(basename "$f")" "$rate"
done
