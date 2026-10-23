#!/usr/bin/env bash
# [Phase-2 functional n=3, 2026-10-22] ONE n=3 run under the SAME
# functional-diagnostic config as phase 1 (180 s per-Select budget
# reaching the searcher). Frozen version = post-fix tree
# (ac2a92d3: driver --scene-tf-config, plugin param-delivery 0.8).
# Stack: production (diagnostic_capture:=false — no capture routing
# hops), joint_capture window kept (plugin-side only, +1.3% measured).
# No skipped boxes, no manual placement, no lowered gates; prior OFF
# (reported separately as NOT adopted). Pass = "relaxed-budget n=3
# functional closed-loop pass" ONLY.
set -o pipefail
unset AMENT_TRACE_SETUP_FILES

WS="/tmp/elfin-master-pack-baseline-g3"
MAIN="/home/adamliao/work/elfin_humble_ws"
OUT="${PHASE2_RUN_OUT:-$WS/docs/status/evidence/occ_owner_b1/attachment_geometry_g4_20261022/live_$(date +%Y%m%d_%H%M%S)}"
RUN_ID="n3func_$(date +%H%M%S)"
DEADLINE=$(( $(date +%s) + 5400 ))
DIAG_ROUTE_BUDGET_S=180
mkdir -p "$OUT"
export ROS_DOMAIN_ID=7

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/runner.log"; }

# Refuse an occupied slot; never stop another agent's stack or sweep ROS.
if pgrep -f 'ros2 launch luggage_gazebo|ign gazebo|gz sim|ros_gz_bridge/[p]arameter_bridge' >/dev/null; then
  log "sim slot or residual bridge busy"; exit 2
fi
# Existing ROS peers can poison identity even when Gazebo/bridges are gone.
python3 - <<'PYPEERS'
from pathlib import Path
import os
busy = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        argv = proc.joinpath('cmdline').read_bytes().split(b'\0')
        env = proc.joinpath('environ').read_bytes().split(b'\0')
    except (OSError, ProcessLookupError):
        continue
    if b'ROS_DOMAIN_ID=7' not in env:
        continue
    if any(b'/install/' in arg and b'/lib/' in arg for arg in argv[:2]):
        busy.append((proc.name, b' '.join(argv[:2]).decode(errors='replace')))
if busy:
    raise SystemExit('live domain-7 peers: ' + repr(busy))
PYPEERS
if [[ $? -ne 0 ]]; then log "ROS peer slot busy"; exit 2; fi
LPID=""
HEARTBEAT_PID=""
cleanup() {
  local status=$?
  if [[ -n "$HEARTBEAT_PID" ]]; then
    kill -TERM "$HEARTBEAT_PID" 2>/dev/null || true
    for _ in $(seq 1 25); do
      kill -0 "$HEARTBEAT_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$HEARTBEAT_PID" 2>/dev/null; then
      kill -KILL "$HEARTBEAT_PID" 2>/dev/null || true
    fi
    wait "$HEARTBEAT_PID" 2>/dev/null || true
  fi
  if [[ -n "$LPID" ]]; then
    if [[ -d "$OUT/joint_capture/ctl" ]]; then
      : > "$OUT/joint_capture/ctl/stop"
      for _ in $(seq 1 30); do
        [[ -f "$OUT/joint_capture/manifest_recorder.json" ]] && break
        sleep 1
      done
    fi
    ELFIN_SIM_PIDFILE="$OUT/launch.pid" bash "$WS/scripts/stop_sim.sh" \
      --owned-pgid "$LPID" >> "$OUT/teardown.log" 2>&1
  fi
  log "runner exit=$status"
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT

# ---- Same A-prime scene rebind + driver scene arg (phase-1 fix) ----
export G3_CONTAINER_MODEL="airport_container_real_aprime"
SCENE="$OUT/scene_tf_used.yaml"
POSES_OUT="$OUT/robot_poses_used.yaml"
python3 - "$SCENE" "$POSES_OUT" << 'PYSCENE'
import hashlib, os, sys, yaml
cfg = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "scene_tf_deploy_update.yaml"))
cfg.setdefault("gazebo", {})["container_model"] = \
    os.environ["G3_CONTAINER_MODEL"]
yaml.safe_dump(cfg, open(sys.argv[1], "w"), sort_keys=False)
scene_md5 = hashlib.md5(open(sys.argv[1], "rb").read()).hexdigest()
poses = yaml.safe_load(open(
    "/tmp/elfin-master-pack-baseline-g3/src/luggage_description/config/"
    "robot_poses.baseline_update.yaml"))
poses.setdefault("identity", {})["scene_revision"] = scene_md5
poses["identity"]["_rebind_note"] = (
    "scene_revision re-bound for n3_functional boot (md5 %s); joint values "
    "unchanged" % scene_md5)
for _name, pose in (poses.get("poses") or {}).items():
    if isinstance(pose, dict) and pose.get("scene_revision_at_capture"):
        pose["scene_revision_at_capture"] = scene_md5
yaml.safe_dump(poses, open(sys.argv[2], "w"), sort_keys=False)
print("scene_md5", scene_md5)
PYSCENE

WEIGHTS="$MAIN/yolov8s-world.pt"
VENDOR="$MAIN/src/luggage_perception/vendor"
PYDEPS="$MAIN/pydeps-frozen"
export ELFIN_BOX_CONTACT_TOPIC=/obs/box_contact
export ELFIN_BOX_SDF_DUMP="$OUT/spawn.sdf"
export ELFIN_HANDOFF_FAILURE_CAPTURE_DIR="$OUT/handoff_failure"
export ELFIN_SELECTION_CAPTURE_DIR="$OUT/selection_boundary"
export ELFIN_PHYSICAL_ATTACHMENT_TRACE="$OUT/physical_attachment.jsonl"

set +u
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
source "$WS/install/setup.bash"
set -u
# No PYTHONPATH src prefixes — driver identity must report INSTALL overlay
# paths (src prefixes triggered WRONG_OVERLAY in the 10-22 attempts).
python3 - "$WS" "$OUT" <<'PYMANIFEST'
import datetime, hashlib, json, os, pathlib, subprocess, sys
ws, out = map(pathlib.Path, sys.argv[1:])
paths = ["scripts/pack_eval_driver.py", "scripts/place_smoke_driver.py"]
hashes = {}
for relative in paths:
    source = ws / "src/luggage_gazebo" / relative
    installed = ws / "install/luggage_gazebo/lib/luggage_gazebo" / source.name
    hashes[relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    if source.read_bytes() != installed.read_bytes():
        raise SystemExit("stale installed driver: " + str(installed))
envelope_source = ws / "src/luggage_planning/config/attachment_geometry.yaml"
envelope_installed = ws / "install/luggage_planning/share/luggage_planning/config/attachment_geometry.yaml"
if envelope_source.read_bytes() != envelope_installed.read_bytes():
    raise SystemExit("stale installed envelope config")
(out / "attachment_geometry.yaml").write_bytes(envelope_source.read_bytes())
import luggage_gazebo.driver_executor as lifecycle
manifest = dict(
    start=datetime.datetime.now().astimezone().isoformat(),
    revision=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ws, text=True).strip(),
    dirty=subprocess.check_output(["git", "status", "--porcelain"], cwd=ws, text=True).splitlines(),
    mode="non-scored phase-2 n=3 functional diagnostic", domain=7,
    selector_budget_s=180, attachment_max_age_s=60,
    prior_adopted=False, seed=0, catalog="inch20", driver_hashes=hashes,
    lifecycle_module=lifecycle.__file__,
    envelope_config_sha256=hashlib.sha256(envelope_source.read_bytes()).hexdigest(),
    acceptance="all three boxes verified, committed and map-confirmed; MAX_BOXES; exit 0; no executor runtime failure",
    aggregation="AND over three boxes in one suite; no success-rate estimate from one suite",
    diagnostic_selection_only=os.environ.get("PHASE2_SELECTION_ONLY", "0") == "1",
)
if manifest["diagnostic_selection_only"]:
    manifest["mode"] = "non-scored post-pick selection-boundary diagnostic"
    manifest["acceptance"] = "request JSON and original response CDR captured; no carry execution; never n=3 PASS"
(out / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
PYMANIFEST
if [[ $? -ne 0 ]]; then log "installed identity gate failed"; exit 6; fi

setsid ros2 launch luggage_gazebo sim_world.launch.py \
    gui:=false use_rviz:=false \
    use_semantic:=true use_motion:=true use_vacuum:=true \
    use_cargo_map:=true use_packing:=true \
    visual_kind:=mesh size_mode:=catalog \
    sequence_ids:=inch20 \
    yaw_mode:=discrete \
    random_seed:=0 \
    observe_pose_name:=pickup_observe_sim \
    frame_runtime:=true frame_driver_peer:=pack_eval_driver \
    semantic_require_backend:=bbox_fill \
    semantic_model_name:="$WEIGHTS" \
    semantic_clip_vendor_dir:="$VENDOR" \
    semantic_pydeps_dir:="$PYDEPS" \
    scene_tf_config:="$SCENE" \
    start_placement_prior:=false \
    robot_poses_config:="$POSES_OUT" \
    diagnostic_route_budget_s:=$DIAG_ROUTE_BUDGET_S \
    attachment_geometry_config:="$OUT/attachment_geometry.yaml" \
    joint_capture_mode:=window joint_capture_run_dir:="$OUT/joint_capture" \
    > "$OUT/launch.log" 2>&1 &
LPID=$!
printf "%s\n" "$LPID" > "$OUT/launch.pid"
sleep 2

if [[ -z "$LPID" ]]; then log "launch process missing"; exit 1; fi
log "launch pid=$LPID out=$OUT run_id=$RUN_ID"

# ---- Fix-verification gate: the plugin MUST report gain 0.8 now ----
GAIN_OK=""
for _ in $(seq 1 60); do
  sleep 5
  if ! kill -0 "$LPID" 2>/dev/null || grep -Eq 'startup_failed|\[ERROR\].*\[spawner-.*process has died' "$OUT/launch.log"; then
    log "startup terminal failure before gain verification"; break
  fi
  G=$(grep -m1 "position_proportional_gain has been set to:" "$OUT/launch.log" 2>/dev/null | sed 's/.*set to: //')
  if [[ -n "$G" ]]; then
    if [[ "$G" == "0.8" ]]; then GAIN_OK=1; else log "GAIN_VERIFY_FAIL runtime=$G (expected 0.8)"; fi
    break
  fi
  [ "$(date +%s)" -gt "$DEADLINE" ] && break
done
if [[ -z "$GAIN_OK" ]]; then
  log "aborting before any motion: param-delivery fix not active"
  exit 5
fi
log "gain verify OK (0.8)"

python3 "$WS/scripts/g3/attachment_preflight.py" "$OUT" > "$OUT/preflight.log" 2>&1
if [[ $? -ne 0 ]]; then log "pre-case capture/readiness gate failed"; exit 7; fi
python3 "$WS/scripts/g3/runtime_heartbeat_capture.py" "$OUT/heartbeat_capture" \
  > "$OUT/heartbeat_capture.log" 2>&1 &
HEARTBEAT_PID=$!
for _ in $(seq 1 20); do
  [[ -f "$OUT/heartbeat_capture/ready.json" ]] && break
  sleep 0.25
 done
if [[ ! -f "$OUT/heartbeat_capture/ready.json" ]]; then
  log "independent heartbeat capture not ready; no box attempted"; exit 8
fi
if [[ "${PHASE2_STARTUP_ONLY:-0}" == "1" ]]; then
  if [[ "${PHASE2_RETURN_REHEARSAL:-0}" == "1" ]]; then
    python3 "$WS/scripts/g3/return_evidence_rehearsal.py" "$OUT" \
      > "$OUT/return_rehearsal.log" 2>&1 || exit 8
  fi
  log "startup-only rehearsal complete; no box attempted"
  exit 0
fi

# ---- Driver: production n=3 + functional-diagnostic budget ----
export JOINT_CAPTURE_DIR="$OUT/joint_capture"
export JOINT_CAPTURE_MODE="window"
export JOINT_CAPTURE_POST_S="5"
log "driver start (n=3)"
driver_diagnostic_args=()
if [[ "${PHASE2_SELECTION_ONLY:-0}" == "1" ]]; then
  driver_diagnostic_args+=(--diagnostic-selection-only)
  log "selection-only diagnostic: stop after selection before carry"
fi
timeout --signal=INT 4200 ros2 run luggage_gazebo pack_eval_driver.py \
  "${driver_diagnostic_args[@]}" \
  --frame-runtime --frame-peer pack_eval_driver \
  --observe-pose pickup_observe_sim \
  --sequence-ids inch20 \
  --max-boxes 3 \
  --goto-timeout 60 \
  --plan-timeout 180 \
  --select-budget-s "$DIAG_ROUTE_BUDGET_S" \
  --scene-tf-config "$SCENE" \
  --seed 0 \
  --auth-max-attempts 8 \
  --auth-max-elapsed-sec 8 \
  --stop-on-first-failure \
  --out "$OUT/driver_out" > "$OUT/driver.log" 2>&1
DRC=$?
log "driver rc=$DRC"

# EXIT trap stops the recorder, then only this launch's process group.
exit "$DRC"
