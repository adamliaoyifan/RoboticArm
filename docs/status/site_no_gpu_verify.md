# 无 GPU 实机验证（除 place）

- Host: ThinkPad, no NVIDIA GPU, ROS 2 Jazzy
- Scope: sensors, preprocessor A/B, detect, 2° FJT, observe, plan-only, vacuum, sealed pick
- Out of scope: place, packing, cargo map, orchestrator, Livox in detection, full HB-1/2/3, PF-R9 D8
- Workstation `master` lineage at write time: `c086a39` plus the site-yaml / CPU-default patch in this tree
- Site historical branch: `origin/ros2_humble` = `48e034b` (nested `RoboticArm/{deployment_ws,elfin_humble_ws}`)
- Histories are unrelated. Do not `git pull` / rebase them together. Do not `checkout master` on a dirty tree.

Field evidence goes under `docs/status/evidence/site_no_gpu_verify/<run>/`, not under `docs/agents/`.

## Copy-paste for the site Cursor agent

You are the site ThinkPad (no NVIDIA). Remote is `git@github.com:adamliaoyifan/RoboticArm.git`. Workstation `master` already contains the site stack, but it has **no merge-base** with `origin/ros2_humble`. `origin` currently publishes `main` and `ros2_humble` only; **do not assume `origin/master` exists**.

Hard rules:

- Jazzy only: `source /opt/ros/jazzy/setup.bash`. Never source Humble in the same shell.
- `ROS_DOMAIN_ID=7`. `enp0s31f6` MTU 9000. No Gazebo. No zero-joint publisher.
- One CPS TCP client. `jazzy_real` and `cps_telemetry` are mutually exclusive.
- Ctrl+C stops nodes. It does **not** BlackOut / cut 48 V.
- Camera profile **640,360,15** only. Never `896x504@30`.
- IPs: arm `192.168.0.10:10003`, D555 `192.168.11.55`, Livox about `192.168.1.120`.
- Vacuum: box DO0=pump, DO1=blow-off, DI0=sealed. Never DO0 and DO1 both 1.
- Semantic: `semantic_device:=cpu`. Launch default is cuda; `hardware_pick.sh` injects cpu when `nvidia-smi` fails. YOLO is capped at `max_rate_hz:=2.0`, `cloud_max_age_sec:=8.0`.
- Preprocessor A: `src/luggage_perception/config/preprocessor_d555_live.yaml` (motion gate on, 5 ms).
- Preprocessor B: `src/luggage_perception/config/preprocessor_d555_site.yaml` (gate off, 50 ms, `use_sim_time` false).
- Numeric gates for this cell (frame rate, YOLO 2 Hz cap, RSS): [site_preprocessor_ab_accept.md](site_preprocessor_ab_accept.md).
- Do **not** pass `preprocessor_d555_replay.yaml` into live pick (`use_sim_time` true).
- Do **not** run place. Stop at sealed pick.
- Already done on this cell (smoke only): D555 HB-1/2/3, Livox overlay, pendant bags, vacuum pin map, CC600 hand-eye. Do not rerun the full HB plan.
- Never verified on this cell: sealed `hardware_pick_driver.py` cycle; master-strict preprocessor A.

First action: Phase 0 dump. Stop and report if any step fails. Do not skip to pick.

## Phase 0 — code (blocking)

From the **current** site checkout (likely `ros2_humble`, nested layout). Do not checkout `master` yet.

```bash
git rev-parse --abbrev-ref HEAD
git rev-parse HEAD
git status -sb
git log -15 --oneline
git stash list
git diff --stat
git diff --stat --cached
```

Preserve unpushed work:

```bash
# Prefer a real commit on a backup branch, not a vanishing stash.
git checkout -b site/unpushed-$(date +%Y%m%d)
git add -A
git status
# Review: do not add bags, weights, or secrets.
git commit -m "site: preserve unpushed cell tree before master verify"
git push -u origin HEAD
```

If commit is refused, send `git diff` and `git diff --cached` back instead. Compare local edits against vacuum IO, preprocessor yaml, launch paths, Livox JSON, and measured `scene_tf`. Keep measured numbers.

Then open a **second** worktree. Do not replace the `ros2_humble` tree.

`master` may not be on `origin`. If `git ls-remote --heads origin master` is empty, stop and ask the workstation to push a snapshot branch (for example `site/no-gpu-verify`) or send a `git bundle`. Do not hard-reset the dirty cell tree onto an unpublished SHA.

When a published snapshot exists:

```bash
git fetch origin
git worktree add /home/adamliao/work/RoboticArm-master origin/<snapshot-branch>
```

Build Jazzy overlays in that worktree only:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
cd /home/adamliao/work/RoboticArm-master
colcon build --symlink-install --packages-select \
  luggage_msgs luggage_description luggage_perception luggage_planning \
  elfin_description elfin_moveit_config
cd deployment_ws
colcon build --packages-select elfin_trajectory_executor
```

Pass condition: `git status` of the old tree is clean or on `site/unpushed-*`, new worktree exists, both overlays source without mixing Humble.

## Phase 1 — no servo enable

Same Jazzy shell. Person may be off e-stop until motion starts, but do not enable the arm.

1. Gate 0:

```bash
cd /home/adamliao/work/RoboticArm-master/deployment_ws
python3 scripts/check_site.py
```

Need ICMP to `192.168.0.10`.

2. D555 smoke:

```bash
source /opt/ros/jazzy/setup.bash
source /home/adamliao/work/RoboticArm-master/install/setup.bash
source /home/adamliao/work/RoboticArm-master/deployment_ws/install/setup.bash
export ROS_DOMAIN_ID=7
export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
ros2 launch elfin_trajectory_executor d555_rgbd.launch.py
```

Confirm `640x360` at about 15 Hz and aligned depth in `d555_color_optical_frame`. Stop this launch before the next exclusive driver.

3. Livox overlay (optional, does not block pick):

```bash
source /home/adamliao/work/RoboticArm-master/deployment_ws/livox_ws/env.sh
ros2 launch elfin_trajectory_executor overlay_livox_d555.launch.py
```

4. Joints without `jazzy_real` (pendant parked, ideally at observe):

```bash
ros2 launch elfin_trajectory_executor cps_telemetry.launch.py
ros2 topic echo /joint_states --once
```

Values must be non-zero. Kill telemetry before starting `jazzy_real`.

5. Preprocessor A then B, CPU semantic. Use `hardware_pick.sh` with executor off:

```bash
# Kill cps_telemetry first if it still holds CPS.
cd /home/adamliao/work/RoboticArm-master/deployment_ws
./scripts/hardware_pick.sh start_executor:=false semantic_device:=cpu
```

Second terminal:

```bash
ros2 topic echo /luggage/preprocessed/status --once
ros2 run luggage_planning hardware_pick_driver.py --detect-only --skip-observe
```

Restart the launch with profile B:

```bash
./scripts/hardware_pick.sh start_executor:=false semantic_device:=cpu \
  preprocessor_config:=/home/adamliao/work/RoboticArm-master/src/luggage_perception/config/preprocessor_d555_site.yaml
```

Pass: `DetectLuggage` succeeds; no persistent `DETECT_STALE_CLOUD`. `--skip-observe` is valid only if the arm is already at a usable observe view.

## Phase 2 — small motion (person on e-stop)

One CPS client: `jazzy_real` only.

7. Gate 1 two-degree move:

```bash
cd /home/adamliao/work/RoboticArm-master/deployment_ws
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
export PYTHONPATH=/home/adamliao/work/RoboticArm-master/third_party/huayan_python_sdk:${PYTHONPATH}
ros2 launch elfin_trajectory_executor jazzy_real.launch.py
```

Other terminal:

```bash
ros2 run elfin_trajectory_executor send_joint_trajectory --delta-deg 2 --and-back
```

Wait for `READY_FOR_NEXT`. Stop this standalone executor before `hardware_pick.sh` (it starts `jazzy_real` itself).

8–10. Observe, detect A/B, plan-only:

```bash
./scripts/hardware_pick.sh semantic_device:=cpu
```

Other terminal, same domain:

```bash
ros2 run luggage_planning hardware_pick_driver.py --detect-only
ros2 run luggage_planning hardware_pick_driver.py --plan-only
```

Repeat detect-only / plan-only with profile B (`preprocessor_config:=.../preprocessor_d555_site.yaml`). Plan-only must print pick segments and must not send `PlanMotion`.

## Phase 3 — vacuum and sealed pick (still no place)

`jazzy_real` must be the CPS owner (via `hardware_pick.sh`).

11. Vacuum IO: after attach pose or at a safe contact, `/vacuum/command` enable then disable. DI0 should rise in about 6.3 s. Never command DO0=1 and DO1=1 together.

12. Sealed pick:

```bash
ros2 run luggage_planning hardware_pick_driver.py
# holds suction; then
ros2 run luggage_planning hardware_pick_driver.py --release
```

On failure: stop, release vacuum if attached, report commit + preprocessor file + logs. Do not auto-retry into the cell.

## Evidence to send back

Record the commit, preprocessor file (A or B), and:

- Phase 0 git dump and `site/unpushed-*` URL or diff
- Gate 0 ping result
- `ros2 topic hz` for D555 colour / aligned depth
- `/joint_states` once (non-zero)
- `/luggage/preprocessed/status` once per profile
- detect log (`DETECT_STALE_CLOUD` present or not)
- FJT result / `READY_FOR_NEXT`
- vacuum DI0 rise time
- pick segment names and attach/seal outcome

## Already verified (smoke only)

See `docs/status/evidence/d555_bringup/README.md` and `docs/status/ros2_bag_site_recording.md`.

- D555 PoE `192.168.11.55`, 640x360@15, colour-aligned depth
- HB-3 mount residual about 2°
- Mid-360 overlay and pendant bag `record_site_pendant_20260911_220406`
- Vacuum pin map measured 2026-09-09
- CC600 hand-eye locked in `docs/architecture/eef_sensor_frames.md`
