# PF-R5B closeout — exact commands and profile

## Environment

- Isolated clean git worktree: `git worktree add /tmp/pfr5b_clean a3dba5e`
  (revision `a3dba5e7e0c2fb219062cfb2a091267f6f2d606d`, dirty=0,
  recorded in `pfr5b_evidence.json:revision`)
- Build: `cd /tmp/pfr5b_clean && source /opt/ros/humble/setup.bash && colcon build
  --packages-select elfin_control elfin_description elfin_moveit_config
  luggage_msgs luggage_description luggage_perception luggage_planning
  luggage_packing luggage_gazebo luggage_bringup`
- Model weights/vendor seeded from the primary workspace (build artifacts,
  not tracked): `yolov8s-world.pt` into the worktree cwd and the CLIP
  vendor tree into `install/luggage_perception/share/.../vendor`.

## Semantic stack (audits, no-box control, positive smokes)

```bash
source /opt/ros/humble/setup.bash && source /tmp/pfr5b_clean/install/setup.bash
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=true use_motion:=true use_vacuum:=false visual_kind:=mesh \
  size_mode:=catalog sequence_ids:=carryon,standard,large \
  observe_pose_name:=pickup_observe
```

Evidence collection (parameter dumps, AST+runtime truth audits, no-box
control, three-tier positive smokes):

```bash
python3 /tmp/pfr5b_evidence.py <evidence_dir> /tmp/pfr5b_clean
```

## Raw-only stack (negative control)

```bash
source /opt/ros/humble/setup.bash && source /tmp/pfr5b_clean/install/setup.bash
ros2 launch luggage_gazebo sim_world.launch.py gui:=false use_rviz:=false \
  use_semantic:=false use_motion:=false use_vacuum:=false visual_kind:=mesh \
  size_mode:=catalog sequence_ids:=standard observe_pose_name:=pickup_observe

python3 scripts/platform_free_height_gate4_eval.py \
  --out <evidence_dir>/raw_only_negative_control \
  --trials 3 --settle-sec 4.0 --negative-control-raw-only \
  --launch-params "<the raw launch line above; worktree a3dba5e>"
```

## Teardown

```bash
scripts/stop_sim.sh
# residual counted by comm name (ign|gz|ruby|ros2|parameter_bridge):
ps -eo comm= | grep -cE "^(ign|gz|ruby|ros2|parameter_bridge)"  # -> 0
```

## Rerun-boundary comparison (against run8-era c5921d5)

```bash
git diff --stat c5921d5..a3dba5e -- scripts/platform_free_height_gate4_eval.py \
  src/luggage_perception/luggage_perception/eval/gate4_scoring.py      # empty
git diff --stat c5921d5..a3dba5e -- src/luggage_perception/scripts/ ...  # empty
(cd src/luggage_description && python3 -m pytest test/test_pf_r5a_gt_fail_closed.py -q)
```

Recorded in `rerun_boundary_check.md`.
