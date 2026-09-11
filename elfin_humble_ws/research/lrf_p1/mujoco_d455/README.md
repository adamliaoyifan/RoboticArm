# LRF-P1 MuJoCo D455 scene

Research-only camera simulator. It does **not** replace Gazebo and is not
wired into production launch files.

Isaac Lab is not installed here. This spike uses MuJoCo 3 with EGL rendering
on the local NVIDIA GPU.

## What the scene is

- One suitcase STL sitting on a table (`z = 0`)
- One Intel RealSense **D455** pinhole RGB-D camera (IMU site present, unused)
- Optional occluder panel
- **Single depth frame** per case, back-projected to world XYZ

Not a lidar scan, not a ROS bag, not time-series, not stereo matching.

## Command

```bash
python3 -m pip install --user 'mujoco>=3.2'
cd /home/adamliao/work/elfin_humble_ws_eng_lrfp1
export MUJOCO_GL=egl
export PYTHONPATH="$PWD:$PWD/src/luggage_description:$PWD/src/luggage_perception"
python3 research/lrf_p1/tests/test_mujoco_d455.py
python3 research/lrf_p1/mujoco_d455/run_validate.py --workspace "$PWD" \
  --out /tmp/lrf_p1_mujoco_d455 --open
```

`--learned d455` (default) trains the residual on MuJoCo D455 loafbrr views.
`--learned stl` is the old mismatched STL-dropout model.
`--learned none` is baseline only.

Open ` /tmp/lrf_p1_mujoco_d455/index.html ` for RGB, depth, and box overlay.
