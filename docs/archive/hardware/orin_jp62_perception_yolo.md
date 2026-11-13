# Orin JetPack 6.2 — perception + YOLO-World

Copy this tree yourself, then bring up Mid-360 + D555 + YOLO on the Orin.
Laptop keeps CPS, scene TF, and planning. Scripts:

- Orin: `src/luggage_perception/scripts/perception_site.sh`
- Laptop ServoJ: `deployment_ws/scripts/hardware_pick_servo_j.sh`
- Previous waypoint all-in-one: `deployment_ws/scripts/hardware_pick.sh`

Topic names are unchanged (`ROS_DOMAIN_ID=7`).

JetPack 6.2 = Ubuntu 22.04, Python 3.10, CUDA 12.6. Install **ROS 2 Humble**
on Orin (not Jazzy). Use system OpenCV. Do **not** `pip install opencv-python`.

Default copy target: `hku_reconova@192.168.0.100:~/ros2_ws`

---

## 1. ThinkPad → Orin copy

Weights in this workspace are **symlinks**. `scp -r` of `luggage_perception`
alone copies broken links. Follow the real files. Do **not** copy
`vendor/deps` (ThinkPad is Python 3.12; Orin is 3.10). Do not copy bags or
`install/`.

```bash
ORIN=hku_reconova@192.168.0.100
REMOTE=~/ros2_ws
SRC=/home/adamliao/work/RoboticArm-master
WT=/home/adamliao/work/RoboticArm/elfin_humble_ws/src/luggage_perception

ssh "$ORIN" "mkdir -p $REMOTE/src/luggage_perception/vendor"

rsync -aH --copy-links --info=progress2 \
  --exclude 'vendor/deps/' \
  "$SRC/src/luggage_msgs" \
  "$SRC/src/luggage_description" \
  "$SRC/src/luggage_perception" \
  "$SRC/deployment_ws/src/elfin_trajectory_executor" \
  "$ORIN:$REMOTE/src/"

ssh "$ORIN" "mkdir -p $REMOTE/config"
scp "$SRC/deployment_ws/config/MID360s_config.json" \
  "$ORIN:$REMOTE/config/MID360s_config.json"

scp "$WT/yolov8s-world.pt" \
  "$ORIN:$REMOTE/src/luggage_perception/yolov8s-world.pt"
scp -r "$WT/vendor/clip_pkg" "$WT/vendor/clip_models" \
  "$ORIN:$REMOTE/src/luggage_perception/vendor/"
```

`elfin_trajectory_executor` is only for `d555_rgbd` + `d555_host_stamp`. Do **not** run `trajectory_executor` on Orin (one CPS owner on the laptop).

Equivalent `scp` only (after `mkdir` as above):

```bash
scp -r \
  "$SRC/src/luggage_msgs" \
  "$SRC/src/luggage_description" \
  "$SRC/src/luggage_perception" \
  "$SRC/deployment_ws/src/elfin_trajectory_executor" \
  "$ORIN:$REMOTE/src/"
scp "$SRC/deployment_ws/config/MID360s_config.json" \
  "$ORIN:$REMOTE/config/MID360s_config.json"
scp "$WT/yolov8s-world.pt" \
  "$ORIN:$REMOTE/src/luggage_perception/yolov8s-world.pt"
scp -r "$WT/vendor/clip_pkg" "$WT/vendor/clip_models" \
  "$ORIN:$REMOTE/src/luggage_perception/vendor/"
```

SSH from this ThinkPad needs the Orin to accept:

```
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIFElx3vWfObgtdLsuxIMUMXojR55Thop/zx2BSuYpjPJ adamliao@adamliao-ThinkPad-T14p-Gen-3
```

On Orin:

```bash
mkdir -p ~/.ssh && chmod 700 ~/.ssh
echo 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIFElx3vWfObgtdLsuxIMUMXojR55Thop/zx2BSuYpjPJ adamliao@adamliao-ThinkPad-T14p-Gen-3' >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
```

Or from the ThinkPad: `ssh-copy-id -i ~/.ssh/id_ed25519.pub hku_reconova@192.168.0.100`

Check on Orin:

```bash
ls -lh ~/ros2_ws/src/luggage_perception/yolov8s-world.pt
ls -lh ~/ros2_ws/src/luggage_perception/vendor/clip_models/ViT-B-32.pt
# expect ~26M and ~338M files, not dangling symlinks
```

---

## 2. Orin: JetPack / OpenCV

```bash
cat /etc/nv_tegra_release
uname -m                    # aarch64
nvcc --version              # CUDA 12.6
python3 -c "import cv2, sys; print(sys.version, cv2.__version__, cv2.__file__)"
```

If `import cv2` already works, leave it. Orin Nano 8 GB: add 16 G swap before torch.

---

## 3. Orin: ROS 2 Humble

```bash
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros \
  python3-colcon-common-extensions \
  python3-pip python3-numpy python3-yaml python3-opencv \
  ros-humble-realsense2-camera
```

---

## 4. Orin: PyTorch (Jetson wheel, not PyPI)

```bash
python3 -m pip install --user -U pip
python3 -m pip uninstall -y torch torchvision torchaudio opencv-python opencv-python-headless || true
python3 -m pip install --user 'numpy<2'
python3 -m pip install --user torch==2.8.0 torchvision==0.23.0 \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

python3 - <<'PY'
import torch
print("torch", torch.__version__, "cuda", torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NO CUDA")
assert torch.cuda.is_available(), "CPU wheel; retry jp6/cu126 index"
PY
```

Do not `pip install torch` from PyPI (CPU aarch64 wheel).

---

## 5. Orin: YOLO-World + CLIP (keep system OpenCV)

```bash
python3 -m pip install --user ftfy regex
python3 -m pip install --user --no-deps ultralytics
python3 -m pip install --user matplotlib pandas seaborn pillow tqdm pyyaml psutil requests
python3 -m pip install --user --target=$HOME/ros2_ws/src/luggage_perception/vendor/deps \
  ftfy regex

python3 - <<'PY'
from ultralytics import YOLOWorld
m = YOLOWorld("/home/hku_reconova/ros2_ws/src/luggage_perception/yolov8s-world.pt")
m.to("cuda")
m.set_classes(["luggage", "box"])
print("YOLO-World cuda ok")
PY
```

---

## 6. Orin: colcon

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  luggage_msgs luggage_description luggage_perception \
  elfin_trajectory_executor
```

Build `livox_ros_driver2` on Orin as well (same overlay as
`deployment_ws/scripts/setup_livox_driver.sh`, or a Humble livox_ws under
`~/ros2_ws/livox_ws`). `perception_site.sh` sources that overlay when present.
Copy `~/ros2_ws/config/MID360s_config.json` host_ip to the Orin NIC.

---

## 7. Laptop: previous waypoint test vs ServoJ

Previous all-in-one waypoint pick (local D555 + YOLO) is unchanged:

```bash
cd /home/adamliao/work/RoboticArm-master/deployment_ws
./scripts/hardware_pick.sh
```

Split: stop that graph, then ServoJ on the laptop with Orin owning sensors:

```bash
cd /home/adamliao/work/RoboticArm-master/deployment_ws
./scripts/hardware_pick_servo_j.sh
```

`hardware_pick_servo_j.sh` injects `execution_backend:=servo_j`,
`start_d555:=false`, `start_perception:=false`. Do not also run local YOLO.

Laptop-only ServoJ (no Orin): `./scripts/hardware_pick_servo_j.sh start_d555:=true start_perception:=true`

### Laptop subscribe (same names, DDS)

Orin publishes; laptop does not remap:

| Role | Name |
|---|---|
| D555 | `/camera/d555/color/image_raw/compressed`, `/camera/d555/color/camera_info` |
| D555 depth | `/camera/d555/aligned_depth_to_color/image_raw/compressed`, `/camera/d555/aligned_depth_to_color/camera_info` |
| Mid-360 | `/livox/lidar`, `/livox/imu` |
| Preprocess | `/luggage/preprocessed/camera/color/image`, `.../depth/image`, camera_info, `/luggage/preprocessed/status` |
| YOLO | `/luggage/semantic/yolo_detections`, `/luggage/semantic/overlay`, `/luggage/semantic/mask` |
| Clouds | `/luggage/semantic/cargo_points`, `/luggage/semantic/obstacle_points` |
| Detect | service `/luggage_detector/detect_luggage` |

Laptop still publishes `/joint_states`, `/tf`, `/tf_static` for Orin.

---

## 8. Orin: run perception (one script)

```bash
chmod +x ~/ros2_ws/src/luggage_perception/scripts/perception_site.sh
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
export LIVOX_WS=$HOME/ros2_ws/livox_ws
export MID360_CONFIG=$HOME/ros2_ws/config/MID360s_config.json
~/ros2_ws/src/luggage_perception/scripts/perception_site.sh
```

Ready when the segmenter logs `backend=bbox_fill:...yolov8s-world.pt` on cuda.

---

## 9. Check (either machine)

```bash
export ROS_DOMAIN_ID=7
unset ROS_LOCALHOST_ONLY
ros2 node list | grep -E 'semantic|preprocessor|luggage_detector|camera.d555|livox|trajectory_executor'
ros2 topic hz /livox/lidar --window 10
ros2 topic hz /camera/d555/color/image_raw/compressed --window 10
ros2 topic hz /luggage/preprocessed/camera/color/image --window 10
ros2 topic hz /luggage/semantic/overlay --window 5
ros2 topic echo /luggage/semantic/yolo_detections --once
```

Pick driver stays on the laptop (`observe=current`). DetectLuggage is served from Orin.

---

## Pitfalls

| Symptom | Fix |
|---|---|
| `torch.cuda.is_available() == False` | Uninstall PyPI torch; use `pypi.jetson-ai-lab.io/jp6/cu126` |
| `cv2` breaks after pip | Uninstall `opencv-python`; use apt `python3-opencv` |
| CLIP import error | Do not copy ThinkPad `vendor/deps`; pip `--target=.../vendor/deps ftfy regex` on Orin |
| Orin sees no `/camera/d555/...` | Same `ROS_DOMAIN_ID=7`, `unset ROS_LOCALHOST_ONLY`, same LAN; D555 plugged to Orin |
| Duplicate YOLO / D555 | Laptop must use `hardware_pick_servo_j.sh` or `start_d555:=false start_perception:=false` |
| Humble vs Jazzy msgs | Build **this** `luggage_msgs` on both. If DetectLuggage is broken, keep detector on the laptop and only YOLO+filter on Orin |
| Tiny `.pt` files on Orin | Symlinks were copied; redo the `scp` of `$WT/yolov8s-world.pt` and `clip_models` |
| Two CPS clients | Do not launch `trajectory_executor` on Orin |
