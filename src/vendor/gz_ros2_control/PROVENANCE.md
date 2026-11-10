# PROVENANCE — vendored gz_ros2_control (instrumented)

## Upstream identity

- Package: `gz_ros2_control` version **0.7.20**
- Source repo: https://github.com/ros-controls/gz_ros2_control
- Tag `0.7.20` → commit **dd35b604a6a60c25da4d4c3838880dea2a24108e**
  (2026-05-13). Only the `gz_ros2_control/` package directory is vendored
  (demos/tests/metapackage shims excluded).
- Installed deb being shadowed: `ros-humble-gz-ros2-control
  0.7.20-1jammy.20260804.212907` (`/opt/ros/humble`).

**A tag name does not prove the deb was built from exactly this source** —
the deb may carry distro patches and different build options. Therefore
(acceptance T1): this file records source repo + commit + the exact patch;
the runtime evidence directory must record the sha256 of the `.so` files
actually loaded (offline sealing step), not just this provenance.

## Local modifications (the entire delta vs upstream)

1. `src/gz_system.cpp`
   - `#include "joint_command_capture/recorder.hpp"` (+ `<chrono>`).
   - `GazeboSimSystemPrivate`: two instrumentation counters
     (`jcc_write_seq`, `jcc_read_seq`).
   - `initSim`: configure `Recorder::instance()` from `JOINT_CAPTURE_*`
     env once per process; logs mode/faults. Refused config is logged,
     never fatal.
   - `write()`: observation-only capture per call (~1 kHz, called from
     PreUpdate every physics step): per joint `cmd_received` (command
     interface value = what the controller wrote),
     `position_used` (state used by the P conversion), `write_applied`
     (target_vel / vel / effort / 0), branch (VELOCITY / POSITION_PV /
     EFFORT / FALLBACK_ZERO_VEL / SKIP_NULL_ENTITY / NO_METHOD) and
     outcome (CREATED / WRITTEN / RETAINED_EMPTY / SKIPPED). The
     `else if (!vel->Data().empty())` retained-previous-command case is
     now an explicit else recording RETAINED_EMPTY (no upstream behavior
     change; upstream fell through identically).
   - `read()`: per call (100 Hz, gated in PostUpdate): position/velocity/
     effort (transmitted-wrench scalar — valid in sim even though the
     URDF exports no effort state interface) + skip flags; skipped joints
     keep NaN.
   - `time` parameters named (they were unnamed `/*time*/`).
2. `src/gz_ros2_control_plugin.cpp`
   - `#include` of the recorder.
   - `PostUpdate` gated control cycle: one `IterationRecord` per cycle
     (gz `iterations` ↔ sim time mapping, dt, paused) + PAUSE_CHANGE
     events.
3. `CMakeLists.txt` / `package.xml`: find/link/depend on
   `joint_command_capture` (recorder core; ROS/Gazebo-free).

**Not modified**: the mimic-joint write() loop (elfin URDF has no mimic
joints; left byte-identical to upstream), all control computations, all
parameters, all ECM writes.

## Reproducing pristine → patched

    git clone --branch 0.7.20 --depth 1 https://github.com/ros-controls/gz_ros2_control.git
    # (tag 0.7.20 == dd35b604a6a60c25da4d4c3838880dea2a24108e)
    diff -ru gz_ros2_control/gz_ros2_control <this directory>   # == the delta above

The generated diff is also stored at `patches/01_instrumentation.patch`
(regenerate with the command above if it drifts).

## Build options

Standard ament_cmake, no extra flags; built in this worktree with
`colcon build --packages-select joint_command_capture gz_ros2_control`
(humble, Fortress branch: `ignition-gazebo6` path).

## Patch 02: param-delivery fix (2026-10-22, settle campaign)

`gz_ros2_control_plugin.cpp` Configure(): the plugin's own node
("gz_ros2_control") is now created WITH the SDF `<parameters>` argument
list (params files + remaps; robot_description appended afterwards as
upstream). In stock 0.7.20 the node is created bare BEFORE the parsed
params file is assigned to the rcl context, so the yaml's
`gz_ros2_control:` section never reached this node —
`position_proportional_gain` ran at the compiled default 0.1 in every
recorded boot (runtime log line "The position_proportional_gain has been
set to: 0.1") while elfin_control/config/elfin_controllers_sim.yaml has
declared 0.8 since the workspace baseline (comment there documents 0.1
as insufficient under camera+panel load). The controller_manager and
everything downstream keep reading the context global arguments exactly
as upstream; the ONLY behavior change is this node honoring its own yaml
section. Evidence: docs/status/evidence/occ_owner_b1/gate3_aprime/
PHASE1_DIAG_BUDGET_20261022.md (hold-window boundary verdict
physical_response at gain 0.1: frozen cmd, faithful −0.1×rate×err
conversion, joints delivering ~1% of commanded velocity and drifting
0.12/0.27 rad per 10 s under load).

Patched-build library hashes (supersede the 01-era values above):
- libign_ros2_control-system.so  5336ee0e2eb05f8be7506268ac2b9ef8596127d147a7b34f3022f6bae00a7c6c
