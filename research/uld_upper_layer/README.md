# Independent upper-layer placement study

This is an offline prototype for upward suction-panel carry, transfer onto a
filled support surface under a ULD ceiling, withdrawal, and rear-face push.
It has no production imports of downward-pick gates or atlas queries and
does not send robot commands. Its geometry comes from the source worktree.

The first measured result is in [RESULTS.md](RESULTS.md): 168/864 parameter
combinations yielded fixed-tool geometry candidates; full-arm paths and
contact physics remain unvalidated.

Run in a shell with ROS Humble Python dependencies available:

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s research/uld_upper_layer/tests -v
python3 research/uld_upper_layer/run.py --repo . --output /absolute/new/evidence-directory
```

`assets.py` expands source xacro and maps source meshes explicitly.
`geometry.py` owns conservative oriented-box collision primitives.
`action.py` generates contact-mode states and checks fixed-tool geometry.
`run.py` captures the fixed 864-case matrix, bounded stage IK and diagrams.

The scope and thresholds are frozen in
`docs/plans/2026-11-10_uld-upper-layer-g1.md`. A geometry candidate means only
that the modeled fixed end-effector passes sampled geometric checks. Stage
IK omits full-arm collision checks; a set of IK solutions is not a connected
path. Pickup/inversion outside the container, transition rotation during
reposition, unknown actual COM, suction peeling, support compliance, friction,
force control, sensor occlusion and live performance remain not_evaluated.

The support is a declared synthetic prism, not measured occupancy. Catalog
dimensions are legal experiment inputs only and must never replace live
perception in a later production action. The eventual caller must establish
a measured upper slot and action-specific feasibility; generic BIN_FULL is
not evidence that such a slot exists.
