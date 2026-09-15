# 实机代码 (site / real-cell). Not part of the Gazebo simulation stack.

Huayan CPS Python bindings (`CPS.py`) used by
`deployment_ws/src/elfin_trajectory_executor`. Place `CPS_python3_Linux.so`
on `PYTHONPATH` / `HUAYAN_SDK` on the site machine; this file is the
import shim, not a Gazebo dependency.
