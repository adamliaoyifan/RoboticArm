# Robot Assets

This directory contains source assets retained from the ROS 1 Elfin-S package
for the ROS 2 migration.

`elfin_description/` contains the S05, S10, S20, and S30 URDF/Xacro geometry,
STL meshes, materials, and Gazebo visual properties. It is intentionally kept
outside `src/` until a ROS 2 description package is created around these
assets. ROS 1 launch files, Catkin metadata, transmissions, controllers,
EtherCAT drivers, and MoveIt 1 configuration were removed from the Humble
workspace.
