# Gazebo scene geometry

`sim_world.launch.py` spawns the pickup platform using the selected
`scene_tf_config` dimensions. `luggage_gazebo.platform_model.platform_sdf`
copies the stock SDF and sets both visual and collision box sizes and their
bottom-centred poses before passing XML to `ros_gz_sim create -string`.
The stock model is a template, not an independent source of platform height.
Platform-top TF and pickup box spawning use the same scene configuration.

This matters because pickup boxes are static until release: a box spawned
above a shorter stock platform will not settle onto it. Measured top-minus-
support height then includes the air gap. Do not compensate by feeding scene
height or reference box dimensions into perception.

Behavioural regression: `test/test_platform_model.py` checks visual/collision
top agreement with platform TF at two sizes, invalid dimensions, and the actual
launch action's generated model arguments. Live depth acquisition is still
required to verify measured geometry after changes to simulation assets.
