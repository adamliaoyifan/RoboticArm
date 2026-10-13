# E.1 deploy profile smoke record (2026-10-03)

    export ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
    ros2 launch src/luggage_planning/launch/deploy_asset_stack.launch.py

Nodes up: robot_state_publisher (hardware-flavour description),
static_joint_state_publisher, move_group (plan-only), scene_manager.
Zero process deaths. scene_manager auto-synced the container mesh
(airport_container_real, 8000 triangles) against the DEPLOY scene.

Plan-only probe through the deploy scene:
/check_state_validity at q=[0.936,-1.025,1.268,-1.819,-1.567,-0.635]
-> valid: True (tool-down mid-workspace state, collision-free against
the mirrored deploy container).

Scope: asset/profile smoke only (nodes up, scene synced, plan-only
validity answers). The full deployment-scene place chain is S5 scope
(scene_tf_deploy mirrored cell: container [-1.5,+0.6] yaw pi, pickup
platform [+1,0]; pickup_observe re-solve pending site measurement).
