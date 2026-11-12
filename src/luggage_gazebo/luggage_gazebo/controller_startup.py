"""Serial standard spawners with one state-aware transport recovery.

A lost load reply can leave a controller loaded but unconfigured. A fresh Humble
spawner queries that actual state and skips duplicate loading. Never launch arm
or MoveIt because a failed predecessor merely exited; stop after one recovery.
"""
from launch.actions import EmitEvent,LogInfo,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def bounded_controller_startup(after_arm):
    def step(name,success,attempt=1):
        node=Node(package='controller_manager',executable='spawner',output='screen',
            arguments=[name,'--controller-manager','/controller_manager',
                       '--controller-manager-timeout','60','--switch-timeout','30'])
        def exited(event,context):
            if event.returncode==0:return success()
            if attempt==1:
                return [LogInfo(msg='CONTROLLER_STARTUP_REQUERY:'+name)]+step(name,success,2)
            return [EmitEvent(event=Shutdown(reason='CONTROLLER_STARTUP_FAILED:'+name))]
        return [RegisterEventHandler(OnProcessExit(target_action=node,on_exit=exited)),node]
    return step('joint_state_broadcaster',lambda:step('elfin_arm_controller',lambda:list(after_arm)))
