"""Bounded readiness predicates; sensor publication precedes controller activation."""
import time


def wait_active_controllers(read, *, deadline, clock=time.monotonic, pause=time.sleep):
    required={'joint_state_broadcaster','elfin_arm_controller'}
    while clock()<deadline:
        controllers=read()
        active={controller.name for controller in controllers.controller if controller.state=='active'}
        if required.issubset(active):return controllers
        pause(min(.2,max(0.,deadline-clock())))
    raise TimeoutError('CONTROLLERS_NOT_ACTIVE_BEFORE_READINESS_DEADLINE')
