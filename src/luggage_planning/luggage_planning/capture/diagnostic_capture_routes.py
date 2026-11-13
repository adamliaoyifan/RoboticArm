"""Explicit action endpoint remaps for Humble clients in diagnostic mode.

Base-name remapping alone does not route all Humble action implementations.
Keep the five underlying endpoints together; behavioural ROS tests exercise
these exact rules against ordinary client names.
"""
PREFIX = '/diagnostic_capture'
EXECUTE = '/execute_trajectory'
FJT = '/elfin_arm_controller/follow_joint_trajectory'


def action_remappings(source):
    return [(source + '/_action/' + endpoint,
             PREFIX + source + '/_action/' + endpoint)
            for endpoint in ('send_goal', 'get_result', 'cancel_goal', 'feedback', 'status')]
