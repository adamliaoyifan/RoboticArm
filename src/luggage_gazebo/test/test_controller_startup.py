"""Actual launch event handlers: failure never advances the startup chain."""
from launch import LaunchContext
from launch.actions import EmitEvent,LogInfo,RegisterEventHandler
from launch.events.process import ProcessExited
from launch_ros.actions import Node
from luggage_gazebo.controller_startup import bounded_controller_startup


def exit_step(actions,code):
    node=next(x for x in actions if isinstance(x,Node))
    handler=next(x.event_handler for x in actions if isinstance(x,RegisterEventHandler))
    event=ProcessExited(action=node,name='controlled-spawner',cmd=[],cwd=None,env=None,pid=1,returncode=code)
    assert handler.matches(event)
    return list(handler.handle(event,LaunchContext()))


def test_successful_controllers_alone_release_following_actions():
    after=[LogInfo(msg='after_arm')]
    arm=exit_step(bounded_controller_startup(after),0)
    assert any(isinstance(x,Node) for x in arm)
    assert exit_step(arm,0)==after


def test_lost_load_reply_uses_fresh_standard_spawner_then_actual_success():
    after=[LogInfo(msg='after_arm')]
    initial=bounded_controller_startup(after);retry=exit_step(initial,1)
    assert next(x for x in initial if isinstance(x,Node)) is not next(x for x in retry if isinstance(x,Node))
    assert exit_step(exit_step(retry,0),0)==after


def test_repeated_broadcaster_failure_only_requests_shutdown():
    result=exit_step(exit_step(bounded_controller_startup([LogInfo(msg='must_not_start')]),1),1)
    assert len(result)==1 and isinstance(result[0],EmitEvent)


def test_repeated_arm_failure_cannot_start_moveit():
    arm=exit_step(bounded_controller_startup([LogInfo(msg='must_not_start')]),0)
    result=exit_step(exit_step(arm,1),1)
    assert len(result)==1 and isinstance(result[0],EmitEvent)
