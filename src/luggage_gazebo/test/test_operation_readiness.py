from types import SimpleNamespace
import pytest
from luggage_gazebo.operation_readiness import wait_active_controllers


def scene(objects=(),diff=False):
    return SimpleNamespace(scene=SimpleNamespace(is_diff=diff,world=SimpleNamespace(collision_objects=[
        SimpleNamespace(id=name,primitives=[object()] if shape else [],meshes=[])
        for name,shape in objects])))


def test_responsive_empty_scene_is_not_static_world_readiness():
    from luggage_gazebo.operation_readiness import wait_static_scene
    samples=iter([scene(),scene([('uld',True)]),scene([('uld',True),('pedestal',True)])])
    now=[0.]
    result=wait_static_scene(lambda:next(samples),{'uld','pedestal'},deadline=1.,clock=lambda:now[0],
        pause=lambda dt:now.__setitem__(0,now[0]+dt))
    assert len(result.scene.world.collision_objects)==2 and now[0]==.4


@pytest.mark.parametrize('sample',[scene(),scene([('uld',False)]),scene([('uld',True)],diff=True)])
def test_missing_geometry_or_scene_diff_cannot_authorize_motion(sample):
    from luggage_gazebo.operation_readiness import wait_static_scene
    now=[0.]
    with pytest.raises(TimeoutError,match='STATIC_WORLD_GEOMETRY_NOT_READY'):
        wait_static_scene(lambda:sample,{'uld'},deadline=.5,clock=lambda:now[0],
            pause=lambda dt:now.__setitem__(0,now[0]+dt))
    assert now[0]==.5


def states(arm, broadcaster='active'):
    return SimpleNamespace(controller=[SimpleNamespace(name='joint_state_broadcaster',state=broadcaster),
        SimpleNamespace(name='elfin_arm_controller',state=arm)])


def test_sensor_early_controller_inactive_is_not_terminal_failure():
    samples=iter([states('unconfigured'),states('inactive'),states('active')]);now=[0.]
    result=wait_active_controllers(lambda:next(samples),deadline=1.,clock=lambda:now[0],
        pause=lambda seconds:now.__setitem__(0,now[0]+seconds))
    assert result.controller[1].state=='active' and now[0]==.4


@pytest.mark.parametrize('sample',[states('inactive'),states('active','inactive'),SimpleNamespace(controller=[])])
def test_neither_half_ready_nor_missing_controllers_authorize_cycle(sample):
    now=[0.]
    with pytest.raises(TimeoutError):
        wait_active_controllers(lambda:sample,deadline=.5,clock=lambda:now[0],
            pause=lambda seconds:now.__setitem__(0,now[0]+seconds))
    assert now[0]==.5


def observation(stamp=1, valid=True, height=True, source=1):
    header=SimpleNamespace(stamp=stamp,frame_id='world')
    domain=SimpleNamespace(valid=valid,box_id='measured',header=header)
    detection=SimpleNamespace(id='measured',header=header,height_valid=height,
        top_surface_valid=True,height_source=source,HEIGHT_SOURCE_MEASURED_SUPPORT=1)
    return SimpleNamespace(success=True,luggage=[detection]),domain


def test_incomplete_height_does_not_borrow_later_valid_domain():
    from luggage_gazebo.operation_readiness import wait_observed_detection
    early,domain=observation(height=False);complete,_=observation()
    responses=iter([early,complete]);now=[0.]
    detection,returned=wait_observed_detection(lambda _:next(responses),lambda:domain,
        deadline=1.,clock=lambda:now[0],pause=lambda dt:now.__setitem__(0,now[0]+dt))
    assert detection.height_valid and returned is domain and now[0]==.1


@pytest.mark.parametrize('kind',['stamp','frame','height','source','invalid','missing'])
def test_observation_join_refuses_incomplete_or_different_acquisition(kind):
    from luggage_gazebo.operation_readiness import wait_observed_detection
    response,domain=observation();now=[0.]
    response.luggage[0].header=SimpleNamespace(stamp=1,frame_id='world')
    if kind=='stamp':domain.header.stamp=2
    if kind=='frame':domain.header.frame_id='camera'
    if kind=='height':response.luggage[0].height_valid=False
    if kind=='source':response.luggage[0].height_source=0
    if kind=='invalid':domain.valid=False
    if kind=='missing':domain=None
    with pytest.raises(TimeoutError,match='COMPLETE_MATCHING_OBSERVATION_UNAVAILABLE'):
        wait_observed_detection(lambda _:response,lambda:domain,deadline=.5,
            clock=lambda:now[0],pause=lambda dt:now.__setitem__(0,now[0]+dt))
    assert now[0]==.5
