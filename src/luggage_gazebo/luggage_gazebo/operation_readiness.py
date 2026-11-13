"""Bounded readiness predicates; sensor publication precedes controller activation."""
import time


def wait_static_scene(read, required_ids, *, deadline, clock=time.monotonic, pause=time.sleep):
    """A responsive scene service can still precede static world insertion."""
    required=set(required_ids)
    if not required:raise ValueError('EXPECTED_STATIC_OBJECTS_REQUIRED')
    while clock()<deadline:
        response=read()
        scene=response.scene
        populated={obj.id for obj in scene.world.collision_objects
                   if obj.primitives or any(mesh.triangles for mesh in obj.meshes)}
        if not scene.is_diff and required.issubset(populated):return response
        pause(min(.2,max(0.,deadline-clock())))
    raise TimeoutError('STATIC_WORLD_GEOMETRY_NOT_READY')


def wait_active_controllers(read, *, deadline, clock=time.monotonic, pause=time.sleep):
    required={'joint_state_broadcaster','elfin_arm_controller'}
    while clock()<deadline:
        controllers=read()
        active={controller.name for controller in controllers.controller if controller.state=='active'}
        if required.issubset(active):return controllers
        pause(min(.2,max(0.,deadline-clock())))
    raise TimeoutError('CONTROLLERS_NOT_ACTIVE_BEFORE_READINESS_DEADLINE')


def wait_observed_detection(detect, read_domain, *, deadline,
                            clock=time.monotonic, pause=time.sleep, after_stamp_ns=None):
    """Wait for measured geometry and its exact live domain, without relaxing either.

    Detection may succeed before support/height is observable after spawning.
    A domain from a later acquisition cannot repair that incomplete detection.
    This bounded diagnostic client retries observation only, never planning.
    """
    if after_stamp_ns is not None and (type(after_stamp_ns) is not int or after_stamp_ns < 0):
        raise ValueError('EXPLICIT_ACQUISITION_LOWER_BOUND_REQUIRED')
    while clock() < deadline:
        domain = read_domain()
        if domain is not None and domain.valid:
            response = detect(max(0., deadline-clock()))
            if response.success and response.luggage:
                domain = read_domain()
                for detection in response.luggage:
                    if not (detection.height_valid and detection.top_surface_valid
                        and detection.height_source == detection.HEIGHT_SOURCE_MEASURED_SUPPORT
                        and domain is not None and domain.valid
                        and domain.box_id == detection.id
                        and domain.header == detection.header):
                        continue
                    if after_stamp_ns is not None:
                        stamp = domain.header.stamp
                        if stamp.sec*10**9+stamp.nanosec <= after_stamp_ns:
                            continue
                    return detection, domain
        pause(min(.1, max(0., deadline-clock())))
    raise TimeoutError('COMPLETE_MATCHING_OBSERVATION_UNAVAILABLE')
