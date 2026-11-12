"""Compose OCC columns into a private scene copy; never publish/apply it.

The world read from GetPlanningScene contributes container/static obstacles.
OCC is authoritative for generated grid columns. Existing unrelated world
objects are retained (conservative); this module never removes them to obtain
clearance. Unknown cells remain full-height obstacles in the prior filter.
"""
import copy
import math
import hashlib
import json

from luggage_planning.placement_prior.contracts import Snapshot, Transform


COLUMN_PREFIX = 'placement-prior-occ-'


def proposal_scene_hash(scene):
    """Hash an ordering context, retaining every field except joint samples.

    Keep joint names/frame, multi-DOF base transforms, attached bodies, world,
    ACM and padding. This does not modify the message or authorize reuse of a
    collision verdict; full scene bytes and identity remain separate.
    """
    from rosidl_runtime_py.convert import message_to_ordereddict
    fields = message_to_ordereddict(scene)
    joints = fields['robot_state']['joint_state']
    joints['header'].pop('stamp')
    for key in ('position', 'velocity', 'effort'):
        joints.pop(key)
    return hashlib.sha256(json.dumps(fields, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def build_private_snapshot(confirmed_surface, baseline_scene, model_hash,
                           world_from_container, *, collision_margin_m, volume=None):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from rclpy.serialization import serialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    surface = confirmed_surface.record()
    if baseline_scene.is_diff:
        raise ValueError('full baseline scene required')
    if not math.isfinite(collision_margin_m) or collision_margin_m < 0:
        raise ValueError('explicit nonnegative collision margin required')
    if surface['frame_id'] != 'container_link':
        raise ValueError('container-frame OCC required')
    # Never mutate the captured ROS message or owner scene.
    scene = copy.deepcopy(baseline_scene)
    scene.world.collision_objects = [o for o in scene.world.collision_objects
        if not o.id.startswith(COLUMN_PREFIX)]
    floor = float(surface['floor_z_container'])
    ceiling = float(surface['ceiling_z_container'])
    resolution = float(surface['resolution_m'])
    if not all(math.isfinite(v) for v in (floor, ceiling, resolution)) or ceiling <= floor or resolution <= 0:
        raise ValueError('invalid occupancy vertical extent/resolution')
    origin = surface['origin_xy_container']
    state = surface['state']
    tops = surface['surface_z_container']
    if volume is not None:
        from luggage_planning.placement_prior.volume import blocked_volume_runs
        for index, (centre, dimensions) in enumerate(blocked_volume_runs(volume, confirmed_surface)):
            world = world_from_container.compose(Transform(centre, (0,0,0,1)))
            obj = CollisionObject(); obj.id = COLUMN_PREFIX+"voxel-"+str(index)
            obj.header.frame_id="world"; obj.operation=CollisionObject.ADD
            shape=SolidPrimitive(); shape.type=SolidPrimitive.BOX
            shape.dimensions=[v+2*collision_margin_m for v in dimensions]
            pose=Pose(); pose.position.x,pose.position.y,pose.position.z=world.translation
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=world.quaternion_xyzw
            obj.primitives=[shape]; obj.primitive_poses=[pose]
            scene.world.collision_objects.append(obj)
    else:
        for row, cells in enumerate(state):
            for column, value in enumerate(cells):
                if value == 'free':
                    continue
                if value == 'unknown':
                    top = ceiling
                elif value == 'occupied':
                    top = tops[row][column]
                    if top is None or not math.isfinite(top) or not floor < top <= ceiling:
                        raise ValueError('occupied cell without valid measured top')
                else:
                    raise ValueError('unknown occupancy state')
                centre = (origin[0]+(row+.5)*resolution,
                          origin[1]+(column+.5)*resolution, (floor+top)/2)
                world = world_from_container.compose(Transform(centre, (0,0,0,1)))
                obj = CollisionObject();obj.id=COLUMN_PREFIX+'%d-%d' % (row,column)
                obj.header.frame_id='world';obj.operation=CollisionObject.ADD
                shape=SolidPrimitive();shape.type=SolidPrimitive.BOX
                shape.dimensions=[resolution+2*collision_margin_m,
                                  resolution+2*collision_margin_m, top-floor]
                pose=Pose();pose.position.x,pose.position.y,pose.position.z=world.translation
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=world.quaternion_xyzw
                obj.primitives=[shape];obj.primitive_poses=[pose]
                scene.world.collision_objects.append(obj)
    return Snapshot(surface['scene_revision'], int(surface['map_revision']),
                    surface['geometry_hash'], model_hash, world_from_container,
                    confirmed_surface.surface_cdr, serialize_message(scene),
                    surface_content_hash=hashlib.sha256(confirmed_surface.record_json).hexdigest(),
                    scene_content_hash=hashlib.sha256(json.dumps(
                        message_to_ordereddict(scene), sort_keys=True,
                        separators=(',', ':'), allow_nan=False).encode()).hexdigest(),
                    proposal_scene_hash=proposal_scene_hash(scene))
