"""Require the configured static ULD before whole-operation planning."""
import numpy as np
from scipy.spatial.transform import Rotation

from luggage_planning.placement_prior_contracts import Transform
from luggage_planning.aperture_geometry import transform_vertices, box_vertices


def _pose(value):
    q = (value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w)
    # MoveIt treats an unset CollisionObject.pose as identity.
    if not any(q): q = (0., 0., 0., 1.)
    return Transform((value.position.x, value.position.y, value.position.z), q)


def _mesh_vertices(obj):
    if (obj.header.frame_id != 'world' or len(obj.meshes) != 1 or len(obj.mesh_poses) != 1
            or obj.primitives or obj.planes or obj.operation != obj.ADD):
        raise ValueError('STATIC_CONTAINER_SHAPE_MISMATCH')
    pose = _pose(obj.pose).compose(_pose(obj.mesh_poses[0]))
    vertices = np.asarray([(v.x, v.y, v.z) for v in obj.meshes[0].vertices])
    faces = np.asarray([tuple(t.vertex_indices) for t in obj.meshes[0].triangles])
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(faces) == 0
            or not np.all(np.isfinite(vertices))):
        raise ValueError('STATIC_CONTAINER_GEOMETRY_MISSING')
    return transform_vertices(pose, vertices), faces


class StaticWorldAdmission:
    def __init__(self, config, world_from_container, *, mesh_path=''):
        from luggage_description.scene_mesh_utils import container_collision_mesh_path, container_model_name
        from luggage_description.scene_tf_config_utils import world_transform
        from luggage_planning.planning_scene_client import load_stl_mesh_msg, build_mesh_collision_object
        xyz, rpy = world_transform(config, 'container_link')
        expected_pose = Transform(xyz, tuple(Rotation.from_euler('xyz', rpy).as_quat()))
        if (not np.allclose(expected_pose.translation, world_from_container.translation, atol=1e-9, rtol=0)
                or abs(np.dot(expected_pose.quaternion_xyzw, world_from_container.quaternion_xyzw)) < 1.-1e-12):
            raise ValueError('APERTURE_CONTAINER_TRANSFORM_MISMATCH')
        self.container_id = container_model_name(config)
        self.pedestal_required = bool(config.get('pedestal', {}).get('enabled', False))
        self.pedestal_vertices = None
        if self.pedestal_required:
            from luggage_description.scene_tf_config_utils import pedestal_collision_pose_in_world, pedestal_dimensions
            centre, orientation = pedestal_collision_pose_in_world(config)
            pose = Transform(centre,tuple(Rotation.from_euler('xyz',orientation).as_quat()))
            self.pedestal_vertices = transform_vertices(pose,box_vertices(pedestal_dimensions(config)))
        mesh = load_stl_mesh_msg(mesh_path or container_collision_mesh_path(config))
        expected = build_mesh_collision_object(self.container_id, expected_pose.translation,
                                               expected_pose.quaternion_xyzw, mesh)
        self.vertices, self.faces = _mesh_vertices(expected)

    def admit(self, scene):
        if scene.is_diff or scene.robot_state.is_diff:
            raise ValueError('FULL_STATIC_SCENE_REQUIRED')
        matches = [o for o in scene.world.collision_objects if o.id == self.container_id]
        if len(matches) != 1:
            raise ValueError('STATIC_CONTAINER_MISSING_OR_DUPLICATE')
        vertices, faces = _mesh_vertices(matches[0])
        if (vertices.shape != self.vertices.shape or faces.shape != self.faces.shape
                or not np.array_equal(faces, self.faces)
                or not np.allclose(vertices, self.vertices, atol=1e-7, rtol=0)):
            raise ValueError('STATIC_CONTAINER_GEOMETRY_OR_POSE_MISMATCH')
        if self.pedestal_required:
            pedestal = [o for o in scene.world.collision_objects if o.id == 'robot_pedestal']
            if len(pedestal) != 1:
                raise ValueError('STATIC_PEDESTAL_MISSING')
            obj = pedestal[0]
            if (obj.header.frame_id != 'world' or obj.operation != obj.ADD
                    or len(obj.primitives) != 1 or len(obj.primitive_poses) != 1
                    or obj.meshes or obj.planes or obj.primitives[0].type != obj.primitives[0].BOX):
                raise ValueError('STATIC_PEDESTAL_GEOMETRY_MISMATCH')
            pose = _pose(obj.pose).compose(_pose(obj.primitive_poses[0]))
            vertices = transform_vertices(pose,box_vertices(obj.primitives[0].dimensions))
            if not np.allclose(vertices,self.pedestal_vertices,atol=1e-7,rtol=0):
                raise ValueError('STATIC_PEDESTAL_GEOMETRY_OR_POSE_MISMATCH')
        return scene
