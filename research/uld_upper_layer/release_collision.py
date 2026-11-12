"""Continuous nominal tool-withdrawal checking in an observed release fixture.

Only the tool translates; payload and environment poses are frozen predictions.
Every source collision is retained. This is neither an arm certificate nor an
error tube, and its result always leaves execution_admitted false.
"""
from dataclasses import asdict
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import fcl
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from assets import sha256
from planning_types import digest
from release_fixture import source_tool_link
from tool_contact_geometry import derive_push_surface


def _rigid(value):
    matrix = np.asarray(value, dtype=float).reshape(4, 4)
    if (not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-8, rtol=0)):
        raise ValueError('RELEASE_COLLISION_POSE_NOT_RIGID')
    return matrix


def _pose(node):
    value = node.find('pose')
    if value is None:
        return np.eye(4)
    if value.attrib:
        raise ValueError('UNSUPPORTED_RELEASE_SDF_POSE_FORMAT')
    values = np.fromstring(value.text, sep=' ')
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError('INVALID_RELEASE_SDF_POSE')
    result = np.eye(4); result[:3, 3] = values[:3]
    result[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    return result


def _canonical(node):
    if node is None:
        raise ValueError('RELEASE_GEOMETRY_MISSING')
    return (node.tag, node.attrib, (node.text or '').strip(), tuple(_canonical(c) for c in node))


class _Shape:
    def __init__(self, name, geometry, transform, hashes):
        transform = _rigid(transform)
        self.name = name; self.local = transform.copy()
        if geometry is None or len(geometry) != 1:
            raise ValueError('RELEASE_GEOMETRY_NOT_UNIQUE')
        node = geometry[0]
        if node.tag == 'box':
            size = np.fromstring(node.findtext('size'), sep=' ')
            if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
                raise ValueError('INVALID_RELEASE_BOX_GEOMETRY')
            self.vertices = trimesh.bounds.corners(np.array([-size / 2, size / 2]))
            body = fcl.Box(*size)
        elif node.tag == 'mesh':
            path = Path(node.findtext('uri')).resolve(strict=True)
            scale = np.fromstring(node.findtext('scale', '1 1 1'), sep=' ')
            if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
                raise ValueError('INVALID_RELEASE_MESH_SCALE')
            mesh = trimesh.load_mesh(path, process=False)
            self.vertices = np.asarray(mesh.vertices) * scale
            if not len(mesh.faces) or not np.isfinite(self.vertices).all():
                raise ValueError('INVALID_RELEASE_MESH')
            body = fcl.BVHModel(); body.beginModel(len(self.vertices), len(mesh.faces))
            body.addSubModel(self.vertices, np.asarray(mesh.faces)); body.endModel()
            hashes[str(path)] = sha256(path)
        else:
            raise ValueError('UNSUPPORTED_RELEASE_GEOMETRY:' + node.tag)
        self.object = fcl.CollisionObject(body, fcl.Transform(transform[:3, :3], transform[:3, 3]))

    def move_tool(self, tool):
        frame = tool @ self.local
        self.object.setTransform(fcl.Transform(frame[:3, :3], frame[:3, 3]))

    def tool_vertices(self):
        return self.vertices @ self.local[:3, :3].T + self.local[:3, 3]


class ReleaseCollisionScene:
    def __init__(self, assets, world_path, sample, dimensions, config):
        self.config = config; self.sample = sample; self.dimensions = tuple(dimensions)
        self.model_hashes = {}; self.world_hash = sha256(world_path)
        self.assets_hashes = dict(assets['hashes'])
        self._tool = []; self._world = []; self._planes = []; self._lower_parts = {}
        root = ET.parse(world_path).getroot(); world = root.find('world')
        if world is None or world.findall('include') or world.findall('model/model'):
            raise ValueError('UNSUPPORTED_RELEASE_WORLD_STRUCTURE')
        world_container = _rigid(assets['world_container']); container_world = np.linalg.inv(world_container)
        _rigid(sample.container_T_tool)
        expected, _ = source_tool_link(assets)
        carriage = world.find("model[@name='release_carriage']")
        if carriage is None:
            raise ValueError('RELEASE_CARRIAGE_MISSING')
        links = carriage.findall('link')
        if ({link.get('name') for link in links} != {'anchor', 'x_carriage', 'tool'}
                or any(not np.allclose(_pose(link), np.eye(4), atol=1e-12, rtol=0) for link in links)
                or len(carriage.findall('joint')) != 3):
            raise ValueError('RELEASE_CARRIAGE_KINEMATICS_CHANGED')
        rotation = (container_world @ _pose(carriage))[:3, :3]
        for name, parent, child, kind, direction in (
                ('world_anchor', 'world', 'anchor', 'fixed', None),
                ('release_x', 'anchor', 'x_carriage', 'prismatic', [1., 0., 0.]),
                ('release_z', 'x_carriage', 'tool', 'prismatic', [0., 0., 1.])):
            joint = carriage.find(f"joint[@name='{name}']")
            if (joint is None or joint.get('type') != kind or joint.findtext('parent') != parent
                    or joint.findtext('child') != child
                    or not np.allclose(_pose(joint), np.eye(4), atol=1e-12, rtol=0)):
                raise ValueError('RELEASE_CARRIAGE_KINEMATICS_CHANGED')
            if direction is not None:
                axis = joint.find('axis/xyz')
                xyz = np.fromstring(axis.text or '', sep=' ') if axis is not None else np.array([])
                if (axis is None or axis.attrib or xyz.shape != (3,)
                        or not np.allclose(rotation @ xyz, direction, atol=1e-12, rtol=0)):
                    raise ValueError('RELEASE_CARRIAGE_AXES_CHANGED')
        tool = carriage.find("link[@name='tool']")
        if tool is None or not np.allclose(_pose(tool), np.eye(4), atol=1e-12, rtol=0):
            raise ValueError('RELEASE_TOOL_FRAME_CHANGED')
        actual_parts = {c.get('name'): c for c in tool.findall('collision')}
        expected_parts = {c.get('name'): c for c in expected.findall('collision')}
        if set(actual_parts) != set(expected_parts):
            raise ValueError('RELEASE_SOURCE_TOOL_SET_CHANGED')
        for name, source in expected_parts.items():
            actual = actual_parts[name]
            if not np.allclose(_pose(actual), _pose(source), atol=1e-12, rtol=0) or _canonical(actual.find('geometry')) != _canonical(source.find('geometry')):
                raise ValueError('RELEASE_SOURCE_TOOL_GEOMETRY_CHANGED:' + name)
            self._tool.append(_Shape('tool/' + name, actual.find('geometry'), _pose(actual), self.model_hashes))
        for link in carriage.findall('link'):
            if link is not tool and link.findall('collision'):
                raise ValueError('UNEXPECTED_CARRIAGE_COLLISION_GEOMETRY')
        lower = {b.name: b for b in sample.lower}; seen_lower = set()
        for model in world.findall('model'):
            name = model.get('name')
            if name in ('release_carriage', 'upper_layer_payload'):
                continue
            if name in lower:
                seen_lower.add(name); base = _rigid(lower[name].container_T_body)
                collisions = model.findall('link/collision')
                if len(collisions) != 1:
                    raise ValueError('RELEASE_LOWER_GEOMETRY_CHANGED')
                size = np.fromstring(collisions[0].findtext('geometry/box/size', ''), sep=' ')
                # scene.numbers serializes original fixture sizes with .9g.
                # Accept that exact round-trip only within the existing 1um
                # enclosure; queries retain actual SDF geometry, not estimates.
                rounded = np.array([float(format(v, '.9g')) for v in lower[name].dimensions])
                if (size.shape != (3,) or not np.allclose(size, rounded, atol=1e-12, rtol=0)
                        or np.max(np.abs(size - lower[name].dimensions)) > 1e-6
                        or not np.allclose(_pose(collisions[0]), np.eye(4), atol=1e-12, rtol=0)):
                    raise ValueError('RELEASE_LOWER_GEOMETRY_CHANGED')
            else:
                if model.findtext('static', 'false').strip() != 'true' and model.findall('link/collision'):
                    raise ValueError('UNOBSERVED_DYNAMIC_RELEASE_BODY:' + name)
                base = container_world @ _pose(model)
            for link in model.findall('link'):
                if name in lower and not np.allclose(_pose(link), np.eye(4), atol=1e-12, rtol=0):
                    raise ValueError('RELEASE_LOWER_FRAME_UNSUPPORTED')
                for collision in link.findall('collision'):
                    frame = base @ _pose(link) @ _pose(collision)
                    geometry = collision.find('geometry'); key = name + '/' + link.get('name') + '/' + collision.get('name')
                    plane = geometry.find('plane')
                    if plane is not None:
                        if len(geometry) != 1:
                            raise ValueError('RELEASE_GEOMETRY_NOT_UNIQUE')
                        normal = np.fromstring(plane.findtext('normal'), sep=' ')
                        if normal.shape != (3,) or not np.isfinite(normal).all() or not np.isclose(np.linalg.norm(normal), 1.):
                            raise ValueError('RELEASE_PLANE_NORMAL_INVALID')
                        self._planes.append((key, frame[:3, :3] @ normal, frame[:3, 3]))
                    else:
                        part = _Shape(key, geometry, frame, self.model_hashes)
                        self._world.append(part)
                        if name in lower:
                            self._lower_parts[name] = part
        if seen_lower != set(lower):
            raise ValueError('RELEASE_OBSERVED_LOWER_SET_CHANGED')
        payload = world.find("model[@name='upper_layer_payload']/link")
        if payload is None or not np.allclose(_pose(payload), np.eye(4), atol=1e-12, rtol=0) or len(payload.findall('collision')) != 1:
            raise ValueError('RELEASE_PAYLOAD_FRAME_CHANGED')
        collision = payload.find('collision')
        size = np.fromstring(collision.findtext('geometry/box/size', ''), sep=' ')
        if size.shape != (3,) or not np.allclose(size, dimensions, atol=1e-12, rtol=0) or not np.allclose(_pose(collision), np.eye(4), atol=1e-12, rtol=0):
            raise ValueError('RELEASE_PAYLOAD_GEOMETRY_CHANGED')
        box = _rigid(sample.container_T_box)
        self._payload = _Shape('payload', collision.find('geometry'), box, self.model_hashes)
        sign = -1 if box[2, 2] >= 0 else 1
        self._box_up = -sign * box[:3, 2]
        self._box_bottom = box[:3, 3] + sign * dimensions[2] / 2 * box[:3, 2]
        contact = derive_push_surface(assets)
        self._interface = 'tool/' + contact.collision_key.replace(':', '_')
        self._footprint = np.asarray(assets['contact']['footprint_size_xy_m'])
        self._queries = 0

    def _validate_motions(self, motions):
        """Bind the certified locus to measured start and commanded samples.

        Only the fixture's declared X/Z axes and two-stage withdrawal are
        supported. Reject a forged intermediate waypoint even when endpoints
        describe a clear straight segment.
        """
        if tuple(m.name for m in motions) != ('separate_down', 'withdraw_outward'):
            raise ValueError('RELEASE_WITHDRAWAL_STAGES_INVALID')
        previous_tool = np.asarray(self.sample.container_T_tool).reshape(4, 4)
        previous_q = np.asarray(self.sample.joint_displacements_xz_m)
        for index, motion in enumerate(motions):
            start = np.asarray(motion.container_T_tool_start).reshape(4, 4)
            end = np.asarray(motion.container_T_tool_end).reshape(4, 4)
            q = np.asarray(motion.joint_displacements_xz_m)
            v = np.asarray(motion.joint_velocities_xz_mps)
            t = np.asarray(motion.time_from_start_s)
            if (t.ndim != 1 or len(t) < 2 or q.shape != (len(t), 2) or v.shape != q.shape
                    or not all(np.isfinite(a).all() for a in (start, end, q, v, t))
                    or t[0] != 0 or np.any(np.diff(t) <= 0)):
                raise ValueError('RELEASE_MOTION_SAMPLES_INVALID')
            delta = q[-1] - q[0]; axis = 1 if index == 0 else 0
            if delta[axis] >= 0 or abs(delta[1-axis]) > 1e-12:
                raise ValueError('RELEASE_MOTION_DIRECTION_INVALID')
            expected_end = start.copy(); expected_end[[0, 2], 3] += delta
            if (not np.allclose(start, previous_tool, atol=1e-12, rtol=0)
                    or not np.allclose(q[0], previous_q, atol=1e-12, rtol=0)
                    or not np.allclose(end, expected_end, atol=1e-12, rtol=0)):
                raise ValueError('RELEASE_MOTION_NOT_CONTINUOUS_TRANSLATION')
            progress = (q[:, axis] - q[0, axis]) / delta[axis]
            if (np.any(np.diff(progress) < -1e-12) or progress.min() < -1e-12 or progress.max() > 1+1e-12
                    or not np.allclose(q[:, 1-axis], q[0, 1-axis], atol=1e-12, rtol=0)
                    or not np.allclose(v[:, 1-axis], 0, atol=1e-12, rtol=0)
                    or np.any(v[:, axis] > 1e-12)):
                raise ValueError('RELEASE_MOTION_NOT_MONOTONE_LINE')
            previous_tool = end; previous_q = q[-1]

    def initial_seal_valid(self, tool):
        box = np.asarray(self.sample.container_T_box).reshape(4, 4)
        relative = np.linalg.inv(box) @ tool; half = np.asarray(self.dimensions) / 2
        axis = int(np.argmax(np.abs(relative[:3, 2]))); sign = np.sign(relative[axis, 2])
        if abs(relative[axis, 2]) < 1 - 1e-7:
            return False
        gap = -sign * (relative[axis, 3] + sign * half[axis])
        if gap < -self.config.contact_penetration_m or gap > self.config.contact_tolerance_m:
            return False
        for u, v in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            point = relative @ np.array([u * self._footprint[0] / 2, v * self._footprint[1] / 2, 0., 1.])
            if any(abs(point[j]) > half[j] + 1e-8 for j in range(3) if j != axis):
                return False
        return True

    def evaluate(self, tool, *, separating_interface):
        # Same experimental 1um numerical enclosure as the existing FCL oracle.
        error = 1e-6; minimum = float('inf'); witness = None; interface = None
        for part in self._tool:
            part.move_tool(tool)
            vertices = part.tool_vertices() @ tool[:3, :3].T + tool[:3, 3]
            for other in self._world + [self._payload]:
                if separating_interface and part.name == self._interface and other is self._payload:
                    separation = float(np.min((self._box_bottom - vertices) @ self._box_up)) - error
                    interface = separation
                    if separation <= 0:
                        return dict(failure='RELEASE_INTERFACE_NOT_SEPARATED', pair=[part.name, 'payload'], separation_m=separation)
                    continue
                self._queries += 1
                collision = fcl.collide(part.object, other.object, fcl.CollisionRequest(), fcl.CollisionResult())
                distance = float(fcl.distance(part.object, other.object, fcl.DistanceRequest(), fcl.DistanceResult())) - error
                if collision or not np.isfinite(distance) or distance < self.config.collision_margin_m:
                    return dict(failure='RELEASE_TOOL_COLLISION_OR_CLEARANCE', pair=[part.name, other.name], distance_m=distance)
                if distance < minimum:
                    minimum = distance; witness = [part.name, other.name]
            for name, normal, point in self._planes:
                distance = float(np.min((vertices - point) @ normal)) - error
                if distance < self.config.collision_margin_m:
                    return dict(failure='RELEASE_TOOL_PLANE_CLEARANCE', pair=[part.name, name], distance_m=distance)
                if distance < minimum:
                    minimum = distance; witness = [part.name, name]
        return dict(minimum_m=minimum, witness=witness, interface_separation_m=interface)

    def evaluate_observation(self, sample, *, separating_interface):
        """Check actual same-step poses without altering certificate inputs.

        Source lower/payload collisions have identity link/collision frames,
        established by the constructor. Restore the frozen scene after each
        observation so a later certificate cannot silently use changed poses.
        """
        lower = {body.name: body for body in sample.lower}
        if set(lower) != set(self._lower_parts):
            raise ValueError('RELEASE_OBSERVED_LOWER_SET_CHANGED')
        before_up = self._box_up.copy(); before_bottom = self._box_bottom.copy()
        moving = [self._payload, *self._lower_parts.values()]
        try:
            for name, part in self._lower_parts.items():
                frame = _rigid(lower[name].container_T_body)
                part.object.setTransform(fcl.Transform(frame[:3, :3], frame[:3, 3]))
            box = _rigid(sample.container_T_box)
            self._payload.object.setTransform(fcl.Transform(box[:3, :3], box[:3, 3]))
            sign = -1 if box[2, 2] >= 0 else 1
            self._box_up = -sign * box[:3, 2]
            self._box_bottom = box[:3, 3] + sign*self.dimensions[2]/2*box[:3, 2]
            return self.evaluate(_rigid(sample.container_T_tool), separating_interface=separating_interface)
        finally:
            self._box_up = before_up; self._box_bottom = before_bottom
            for part in moving:
                part.object.setTransform(fcl.Transform(part.local[:3, :3], part.local[:3, 3]))

    def certify(self, motions, deadline):
        motions = tuple(motions)
        self._validate_motions(motions)
        checks = leaves = 0; lower = float('inf'); interface_lower = float('inf')
        previous = None
        for index, motion in enumerate(motions):
            start = np.asarray(motion.container_T_tool_start).reshape(4, 4)
            end = np.asarray(motion.container_T_tool_end).reshape(4, 4)
            if not np.allclose(start[:3, :3], end[:3, :3], atol=1e-12, rtol=0) or (previous is not None and not np.allclose(start, previous, atol=1e-12, rtol=0)):
                raise ValueError('RELEASE_MOTION_NOT_CONTINUOUS_TRANSLATION')
            separating = index == 0 and motion.name == 'separate_down' and self.initial_seal_valid(start)
            pending = [(0., 1.)]
            while pending:
                if time.monotonic() >= deadline:
                    return dict(valid=False, reason='RELEASE_CERTIFICATE_DEADLINE', checks=checks, execution_admitted=False)
                low, high = pending.pop(); mid = (low + high) / 2
                tool = start.copy(); tool[:3, 3] = start[:3, 3] + mid * (end[:3, 3] - start[:3, 3])
                result = self.evaluate(tool, separating_interface=separating); checks += 1
                if result.get('failure'):
                    return dict(valid=False, reason=result['failure'], diagnostic=result, stage=index, fraction=mid, checks=checks, execution_admitted=False)
                bound = float(np.linalg.norm(end[:3, 3] - start[:3, 3])) * (high - low) / 2
                if result['minimum_m'] - bound >= self.config.collision_margin_m and (not separating or result['interface_separation_m'] - bound > 0):
                    leaves += 1; lower = min(lower, result['minimum_m'] - bound)
                    if separating:
                        interface_lower = min(interface_lower, result['interface_separation_m'] - bound)
                elif mid in (low, high):
                    return dict(valid=False, reason='RELEASE_UNCERTIFIABLE_INTERVAL', checks=checks, execution_admitted=False)
                else:
                    pending.extend(((mid, high), (low, mid)))
            previous = end
        return dict(valid=True, checks=checks, certified_intervals=leaves, pair_queries=self._queries,
            noncontact_lower_m=lower, interface_lower_m=interface_lower if np.isfinite(interface_lower) else None,
            source_tool_parts=len(self._tool), world_parts=len(self._world), planes=len(self._planes),
            inputs_digest=digest(dict(sample=asdict(self.sample), motions=[asdict(m) for m in motions], config=self.config.revision,
                world=self.world_hash, assets=self.assets_hashes, meshes=self.model_hashes)),
            numeric_error_m=1e-6, execution_admitted=False,
            scope='nominal_translation_locus_frozen_payload_and_environment_not_error_tube_or_arm_certificate')
