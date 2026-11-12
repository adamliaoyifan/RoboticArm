"""Ordered opening routing inside planning; no controller or map writes.

The six-leg public plan remains unchanged: these waypoints belong to its carry
leg. Exact shared joint rows preserve winding. Every edge uses the same attached
scene; no unconstrained direct fallback can silently skip a required opening.
"""
import math
import time

import numpy as np
from scipy.spatial.transform import Rotation

from luggage_planning.joint_operation.aperture_geometry import box_vertices, transform_vertices
from luggage_planning.joint_operation.connection import GeometricConnection
from luggage_planning.placement_prior.contracts import Transform


def passage_constraints(aperture, region, world_from_container, tolerance):
    from moveit_msgs.msg import Constraints, PositionConstraint
    from shape_msgs.msg import SolidPrimitive
    from luggage_planning.joint_operation.scenes import _pose
    centre = aperture.point((region.outside_n+region.inside_n)/2,
                            (np.asarray(region.lower_uv)+region.upper_uv)/2)
    orientation = tuple(Rotation.from_matrix(aperture.axes).as_quat())
    pose = world_from_container.compose(Transform(centre, orientation))
    dimensions = (region.inside_n-region.outside_n,
                  *(np.asarray(region.upper_uv)-region.lower_uv))
    constraint = PositionConstraint()
    constraint.header.frame_id = 'world'
    constraint.link_name = 'suction_contact_frame'
    constraint.weight = 1.
    # Numerical FK slack, not an added/removed physical collision margin.
    constraint.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
        dimensions=[float(v+2*tolerance) for v in dimensions])]
    constraint.constraint_region.primitive_poses = [_pose(pose)]
    return Constraints(position_constraints=[constraint])


class ApertureRouter:
    def __init__(self, aperture, model, ik, connections, *, rigid_vertices,
                 max_points, max_seeds, atlas=None, hull=None, clock=time.monotonic):
        if (type(max_points) is not int or not 1 <= max_points <= 32
                or type(max_seeds) is not int or not 1 <= max_seeds <= 8):
            raise ValueError('bounded opening points and IK seeds required')
        self.aperture, self.model, self.ik, self.connections = aperture, model, ik, connections
        self.rigid_vertices = np.asarray(rigid_vertices)
        self.max_points, self.max_seeds, self.atlas, self.clock = max_points, max_seeds, atlas, clock
        self.hull = hull

    def _container_tool(self, snapshot, q):
        world = Transform.from_matrix(self.model.urdf.tcp(q, self.model.path))
        return snapshot.world_from_container.inverse().compose(world)

    def _seeds(self, target, start, goal):
        # Keep both coupled endpoints represented before applying the seed cap.
        # A full atlas cell must not silently remove all online fallbacks.
        result = [('current-carry', start), ('place-entry', goal)]
        if self.atlas is not None:
            x_axis = Rotation.from_quat(target.quaternion_xyzw).apply((1., 0., 0.))
            yaw = math.atan2(x_axis[1], x_axis[0])
            proposals = self.atlas.query(*target.translation, yaw)
            # Rank proposals by raw joint travel via the endpoints. Never wrap
            # J6 (or any other bounded joint) to make a distant winding cheap.
            branches = sorted(proposals.branches,key=lambda branch:
                np.linalg.norm(np.asarray(branch.contact_joints)-start)
                + np.linalg.norm(np.asarray(branch.contact_joints)-goal))
            for branch in branches:
                result.append(('atlas:'+branch.branch_id, branch.contact_joints))
        unique = []
        for name, q in result:
            if not any(tuple(q) == old[1] for old in unique):
                unique.append((name, tuple(q)))
        return tuple(unique[:self.max_seeds])

    def _already_inside(self, candidate, tool):
        if self.hull is None:
            # Explicit isolated fixtures may use an extruded rectangle. The
            # production root always provides its normalized complete hull.
            point = self.aperture.coordinates(tool.translation)
            return (point[0] >= 0. and all(a <= v <= b for a,v,b in
                    zip(self.aperture.lower,point[1:],self.aperture.upper)))
        from luggage_description.container_geometry import contains_point
        envelope = candidate.predicted_attachment.conservative
        tool_box = Transform(envelope.t_tcp_box_translation,envelope.t_tcp_box_quaternion_xyzw)
        vertices = transform_vertices(tool.compose(tool_box),box_vertices(envelope.dimensions_wdh))
        return (contains_point(self.hull,tool.translation)
                and all(contains_point(self.hull,p) for p in vertices))

    def connect(self, candidate, start, goal, *, request_id, deadline, cancelled, trace):
        scene = candidate.scenes.carry
        def stopped(): return cancelled() or self.clock() >= deadline
        def failure(reason): return GeometricConnection(False, False, reason)
        if stopped(): return failure('CANCELLED_OR_DEADLINE')
        initial = self._container_tool(scene, start)
        end = self._container_tool(scene, goal)
        tolerance = self.model.position_tolerance_m
        if self._already_inside(candidate,initial):
            if self.aperture.coordinates(end.translation)[0] < -tolerance:
                return failure('PLACEMENT_ENTRY_OUTSIDE_APERTURE')
            trace.append(('aperture', 'ALREADY_INSIDE_NO_FORCED_REENTRY'))
            return self.connections.connect(scene, start, goal, request_id=request_id+':inside',
                deadline=deadline, cancelled=cancelled)
        if self.aperture.coordinates(end.translation)[0] < -tolerance:
            return failure('PLACEMENT_ENTRY_OUTSIDE_APERTURE')
        reference = scene.world_from_container.inverse().compose(candidate.world_from_release_tool)
        for heading in range(4):
            rotation = Rotation.from_rotvec(self.aperture.axes[:, 2]*(heading*math.pi/2))
            quat = tuple((rotation*Rotation.from_quat(reference.quaternion_xyzw)).as_quat())
            target = Transform(reference.translation, quat)
            region = self.aperture.region(candidate.predicted_attachment, target,
                                          rigid_vertices=self.rigid_vertices)
            if region.empty:
                # Whole fixed-tool projection is sufficient but may exclude a
                # head that need not enter fully. Native geometry still applies.
                trace.append(('aperture', heading, 'FIXED_TOOL_PROJECTION_EMPTY_TRY_PAYLOAD'))
                region = self.aperture.region(candidate.predicted_attachment, target)
            points = self.aperture.proposals(region, end.translation, self.max_points)
            trace.append(('aperture_region', heading, region.proposal_kind,
                          region.lower_uv, region.upper_uv, len(points)))
            for index, point in enumerate(points):
                if stopped(): return failure('CANCELLED_OR_DEADLINE')
                gate = Transform(point, quat)
                coordinates = self.aperture.coordinates(point)
                targets = tuple(scene.world_from_container.compose(Transform(
                    self.aperture.point(n, coordinates[1:]), quat))
                    for n in (region.outside_n-tolerance, 0., region.inside_n+tolerance))
                for seed_name, seed in self._seeds(gate, start, goal):
                    tag = request_id+':aperture:'+str(heading)+':'+str(index)+':'+seed_name
                    began = self.clock()
                    centre = self.ik.refine(scene, targets[1], seed, request_id=tag+':plane',
                        deadline=deadline, cancelled=cancelled)
                    if not centre.success or not centre.complete:
                        trace.append(('aperture_ik', tag, centre.reason, self.clock()-began))
                        continue
                    outside = self.ik.refine(scene, targets[0], centre.joints, request_id=tag+':outside',
                        deadline=deadline, cancelled=cancelled)
                    inside = self.ik.refine(scene, targets[2], centre.joints, request_id=tag+':inside',
                        deadline=deadline, cancelled=cancelled)
                    if not all(r.success and r.complete for r in (outside, inside)):
                        trace.append(('aperture_crossing_ik', tag, outside.reason, inside.reason))
                        continue
                    constraints = passage_constraints(self.aperture, region,
                                                       scene.world_from_container, 2*tolerance)
                    waypoints = (tuple(start), outside.joints, centre.joints, inside.joints, tuple(goal))
                    rows, checked = [], 0
                    for edge, (a, b) in enumerate(zip(waypoints, waypoints[1:])):
                        kwargs = dict(request_id=tag+':edge:'+str(edge), deadline=deadline, cancelled=cancelled)
                        if edge in (1, 2): kwargs['path_constraints'] = constraints
                        result = self.connections.connect(scene, a, b, **kwargs)
                        from luggage_planning.joint_operation.planner import _connection_rows
                        segment = _connection_rows(result, a, b)
                        if segment is None:
                            trace.append(('aperture_edge', tag, edge, result.reason))
                            break
                        rows.extend(segment if not rows else segment[1:])
                        checked += result.checked_states
                    else:
                        if stopped(): return failure('CANCELLED_OR_DEADLINE')
                        trace.append(('aperture_selected', tag,
                            [dict(position_world=t.translation, quaternion_xyzw=t.quaternion_xyzw,
                                  joints=q) for t, q in zip(targets, waypoints[1:4])], self.clock()-began))
                        return GeometricConnection(True, True, 'ORDERED_APERTURE_CONNECTION', tuple(rows), checked)
        return failure('APERTURE_SEARCH_INCOMPLETE')

    def has_plane_witness(self, draft, candidate):
        """Independent FK check before native whole-chain certification."""
        leg = draft.legs[3]
        scene = candidate.scenes.carry
        tolerance = self.model.position_tolerance_m
        initial = self._container_tool(scene, leg.knots[0].positions)
        if self._already_inside(candidate,initial):
            return True
        for knot in leg.knots:
            tool = self._container_tool(scene, knot.positions)
            coordinates = self.aperture.coordinates(tool.translation)
            if abs(coordinates[0]) > tolerance:
                continue
            region = self.aperture.region(candidate.predicted_attachment, tool)
            if all(a-tolerance <= v <= b+tolerance for a, v, b in
                   zip(region.lower_uv, coordinates[1:], region.upper_uv)):
                return True
        return False
