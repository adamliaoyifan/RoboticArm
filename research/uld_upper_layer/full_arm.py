"""Source-mesh FCL collision queries for the research robot, in world frame.

This owns a private scene, never a live MoveIt planning scene. Robot kinematics
come from the expanded source URDF. Existing SRDF self-pair policy is explicit;
world, payload and lower-cargo pairs are not disabled by that policy.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from assets import origin, source_mesh, sha256
from geometry import pose


def mesh_from_geometry(geometry, repo):
    mesh=geometry.find('mesh')
    if mesh is not None:
        loaded=trimesh.load_mesh(source_mesh(repo,mesh.get('filename')),process=False)
        loaded.apply_scale([float(v) for v in mesh.get('scale','1 1 1').split()])
        return loaded
    box=geometry.find('box')
    if box is not None:return trimesh.creation.box([float(v) for v in box.get('size').split()])
    cylinder=geometry.find('cylinder')
    if cylinder is not None:
        return trimesh.creation.cylinder(float(cylinder.get('radius')),float(cylinder.get('length')),sections=64)
    sphere=geometry.find('sphere')
    if sphere is not None:return trimesh.creation.icosphere(subdivisions=3,radius=float(sphere.get('radius')))
    raise ValueError('unsupported robot collision geometry')


class FullArmScene:
    def __init__(self,assets,snapshot,config=None):
        from trimesh.collision import CollisionManager
        self.assets=assets;self.urdf=assets['urdf'];self.snapshot=snapshot
        from planning_types import UpperConfig
        self.config=config or UpperConfig.load()
        from tool_contact_geometry import derive_push_surface
        self.push_geometry=derive_push_surface(assets)
        self.robot=CollisionManager();self.world=CollisionManager()
        self.robot_parts={};self.world_parts={};self.model_hashes={}
        self.world_managers={}
        self.minimum_clearance_m=float('inf');self.clearance_witness=None
        self.query_observer=None;self.exact_distances=False
        self.robot_bounds={};self.world_bounds={}
        self.world_container=assets['world_container']
        self.container_world=np.linalg.inv(self.world_container)
        srdf=assets['repo']/'src/luggage_description/config/S20_with_camera.srdf'
        self.source_disabled={frozenset((e.get('link1'),e.get('link2')))
                       for e in ET.parse(srdf).getroot().findall('disable_collisions')}
        # New inversion/push poses invalidate a sampled SRDF 'Never' assumption.
        # Only adjacent links and geometrically rigid fixed assemblies are
        # exempt. Sensor-to-moving-arm pairs receive no blanket exemption.
        self.disabled={frozenset((e.get('link1'),e.get('link2')))
                       for e in ET.parse(srdf).getroot().findall('disable_collisions')
                       if e.get('reason')=='Adjacent'}
        fixed={name:{name} for name in [link.get('name') for link in self.urdf.root.findall('link')]}
        for joint in self.urdf.joints.values():
            if joint['type']=='fixed':
                merged=fixed[joint['parent']]|fixed[joint['child']]
                for name in merged:fixed[name]=merged
        for name,group in fixed.items():
            self.disabled.update(frozenset((name,other)) for other in group if other!=name)
        self.model_hashes[str(srdf)]=sha256(srdf)
        for link in self.urdf.root.findall('link'):
            name=link.get('name')
            for index,collision in enumerate(link.findall('collision')):
                key=f'{name}:{index}'
                geometry=collision.find('geometry')
                mesh=mesh_from_geometry(geometry,assets['repo'])
                uri=geometry.find('mesh')
                if uri is not None:
                    path=source_mesh(assets['repo'],uri.get('filename'))
                    self.model_hashes[str(path)]=sha256(path)
                self.robot_parts[key]=(name,mesh,origin(collision))
                self.robot_bounds[key]=trimesh.bounds.corners(mesh.bounds)
                self.robot.add_object(key,mesh)
        self.pair_managers={}
        for key,(link,mesh,local) in self.robot_parts.items():
            manager=CollisionManager();manager.add_object(key,mesh)
            self.pair_managers[key]=manager
        self.self_pairs=[]
        keys=list(self.robot_parts)
        for i,a in enumerate(keys):
            for b in keys[i+1:]:
                la,lb=self.robot_parts[a][0],self.robot_parts[b][0]
                if la!=lb and frozenset((la,lb)) not in self.disabled:self.self_pairs.append((a,b))
        self._add_environment()
        self.last_witness=None
        self.payload=None
        self.payload_dynamics=None
        from motion_bounds import SourceMotionBounds
        self._motion_bounds=SourceMotionBounds(self.urdf,self.robot_parts,self.world_parts)
        self.set_joints(np.zeros(6))

    def add_world(self,name,mesh,transform):
        self.world.add_object(name,mesh,transform)
        self.world_parts[name]=(mesh,np.asarray(transform))
        self.world_bounds[name]=self.transformed_bounds(mesh,transform)
        from trimesh.collision import CollisionManager
        manager=CollisionManager();manager.add_object(name,mesh,transform)
        self.world_managers[name]=manager
        if hasattr(self,'_motion_bounds'):self._motion_bounds.register_static(name)

    def remove_world(self,name):
        self.world.remove_object(name)
        if hasattr(self,'_motion_bounds'):self._motion_bounds.remove_static(name)
        del self.world_parts[name];del self.world_managers[name];del self.world_bounds[name]

    def _add_environment(self):
        from luggage_description.scene_tf_config_utils import (
            gazebo_pedestal_spawn_pose,gazebo_pickup_platform_spawn_pose)
        scene=self.assets['scene'];repo=self.assets['repo']
        model=scene['gazebo']['container_model']
        path=repo/f'src/luggage_gazebo/models/{model}/meshes/container_collision_physics.stl'
        self.model_hashes[str(path)]=sha256(path)
        self.add_world('uld_mesh',trimesh.load_mesh(path,process=False),self.world_container)
        for name,spawn in (('robot_pedestal',gazebo_pedestal_spawn_pose(scene)),
                           ('pickup_platform',gazebo_pickup_platform_spawn_pose(scene))):
            path=repo/f'src/luggage_gazebo/models/{name}/model.sdf'
            self.model_hashes[str(path)]=sha256(path)
            from luggage_description.configured_scene_models import configured_model
            root=configured_model(path,scene,name)
            base=pose([spawn['x'],spawn['y'],spawn['z']],Rotation.from_euler(
                'xyz',[spawn['R'],spawn['P'],spawn['Y']]).as_matrix())
            for link in root.findall('link'):
                for index,collision in enumerate(link.findall('collision')):
                    extents=[float(v) for v in collision.find('geometry/box/size').text.split()]
                    xyzrpy=[float(v) for v in collision.find('pose').text.split()]
                    local=pose(xyzrpy[:3],Rotation.from_euler('xyz',xyzrpy[3:]).as_matrix())
                    self.add_world(f'{name}:{index}',trimesh.creation.box(extents),base@local)
        self.add_world('ground',trimesh.creation.box([20,20,.1]),pose([0,0,-.05]))
        for item in self.snapshot.occupied:
            self.add_world(item.name,trimesh.creation.box(item.size),
                           self.world_container@pose(item.center))
        hull=self.assets['hull']
        self.add_world('canonical_floor',trimesh.creation.box([hull.length,hull.width,.01]),
                       self.world_container@pose([0,0,hull.floor_z-.005]))

    def link_transforms(self,q):
        values=dict(zip([f'elfin_joint{i}' for i in range(1,7)],q))
        transforms={self.urdf.root_link:np.eye(4)}
        pending=[self.urdf.root_link]
        while pending:
            parent=pending.pop()
            for name in self.urdf.children.get(parent,()):
                joint=self.urdf.joints[name]
                rotation=joint['rot']
                if joint['type']=='revolute':
                    rotation=rotation@Rotation.from_rotvec(joint['axis']*values.get(name,0.)).as_matrix()
                transforms[joint['child']]=transforms[parent]@pose(joint['xyz'],rotation)
                pending.append(joint['child'])
        return transforms

    def set_joints(self,q):
        self.current_q=list(q)
        transforms=self.link_transforms(q)
        for key,(link,mesh,local) in self.robot_parts.items():
            self.robot.set_transform(key,transforms[link]@local)
            if key in self.pair_managers:self.pair_managers[key].set_transform(key,transforms[link]@local)
        self.transforms=transforms
        self.current_bounds={}
        for key,vertices in self.robot_bounds.items():
            link,_,local=self.robot_parts[key];tf=transforms[link]@local
            points=vertices@tf[:3,:3].T+tf[:3,3]
            self.current_bounds[key]=np.array([points.min(axis=0),points.max(axis=0)])

    @staticmethod
    def transformed_bounds(mesh,transform):
        points=trimesh.bounds.corners(mesh.bounds)@transform[:3,:3].T+transform[:3,3]
        return np.array([points.min(axis=0),points.max(axis=0)])

    @staticmethod
    def bounds_distance(a,b):
        return float(np.linalg.norm(np.maximum(np.maximum(a[0]-b[1],b[0]-a[1]),0)))

    def observed_distance(self,a,b,pair,q=None):
        import time
        event=dict(pair=pair,q=list(q) if q is not None else self.current_q,wall=time.time())
        if self.query_observer:self.query_observer(dict(phase='begin',**event))
        result=a.min_distance_other(b)
        if self.query_observer:self.query_observer(dict(phase='end',distance_m=float(result),elapsed_s=time.time()-event['wall'],**event))
        return result

    def record_clearance(self,distance,pair):
        self.sample_clearance_m=min(getattr(self,'sample_clearance_m',float('inf')),float(distance))
        if hasattr(self,'sample_distances'):self.sample_distances.append((float(distance),pair))
        if distance<self.minimum_clearance_m:
            self.minimum_clearance_m=float(distance);self.clearance_witness=pair

    def check_robot(self,q,margin=0.):
        """Exact mesh collision. Distance margin covers robot/world separately.

        Mount contact between elfin_base and pedestal is intentional; no moving
        link obtains that exception. Existing self SRDF pairs are reported policy.
        """
        limits=np.array(self.urdf.joint_limits())
        if np.any(np.asarray(q)<limits[:,0]) or np.any(np.asarray(q)>limits[:,1]):
            return dict(reason='JOINT_LIMIT',q=list(q))
        self.set_joints(q)
        # Query only non-exempt pairs. Asking the global manager first also
        # computes thousands of permanent fixed-assembly contacts which are
        # subsequently discarded; profile_g11 measured these as the main cost.
        for a,b in self.self_pairs:
            if self.bounds_distance(self.current_bounds[a],self.current_bounds[b])>0:continue
            if self.pair_managers[a].in_collision_other(self.pair_managers[b]):
                return dict(reason='SELF_COLLISION',pair=[a,b],q=list(q))
        for key,(link,mesh,local) in self.robot_parts.items():
            for other,manager in self.world_managers.items():
                if link=='elfin_base' and other.startswith('robot_pedestal:'):continue
                if self.bounds_distance(self.current_bounds[key],self.world_bounds[other])>0:continue
                if self.pair_managers[key].in_collision_other(manager):
                    return dict(reason='ROBOT_WORLD_COLLISION',pair=[key,other],q=list(q))
        if margin>0:
            for a,b in self.self_pairs:
                distance=self.bounds_distance(self.current_bounds[a],self.current_bounds[b])
                if self.exact_distances or distance<margin+2*self.config.cartesian_step_m:
                    distance=self.observed_distance(self.pair_managers[a],self.pair_managers[b],[a,b],q)
                self.record_clearance(distance,[a,b])
                if distance<margin-1e-8:
                    return dict(reason='SELF_CLEARANCE',pair=[a,b],distance_m=float(distance),q=list(q))
            # Static base mounting cannot set every moving-arm score to zero.
            for key,(link,mesh,local) in self.robot_parts.items():
                if link=='elfin_base':continue
                for other,manager in self.world_managers.items():
                    distance=self.bounds_distance(self.current_bounds[key],self.world_bounds[other])
                    if self.exact_distances or distance<margin+2*self.config.cartesian_step_m:
                        distance=self.observed_distance(self.pair_managers[key],manager,[key,other],q)
                    self.record_clearance(distance,[key,other])
                    if distance < margin-1e-8:
                        return dict(reason='ROBOT_WORLD_CLEARANCE',pair=[key,other],
                                    distance_m=float(distance),q=list(q))
        return None

    def set_payload(self,size,mode,world_pose=None,box_T_tool=None,allow_support=False):
        if mode not in ('absent','world','attached'):
            raise ValueError('invalid payload mode')
        self.payload=None if mode=='absent' else dict(mesh=trimesh.creation.box(size),
            size=np.array(size),mode=mode,world_pose=world_pose,box_T_tool=box_T_tool,
            allow_support=allow_support)
        self._motion_bounds.set_payload(self.payload)

    def relative_motion_bound(self,pair,absolute_joint_delta):
        return self._motion_bounds.relative_motion_bound(pair,absolute_joint_delta)

    def payload_transform(self):
        p=self.payload
        if p['mode']=='attached':
            return self.transforms['suction_contact_frame']@np.linalg.inv(p['box_T_tool'])
        return np.asarray(p['world_pose'])

    def check_payload(self,contact_tool=False,margin=None):
        """Validate designated suction/push contact patch, not whole-link ACM.

        Support contact is permitted only at the horizontal box bottom, with
        actual lower top/ground/platform geometry and bounded penetration.
        """
        if self.payload is None:return None
        margin=self.config.collision_margin_m if margin is None else margin
        contact_tolerance=self.config.contact_tolerance_m
        p=self.payload;transform=self.payload_transform();mesh=p['mesh']
        contact_mode=self.contact_mode(contact_tool)
        contact_frame=self.contact_frame(contact_mode)
        patch_allowed=self.contact_patch_valid(transform,contact_mode) if contact_mode else False
        hit,names,contacts=self.world.in_collision_single(mesh,transform,return_names=True,return_data=True)
        if hit:
            from support_contact import support_contact_failure
            for c in contacts:
                other=next(n for n in c.names if n!='__external')
                support_name=other.startswith('lower_') or other.startswith('pickup_platform:')
                support_failure=None
                if p['allow_support'] and support_name:
                    support_mesh,support_pose=self.world_parts[other]
                    support_failure=support_contact_failure(transform,p['size'],support_pose,
                        support_mesh.bounds,c.point,c.normal,contact_tolerance,
                        self.config.contact_penetration_m,self.config.contact_normal_alignment_deg,
                        c.depth,normal_toward_box=False)
                    if support_failure is None:continue
                    edge_validator=getattr(self,'support_edge_validator',None)
                    if edge_validator and edge_validator(other,transform,support_pose,
                            support_mesh.bounds,c.point,c.normal,c.depth):continue
                return dict(reason='PAYLOAD_WORLD_COLLISION',pair=['payload',other],
                            point=c.point.tolist(),depth_m=float(c.depth),support_diagnostic=support_failure)
        hit,names,contacts=self.robot.in_collision_single(mesh,transform,return_names=True,return_data=True)
        if hit:
            inverse=np.linalg.inv(contact_frame)
            from shapely.geometry import Point
            patch=self.contact_polygon(contact_mode).buffer(contact_tolerance)
            for c in contacts:
                other=next(n for n in c.names if n!='__external')
                local=inverse@np.r_[c.point,1.]
                in_patch=(patch.covers(Point(local[:2])) and
                          abs(local[2])<=contact_tolerance)
                if patch_allowed and other==self.push_geometry.collision_key and in_patch and abs(c.depth)<=self.config.contact_penetration_m:
                    continue
                return dict(reason='ROBOT_PAYLOAD_COLLISION',pair=[other,'payload'],
                            point=c.point.tolist(),depth_m=float(c.depth),point_tool=local[:3].tolist())
        # Distance checks include carried/released cargo, except the declared
        # horizontal support and the validated panel approach/contact patch.
        from trimesh.collision import CollisionManager
        pm=CollisionManager();pm.add_object('payload',mesh,transform)
        bottom=float(transform[2,3]-p['size'][2]/2)
        for name,manager in self.world_managers.items():
            other_mesh,other_tf=self.world_parts[name]
            other_top=float((np.array(other_mesh.vertices)@other_tf[:3,:3].T+other_tf[:3,3])[:,2].max())
            support=(p['allow_support'] and (name.startswith('lower_') or name.startswith('pickup_platform:'))
                     and abs(transform[2,2])>=np.cos(np.deg2rad(self.config.contact_normal_alignment_deg))
                     and bottom>=other_top-self.config.contact_penetration_m)
            if support:continue
            distance=self.bounds_distance(self.transformed_bounds(mesh,transform),self.world_bounds[name])
            if self.exact_distances or distance<margin+2*self.config.cartesian_step_m:
                distance=self.observed_distance(pm,manager,['payload',name])
            self.record_clearance(distance,['payload',name])
            if distance<margin-1e-8:
                return dict(reason='PAYLOAD_WORLD_CLEARANCE',pair=['payload',name],distance_m=float(distance))
        for name,manager in self.pair_managers.items():
            if patch_allowed and name==self.push_geometry.collision_key:continue
            distance=self.bounds_distance(self.transformed_bounds(mesh,transform),self.current_bounds[name])
            if self.exact_distances or distance<margin+2*self.config.cartesian_step_m:
                distance=self.observed_distance(pm,manager,['payload',name])
            self.record_clearance(distance,['payload',name])
            if distance<margin-1e-8:
                return dict(reason='ROBOT_PAYLOAD_CLEARANCE',pair=[name,'payload'],distance_m=float(distance))
        return None

    @staticmethod
    def contact_mode(value):
        if value is True:return 'seal'
        if value is False or value is None:return None
        if value not in ('seal','push_plate'):raise ValueError('UNKNOWN_TOOL_CONTACT_MODE')
        return value

    def contact_frame(self,mode):
        tcp=self.transforms['suction_contact_frame']
        return tcp@np.array(self.push_geometry.tool_T_push_surface).reshape(4,4) if mode=='push_plate' else tcp

    def contact_polygon(self,mode):
        from shapely.geometry import Polygon,box
        if mode=='push_plate':return Polygon(self.push_geometry.push_polygon_xy)
        half=np.array(self.assets['contact']['footprint_size_xy_m'])/2
        return box(-half[0],-half[1],half[0],half[1])

    def validate_push_geometry(self,action):
        if (action.push_contact_geometry_id!=self.push_geometry.geometry_id or
            tuple(action.tool_T_push_surface)!=self.push_geometry.tool_T_push_surface):
            raise ValueError('STALE_PUSH_CONTACT_GEOMETRY')

    def contact_patch_valid(self,box_pose,mode='seal'):
        """Panel normal enters one box face, its whole footprint lies on it.

        Positive separation is allowed along a declared contact approach. Actual
        contact existence/dwell is a distinct execution gate, never inferred
        from this exemption. Tilted/non-face contacts do not obtain a waiver.
        """
        mode=self.contact_mode(mode)
        if mode is None:return False
        tool_in_box=np.linalg.inv(box_pose)@self.contact_frame(mode)
        normal=tool_in_box[:3,2];axis=int(np.argmax(np.abs(normal)))
        minimum_alignment=(np.cos(np.deg2rad(self.config.contact_normal_alignment_deg))
                           if mode=='push_plate' else 1-1e-7)
        if abs(normal[axis])<minimum_alignment:return False
        sign=np.sign(normal[axis]);half=self.payload['size']/2
        gap=-sign*(tool_in_box[axis,3]+sign*half[axis])
        if gap < -self.config.contact_penetration_m:return False
        for u,v in self.contact_polygon(mode).exterior.coords:
            point=tool_in_box@np.array([u,v,0,1])
            for j in range(3):
                if j!=axis and abs(point[j])>half[j]+1e-8:return False
        return True

    def relative_joint_indices(self,pair):
        def ancestors(body):
            if body=='payload':
                if self.payload is None or self.payload['mode']!='attached':return set()
                link='suction_contact_frame'
            elif body in self.robot_parts:link=self.robot_parts[body][0]
            else:return set()
            path=self.urdf.path_to(link)
            return {i for i in range(6) if f'elfin_joint{i+1}' in path}
        return ancestors(pair[0])^ancestors(pair[1])

    def configure_payload_dynamics(self,state):
        if not self.config.payload_effort_screen_enabled:return
        mass=getattr(state,'mass_kg',None)
        if mass is None:raise ValueError('PAYLOAD_MASS_REQUIRED_FOR_EFFORT_SCREEN')
        from payload_dynamics import PayloadGravityModel
        self.payload_dynamics=PayloadGravityModel(self.urdf,mass,state.com_local,
            self.config.gravity_world_mps2,self.config.payload_effort_reserve_nm)

    def check_state(self,q,margin,contact_tool=False):
        self.sample_distances=[]
        self.sample_clearance_m=float('inf')
        failure=self.check_robot(q,margin)
        failure=failure or self.check_payload(contact_tool=contact_tool,margin=margin)
        if failure:return failure
        if self.payload_dynamics is not None and self.payload is not None:
            result=self.payload_dynamics.evaluate(self.transforms,
                self.payload_transform(),self.payload['mode']=='attached')
            if not result['valid']:return result
        return None

    def endpoint_solutions(self,target_container,seeds,margin,budget_s):
        import time
        from luggage_planning.motion.offline_ik import refine_to_pose
        start=time.monotonic();target=self.world_container@np.asarray(target_container).reshape(4,4)
        path=self.urdf.path_to('suction_contact_frame')
        solutions=[];attempts=[]
        for index,seed in enumerate(seeds):
            if time.monotonic()-start>=budget_s:break
            q,info=refine_to_pose(self.urdf,path,target[:3,3],
                                Rotation.from_matrix(target[:3,:3]).as_quat(),seed)
            row=dict(seed_index=index,ik=info)
            if q is not None:
                failure=self.check_robot(q,margin)
                row.update(q=q,collision=failure)
                if failure is None and not any(np.max(np.abs(np.array(q)-old))<1e-3 for old in solutions):
                    solutions.append(q)
            attempts.append(row)
        return solutions,dict(attempts=attempts,elapsed_s=time.monotonic()-start,
                             scope='ROBOT_ONLY_ENDPOINT_NOT_FULL_ACTION')
