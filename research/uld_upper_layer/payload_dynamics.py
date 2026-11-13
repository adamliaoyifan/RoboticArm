"""Source URDF quasistatic payload effort screen; not measured joint torque.

Gravity, Coulomb friction and a configurable damping reserve are necessary
planning checks. Acceleration/coupling/contact forces need physical validation.
No actuator limit or simulation material is changed by this module.
"""
import time
import numpy as np


class PayloadGravityModel:
    def __init__(self,urdf,mass_kg,com_local,gravity,reserve_nm):
        self.urdf=urdf;self.mass=float(mass_kg);self.com=np.asarray(com_local)
        self.gravity=np.asarray(gravity);self.reserve=float(reserve_nm)
        self.joints=[];self.links=[]
        for i in range(6):
            name=f'elfin_joint{i+1}';source=urdf.root.find(f"joint[@name='{name}']")
            dynamics=source.find('dynamics')
            self.joints.append((name,urdf.joints[name],float(source.find('limit').get('effort')),
                float(dynamics.get('friction','0')) if dynamics is not None else 0.,
                float(dynamics.get('damping','0')) if dynamics is not None else 0.))
        for link in urdf.root.findall('link'):
            inertial=link.find('inertial')
            if inertial is None:continue
            origin=inertial.find('origin');com=np.array([float(x) for x in origin.get('xyz','0 0 0').split()]) if origin is not None else np.zeros(3)
            self.links.append((link.get('name'),float(inertial.find('mass').get('value')),com,
                set(urdf.path_to(link.get('name')))))
        self.damping=np.array([j[4] for j in self.joints])

    def evaluate(self,transforms,box_pose,attached):
        points=[];forces=[];ancestors=[]
        for name,mass,com,path in self.links:
            tf=transforms[name];points.append(tf[:3,3]+tf[:3,:3]@com)
            forces.append(mass*self.gravity);ancestors.append(path)
        if attached:
            points.append(box_pose[:3,3]+box_pose[:3,:3]@self.com)
            forces.append(self.mass*self.gravity);ancestors.append({j[0] for j in self.joints})
        points=np.array(points).reshape(-1,3);forces=np.array(forces).reshape(-1,3);torques=[];budgets=[]
        for name,joint,limit,friction,_ in self.joints:
            parent=transforms[joint['parent']]
            origin=parent[:3,3]+parent[:3,:3]@joint['xyz']
            axis=parent[:3,:3]@joint['rot']@joint['axis']
            mask=np.array([name in path for path in ancestors],dtype=bool)
            torque=-float(np.sum(np.cross(points[mask]-origin,forces[mask])@axis))
            torques.append(torque);budgets.append(limit-friction-self.reserve)
        required=np.abs(torques);budgets=np.array(budgets);index=int(np.argmax(required-budgets))
        return dict(valid=bool(np.all(required<=budgets)),gravity_torque_nm=torques,
            gravity_budget_nm=budgets.tolist(),joint=index+1,
            reason='QUASISTATIC_EFFORT_SCREEN' if np.all(required<=budgets) else 'QUASISTATIC_EFFORT_LIMIT',
            scope='source_gravity_friction_damping_reserve_not_full_inverse_dynamics')


def screen_path(scene,path,config,deadline=None):
    model=getattr(scene,'payload_dynamics',None)
    if model is None:return dict(valid=True,scope='not_enabled')
    checks=0
    for qa,qb in zip(path,path[1:]):
        count=max(1,int(np.ceil(np.max(np.abs(np.array(qb)-qa))/config.payload_effort_joint_step_rad)))
        for t in np.linspace(0,1,count+1):
            if deadline is not None and time.monotonic()>deadline:
                return dict(valid=False,reason='EFFORT_SCREEN_DEADLINE',checks=checks)
            q=(1-t)*np.array(qa)+t*np.array(qb);tf=scene.link_transforms(q)
            attached=scene.payload['mode']=='attached'
            box=(tf['suction_contact_frame']@np.linalg.inv(scene.payload['box_T_tool']) if attached else None)
            result=model.evaluate(tf,box,attached);checks+=1
            if not result['valid']:return dict(result,q=q.tolist(),checks=checks)
    return dict(valid=True,checks=checks,joint_step_rad=config.payload_effort_joint_step_rad,
                scope='sampled_source_quasistatic_effort_only')
