"""Configuration-independent point travel bounds from the source kinematic tree.

For a revolute joint, the downstream sum of origin translations plus the body's
maximum local point norm bounds its distance from that joint axis at every q.
Shared ancestors move both bodies rigidly and cancel from relative distance.
"""
import numpy as np


class SourceMotionBounds:
    def __init__(self,urdf,robot_parts,static_names=()):
        self.urdf=urdf;self._static_names=set(static_names);self._radii={};self._axes={};self._pair_weights={}
        for name,(link,mesh,local) in robot_parts.items():
            points=np.array(mesh.vertices)@local[:3,:3].T+local[:3,3]
            self._set_body(name,link,float(np.linalg.norm(points,axis=1).max()))
        self.set_payload(None)

    def _set_body(self,name,link,extent):
        path=self.urdf.path_to(link);radii=np.zeros(6);axes=set()
        for j in range(6):
            joint=f'elfin_joint{j+1}'
            if joint not in path:continue
            position=path.index(joint)
            radii[j]=extent+sum(np.linalg.norm(self.urdf.joints[k]['xyz'])
                               for k in path[position+1:])
            axes.add(j)
        self._radii[name]=radii;self._axes[name]=axes

    def set_payload(self,payload):
        # World payload has no moving robot ancestors. Attached payload extent
        # includes its measured/candidate TCP-to-box offset and every box corner.
        self._radii['payload']=np.zeros(6);self._axes['payload']=set()
        if payload is not None and payload['mode']=='attached':
            tool_box=np.linalg.inv(payload['box_T_tool'])
            points=np.array(payload['mesh'].vertices)@tool_box[:3,:3].T+tool_box[:3,3]
            self._set_body('payload','suction_contact_frame',float(np.linalg.norm(points,axis=1).max()))
        self._pair_weights.clear()

    def register_static(self,name):
        if name in self._radii:raise ValueError('STATIC_BODY_NAME_COLLISION:'+name)
        self._static_names.add(name)

    def remove_static(self,name):
        self._static_names.remove(name);self._pair_weights.clear()

    def joint_point_radii(self,body):
        return tuple(self._radii[body])

    def relative_motion_coefficients(self,pair):
        return tuple(self._weights(pair))

    def _weights(self,pair):
        unknown=set(pair)-self._radii.keys()-self._static_names
        if unknown:raise ValueError('UNKNOWN_MOTION_BODY:'+','.join(sorted(unknown)))
        key=tuple(sorted(pair))
        if key not in self._pair_weights:
            a,b=key;axes=self._axes.get(a,set())^self._axes.get(b,set())
            weights=np.zeros(6)
            for j in axes:weights[j]=self._radii.get(a,np.zeros(6))[j]+self._radii.get(b,np.zeros(6))[j]
            self._pair_weights[key]=weights
        return self._pair_weights[key]

    def relative_motion_bound(self,pair,absolute_joint_delta):
        return float(self._weights(pair)@absolute_joint_delta)
