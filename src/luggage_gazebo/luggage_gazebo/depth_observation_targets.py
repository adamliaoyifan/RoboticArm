"""Bounded camera intents for non-scored depth acquisition rehearsals."""
import json
import math


def load_depth_targets(path, scene_revision):
    if not path:
        return ()
    with open(path, encoding='utf-8') as stream:
        record=json.load(stream)
    if record.get('frame_id') != 'world' or record.get('scene_revision') != scene_revision:
        raise ValueError('observation target frame/scene mismatch')
    rows=record.get('targets')
    if not isinstance(rows,list) or not 1<=len(rows)<=3:
        raise ValueError('one to three camera targets required')
    result=[]
    for row in rows:
        xyz=tuple(float(v) for v in row['position'])
        quat=tuple(float(v) for v in row['quaternion_xyzw'])
        if len(xyz)!=3 or len(quat)!=4 or not all(map(math.isfinite,xyz+quat)):
            raise ValueError('invalid camera target')
        if abs(sum(v*v for v in quat)-1.)>1e-6:
            raise ValueError('camera quaternion must be unit length')
        result.append((xyz,quat))
    return tuple(result)
