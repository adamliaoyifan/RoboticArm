"""Independent upward-carry -> support transfer -> rear-push hypothesis.

No production pick gate, downward atlas, ROS node, or simulator is called.
The generated states are research targets, never executable robot commands.
"""
from itertools import product
import numpy as np
from scipy.spatial.transform import Rotation
from geometry import axis_box, clipped_at_x, handoff_interval, pose, separated

SETTINGS = dict(stability_margin_m=.02, clearance_m=.005,
                placement_lift_m=.005, step_m=.01,
                release_drop_m=.06, withdraw_m=.15)


def cases(assets):
    catalog = {r['id']:r for r in assets['catalog']['box_catalog']}
    for index, values in enumerate(product(
            ('carryon','standard','large'), (0,1), (.01,.03,.05),
            (0.,.10), (.4,.5,.6), ('center','rear'), (0,90,180,270))):
        name, axis, gap, inset, com, grasp, yaw = values
        size = list(catalog[name]['size'])
        if axis:
            size[0], size[1] = size[1], size[0]
        yield dict(id=f'case_{index:04d}', box=name, insertion_axis=axis,
                   size_m=size, ceiling_gap_m=gap, support_inset_m=inset,
                   com_from_front_fraction=com, grasp=grasp, tool_yaw_deg=yaw)


def payload_at(case, front, bottom, lateral):
    length, width, height = case['size_m']
    return axis_box('payload', [front-length,lateral-width/2,bottom],
                    [front,lateral+width/2,bottom+height])


def build(assets, case):
    """Compute the conservative handoff window and contact-mode states."""
    settings = SETTINGS
    aperture = np.array(assets['scene']['container']['opening']['aperture']['corners'])
    opening_x = float(aperture[0,0]); lateral = float(aperture[:,1].mean())
    length, width, height = case['size_m']
    support_x = opening_x + case['support_inset_m']
    support_z = assets['hull'].ceiling_z - case['ceiling_gap_m'] - height
    rotation = Rotation.from_euler('z', case['tool_yaw_deg'], degrees=True).as_matrix()
    footprint = assets['contact']['footprint_size_xy_m']
    edge = assets['contact']['boundary_margin_m']
    # Footprint width along insertion after tool yaw. Current square model is
    # invariant, but this also handles a future rectangular contact footprint.
    footprint_x = abs(rotation[0,0])*footprint[0] + abs(rotation[0,1])*footprint[1]
    footprint_y = abs(rotation[1,0])*footprint[0] + abs(rotation[1,1])*footprint[1]
    b = length/2 if case['grasp']=='center' else length-footprint_x/2-edge
    rotated = [box.moved(pose([0,0,0], rotation)) for box in assets['tool']]
    forward = max(float(box.corners()[:,0].max()) for box in rotated)
    interval = handoff_interval(length, case['com_from_front_fraction'], b,
                                forward, settings['stability_margin_m'],
                                settings['clearance_m'])
    result = dict(case=case, interval=interval, contact_from_front_m=b,
                  tool_forward_extent_m=forward, support_x_m=support_x,
                  support_z_m=support_z, lateral_m=lateral, stages=[])
    if width < footprint_y+2*edge or min(b,length-b) < footprint_x/2+edge-1e-9:
        return dict(result, outcome='rejected', reason='contact_footprint_outside_box')
    if not interval['feasible']:
        return dict(result, outcome='rejected', reason='handoff_window_empty')
    depth = (interval['min_m']+interval['max_m'])/2
    front = support_x+depth
    settled_box = payload_at(case, front, support_z, lateral)
    stages = []

    def add(name, position, rot, box, mode):
        stages.append(dict(name=name, tcp=pose(position,rot), payload=box,
                           contact_mode=mode))

    before_front = opening_x-.05
    lifted_z = support_z+settings['placement_lift_m']
    add('approach_up', [before_front-b,lateral,lifted_z], rotation,
        payload_at(case,before_front,lifted_z,lateral), 'tool_carries')
    add('insert_up', [front-b,lateral,lifted_z], rotation,
        payload_at(case,front,lifted_z,lateral), 'tool_carries')
    add('handoff', [front-b,lateral,support_z], rotation,
        settled_box, 'tool_and_support')
    add('release_down', [front-b,lateral,support_z-settings['release_drop_m']],
        rotation,settled_box,'support_only')
    add('withdraw', [front-b-settings['withdraw_m'],lateral,
                    support_z-settings['release_drop_m']],rotation,settled_box,'support_only')
    push_rotation = Rotation.from_euler('y',np.pi/2).as_matrix()
    rear = front-length; push_z = support_z+height/2
    add('push_pre', [rear-.05,lateral,push_z],push_rotation,settled_box,'support_only')
    add('push_start', [rear,lateral,push_z],push_rotation,settled_box,'support_and_pusher')
    final_rear = support_x+.02
    add('push_end', [final_rear,lateral,push_z],push_rotation,
        payload_at(case,final_rear+length,support_z,lateral),'support_and_pusher')
    return dict(result, outcome='candidate', reason='handoff_window_found',
                insertion_m=depth, stages=stages)


def boundary_failure(box, assets, margin=None):
    from luggage_description.container_geometry import contains_point_through_aperture
    aperture = np.array(assets['scene']['container']['opening']['aperture']['corners'])
    opening_x = float(aperture[0,0]); margin = SETTINGS['clearance_m'] if margin is None else margin
    clipped = clipped_at_x(box, opening_x)
    for vertex in clipped:
        if not contains_point_through_aperture(assets['hull'],vertex,margin=margin):
            return dict(reason='uld_hull_clearance', body=box.name, vertex=vertex.tolist())
        if abs(vertex[0]-opening_x)<1e-8:
            if (vertex[1]<aperture[:,1].min()+margin-1e-9 or
                vertex[1]>aperture[:,1].max()-margin+1e-9 or
                vertex[2]<aperture[:,2].min()+margin-1e-9 or
                vertex[2]>aperture[:,2].max()-margin+1e-9):
                return dict(reason='aperture_clearance',body=box.name,vertex=vertex.tolist())
    return None


def check_state(assets, result, stage):
    hull = assets['hull']
    # This support prism deliberately over-approximates filled cargo, including
    # the lower chamfer cutout; reject witnesses at that cutout are conservative.
    support = axis_box('support',[result['support_x_m'],-hull.half_y,hull.floor_z],
                        [hull.half_x,hull.half_y,result['support_z_m']])
    payload = stage['payload']
    failure = boundary_failure(payload,assets)
    if failure:
        return failure
    if not separated(payload,support,0):
        return dict(reason='payload_support_penetration',body=payload.record(),obstacle=support.record())
    for fixed in assets['tool']:
        body = fixed.moved(stage['tcp'])
        failure = boundary_failure(body,assets)
        if failure:
            return failure
        if not separated(body,support,SETTINGS['clearance_m']):
            return dict(reason='tool_support_clearance',body=body.record(),obstacle=support.record())
        # Only panel/payload touch is allowed; positive penetration remains
        # rejected even while attached. Other bodies require full clearance.
        clearance = 0 if fixed.name.startswith('suction_panel:') else SETTINGS['clearance_m']
        if not separated(body,payload,clearance):
            return dict(reason='tool_payload_clearance',body=body.record(),obstacle=payload.record())
    return None


def validate(assets, result):
    if result['outcome']=='rejected':
        return result
    checked = 0
    unchecked = []
    stages = result['stages']
    for i, current in enumerate(stages):
        samples = [current]
        if i:
            previous = stages[i-1]
            if not np.allclose(previous['tcp'][:3,:3],current['tcp'][:3,:3]):
                unchecked.append(f"{previous['name']}->{current['name']}:orientation_change")
            else:
                distance = max(np.linalg.norm(current['tcp'][:3,3]-previous['tcp'][:3,3]),
                               np.linalg.norm(current['payload'].center-previous['payload'].center))
                count = max(1,int(np.ceil(distance/SETTINGS['step_m'])))
                samples=[]
                for fraction in np.linspace(0,1,count+1)[1:]:
                    transform=current['tcp'].copy()
                    transform[:3,3]=(1-fraction)*previous['tcp'][:3,3]+fraction*current['tcp'][:3,3]
                    payload=current['payload']
                    from geometry import Box
                    moving_box=Box('payload',(1-fraction)*previous['payload'].center+
                                   fraction*payload.center,payload.axes,payload.half)
                    samples.append(dict(current,tcp=transform,payload=moving_box))
        for sample in samples:
            checked+=1
            witness=check_state(assets,result,sample)
            if witness:
                return dict(result,outcome='rejected',reason=witness['reason'],
                            first_failed_stage=current['name'],witness=witness,
                            failed_tcp=sample['tcp'].tolist(),
                            failed_payload=sample['payload'].record(),samples_checked=checked,
                            unchecked_transitions=unchecked)
    return dict(result,outcome='geometry_candidate',reason='sampled_fixed_tool_checks_pass',
                samples_checked=checked,unchecked_transitions=unchecked)


def record(result):
    result = dict(result)
    result['stages']=[dict(s,tcp=s['tcp'].tolist(),payload=s['payload'].record())
                      for s in result['stages']]
    return result
