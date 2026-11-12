"""Offline boundary probe, not a successful native/full-operation run."""
from pathlib import Path
import hashlib
import json

from moveit_msgs.msg import PlanningScene
from rclpy.serialization import deserialize_message
from luggage_planning.placement_prior_contracts import Transform
from luggage_planning.placement_prior_snapshot import build_private_snapshot
from luggage_planning.placement_prior_surface import ConfirmedSurface

surface = dict(frame_id='container_link', scene_revision='offline-probe', map_revision=1,
    geometry_hash='offline-hull', floor_z_container=.53, ceiling_z_container=2.01,
    resolution_m=.01, origin_xy_container=[0, 0], state=[['free']],
    surface_z_container=[[.53]])
frozen = ConfirmedSurface(json.dumps(surface).encode(), b'surface', b'confirmation')
baseline = PlanningScene()
baseline.is_diff = False
snapshot = build_private_snapshot(frozen, baseline, 'offline-model',
    Transform((0, 0, 0), (0, 0, 0, 1)), collision_margin_m=.05)
decoded = deserialize_message(snapshot.scene_cdr, PlanningScene)
result = dict(scope='Synthetic ROS message boundary only; no native or motion run',
    baseline_world_objects=len(baseline.world.collision_objects),
    private_world_objects=len(decoded.world.collision_objects),
    empty_static_world_accepted=True,
    implication='Snapshot builder does not require deployment static geometry; callers must enforce it',
    source_sha256=hashlib.sha256(Path(
        'src/luggage_planning/luggage_planning/placement_prior_snapshot.py').read_bytes()).hexdigest())
assert result['private_world_objects'] == 0
Path(__file__).with_name('summary.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
