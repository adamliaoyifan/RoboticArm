"""Production placement service in an explicitly isolated replay fixture.

This uses the production subscription admission and service callback, with a
recorded surface. Runtime registry admission is not emulated; this is not F2.
"""
import importlib.util
from pathlib import Path
from rclpy.parameter import Parameter


def create_baseline_placement(scene_path, scene_revision, surface, parameters, recorder):
    if not isinstance(parameters, dict):
        raise ValueError('explicit placement parameter mapping required')
    reserved = {'frame_runtime','frame_identity_json','frame_scene_revision','scene_tf_config','use_sim_time'}
    if reserved.intersection(parameters):
        raise ValueError('fixture identity and runtime parameters cannot be overridden')
    path = Path(__file__).resolve().parents[2]/'src/luggage_packing/scripts/placement_planner_node.py'
    spec = importlib.util.spec_from_file_location('prior_baseline_placement_node',path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    class CapturedPlacementPlanner(module.PlacementPlannerNode):
        def _handle_v2(self, request, response):
            recorder('compute_server:request',request)
            response = super()._handle_v2(request,response)
            recorder('compute_server:response',response)
            return response

    configured = dict(parameters, scene_tf_config=str(scene_path),
                      frame_scene_revision=scene_revision,frame_runtime=False,use_sim_time=False)
    node = CapturedPlacementPlanner(use_global_arguments=False,
        parameter_overrides=[Parameter(key,value=value) for key,value in configured.items()])
    try:
        for key in parameters:
            node.get_parameter(key)  # Unknown overrides must not be silently ignored.
        node._on_surface_v2(surface)
        if node._admitted_map_revision != surface.map_revision:
            raise ValueError('production placement subscriber did not admit recorded surface')
    except Exception:
        node.destroy_node()
        raise
    return node
