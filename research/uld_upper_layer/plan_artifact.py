"""Read-only scored-plan identity and source validation for execution consumers."""
import json
from pathlib import Path
from assets import sha256
from planning_types import UpperConfig,digest
from selection_policy import SelectionPolicy
from plan_selection import rank_nominal_plans


def load_scored_plan(planning_run,policy):
    planning_run=Path(planning_run)
    planning_manifest=json.loads((planning_run/'manifest.json').read_text())
    config=UpperConfig.load(planning_run/'source/upper_config.yaml')
    scene_path=planning_manifest.get('scene_config')
    if scene_path and sha256(scene_path)!=planning_manifest['scene_config_sha256']:
        raise ValueError('PLANNED_SCENE_CONFIG_CHANGED')
    plans=json.loads((planning_run/'plans.json').read_text())
    if not plans:raise ValueError('NO_COMPLETE_NOMINAL_PLAN')
    selection_policy=SelectionPolicy.load(planning_run/'source/selection_policy.yaml')
    if (sha256(planning_run/'source/selection_policy.yaml')!=planning_manifest['source_hashes'].get('selection_policy.yaml') or
        sha256(planning_run/'source/upper_config.yaml')!=planning_manifest['source_hashes'].get('upper_config.yaml')):
        raise ValueError('PLANNING_CONFIG_ARCHIVE_CHANGED')
    if (sha256(planning_run/'source/execution_policy.yaml')!=planning_manifest['source_hashes'].get('execution_policy.yaml') or
        planning_manifest.get('execution_policy_revision')!=policy.revision):
        raise ValueError('PLANNED_EXECUTION_POLICY_CHANGED')
    models=json.loads((planning_run/'models.json').read_text())
    planning_inputs=json.loads((planning_run/'inputs.json').read_text())
    selection=rank_nominal_plans(plans,planning_inputs['snapshot']['snapshot_id'],config,selection_policy,
        digest(dict(assets=models['asset_hashes'],meshes=models['hashes'])),policy)
    if not selection['nominal_minimum_met']:raise ValueError('NO_SCORED_NOMINAL_PLAN:'+str(selection['rejected']))
    plan=plans[selection['selected_index']]
    if planning_manifest.get('request_revision'):
        from request_planner import request_from_record
        request=request_from_record(json.loads((planning_run/'request.json').read_text()))
        if (request.revision!=planning_manifest['request_revision'] or
                plan.get('request_revision')!=request.revision):
            raise ValueError('PLANNED_REQUEST_BINDING_CHANGED')
    for name in ('candidates.py','placement_request.py','request_planner.py','planning_types.py','full_arm.py','tool_contact_geometry.py','support_contact.py',
                 'payload_dynamics.py','geometry.py','assets.py','path_planning.py','stage_planner.py','action_screen.py','coupled_chain.py','interval_certificate.py','motion_bounds.py',
                 'plan_clearance.py','push_seam_events.py','clearance_bounds.py','selection_policy.py','plan_selection.py',
                 'free_box_validation.py','noncontact_admission.py','payload_envelope.py','payload_tube.py','envelope_planning.py','execution_policy.py'):
        if sha256(Path(__file__).with_name(name))!=planning_manifest['source_hashes'].get(name):
            raise ValueError('PLANNED_SOURCE_CHANGED:'+name)
    return dict(plan=plan,config=config,scene_path=scene_path,manifest=planning_manifest,
                selection=selection,selection_policy=selection_policy)
