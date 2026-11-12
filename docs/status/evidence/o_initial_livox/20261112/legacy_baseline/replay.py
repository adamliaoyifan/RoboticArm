from pathlib import Path
import runpy
import sys
runpy.run_path('scripts/production/test_source.py',run_name='_source_environment')
original=runpy.run_path
head_source=Path(__file__).parent/'placement_planner_node.py'
def load(path,*args,**kwargs):
    if str(path).endswith('/scripts/placement_planner_node.py'):
        path=str(head_source)
    return original(path,*args,**kwargs)
runpy.run_path=load
import pytest
raise SystemExit(pytest.main(['-q','src/luggage_packing/test/test_frame_migration_placement.py','-k',
    'review3_v2_surface_to_service_continuity or review_expected_map_revision_gate or phantom_placed_copy_rejected_ledger_source_used']))
