import importlib.util
from pathlib import Path
_original=importlib.util.spec_from_file_location

def pytest_configure(config):
    def load(name,location,*args,**kwargs):
        if str(location).endswith('/scripts/placement_planner_node.py'):
            location=Path(__file__).parent/'placement_planner_node.py'
        return _original(name,location,*args,**kwargs)
    importlib.util.spec_from_file_location=load
