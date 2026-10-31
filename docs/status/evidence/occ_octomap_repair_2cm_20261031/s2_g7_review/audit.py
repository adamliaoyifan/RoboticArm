"""Offline response injection, no live ROS or performance measurement."""
import json
import sys
from types import SimpleNamespace as NS, ModuleType
from unittest.mock import patch
sys.path.insert(0, '/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark')
import campaign_executor as ce
srv = ModuleType('uld_occ_observer.srv')
for name in ('ReadCells', 'ReadSurface', 'ReleaseSnapshot'):
    setattr(srv, name, NS(Request=NS))

def run(count, observed=7, minimum=None):
    identity = NS(epoch=1, map_revision=1, observed_revision=observed,
                  last_integrated_stamp_ns=100, geometry_hash='geometry',
                  config_digest='config', clock_domain='ros')
    class Session:
        node = NS(create_client=lambda *args: args[1])
        def wait_for(self, *args):
            return NS(status='OK', reason='', age_ns=10, handle='lease', identity=identity)
        def wait_service(self, *args, **kwargs):
            pass
        def call(self, client, request, timeout):
            if client.endswith('/read_surface'):
                return NS(status='OK', done=True, identity=identity, x=[0], y=[0],
                          active_count=[1], known_count=[1], has_occupied=[True], top_z=[0.02])
            if client.endswith('/read_cells'):
                return NS(status='OK', identity=identity, states=[2]*count)
            return NS(status='OK')
    expected = {'epoch': 1, 'sensor_id': 'cam0', 'observation_id': 'target'}
    if minimum is not None:
        expected['min_observed_revision'] = minimum
    with patch.dict(sys.modules, {'uld_occ_observer': ModuleType('uld_occ_observer'), 'uld_occ_observer.srv': srv}):
        return ce._read_snapshot_chain(Session(), expected)

results = {'full_roi_control': run(4096), 'missing_4095_roi_cells': run(1),
           'target_minimum_supplied': run(4096, 6, 7),
           'target_minimum_omitted_like_live_caller': run(4096, 6)}
print(json.dumps(results, indent=2))
assert results['missing_4095_roi_cells']['identity_stable'] is True
assert results['target_minimum_supplied']['identity_stable'] is False
assert results['target_minimum_omitted_like_live_caller']['identity_stable'] is True
