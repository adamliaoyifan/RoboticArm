"""Offline identity fault injection into the actual short-chain reader; no ROS."""
import json
import sys
from types import SimpleNamespace as NS, ModuleType
from unittest.mock import patch

sys.path.insert(0, '/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/benchmark')
import campaign_executor as ce

srv = ModuleType('uld_occ_observer.srv')
for name in ('GetSnapshot', 'ReadCells', 'ReadSurface', 'ReleaseSnapshot'):
    setattr(srv, name, NS(Request=NS))

def identity(epoch, observed, revision=1):
    return NS(epoch=epoch, observed_revision=observed, map_revision=revision,
              geometry_hash='geometry', config_digest='config',
              last_integrated_stamp_ns=100, clock_domain='ros')

def run(wait_identity, read_identity):
    class Session:
        node = NS(create_client=lambda *args: args[1])
        def wait_for(self, *args):
            return NS(status='OK', reason='', age_ns=10, handle='', identity=wait_identity)
        def wait_service(self, *args, **kwargs):
            pass
        def call(self, client, request, timeout):
            if client.endswith('/get_snapshot'):
                return NS(status='OK', age_ns=20, handle='fixture-handle', identity=read_identity)
            if client.endswith('/read_surface'):
                return NS(status='OK', done=True, identity=read_identity, x=[0], next_cursor=0)
            if client.endswith('/read_cells'):
                return NS(status='OK', identity=read_identity, states=[0]*4096)
            return NS(status='OK')
    with patch.dict(sys.modules, {'uld_occ_observer': ModuleType('uld_occ_observer'), 'uld_occ_observer.srv': srv}):
        return ce._read_snapshot_chain(Session(), {'epoch': 1, 'observation_id': 'target'})

cases = {
    'healthy_control': run(identity(1, 7), identity(1, 7)),
    'wrong_epoch': run(identity(1, 7), identity(2, 1)),
    'target_not_covered': run(identity(1, 7), identity(1, 6)),
    'map_revision_mismatch': run(identity(1, 7, 2), identity(1, 7, 1)),
}
print(json.dumps(cases, indent=2))
assert all(case['identity_stable'] for case in cases.values())
