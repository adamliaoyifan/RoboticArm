"""Small executable checks; not a throughput benchmark."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import yaml
import pytest

BINARY = os.environ.get('OBSERVER_PROFILE_CLI')
pytestmark = pytest.mark.skipif(not BINARY, reason='set OBSERVER_PROFILE_CLI to the freshly built executable')


def setup_fixture(root):
    descriptor=dict(schema_version=1,frame_id='container_link',length=.8,width=.8,
                    floor_z=0,ceiling_z=1.2,geometry_hash='fixture')
    data=json.dumps(descriptor,sort_keys=True,separators=(',',':'))
    (root/'geometry').write_text(yaml.safe_dump(dict(schema='uld_geometry_export_v1',descriptor_json=data,
                        descriptor_sha256=hashlib.sha256(data.encode()).hexdigest())))
    (root/'config').write_text('resolution_m: 0.05\ndepth_z_min_m: 0.52\ndepth_z_max_m: 6.0\nclock_domain: ros\n')
    frame=dict(sensor_id='test',observation_id='1',optical_frame='optical',clock_domain='ros',
               geometry_hash='fixture',epoch=1,stamp_ns=1,points_m=[[0,0,.6]],
               transform=dict(stamp_ns=1,translation=[0,0,0],rotation=[1,0,0,0,1,0,0,0,1]))
    (root/'frame').write_text(yaml.safe_dump(frame))
    manifest=dict(case_id='test',mode='stages',surface=True,frames=[dict(path='frame',sha256=hashlib.sha256((root/'frame').read_bytes()).hexdigest())])
    (root/'manifest').write_text(yaml.safe_dump(manifest))
    return manifest


def run(root):
    return subprocess.run([BINARY,'--geometry',str(root/'geometry'),'--config',str(root/'config'),
                           '--manifest',str(root/'manifest'),'--out',str(root/'out')],capture_output=True,text=True)


def test_valid_record_and_cache(tmp_path):
    setup_fixture(tmp_path)
    result=run(tmp_path)
    assert result.returncode==0, result.stderr
    row=list(yaml.safe_load_all((tmp_path/'out').read_text()))[0]
    assert row['receipt']['profile']['rays_attempted']==1
    assert row['receipt']['profile']['split_key_timers']=='not_provided'
    assert 'domain_check_ms' not in row['receipt']['profile']
    first=row['surface_first_call']['profile']
    second=row['surface_second_call']['profile']
    assert first['domain_init']=='collected' and first['domain_init_performed'] is True
    assert first['domain_init_scanned_cells']>0 and first['scanned_cells']>=0
    assert second['cache_hit'] is True and second['scanned_cells']==0
    assert second['domain_init_performed'] is False and second['domain_init_scanned_cells']==0
    assert isinstance(row['receipt']['profile']['stage_ms']['fingerprint'],(int,float))


def test_off_omits_uncollected_diagnostics(tmp_path):
    manifest=setup_fixture(tmp_path)
    manifest['mode']='off'
    (tmp_path/'manifest').write_text(yaml.safe_dump(manifest))
    result=run(tmp_path)
    assert result.returncode==0, result.stderr
    row=list(yaml.safe_load_all((tmp_path/'out').read_text()))[0]
    assert row['receipt']['profile']['split_key_timers']=='not_provided'
    assert row['receipt']['profile']['rays_attempted']==0
    assert 'domain_check_ms' not in row['receipt']['profile']
    assert row['surface_first_call']['profile_enabled'] is False
    assert 'profile' not in row['surface_first_call']


def test_bad_hash_preserves_named_failure(tmp_path):
    manifest=setup_fixture(tmp_path)
    manifest['frames'][0]['sha256']='bad'
    (tmp_path/'manifest').write_text(yaml.safe_dump(manifest))
    assert run(tmp_path).returncode==3
    row=list(yaml.safe_load_all((tmp_path/'out').read_text()))[0]
    assert row['status']=='INPUT_HASH_MISMATCH'
    assert 'receipt' not in row


def test_existing_evidence_not_overwritten(tmp_path):
    setup_fixture(tmp_path)
    (tmp_path/'out').write_text('original evidence')
    assert run(tmp_path).returncode==3
    assert (tmp_path/'out').read_text()=='original evidence'


def test_invalid_mode_rejected(tmp_path):
    manifest=setup_fixture(tmp_path);manifest['mode']='auto'
    (tmp_path/'manifest').write_text(yaml.safe_dump(manifest))
    assert run(tmp_path).returncode==3
