"""Recorder main under controlled stream/signal ports, preserving raw failures."""
import hashlib,importlib.util,json,signal,subprocess,sys
from pathlib import Path
from types import SimpleNamespace
import pytest
from pickup_continuation_fixtures import case_archive


def recorder_module():
    spec=importlib.util.spec_from_file_location('reference_lifecycle_fixture',Path(__file__).with_name('physics_reference.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


@pytest.mark.parametrize('stop_kind',['requested','unexpected'])
@pytest.mark.parametrize('pose_present',[True,False])
def test_recorder_main_distinguishes_requested_stop_from_lost_stream(case_archive,monkeypatch,stop_kind,pose_present):
    module=recorder_module(); events=[]; handlers={}; clock=[100.]; reads=[0]
    raw=(b'header { stamp { sec: 7 nsec: 12 } }\n'
         b'collision1 { name: "box::body" }\ncollision2 { name: "uld::floor" }\n\n')
    capture=case_archive.directory/'reference';run=case_archive.directory/'runtime';run.mkdir()
    fake_stdout=(case_archive.directory/'stream_handle').open('w+b')
    process=SimpleNamespace(stdout=fake_stdout,returncode=None)
    def monotonic():clock[0]+=.1;return clock[0]
    def terminate():events.append('stream_terminate');process.returncode=0
    process.poll=lambda:process.returncode;process.terminate=terminate
    process.wait=lambda **kwargs:process.returncode;process.kill=terminate
    def select(readers,*args):
        events.append('stream_read_ready')
        if reads[0]:
            process.returncode=0
            if stop_kind=='requested':
                events.append('requested_stop');handlers[signal.SIGINT]()
        return readers,[],[]
    def read(*args):
        reads[0]+=1;events.append('contact_data' if reads[0]==1 else 'contact_eof')
        return raw if reads[0]==1 else b''
    def pose(*args,**kwargs):
        events.append('pose_read');return SimpleNamespace(returncode=0,stdout=b'controlled_pose_payload' if pose_present else b'')
    monkeypatch.setattr(module,'signal',SimpleNamespace(SIGINT=signal.SIGINT,SIGTERM=signal.SIGTERM,
        signal=lambda number,callback:handlers.__setitem__(number,callback)))
    monkeypatch.setattr(module,'subprocess',SimpleNamespace(Popen=lambda *a,**k:process,run=pose,TimeoutExpired=subprocess.TimeoutExpired,PIPE=subprocess.PIPE))
    monkeypatch.setattr(module,'select',SimpleNamespace(select=select))
    monkeypatch.setattr(module,'os',SimpleNamespace(set_blocking=lambda *a:None,read=read,environ={}))
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=monotonic,time=lambda:clock[0]))
    monkeypatch.setattr(sys,'argv',['physics_reference.py','--run-dir',str(run),'--output',str(capture),
        '--contact-topic','/fixture/contacts'])
    case_archive.save('reference_fixture',dict(stop_kind=stop_kind,pose_present=pose_present,
        source_sha256=hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest(),
        ports='Actual recorder main with controlled process/pipe/signal and pose payload; no physical capture claim'))
    case_archive.save('raw_contact_bytes',raw)
    try:
        code=module.main(); summary=json.loads((capture/'summary.json').read_text())
        case_archive.save('reference_terminal_summary',summary)
        assert code==(0 if stop_kind=='requested' else 1)
        assert summary['capture_complete']==(stop_kind=='requested' and pose_present)
        assert summary['messages']==1
        if stop_kind=='unexpected':assert summary['error']=='RuntimeError:CONTACT_STREAM_EXIT'
    finally:
        fake_stdout.close();case_archive.save('reference_boundary_events',dict(events=events,clock=clock[0],reads=reads[0]))
        for path in capture.glob('*'):
            if path.is_file():case_archive.save('capture_'+path.name,path.read_bytes())
