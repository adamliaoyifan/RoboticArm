"""T1 preserves join data, bounded memory and visible overflow."""
import gc
import importlib.util
from pathlib import Path


def buffer(capacity):
    spec=importlib.util.spec_from_file_location('trace_buffer',Path(__file__).with_name('occ_query_trace_buffer.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.SnapshotTraceBuffer(capacity)


def test_atomic_event_fields_roundtrip_and_explicit_overflow():
    trace=buffer(2)
    trace.record('roi-0',1_000_000_000_000_000,123)
    trace.record('distance-1',1_000_000_000_000_456,789)
    trace.record('overflow',7,8)
    assert list(trace.rows())==[
        dict(stage='snapshot',request_id='roi-0',started_ns=1_000_000_000_000_000,elapsed_ns=123),
        dict(stage='snapshot',request_id='distance-1',started_ns=1_000_000_000_000_456,elapsed_ns=789)]
    assert trace.dropped==1 and trace.size==2


def test_recording_retains_no_per_request_gc_containers():
    trace=buffer(5000)
    gc.collect()
    # The buffer itself is already allocated. Atomic request strings and
    # counters need no tracked event dict/tuple per call.
    before=len(gc.get_objects())
    for i in range(5000):trace.record('request-'+str(i),10000+i,5)
    after=len(gc.get_objects())
    assert after-before<10
    assert trace.size==5000 and trace.dropped==0
