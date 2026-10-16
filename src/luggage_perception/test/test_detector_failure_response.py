"""Exercise the real service failure branches that previously killed the node."""
import importlib.util
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest


@pytest.fixture
def detector_class():
    path = Path(__file__).parents[1] / 'scripts/luggage_detector_node.py'
    spec = importlib.util.spec_from_file_location('detector_failure_response', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LuggageDetector


@pytest.mark.parametrize('no_patch', [False, True])
def test_failure_response_returns_without_raising(detector_class, no_patch):
    from luggage_msgs.msg import DetectedLuggage
    detected = DetectedLuggage() if no_patch else None
    if detected is not None:
        detected.top_surface_valid = True
    fake = SimpleNamespace(
        _wait_suitcase_view=lambda: True, _wait_instance_ready=lambda: True,
        _detect_from_window_with_retries=lambda: (detected, .8),
        _use_semantic=True, _top_surface_mode='dynamic',
        _last_failure_reason='DETECT_SUPPORT_UNOBSERVABLE',
        _last_plane_detail='preserved measured failure',
        _publish_diagnostics=lambda *args: None,
        get_logger=lambda: SimpleNamespace(warning=lambda text: None))
    fake._failure_message = MethodType(detector_class._failure_message, fake)
    response = detector_class.handle_detect(fake, None, None)
    assert not response.success
    expected = 'DETECT_NO_SEALABLE_PATCH' if no_patch else 'DETECT_SUPPORT_UNOBSERVABLE'
    assert response.message == expected + ': preserved measured failure'
    assert len(response.luggage) == int(no_patch)


def test_implicit_stale_reason_keeps_age_diagnostic(detector_class):
    fake = SimpleNamespace(_last_failure_reason='DETECT_STALE_CLOUD',
                           _last_cloud_age_sec=2., _cloud_max_age=1.)
    assert detector_class._failure_message(fake) == 'DETECT_STALE_CLOUD (age=2.00s max=1.0s)'
