"""Bounded typed contact-quality service and perception publication boundary."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_msgs.msg import DirectedSuctionQueryV2
from luggage_msgs.srv import EvaluateDirectedSuctionV2
from luggage_perception.directed_suction_quality import DirectedSuctionContext
from luggage_perception.directed_suction_service import DirectedSuctionService
from test_directed_suction_quality import context_inputs


def request_and_context():
    domain, hmap, model = context_inputs()
    context = DirectedSuctionContext(domain, hmap, model)
    request = EvaluateDirectedSuctionV2.Request()
    request.domain_sha256 = context.domain_sha256
    request.generation = domain.generation
    box = request.detection
    box.id = domain.box_id; box.header.frame_id = 'world'; box.header.stamp.sec = 12
    box.pose.position.x, box.pose.position.y, box.pose.position.z = domain.box_xyz_world
    box.pose.orientation.w = 1.
    box.width, box.depth, box.height = domain.box_wdh
    box.top_surface_valid = box.height_valid = True
    box.height_source = box.HEIGHT_SOURCE_MEASURED_SUPPORT
    first = DirectedSuctionQueryV2(query_id='clear', center_uv_top_m=[.0013, -.0027], yaw_rad=.17)
    second = DirectedSuctionQueryV2(query_id='overhang', center_uv_top_m=[0., .12], yaw_rad=np.pi/2)
    request.queries = [first, second]
    request.max_refinements, request.budget_s = 3, 1.
    return request, context


def test_batch_returns_ordered_individual_verdicts_and_generated_cdr_preserves_identity():
    from rclpy.serialization import serialize_message, deserialize_message
    request, context = request_and_context()
    request = deserialize_message(serialize_message(request), EvaluateDirectedSuctionV2.Request)
    handler = DirectedSuctionService(clock=lambda:0.); handler.replace_context(context)
    response = handler.handle(request, EvaluateDirectedSuctionV2.Response())
    response = deserialize_message(serialize_message(response), EvaluateDirectedSuctionV2.Response)
    assert response.success and response.reason == 'QUALITY_BATCH_COMPLETE'
    assert response.header.frame_id == 'world' and response.header.stamp.sec == 12
    assert response.domain_sha256 == context.domain_sha256
    assert response.quality_policy_sha256 == context.policy_sha256
    assert response.samples_sha256 == context.samples_sha256
    assert [c.query_id for c in response.contacts] == ['clear', 'overhang']
    assert [c.accepted for c in response.contacts] == [True, False]
    first = response.contacts[0]
    assert np.array(first.world_from_contact_frame).reshape(4, 4)[:2, 3] == pytest.approx([.0013, -.0027])
    assert json.loads(first.quality_metrics_json)['rms_residual'] < 1e-14
    assert not np.any(response.contacts[1].world_from_contact_frame)


@pytest.mark.parametrize('damage', ['domain', 'policy', 'generation', 'geometry', 'stamp',
    'configured_height', 'no_context', 'duplicate_id', 'empty_id', 'nan', 'no_queries',
    'zero_budget', 'zero_refinements', 'excess_refinements'])
def test_invalid_or_unbound_batch_never_returns_contacts(damage):
    request, context = request_and_context()
    if damage == 'domain': request.domain_sha256 = 'other'
    elif damage == 'policy': request.expected_quality_policy_sha256 = 'other'
    elif damage == 'generation': request.generation += 1
    elif damage == 'geometry': request.detection.width += .001
    elif damage == 'stamp': request.detection.header.stamp.sec += 1
    elif damage == 'configured_height': request.detection.height_source = request.detection.HEIGHT_SOURCE_CONFIGURED_SUPPORT
    elif damage == 'no_context': context = None
    elif damage == 'duplicate_id': request.queries[1].query_id = request.queries[0].query_id
    elif damage == 'empty_id': request.queries[0].query_id = ''
    elif damage == 'nan': request.queries[0].yaw_rad = float('nan')
    elif damage == 'no_queries': request.queries = []
    elif damage == 'zero_budget': request.budget_s = 0.
    elif damage == 'zero_refinements': request.max_refinements = 0
    else: request.max_refinements = 9
    handler = DirectedSuctionService(clock=lambda:0.); handler.replace_context(context)
    response = handler.handle(request, EvaluateDirectedSuctionV2.Response())
    assert not response.success and not response.contacts


def test_batch_deadline_discards_previously_computed_contacts_and_reused_response():
    request, context = request_and_context()
    clock = SimpleNamespace(value=0.)
    handler = DirectedSuctionService(clock=lambda:clock.value)
    handler.replace_context(context)
    response = handler.handle(request, EvaluateDirectedSuctionV2.Response())
    assert response.success

    class SlowContext:
        def __getattr__(self, name): return getattr(context, name)
        def evaluate(self, *args, **kwargs):
            result = context.evaluate(*args, **kwargs)
            clock.value += .6
            return result

    handler.replace_context(SlowContext())
    result = handler.handle(request, response)
    assert result.reason == 'QUALITY_DEADLINE'
    assert not result.success and not result.contacts


def test_inflight_batch_retains_its_pinned_snapshot_when_producer_invalidates_context():
    request, context = request_and_context()
    handler = DirectedSuctionService(clock=lambda:0.)

    class ReplacedContext:
        def __getattr__(self, name): return getattr(context, name)
        def evaluate(self, *args, **kwargs):
            handler.replace_context(None)
            return context.evaluate(*args, **kwargs)

    handler.replace_context(ReplacedContext())
    first = handler.handle(request, EvaluateDirectedSuctionV2.Response())
    second = handler.handle(request, EvaluateDirectedSuctionV2.Response())
    assert first.success and len(first.contacts) == 2
    assert not second.success and second.reason == 'OBSERVED_QUALITY_CONTEXT_UNAVAILABLE'


def test_detector_pins_matching_context_and_invalid_current_frame_clears_it():
    import importlib.util
    from luggage_msgs.msg import DetectionFrame
    from test_observed_suction_domain import detection_and_evaluation
    request, _ = request_and_context()
    detection, evaluation = detection_and_evaluation()
    from luggage_perception.ros_suction_domain import observed_domain_for_detection
    domain = observed_domain_for_detection(detection, evaluation)
    context = SimpleNamespace(domain_sha256=domain.digest())
    path = Path(__file__).parents[1]/'scripts/luggage_detector_node.py'
    spec = importlib.util.spec_from_file_location('detector_directed_quality', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    outputs = []
    service = DirectedSuctionService()
    fake = SimpleNamespace(_suction_top_domain_pub=SimpleNamespace(publish=outputs.append),
        _last_suction_eval=evaluation, _directed_suction_service=service,
        _suction_evaluator=SimpleNamespace(directed_quality_context=lambda box:context))
    frame = DetectionFrame(header=detection.header, box=detection, pca_valid=True,
        instance_id='instance', generation=3)
    module.LuggageDetector._publish_suction_top_domain(fake, frame)
    assert outputs[-1].valid and service._context is context
    frame.generation += 1
    module.LuggageDetector._publish_suction_top_domain(fake, frame)
    assert not outputs[-1].valid and service._context is None
    frame.generation = 3
    context.domain_sha256 = 'wrong-samples'
    module.LuggageDetector._publish_suction_top_domain(fake, frame)
    assert not outputs[-1].valid and not outputs[-1].support and service._context is None


@pytest.mark.skipif(not os.environ.get('JOINT_DIRECTED_QUALITY_ROS_DOMAIN_ID'),
    reason='opt-in isolated ROS service transport fixture')
def test_native_ros_service_transport_and_terminal_teardown():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import serialize_message
    root = Path(os.environ['JOINT_DIRECTED_SERVICE_NATIVE_EVIDENCE'])
    root.mkdir(parents=True, exist_ok=False)
    request, snapshot = request_and_context()
    handler = DirectedSuctionService(); handler.replace_context(snapshot)
    context = Context()
    rclpy.init(context=context, domain_id=int(os.environ['JOINT_DIRECTED_QUALITY_ROS_DOMAIN_ID']))
    server = rclpy.create_node('directed_suction_fixture_server', context=context)
    client_node = rclpy.create_node('directed_suction_fixture_client', context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(server); executor.add_node(client_node)
    endpoint = '/fixture/evaluate_directed_suction_v2'
    server.create_service(EvaluateDirectedSuctionV2, endpoint, handler.handle)
    client = client_node.create_client(EvaluateDirectedSuctionV2, endpoint)
    try:
        assert client.wait_for_service(timeout_sec=3.)
        for case, valid in [('positive', True), ('missing_context', False)]:
            if not valid: handler.replace_context(None)
            (root/(case+'_request.cdr')).write_bytes(serialize_message(request))
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=3.)
            assert future.done()
            response = future.result()
            (root/(case+'_response.cdr')).write_bytes(serialize_message(response))
            assert response.success is valid
            if valid: assert [c.accepted for c in response.contacts] == [True, False]
            else: assert not response.contacts
    finally:
        executor.remove_node(server); executor.remove_node(client_node)
        executor.shutdown(timeout_sec=3.)
        server.destroy_node(); client_node.destroy_node(); context.shutdown()
        (root/'teardown.json').write_text(json.dumps(dict(context_terminal=not context.ok(),
            external_processes_launched=0)))
