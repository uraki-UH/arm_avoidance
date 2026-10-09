"""クラス包含・属性重複・観測不足・期限切れ・ROS配信の検証。"""

from copy import deepcopy
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

import pytest
import yaml

share = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(share / 'scripts'))
from object_class_recognition import class_recognizer


def config():
    return yaml.safe_load((share / 'config/object_class_recognition.yaml').read_text())


def candidate(template_id='truck_surface', **values):
    return {'template_id': template_id, 'state': 'candidate', 'score': 0.80,
            'visible_ratio': 0.70, 'is_falsified': False, **values}


def test_overlapping_classes_attributes_and_independent_detail():
    recognizer = class_recognizer(config(), ['truck_surface', 'mug_complete'])
    recognizer.update('truck_surface', candidate(), 0.0)
    recognizer.update('mug_complete', candidate('mug_complete', score=0.75), 0.1)
    result = recognizer.snapshot(0.2)
    assert result['classes']['vehicle']['membership'] == pytest.approx(0.9)
    assert result['classes']['truck']['membership'] == pytest.approx(0.9)
    assert result['attributes']['large']['membership'] == 0.85
    assert result['classes']['mug']['membership'] == pytest.approx(0.8)
    assert result['scope'] == 'scene_presence'
    assert not result['is_probability'] and not result['is_calibrated']
    assert result['hypotheses'][0]['match_score'] == 0.8
    assert 'mug' not in result['hypotheses'][0]['classes']
    assert 'truck' not in result['hypotheses'][1]['classes']
    assert result['classes']['kei_car']['membership'] is None
    json.dumps(result, allow_nan=False)


def test_parent_supported_while_subclass_and_attribute_are_unobserved():
    recognizer = class_recognizer(config(), ['truck_surface'])
    recognizer.update('truck_surface', candidate(visible_ratio=0.20), 0.0)
    result = recognizer.snapshot(0.0)
    assert result['classes']['vehicle']['state'] == 'supported'
    assert result['classes']['truck']['state'] == 'insufficient'
    assert result['classes']['truck']['membership'] is None
    assert result['attributes']['large']['membership'] is None


def test_fuzzy_class_annotations_and_diamond_hierarchy():
    data = config()
    data['classes']['commercial'] = {'label': '商用車', 'parents': ['vehicle']}
    data['classes']['truck']['parents'].append('commercial')
    data['templates']['truck_surface']['classes'].update(passenger_car=0.3)
    data['classes']['vehicle'].update(min_score=0.7, max_score=0.95)
    recognizer = class_recognizer(data, ['truck_surface'])
    recognizer.update('truck_surface', candidate(), 0.0)
    classes = recognizer.snapshot(0.0)['classes']
    assert classes['vehicle']['membership'] == pytest.approx(0.9)
    assert classes['commercial']['membership'] == pytest.approx(0.9)
    assert classes['passenger_car']['membership'] == 0.3


def test_same_class_prototypes_use_max_without_count_boost():
    data = config()
    data['templates']['truck_two'] = deepcopy(data['templates']['truck_surface'])
    recognizer = class_recognizer(data, ['truck_surface', 'truck_two'])
    recognizer.update('truck_surface', candidate(score=0.6), 0.0)
    recognizer.update('truck_two', candidate('truck_two', score=0.7), 0.0)
    result = recognizer.snapshot(0.0)['classes']['truck']
    assert result['membership'] == pytest.approx(0.7)
    assert result['support_template_id'] == 'truck_two'


@pytest.mark.parametrize('score, expected', [(0.0, 0.0), (0.35, 0.0), (0.60, 0.5), (0.85, 1.0), (1.0, 1.0)])
def test_membership_curve(score, expected):
    recognizer = class_recognizer(config(), ['mug'])
    recognizer.update('mug', candidate('mug', score=score), 0.0)
    assert recognizer.snapshot(0.0)['classes']['mug']['membership'] == pytest.approx(expected)


def test_contradiction_and_expiry_do_not_become_class_absence():
    recognizer = class_recognizer(config(), ['truck_surface'])
    assert recognizer.snapshot(0.0)['classes']['truck']['state'] == 'unobserved'
    recognizer.update('truck_surface', candidate(), 0.0)
    recognizer.update('truck_surface', {'template_id': 'truck_surface', 'state': 'no_hypothesis',
                                      'is_falsified': True, 'score': 0.0}, 1.0)
    result = recognizer.snapshot(1.0)['classes']['truck']
    assert result['state'] == 'rejected' and result['membership'] is None
    recognizer.update('truck_surface', candidate(), 2.0)
    assert recognizer.snapshot(7.0)['classes']['truck']['state'] == 'supported'
    expired = recognizer.snapshot(7.01)['classes']['truck']
    assert expired['state'] == 'stale' and expired['membership'] is None


def test_unconfigured_template_keeps_detail_without_inventing_class():
    recognizer = class_recognizer(config(), ['unregistered_mug'])
    recognizer.update('unregistered_mug', candidate('unregistered_mug'), 0.0)
    result = recognizer.snapshot(0.0)
    assert result['unconfigured_template_ids'] == ['unregistered_mug']
    assert result['hypotheses'][0]['match_score'] == 0.8
    assert all(entry['membership'] is None for entry in result['classes'].values())


@pytest.mark.parametrize('change', [
    lambda data: data.update(version=True),
    lambda data: data.update(unknown=1),
    lambda data: data.update(max_candidate_age_sec=0),
    lambda data: data['defaults'].update(min_score=0.9),
    lambda data: data['classes']['vehicle'].update(parents=['truck']),
    lambda data: data['classes']['truck'].update(parents=['missing']),
    lambda data: data['classes']['truck'].update(parents='vehicle'),
    lambda data: data['classes']['vehicle'].update(min_visible_ratio=0.9),
    lambda data: data['classes']['truck'].update(membership_th=float('nan')),
    lambda data: data['templates']['truck_surface']['classes'].update(missing=1.0),
    lambda data: data['templates']['truck_surface']['attributes'].update(large=True),
    lambda data: data['attributes']['large'].update(parents=['vehicle']),
])
def test_invalid_config_rejected(change):
    data = config()
    change(data)
    with pytest.raises(ValueError):
        class_recognizer(data, ['truck_surface'])


@pytest.mark.parametrize('values', [
    {'template_id': 'mug'}, {'state': 'confirmed'}, {'score': None}, {'score': float('inf')},
    {'score': True}, {'score': -0.1}, {'visible_ratio': 1.1}, {'is_falsified': 'false'},
])
def test_invalid_candidate_preserves_last_valid_result(values):
    recognizer = class_recognizer(config(), ['truck_surface'])
    recognizer.update('truck_surface', candidate(), 0.0)
    with pytest.raises(ValueError):
        recognizer.update('truck_surface', candidate(**values), 1.0)
    assert recognizer.snapshot(1.0)['classes']['truck']['membership'] == pytest.approx(0.9)


def test_clock_regression_and_bounded_cache():
    recognizer = class_recognizer(config(), ['mug'])
    for idx in range(100):
        recognizer.update('mug', candidate('mug'), float(idx))
    assert len(recognizer.candidates) == 1
    with pytest.raises(ValueError):
        recognizer.snapshot(98.0)


def test_matching_launch_enable_disable_and_template_topics(tmp_path):
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    spec = importlib.util.spec_from_file_location('matching_launch', share / 'launch/object_template_matching.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    dataset = tmp_path / 'mug.json'
    dataset.write_text(json.dumps({'kind': 'object_template', 'template_id': 'mug'}))
    context = LaunchContext()
    context.launch_configurations.update({
        'dataset_dir': str(tmp_path), 'dataset_file': 'mug.json', 'template_sources_file': '',
        'profile_file': str(share / 'config/object_template_matching.yaml'),
        'environment_topological_map_topic': '/test_environment', 'plane_clusters_topic': '/test_planes',
        'enable_class_recognition': 'true', 'class_config_file': str(share / 'config/object_class_recognition.yaml'),
        'class_output_topic': '/test_classes', 'frame_id': 'object_template', 'publish_hz': '1.0'})
    nodes = module.create_matching_nodes(context)
    recognition = next(node for node in nodes if node.condition is not None)
    assert recognition.condition.evaluate(context)
    params = evaluate_parameters(context, recognition._Node__parameters)[0]
    assert tuple(params['template_ids']) == ('mug',)
    assert tuple(params['candidate_topics']) == ('/mug/object_template_match_candidates',)
    assert params['output_topic'] == '/test_classes'
    context.launch_configurations['enable_class_recognition'] = 'false'
    assert not recognition.condition.evaluate(context)
    assert len(nodes) == 4


@pytest.mark.skipif(os.environ.get('RUN_OBJECT_CLASS_ROS_TEST') != '1', reason='ROS通信試験の明示起動')
def test_ros_candidate_delivery_rejection_and_expiry(tmp_path):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from object_class_recognition_node import object_class_recognition_node

    data = config()
    data['max_candidate_age_sec'] = 0.5
    profile = tmp_path / 'classes.yaml'
    profile.write_text(yaml.safe_dump(data))
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        recognizer = object_class_recognition_node(parameter_overrides=[
            Parameter('config_file', value=str(profile)),
            Parameter('template_ids', value=['truck_surface']),
            Parameter('candidate_topics', value=['/class_test/candidate']),
            Parameter('output_topic', value='/class_test/result'), Parameter('publish_hz', value=20.0)])
        nodes.append(recognizer)
        probe = Node('class_recognition_probe')
        nodes.append(probe)
        for node in nodes:
            executor.add_node(node)
        results = []
        probe.create_subscription(String, '/class_test/result', lambda message: results.append(json.loads(message.data)), 10)
        publisher = probe.create_publisher(String, '/class_test/candidate', 10)

        def wait_for(check):
            deadline = time.monotonic() + 5.0
            while not check() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
            assert check()

        wait_for(lambda: publisher.get_subscription_count() > 0 and len(results) > 0)
        publisher.publish(String(data=json.dumps(candidate(visible_ratio=0.2))))
        wait_for(lambda: results[-1]['classes']['vehicle']['state'] == 'supported')
        assert results[-1]['classes']['truck']['membership'] is None
        publisher.publish(String(data=json.dumps(candidate())))
        wait_for(lambda: results[-1]['classes']['truck']['state'] == 'supported')
        assert results[-1]['attributes']['large']['membership'] == 0.85
        publisher.publish(String(data=json.dumps(candidate(is_falsified=True))))
        wait_for(lambda: results[-1]['classes']['truck']['state'] == 'rejected')
        assert results[-1]['classes']['truck']['membership'] is None
        publisher.publish(String(data=json.dumps(candidate())))
        wait_for(lambda: results[-1]['classes']['truck']['state'] == 'supported')
        wait_for(lambda: results[-1]['classes']['truck']['state'] == 'stale')
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
