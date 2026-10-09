"""状態遷移・速度制約・周期角・保存互換性の検証。"""
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from mobile_state_graph import (drive_model, graph_config, generate_graph, has_valid_speed,
                                load_or_build, rollout, state_cell, state_error, validate_graph)


MODEL = drive_model(0.06, 0.149, 8.063)


def test_straight_acceleration_and_constant_circle():
    config = graph_config()
    straight = rollout([0, 0, 0, 0, 0], 0.4, 0, config, MODEL)
    assert straight[-1] == pytest.approx([0.05, 0, 0, 0.2, 0], abs=1e-10)
    circle = rollout([0, 0, 0, 0.2, 0.6], 0, 0, config, MODEL)
    angle = 0.6*config.duration_sec
    assert circle[-1] == pytest.approx([0.2/0.6*math.sin(angle),
                                      0.2/0.6*(1-math.cos(angle)), angle, 0.2, 0.6], abs=1e-8)


def test_turning_reverse_and_yaw_seam():
    config = graph_config()
    turning = rollout([0, 0, math.pi-0.05, 0, 0.6], 0, 0, config, MODEL)
    assert turning[-1][0:2] == [0, 0]
    assert turning[-1][2] == pytest.approx(-math.pi+0.25)
    reverse = rollout([0, 0, 0, 0, 0], -0.4, 0, config, MODEL)
    assert reverse[-1][0] < 0 and reverse[-1][3] == pytest.approx(-0.2)
    assert state_cell([0, 0, math.pi, 0, 0], config) == state_cell([0, 0, -math.pi, 0, 0], config)


@pytest.mark.parametrize('state,acceleration,angular', [
    ([0, 0, 0, 0.4, 0], .4, 0), ([0, 0, 0, 0, 1.2], 0, 1.2),
    ([0, 0, 0, .4, 1.2], 0, 0), ([0, 0, 0, 0, 0], .5, 0),
    ([0, 0, 0, 0, 0], 0, 1.3), ([2.0, 0, 0, .2, 0], 0, 0),
    ([math.nan, 0, 0, 0, 0], 0, 0),
])
def test_invalid_motion_rejected(state, acceleration, angular):
    assert rollout(state, acceleration, angular, graph_config(), MODEL) is None


@pytest.mark.parametrize('overrides', [
    {'radius': math.nan}, {'duration_sec': 0}, {'min_speed': 1}, {'max_nodes': 65536},
    {'max_depth': 1.5}, {'position_step': 0}, {'initial_angular_speed': 9},
    {'integration_step_sec': 1},
])
def test_invalid_config(overrides):
    with pytest.raises(ValueError):
        graph_config(**overrides)


def test_graph_all_edges_and_reachability():
    config = graph_config(max_nodes=300, max_depth=6)
    graph = generate_graph(config, MODEL)
    validate_graph(graph, config, MODEL)
    assert 20 < len(graph['states']) <= config.max_nodes
    assert graph['collision_checked'] is False
    assert all(has_valid_speed(state, config, MODEL) for state in graph['states'])
    reached = {0}
    for _ in graph['states']:
        for source, target, acceleration, angular, duration in graph['edges']:
            if source in reached:
                reached.add(target)
            samples = rollout(graph['states'][source], acceleration, angular, config, MODEL)
            assert state_error(samples[-1], graph['states'][target]) <= 1e-9
            assert duration == config.duration_sec
        if len(reached) == len(graph['states']):
            break
    assert reached == set(range(len(graph['states'])))
    again = generate_graph(config, MODEL)
    assert graph['states'] == again['states'] and graph['edges'] == again['edges']


def test_budget_and_initial_velocity():
    config = graph_config(max_nodes=12, initial_speed=.2)
    graph = generate_graph(config, MODEL)
    assert len(graph['states']) == 12 and graph['has_node_limit']
    assert graph['states'][0] == [0, 0, 0, .2, 0]
    with pytest.raises(ValueError, match='車輪速度'):
        generate_graph(replace(config, initial_speed=.4, initial_angular_speed=1.2), MODEL)


def test_cache_and_invalid_edges(tmp_path):
    config = graph_config(max_nodes=50)
    path = tmp_path/'graph.json'
    graph, has_generated = load_or_build(path, config, MODEL)
    assert has_generated
    original = path.read_bytes()
    loaded, has_generated = load_or_build(path, config, MODEL)
    assert not has_generated and loaded == graph and path.read_bytes() == original
    with pytest.raises(ValueError, match='設定が異なります'):
        load_or_build(path, replace(config, max_nodes=51), MODEL)
    assert path.read_bytes() == original
    load_or_build(path, replace(config, max_nodes=51), MODEL, enable_rebuild=True)
    corrupted = json.loads(path.read_text())
    corrupted['edges'][0][1] = 0
    path.write_text(json.dumps(corrupted))
    with pytest.raises(ValueError, match='運動モデル'):
        load_or_build(path, replace(config, max_nodes=51), MODEL)


def test_urdf_drive_values():
    xml = '''<robot><joint name="left"><limit velocity="8.063"/></joint>
      <joint name="right"><limit velocity="9.0"/></joint><gazebo>
      <plugin filename="libgazebo_ros_diff_drive.so"><left_joint>left</left_joint>
      <right_joint>right</right_joint><wheel_diameter>0.12</wheel_diameter>
      <wheel_separation>0.149</wheel_separation></plugin></gazebo></robot>'''
    assert drive_model.from_urdf(xml) == MODEL
    with pytest.raises(ValueError):
        drive_model.from_urdf('<robot/>')
