"""回避・復帰の安全継続確認と、危険時の即時再回避の検証。"""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'launch'))
import dual_arm_gng_lidar_demo as module
from pointcloud_avoidance_config import load_config
from test_pointcloud_avoidance import robot_config


@pytest.fixture
def retreat(monkeypatch):
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    observed = {'label': 1, 'gap': .12, 'can_bridge': True}
    state = SimpleNamespace(positions=np.array([.3]), home=np.array([0.]),
        native_target=np.array([.8]), arm_indices=[0], native_node_path=[1, 2], num_selected_gng=0,
        phase='avoiding', max_home_error_th=.015,
        config={'min_cloud_clearance_th': .015, 'min_clearance_th': .015,
                'target_clearance': .1, 'return_clear_sec': .5, 'max_state_age_sec': 1.},
        select_active_arms=lambda: np.array([0]),
        has_safe_first_neighbors=lambda: observed['label'] == 1,
        cloud_clearance=lambda _: (observed['gap'], None),
        can_bridge=lambda *args: observed['can_bridge'],
        refine_target=lambda step: (np.array([.3+step]), True))
    return state, observed, clock


@pytest.mark.parametrize('cause', ['danger', 'collision', 'missing', 'distance', 'path'])
def test_return_requires_uninterrupted_safety_and_risk_is_immediate(retreat, cause):
    state, observed, clock = retreat
    value, is_valid = module.gng_lidar_demo.select_native_target(state, .032)
    assert is_valid and state.phase == 'waiting_for_clearance'
    np.testing.assert_allclose(value, [.3])
    clock[0] += .49
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    np.testing.assert_allclose(value, [.3])
    if cause in ('danger', 'collision', 'missing'):
        observed['label'] = {'danger': 3, 'collision': 2, 'missing': 0}[cause]
    elif cause == 'distance':
        observed['gap'] = .08
    else:
        observed['can_bridge'] = False
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    assert state.return_clear_since_sec is None
    assert state.phase == ('waiting_for_clearance' if cause == 'path' else 'avoiding')
    if cause != 'path':
        assert value[0] > .3
    observed.update(label=1, gap=.12, can_bridge=True)
    clock[0] += .02
    module.gng_lidar_demo.select_native_target(state, .032)
    clock[0] += .49
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    np.testing.assert_allclose(value, [.3])
    clock[0] += .02
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    assert state.phase == 'returning' and value[0] < .3
    # 復帰中の距離減少だけによる逆転なし。隣接危険は即時再回避
    observed['gap'] = .08
    clock[0] += .05
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    assert state.phase == 'returning' and value[0] < .3
    observed['label'] = 3
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    assert state.phase == 'avoiding' and value[0] > .3


@pytest.mark.parametrize('jump', [-1., 2.])
def test_confirmation_does_not_cross_clock_reset_or_check_gap(retreat, jump):
    state, _, clock = retreat
    module.gng_lidar_demo.select_native_target(state, .032)
    clock[0] += jump
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    assert state.phase == 'waiting_for_clearance'
    np.testing.assert_allclose(value, [.3])
    clock[0] += .51
    module.gng_lidar_demo.select_native_target(state, .032)
    assert state.phase == 'returning'


def test_python_planner_also_waits_before_return(retreat):
    state, _, clock = retreat
    search = module.gng_path_search()
    search.__dict__.update(state.__dict__)
    search.enable_native_planner = False
    search.angles, search.labels, search.adjacency = {1: np.array([.3])}, {1: 1}, {1: []}
    search.active_angle_indices = np.array([0])
    search.path = []
    search.config['min_retreat_dist_th'] = .1
    value, is_valid = module.gng_lidar_demo.select_target(search, None, None, .032)
    assert is_valid
    np.testing.assert_allclose(value, [.3])
    clock[0] += .51
    value, is_valid = module.gng_lidar_demo.select_target(search, None, None, .032)
    assert is_valid and value[0] < .3


def test_new_run_and_software_stop_clear_confirmation(retreat, monkeypatch):
    state, _, _ = retreat
    target = module.gng_lidar_demo.__new__(module.gng_lidar_demo)
    target.__dict__.update(state.__dict__)
    target.return_clear_since_sec = 1.
    target.plan_future = None
    monkeypatch.setattr(module.avoidance_demo, 'on_start', lambda *args: SimpleNamespace(success=True))
    target.on_start(None, None)
    assert target.return_clear_since_sec is None
    target.return_clear_since_sec = 1.
    target.on_safety_stop(SimpleNamespace(data=True))
    assert target.return_clear_since_sec is None and target.is_stop_latched


@pytest.mark.parametrize('value', [-.1, float('nan'), float('inf'), True, '0.5'])
def test_invalid_return_delay_rejected_before_launch(robot_config, value):
    path, _ = robot_config
    overlay = path.parent/'delay.yaml'
    overlay.write_text(yaml.safe_dump({'return_clear_sec': value}))
    with pytest.raises(ValueError, match='return_clear_sec'):
        load_config(path, overlay)


def test_zero_delay_allows_immediate_return(retreat):
    state, _, _ = retreat
    state.config['return_clear_sec'] = 0.
    value, _ = module.gng_lidar_demo.select_native_target(state, .032)
    assert state.phase == 'returning' and value[0] < .3
