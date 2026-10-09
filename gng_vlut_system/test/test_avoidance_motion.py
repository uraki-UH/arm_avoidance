"""回避動作の選択・差替え・復帰継続・停止優先・指令周期の検証。"""
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import json
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'launch'))

from avoidance_motion import motion_flags, motion_input, motion_result, motion_components, select_motion
from dual_arm_avoidance_demo import avoidance_demo, next_control_time
from gng_avoidance_planner import gng_avoidance_policy
import dual_arm_gng_lidar_demo as ros_module
import gng_avoidance_planner as module


@pytest.mark.parametrize('flags, expected', [
    (motion_flags(is_stop_requested=True, has_valid_input=False), 'stopped'),
    (motion_flags(has_valid_input=False), 'fault'),
    (motion_flags(), 'monitoring'),
    (motion_flags(has_active_joints=True), 'avoiding'),
    (motion_flags(has_active_joints=True, has_safe_neighbors=True), 'avoiding'),
    (motion_flags(has_active_joints=True, has_safe_neighbors=True, can_finish_retreat=True), 'waiting_for_clearance'),
    (motion_flags(has_active_joints=True, has_safe_neighbors=True, can_finish_retreat=True, can_return=True), 'returning'),
    (motion_flags(has_active_joints=True, has_safe_neighbors=True, can_finish_retreat=True, can_return=True, is_home=True), 'monitoring'),
])
def test_motion_priority(flags, expected):
    assert select_motion(flags) == expected


@pytest.mark.parametrize('action, source, has_candidate', [
    ('returning', 'home', True),
    ('waiting_for_clearance', 'positions', True),
    ('monitoring', 'positions', True),
    ('stopped', 'positions', True),
    ('fault', 'positions', False),
])
def test_default_components_need_only_pose_input(action, source, has_candidate):
    request = motion_input(np.array([.3, .2]), np.zeros(2), .032)
    retreat = Mock(side_effect=AssertionError('退避以外の計画器呼出し'))
    result = motion_components(retreat=retreat).execute(action, request)
    assert isinstance(result, motion_result)
    assert result.has_candidate is has_candidate and not result.is_gng_target
    expected = getattr(request, source)
    np.testing.assert_array_equal(result.target, expected)
    assert not np.shares_memory(result.target, expected)
    retreat.assert_not_called()


def test_retreat_component_receives_explicit_input():
    request = motion_input(np.array([.3]), np.zeros(1), .032)
    expected = motion_result(np.array([.8]), True, True)
    retreat = Mock(return_value=expected)
    assert motion_components(retreat=retreat).execute('avoiding', request) is expected
    retreat.assert_called_once_with(request)


@pytest.fixture
def policy():
    state = gng_avoidance_policy()
    state.positions = np.array([.3, .2, .1])
    state.home = np.zeros(3)
    state.arm_indices = [0, 1]
    state.active_angle_indices = np.array([0, 1])
    state.coordination_source_indices = np.array([], dtype=int)
    state.path, state.plan_future = [], None
    state.max_home_error_th = .015
    state.num_selected_gng = 0
    state.config = dict(min_cloud_clearance_th=.015, min_clearance_th=.035,
                        target_clearance=.1, return_clear_sec=0.)
    state.has_safe_measured_neighbors = lambda: False
    state.has_safe_target_neighbors = lambda _: True
    state.select_active_arms = lambda: np.array([0])
    state.cloud_clearance = lambda _: (.07, None)
    state.can_bridge = lambda *args: True
    state.refine_target = lambda step: (state.positions.copy(), True)
    state.motion_components = motion_components(
        retreat=lambda request: motion_result(np.array([.8, .7, .1]), True, True))
    return state


def test_replaced_retreat_keeps_speed_limit_and_collision_check(policy):
    value, is_valid = policy.select_target(None, None, .032)
    assert is_valid and policy.num_selected_gng == 1
    np.testing.assert_allclose(value, [.332, .232, .1])
    policy.can_bridge = lambda *args: False
    value, _ = policy.select_target(None, None, .032)
    np.testing.assert_allclose(value, policy.positions)


@pytest.mark.parametrize('position, is_home', [(.3, False), (0., True)])
def test_policy_flags_keep_boolean_type_in_json_diagnostics(policy, position, is_home):
    """NumPy比較結果の真偽値統一と、診断JSONの直列化。"""
    policy.positions[0] = position
    policy.has_safe_measured_neighbors = lambda: True
    policy.cloud_clearance = lambda _: (.12, None)
    policy.select_target(None, None, .032)
    flags = asdict(policy.motion_flags)
    assert flags['is_home'] is is_home
    assert all(type(value) is bool for value in flags.values())
    assert json.loads(json.dumps({'motion_flags': flags})) == {'motion_flags': flags}


def test_default_retreat_adapter_keeps_planner_and_common_constraints(policy):
    policy.motion_components = None
    policy.retreat_target = Mock(return_value=(np.array([.8, .7, .1]), True, True))
    value, is_valid = policy.select_target(None, None, .032)
    policy.retreat_target.assert_called_once_with(.032)
    assert is_valid and policy.num_selected_gng == 1
    np.testing.assert_allclose(value, [.332, .232, .1])


def test_component_input_mutation_does_not_change_policy_state(policy):
    positions, home = policy.positions.copy(), policy.home.copy()

    def retreat(request):
        assert isinstance(request, motion_input)
        request.positions[:] = 9.
        request.home[:] = 9.
        return motion_result(np.array([.8, .7, .1]), True)

    policy.motion_components = motion_components(retreat=retreat)
    value, is_valid = policy.select_target(None, None, .032)
    assert is_valid
    np.testing.assert_array_equal(policy.positions, positions)
    np.testing.assert_array_equal(policy.home, home)
    np.testing.assert_allclose(value, [.332, .232, .1])


def test_neighbor_risk_uses_all_planned_joints_even_at_large_gap(policy):
    policy.cloud_clearance = lambda _: (.4, None)
    value, _ = policy.select_target(None, None, .032)
    assert policy.phase == 'avoiding'
    np.testing.assert_array_equal(policy.active_angle_indices, [0, 1])
    np.testing.assert_allclose(value, [.332, .232, .1])


def test_stop_cancels_pending_plan_before_reading_environment(policy):
    future = policy.plan_future = Mock()
    policy.is_stop_latched = True
    policy.cloud_clearance = Mock(side_effect=AssertionError('停止中の環境探索'))
    stop = Mock(return_value=motion_result(policy.positions.copy(), True))
    policy.motion_components = replace(policy.motion_components, stop=stop)
    value, is_valid = policy.select_target(None, None, .032)
    assert is_valid and policy.plan_future is None
    future.cancel.assert_called_once()
    stop.assert_called_once()
    np.testing.assert_array_equal(value, policy.positions)


def test_return_component_holds_inactive_joints_and_obeys_speed_limit(policy):
    policy.has_safe_measured_neighbors = lambda: True
    policy.cloud_clearance = lambda _: (.12, None)
    returning = Mock(side_effect=lambda request: motion_result(request.home.copy(), True))
    policy.motion_components = replace(policy.motion_components, returning=returning)
    value, _ = policy.select_target(None, None, .032)
    assert policy.phase == 'returning'
    returning.assert_called_once()
    np.testing.assert_allclose(value, [.268, .2, .1])


def test_monitoring_and_new_obstacle_cycle(policy):
    policy.has_safe_measured_neighbors = lambda: True
    policy.select_active_arms = lambda: np.array([], dtype=int)
    value, _ = policy.select_target(None, None, .032)
    assert policy.phase == 'monitoring'
    np.testing.assert_array_equal(value, policy.positions)
    policy.has_safe_measured_neighbors = lambda: False
    value, _ = policy.select_target(None, None, .032)
    assert policy.phase == 'avoiding' and value[0] > policy.positions[0]


@pytest.mark.parametrize('candidate', [np.array([float('nan'), .2, .1]), np.array([.8])])
def test_invalid_component_output_is_rejected(policy, candidate):
    policy.motion_components = motion_components(retreat=lambda request: motion_result(candidate, True))
    value, is_valid = policy.select_target(None, None, .032)
    assert not is_valid
    np.testing.assert_array_equal(value, policy.positions)


def test_component_cannot_move_unplanned_joints(policy):
    policy.motion_components = motion_components(retreat=lambda request: motion_result(np.array([.8, .7, 9.]), True))
    value, _ = policy.select_target(None, None, .032)
    assert value[2] == policy.positions[2]


@pytest.fixture
def retreat(monkeypatch):
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    observed = {'label': 1, 'gap': .12, 'can_bridge': True}
    state = module.gng_avoidance_policy()
    state.__dict__.update(positions=np.array([.3]), home=np.array([0.]),
        arm_indices=[0], num_selected_gng=0, path=[], plan_future=None,
        active_angle_indices=np.array([0]), coordination_source_indices=np.array([], dtype=int),
        phase='avoiding', max_home_error_th=.015,
        config={'min_cloud_clearance_th': .015, 'min_clearance_th': .015,
                'target_clearance': .1, 'return_clear_sec': .5, 'max_state_age_sec': 1.},
        select_active_arms=lambda: np.array([0]),
        has_safe_measured_neighbors=lambda: observed['label'] == 1,
        has_safe_target_neighbors=lambda _: observed['label'] == 1,
        cloud_clearance=lambda _: (observed['gap'], None),
        can_bridge=lambda *args: observed['can_bridge'],
        refine_target=lambda step: (np.array([.3+step]), True))
    state.motion_components = motion_components(retreat=lambda request: motion_result(np.array([.8]), True, True))
    return state, observed, clock


def test_unsafe_home_never_resumes(retreat):
    state, _, clock = retreat
    state.has_safe_target_neighbors = lambda _: False
    state.config['return_clear_sec'] = 0.
    for _ in range(3):
        target, is_valid = state.select_target(None, None, .032)
        assert is_valid and state.phase == 'waiting_for_clearance'
        np.testing.assert_allclose(target, state.positions)
        clock[0] += .1


@pytest.mark.parametrize('cause', ['danger', 'collision', 'missing', 'distance', 'path'])
def test_return_requires_uninterrupted_safety_and_risk_is_immediate(retreat, cause):
    state, observed, clock = retreat
    value, is_valid = state.select_target(None, None, .032)
    assert is_valid and state.phase == 'waiting_for_clearance'
    np.testing.assert_allclose(value, [.3])
    clock[0] += .49
    value, _ = state.select_target(None, None, .032)
    np.testing.assert_allclose(value, [.3])
    if cause in ('danger', 'collision', 'missing'):
        observed['label'] = {'danger': 3, 'collision': 2, 'missing': 0}[cause]
    elif cause == 'distance':
        observed['gap'] = .08
    else:
        observed['can_bridge'] = False
    value, _ = state.select_target(None, None, .032)
    assert state.return_clear_since_sec is None
    assert state.phase == ('waiting_for_clearance' if cause == 'path' else 'avoiding')
    if cause != 'path':
        assert value[0] > .3
    observed.update(label=1, gap=.12, can_bridge=True)
    clock[0] += .02
    state.select_target(None, None, .032)
    clock[0] += .49
    value, _ = state.select_target(None, None, .032)
    np.testing.assert_allclose(value, [.3])
    clock[0] += .02
    value, _ = state.select_target(None, None, .032)
    assert state.phase == 'returning' and value[0] < .3
    # 復帰中の距離減少だけによる逆転なし。隣接危険は即時再回避
    observed['gap'] = .08
    clock[0] += .05
    value, _ = state.select_target(None, None, .032)
    assert state.phase == 'returning' and value[0] < .3
    observed['label'] = 3
    value, _ = state.select_target(None, None, .032)
    assert state.phase == 'avoiding' and value[0] > .3


@pytest.mark.parametrize('jump', [-1., 2.])
def test_confirmation_does_not_cross_clock_reset_or_check_gap(retreat, jump):
    state, _, clock = retreat
    state.select_target(None, None, .032)
    clock[0] += jump
    value, _ = state.select_target(None, None, .032)
    assert state.phase == 'waiting_for_clearance'
    np.testing.assert_allclose(value, [.3])
    clock[0] += .51
    state.select_target(None, None, .032)
    assert state.phase == 'returning'


def test_new_run_and_software_stop_clear_confirmation(retreat, monkeypatch):
    state, _, _ = retreat
    target = ros_module.gng_lidar_demo.__new__(ros_module.gng_lidar_demo)
    target.__dict__.update(state.__dict__)
    target.return_clear_since_sec = 1.
    target.plan_future = None
    monkeypatch.setattr(ros_module.avoidance_demo, 'on_start', lambda *args: SimpleNamespace(success=True))
    target.on_start(None, None)
    assert target.return_clear_since_sec is None
    target.return_clear_since_sec = 1.
    target.on_safety_stop(SimpleNamespace(data=True))
    assert target.return_clear_since_sec is None and target.is_stop_latched


def test_zero_delay_allows_immediate_return(retreat):
    state, _, _ = retreat
    state.config['return_clear_sec'] = 0.
    value, _ = state.select_target(None, None, .032)
    assert state.phase == 'returning' and value[0] < .3


def test_absolute_schedule_does_not_halve_rate():
    scheduled = 1.
    commands = []
    for idx in range(101):
        now = 1.+idx*.05*.88
        if now >= scheduled:
            commands.append(now)
            scheduled = next_control_time(now, scheduled, .05)
    assert 87 <= len(commands) <= 89
    assert all(.043 < b-a < .089 for a, b in zip(commands, commands[1:]))


def test_skips_missed_slots_and_keeps_future_slot():
    assert next_control_time(3.17, 1., .05) == pytest.approx(3.2)
    assert next_control_time(3.18, 3.2, .05) == pytest.approx(3.2)
    assert next_control_time(3.17, 0., .05) == pytest.approx(3.22)


def test_stop_latch_keeps_original_fault():
    state = SimpleNamespace(state='fault', error='joint_age_sec: 1.2')
    avoidance_demo.on_safety_stop(state, SimpleNamespace(data=True))
    assert state.state == 'stopped' and state.is_stop_latched
    assert state.error == 'joint_age_sec: 1.2'
    assert not state.enable_auto_start
