"""動作優先順位と差替え部品に対する共通安全制約の検証。"""
from dataclasses import replace
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from avoidance_motion import motion_flags, motion_components, select_motion
from gng_avoidance_planner import gng_avoidance_policy


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
    state.select_active_arms = lambda: np.array([0])
    state.cloud_clearance = lambda _: (.07, None)
    state.can_bridge = lambda *args: True
    state.refine_target = lambda step: (state.positions.copy(), True)
    state.motion_components = motion_components(
        retreat=lambda state, home, step: (np.array([.8, .7, .1]), True, True))
    return state


def test_replaced_retreat_keeps_speed_limit_and_collision_check(policy):
    value, is_valid = policy.select_target(None, None, .032)
    assert is_valid and policy.num_selected_gng == 1
    np.testing.assert_allclose(value, [.332, .232, .1])
    policy.can_bridge = lambda *args: False
    value, _ = policy.select_target(None, None, .032)
    np.testing.assert_allclose(value, policy.positions)


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
    stop = Mock(return_value=(policy.positions.copy(), True, False))
    policy.motion_components = replace(policy.motion_components, stop=stop)
    value, is_valid = policy.select_target(None, None, .032)
    assert is_valid and policy.plan_future is None
    future.cancel.assert_called_once()
    stop.assert_called_once()
    np.testing.assert_array_equal(value, policy.positions)


def test_return_component_holds_inactive_joints_and_obeys_speed_limit(policy):
    policy.has_safe_measured_neighbors = lambda: True
    policy.cloud_clearance = lambda _: (.12, None)
    returning = Mock(side_effect=lambda state, home, step: (home.copy(), True, False))
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
    policy.motion_components = motion_components(retreat=lambda *args: (candidate, True, False))
    value, is_valid = policy.select_target(None, None, .032)
    assert not is_valid
    np.testing.assert_array_equal(value, policy.positions)


def test_component_cannot_move_unplanned_joints(policy):
    policy.motion_components = motion_components(retreat=lambda *args: (np.array([.8, .7, 9.]), True, False))
    value, _ = policy.select_target(None, None, .032)
    assert value[2] == policy.positions[2]
