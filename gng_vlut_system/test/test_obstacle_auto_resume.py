"""点群接近の自動再開と、手動停止・入力失効のラッチ維持の検証。"""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
from std_msgs.msg import Bool

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import dual_arm_avoidance_demo as module
from dual_arm_gng_lidar_demo import gng_lidar_demo


@pytest.fixture
def waiting(monkeypatch):
    clock = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    holding = Mock()
    monkeypatch.setattr(module.avoidance_demo, 'publish_target', holding)
    state = SimpleNamespace(
        state='running', phase='avoiding', error='', positions=np.array([.2]), home=np.array([.1]),
        config={'enable_live_obstacles': True, 'enable_obstacle_auto_resume': True,
                'enable_native_planner': True, 'min_clearance_th': .02, 'target_clearance': .05,
                'sides': ['left'], 'control_period_sec': .05, 'max_joint_velocity': 1.2},
        geometry=SimpleNamespace(spheres=[('L_gripper_base', None, .04)], radii=[.04],
            root_link='base_link', joint_names=['L_joint1'], has_internal_clearance=Mock(return_value=True)),
        min_observed_clearance=float('inf'), min_home_clearance=float('inf'), max_excursion=0.,
        stop_clearance=None, trails={}, last_visual=None, side_idx=0, run_generation=3,
        run_start_stamp_sec=1., is_stop_latched=False, resume_clear_sec=.3,
        obstacle_hold_positions=None, clear_since_sec=None, next_control_sec=0.,
        joint_time=0., obstacle_time=0., is_fresh=Mock(return_value=True), update_obstacle=Mock(),
        observe_clearance=Mock(return_value=(.015, np.array([[.2,.1,.3]]), 0, np.array([.21,.11,.31]))),
        publish_markers=Mock(), status=Mock(), hold=Mock(), get_logger=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9))),
        can_resume_obstacle=Mock(return_value=True), freshness_detail=lambda: 'voxel_age_sec=2',
        select_target=Mock(return_value=(np.array([.15]),True)), publish_target=Mock())
    state.fail = lambda error: module.avoidance_demo.fail(state, error)
    state.wait_for_obstacle = lambda *args: module.avoidance_demo.wait_for_obstacle(state, *args)
    return state, clock, holding


def set_gap(state, gap):
    state.observe_clearance.return_value = (gap, *state.observe_clearance.return_value[1:])


def test_close_cloud_holds_fixed_pose_and_resumes_same_run(waiting):
    state, clock, holding = waiting
    module.avoidance_demo.tick(state)
    assert state.state == 'running' and state.phase == 'obstacle_wait'
    assert state.stop_clearance['event'] == 'obstacle_wait'
    assert not state.is_stop_latched
    state.positions[0] = .201
    module.avoidance_demo.tick(state)
    np.testing.assert_array_equal(holding.call_args.args[1], [.2])
    set_gap(state, .051)
    clock[0] = 10.1
    module.avoidance_demo.tick(state)
    clock[0] = 10.39
    module.avoidance_demo.tick(state)
    state.select_target.assert_not_called()
    clock[0] = 10.41
    module.avoidance_demo.tick(state)
    assert state.phase == 'avoiding' and state.state == 'running'
    state.select_target.assert_called_once()
    assert state.run_generation == 3
    np.testing.assert_array_equal(state.home, [.1])


@pytest.mark.parametrize('margin, expected_state', [(.01, 'running'), (.03, 'fault')])
def test_configured_internal_stop_margin(waiting, margin, expected_state):
    state, _, _ = waiting
    state.config['min_internal_clearance_th'] = margin
    state.geometry.has_internal_clearance.side_effect = lambda centers, min_clearance_th: .02 >= min_clearance_th
    module.avoidance_demo.tick(state)
    assert state.state == expected_state
    if expected_state == 'fault':
        assert '自己干渉' in state.error
    else:
        assert state.phase == 'obstacle_wait'


@pytest.mark.parametrize('cause', ['distance', 'graph'])
def test_safe_interval_restarts_on_new_obstacle_or_unsafe_neighbor(waiting, cause):
    state, clock, _ = waiting
    module.avoidance_demo.tick(state)
    set_gap(state, .06)
    clock[0] = 10.1
    module.avoidance_demo.tick(state)
    if cause == 'distance':
        set_gap(state, .04)
    else:
        state.can_resume_obstacle.return_value = False
    clock[0] = 10.3
    module.avoidance_demo.tick(state)
    assert state.clear_since_sec is None
    set_gap(state, .06)
    state.can_resume_obstacle.return_value = True
    clock[0] = 10.5
    module.avoidance_demo.tick(state)
    assert state.phase == 'obstacle_wait'
    state.select_target.assert_not_called()


def test_space_latch_never_auto_resumes_even_after_release(waiting):
    state, clock, _ = waiting
    module.avoidance_demo.tick(state)
    module.avoidance_demo.on_safety_stop(state, Bool(data=True))
    set_gap(state, .1)
    clock[0] += 1.
    module.avoidance_demo.tick(state)
    assert state.state == 'stopped' and state.phase == 'software_stop'
    module.avoidance_demo.on_safety_stop(state, Bool(data=False))
    module.avoidance_demo.tick(state)
    assert state.state == 'stopped'
    state.select_target.assert_not_called()


def test_stale_input_during_obstacle_wait_is_still_fault(waiting):
    state, clock, _ = waiting
    module.avoidance_demo.tick(state)
    state.is_fresh.return_value = False
    clock[0] += 2.
    module.avoidance_demo.tick(state)
    assert state.state == 'fault' and '失効' in state.error
    state.is_fresh.return_value = True
    set_gap(state, .1)
    module.avoidance_demo.tick(state)
    assert state.state == 'fault'
    state.select_target.assert_not_called()


def test_internal_collision_cannot_enter_auto_resume_wait(waiting):
    state, _, holding = waiting
    state.geometry.has_internal_clearance.return_value = False
    module.avoidance_demo.tick(state)
    assert state.state == 'fault' and '自己干渉' in state.error
    holding.assert_not_called()


def test_disabled_auto_resume_keeps_distance_fault(waiting):
    state, _, _ = waiting
    state.config['enable_obstacle_auto_resume'] = False
    module.avoidance_demo.tick(state)
    assert state.state == 'fault' and '停止距離' in state.error


def test_resume_checks_direct_gng_neighbors():
    state = SimpleNamespace(has_safe_first_neighbors=Mock(return_value=False))
    assert not gng_lidar_demo.can_resume_obstacle(state)
    state.has_safe_first_neighbors.return_value = True
    assert gng_lidar_demo.can_resume_obstacle(state)
