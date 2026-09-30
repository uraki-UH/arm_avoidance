"""排他的モードの入力失効・切替後目標・速度制限の検証。"""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dual_arm_mode_model import mode_model


@pytest.fixture
def model(tmp_path):
    path = tmp_path/'robot.urdf'
    path.write_text('''<robot name="test">
      <joint name="arm" type="revolute"><limit lower="-2" upper="2" velocity="1"/></joint>
      <joint name="grip" type="revolute"><limit lower="0" upper="0.8" velocity="0.1"/></joint>
      <joint name="mimic" type="revolute"><limit lower="-0.8" upper="0" velocity="0.1"/>
        <mimic joint="grip" multiplier="-1"/></joint>
      <joint name="waist" type="continuous"><limit velocity="1"/></joint>
    </robot>''')
    return mode_model(path)


def state(model, stamp_sec=0.0, now=0.0, velocity=0.0, arm=0.1):
    return model.update_state(['arm', 'grip', 'waist'], [arm, 0.2, 0.3],
                              [velocity, 0.0, 0.0], stamp_sec, now)


def leader(model, stamp_sec=0.0, now=0.0, arm=1.0):
    return model.update_leader(['arm'], [arm], stamp_sec, now)


def test_initial_mode_has_no_output(model):
    assert model.mode == 'stopped'
    assert not model.is_fresh(0.0)
    assert not model.is_stationary(0.0)
    assert model.command(0.0, 0.0) is None


def test_full_feedback_and_mimic_velocity_required(model):
    assert state(model)
    assert not model.update_state(['arm', 'grip'], [0.0, 0.0], [0.0, 0.0], 0.1, 0.1)
    assert not model.is_fresh(0.1)
    assert not model.update_state(['arm', 'grip', 'waist', 'mimic'],
                                  [0.0, 0.2, 0.0, -0.199], [0.0, 0.0, 0.0, math.nan], 0.2, 0.2)
    assert model.update_state(['arm', 'grip', 'waist', 'mimic'],
                              [0.0, 0.2, 0.0, -0.199], [0.0, 0.0, 0.0, 0.03], 0.3, 0.3)
    assert model.measured['grip'] == 0.2
    assert model.max_velocity == 0.03


@pytest.mark.parametrize('positions,velocities', [([math.nan, 0, 0], [0, 0, 0]),
                                                  ([0, 0, 0], [0, math.inf, 0]),
                                                  ([0, 0], [0, 0, 0]),
                                                  ([True, 0, 0], [0, 0, 0])])
def test_invalid_state_clears_freshness(model, positions, velocities):
    assert state(model)
    assert not model.update_state(['arm', 'grip', 'waist'], positions, velocities, 0.1, 0.1)
    assert not model.is_fresh(0.1)


def test_state_stamp_requires_strict_progress(model):
    assert state(model, 1.0, 0.0)
    assert not state(model, 1.0, 0.1)
    assert not state(model, 0.9, 0.2)
    assert state(model, 1.1, 0.3)
    assert not model.is_fresh(0.2)
    assert not model.is_fresh(0.81)


def test_stationary_requires_continuous_sim_interval(model):
    assert state(model, 0.0, 0.0)
    assert not model.is_stationary(0.0)
    assert state(model, 0.25, 0.1, 0.01)
    assert model.is_stationary(0.1)
    assert state(model, 0.3, 0.2, 0.011)
    assert not model.is_stationary(0.2)
    assert state(model, 0.4, 0.3)
    assert state(model, 0.64, 0.4)
    assert not model.is_stationary(0.4)
    assert state(model, 0.65, 0.5)
    assert model.is_stationary(0.5)
    assert not model.is_stationary(1.01)


def test_state_gap_resets_stationary_interval(model):
    assert state(model, 0.0, 0.0)
    assert state(model, 0.3, 0.6)
    assert not model.is_stationary(0.6)
    assert state(model, 0.55, 0.7)
    assert model.is_stationary(0.7)
    assert not state(model, 0.56, 0.69)
    assert not model.is_stationary(0.7)


@pytest.mark.parametrize('names,positions', [(['arm'], [2.01]), (['missing'], [0.0]),
                                           (['grip', 'mimic'], [0.2, -0.3]),
                                           (['arm'], [math.nan]), ([], [])])
def test_invalid_leader_clears_cache(model, names, positions):
    assert leader(model)
    assert not model.update_leader(names, positions, 0.1, 0.1)
    assert not model.has_fresh_leader(0.1)


def test_leader_stamp_requires_strict_progress(model):
    assert leader(model, 1.0, 0.0)
    assert not leader(model, 1.0, 0.1)
    assert not leader(model, 0.9, 0.2)
    assert leader(model, 1.1, 0.3)
    assert model.has_fresh_leader(0.3)
    assert not model.has_fresh_leader(0.81)


def test_enter_requires_fresh_inputs(model):
    with pytest.raises(ValueError):
        model.enter('hold', 0.0)
    assert state(model)
    with pytest.raises(ValueError):
        model.enter('leader', 0.0)
    with pytest.raises(ValueError):
        model.enter('unknown', 0.0)
    model.enter('hold', 0.0)
    assert model.command(0.0, 0.0) == model.measured


def test_hold_retains_captured_pose(model):
    assert state(model)
    model.enter('hold', 0.0)
    assert state(model, 0.1, 0.1, arm=0.3)
    assert model.command(0.1, 0.1)['arm'] == 0.1
    with pytest.raises(ValueError):
        model.command(0.61, 0.2)


def test_leader_ignores_pre_switch_target(model):
    assert state(model)
    assert leader(model)
    model.enter('leader', 0.0)
    assert model.command(0.0, 0.0)['arm'] == 0.1
    assert model.command(0.1, 0.1)['arm'] == 0.1
    assert leader(model, 0.2, 0.2)
    assert model.command(0.2, 0.2)['arm'] == pytest.approx(0.115)


def test_leader_first_frame_timeout_is_not_renewed_by_same_mode(model):
    assert state(model)
    assert leader(model)
    model.enter('leader', 0.0)
    model.enter('leader', 0.1)
    assert state(model, 0.6, 0.6)
    with pytest.raises(ValueError, match='未受信'):
        model.command(0.6, 0.6)


def test_invalid_first_frame_rejects_even_within_grace(model):
    assert state(model)
    assert leader(model)
    model.enter('leader', 0.0)
    assert not leader(model, 0.1, 0.1, math.nan)
    with pytest.raises(ValueError, match='不正'):
        model.command(0.1, 0.1)


def test_leader_speed_and_sim_delta_are_limited(model):
    assert state(model)
    assert leader(model)
    model.enter('leader', 0.0)
    model.command(0.0, 0.0)
    assert model.update_leader(['arm', 'grip'], [1.0, 0.5], 0.1, 0.1)
    out = model.command(0.1, 10.0)
    assert out['arm'] == pytest.approx(0.115)
    assert out['grip'] == pytest.approx(0.205)
    assert out['waist'] == 0.3
    assert model.command(0.1, 10.0) == out
    with pytest.raises(ValueError):
        model.command(0.2, 9.0)


def test_stale_partial_target_cannot_continue(model):
    assert state(model)
    assert leader(model)
    model.enter('leader', 0.0)
    model.command(0.0, 0.0)
    assert leader(model, 0.1, 0.1)
    previous = model.command(0.1, 0.1)
    assert state(model, 0.7, 0.7)
    assert model.update_leader(['grip'], [0.5], 0.7, 0.7)
    result = model.command(0.7, 0.7)
    assert result['arm'] == previous['arm']
    assert result['grip'] > previous['grip']


def test_stale_leader_raises_without_changing_command(model):
    assert state(model)
    assert leader(model)
    model.enter('leader', 0.0)
    model.command(0.0, 0.0)
    assert leader(model, 0.1, 0.1)
    previous = model.command(0.1, 0.1)
    assert state(model, 0.7, 0.7)
    with pytest.raises(ValueError, match='失効'):
        model.command(0.7, 0.7)
    assert model.commanded == previous


def test_stop_discards_leader_and_requires_new_mode(model):
    assert state(model)
    assert leader(model)
    model.enter('leader', 0.0)
    assert leader(model, 0.1, 0.1)
    model.stop()
    assert model.mode == 'stopped'
    assert model.command(0.1, 0.1) is None
    assert not model.has_fresh_leader(0.1)
    assert not model.targets
    with pytest.raises(ValueError):
        model.enter('leader', 0.1)
    model.enter('hold', 0.1)
    assert model.command(0.1, 0.1) == model.measured


def test_avoidance_does_not_publish_model_commands(model):
    assert state(model)
    model.enter('avoidance', 0.0)
    assert model.command(0.0, 0.0) is None
    assert leader(model, 0.1, 0.1)
    assert not model.targets
    assert state(model, 0.1, 0.1, arm=0.4)
    model.enter('leader', 0.1)
    assert model.command(0.1, 0.1)['arm'] == 0.4


def test_mimic_leader_uses_parent_velocity_limit(model):
    assert state(model)
    assert model.update_leader(['mimic'], [-0.5], 0.0, 0.0)
    model.enter('leader', 0.0)
    model.command(0.0, 0.0)
    assert model.update_leader(['mimic'], [-0.5], 0.1, 0.1)
    result = model.command(0.1, 0.1)
    assert result['grip'] == pytest.approx(0.205)
    assert 'mimic' not in result


def test_returned_command_is_not_internal_storage(model):
    assert state(model)
    model.enter('hold', 0.0)
    result = model.command(0.0, 0.0)
    result['arm'] = 1.9
    assert model.command(0.1, 0.1)['arm'] == 0.1


@pytest.mark.parametrize('robot_name', ['topo_dual_arm_max', 'topo_dual_arm_max_long'])
def test_target_robot_requires_all_independent_joints(robot_name):
    path = Path(__file__).resolve().parents[2]/'urdf'/robot_name/'topo_dual_arm_max.urdf'
    model = mode_model(path)
    values = model.model.initial_positions()
    names = list(values)
    assert len(names) == 19
    assert model.update_state(names, list(values.values()), [0.0]*len(names), 0.0, 0.0)
    model.enter('hold', 0.0)
    assert model.command(0.0, 0.0) == values
    assert not model.update_state(names[:-1], list(values.values())[:-1], [0.0]*(len(names)-1), 0.1, 0.1)


@pytest.mark.parametrize('name,value', [('max_joint_velocity', 0), ('max_joint_velocity', math.nan),
                                       ('max_state_age_sec', -1), ('max_state_age_sec', True)])
def test_invalid_configuration_rejected(model, name, value, tmp_path):
    with pytest.raises(ValueError):
        mode_model(tmp_path/'robot.urdf', **{name: value})
