"""左腕7軸の部分欠測・再送・異常実測による停止判定の検証。"""
import math
from pathlib import Path
import sys
import time
from unittest.mock import Mock, patch

import pytest
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamixel_sim_output import dynamixel_sim_output


@pytest.fixture
def output():
    node = dynamixel_sim_output.__new__(dynamixel_sim_output)
    node.names = ['L_joint'+str(value) for value in range(1, 8)]
    node.motor_names = {str(40+value): name for value, name in enumerate(node.names, 1)}
    node.mapping = SimpleNamespace(entries={name: (int(motor), -1., 0.) for motor, name in node.motor_names.items()})
    node.config = {'max_state_age_sec': .3, 'max_stop_velocity_th': .01, 'min_stop_duration_sec': .3}
    node.measured, node.velocity, node.stamps = {}, {}, {}
    node.low_since = None
    return node


def test_sim_target_uses_reference_without_deprecated_fields(output):
    message = JointTrajectoryControllerState()
    message.header.stamp.sec = 1
    message.joint_names = output.names
    message.reference.positions = [.1]*len(output.names)
    message.feedback.positions = [.9]*len(output.names)
    output.last_sim_stamp = -1
    output.sim_target, output.sim_sec = {}, -math.inf
    output.on_sim_target(message)
    assert output.sim_target == dict.fromkeys(output.names, .1)
    assert math.isfinite(output.sim_sec)
    assert not message.desired.positions and not message.actual.positions


def test_sim_target_without_reference_never_uses_feedback(output):
    message = JointTrajectoryControllerState()
    message.header.stamp.sec = 1
    message.joint_names = output.names
    message.feedback.positions = [.9]*len(output.names)
    output.last_sim_stamp = -1
    output.sim_target, output.sim_sec = {}, -math.inf
    output.on_sim_target(message)
    assert output.sim_target == {} and output.sim_sec == -math.inf


@pytest.mark.parametrize('allow_output', [False, True])
def test_torque_off_without_feedback_blocks_all_position_goals(output, allow_output):
    output.ids = list(range(41, 48))
    output.torque_pub = Mock() if allow_output else None
    output.goal_pub = Mock()
    output.is_torque_off_latched = False
    output.target = output.commanded = {'L_joint7': 1.}
    response = output.on_torque_off(None, SimpleNamespace())
    assert response.success is allow_output
    assert output.mode == 'torque_off' and output.is_stop_latched
    output.send_goal({'L_joint7': 2.})
    output.stop('通常保持停止')
    assert output.mode == 'torque_off' and not output.target
    output.goal_pub.publish.assert_not_called()
    if allow_output:
        message = output.torque_pub.publish.call_args.args[0]
        assert list(message.id_list) == list(range(41, 48))
        assert list(message.torque) == [False]*7
        assert not message.error and not message.mode and not message.ping


def test_reset_needs_torque_off_report_even_when_stationary(output):
    output.mode = 'torque_off'
    output.is_torque_off_latched = True
    output.is_stop_latched = True
    output.is_stationary = lambda: True
    output.has_torque_off_report = lambda: False
    response = output.on_reset(None, SimpleNamespace())
    assert not response.success and output.is_torque_off_latched


def test_enable_without_explicit_current_limit_has_no_output(output):
    output.config.update(allow_hardware_output=True, max_current_ma=0.)
    output.mode = 'off'
    output.is_stop_latched = False
    output.goal_pub, output.torque_pub = Mock(), Mock()
    response = output.on_enable(SimpleNamespace(data=True), SimpleNamespace())
    assert not response.success and 'max_current_ma' in response.message
    output.goal_pub.publish.assert_not_called()
    output.torque_pub.publish.assert_not_called()


def sample(ids, stamp, velocity=0.):
    message = JointState(name=[str(value) for value in ids], position=[0.]*len(ids), velocity=[velocity]*len(ids))
    message.header.frame_id = 'dynamixel_motor'
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    return message


def test_missing_one_left_arm_id_never_confirms_stop(output):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 47), now))
        assert not output.has_fresh_state()
        assert not output.is_stationary()
        output.on_measured(sample([47], now))
        assert output.has_fresh_state()
        assert output.low_since is not None


@pytest.mark.parametrize('kind', ['stale', 'future', 'replayed', 'wrong_frame', 'nan_velocity', 'duplicate_id'])
def test_invalid_feedback_does_not_refresh_left_arm(output, kind):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 48), now-100_000_000))
        stamps = dict(output.stamps)
        message = sample(range(41, 48), now)
        if kind in ('stale', 'future', 'replayed'):
            message = sample(range(41, 48), now+{'stale': -400_000_000, 'future': 1_000_000, 'replayed': -100_000_000}[kind])
        elif kind == 'wrong_frame':
            message.header.frame_id = 'base_link'
        elif kind == 'nan_velocity':
            message.velocity = [math.nan]*7
        else:
            message.name = ['47']*7
        output.on_measured(message)
        assert output.stamps == stamps
    with patch('dynamixel_sim_output.time.time_ns', return_value=now+300_000_000):
        assert not output.has_fresh_state()
        assert not output.is_stationary()


def test_one_moving_joint_cancels_stationary_duration(output):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 48), now-1_000))
        output.on_measured(sample([44], now, velocity=.1))
        assert output.low_since is None
        assert not output.is_stationary()


def test_zero_velocity_register_with_position_motion_is_not_stopped(output):
    now = time.time_ns()
    with patch('dynamixel_sim_output.time.time_ns', return_value=now):
        output.on_measured(sample(range(41, 48), now-20_000_000))
        message = sample([47], now)
        message.position = [math.radians(.08789)]
        output.on_measured(message)
        assert output.velocity['L_joint7'] == 0.
        assert not output.is_stationary()
