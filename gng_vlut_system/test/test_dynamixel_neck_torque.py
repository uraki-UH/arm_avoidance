"""首の電流抵抗・対象ID・有効化条件・終了処理の実機非接続試験。"""
from pathlib import Path
import math
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dynamixel_neck_torque as module


@pytest.fixture
def rig(monkeypatch):
    clock = SimpleNamespace(now=100.)
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(module.time, 'time_ns', lambda: int(clock.now * 1e9))
    node = module.neck_torque.__new__(module.neck_torque)
    node.driver = '/dynamixel'
    node.state, node.stage_sec, node.has_owned_output = 'waiting', 100., False
    node.max_current, node.gains = [20., 30.], [10., 20.]
    node.enable_gravity_compensation = False
    node.gravity_cos_ma, node.gravity_sin_ma = [0., 0.], [0., 0.]
    node.gravity_ramp_sec = 1.
    node.status = {motor_id: (False, False, True, 'current') for motor_id in node.ids}
    node.extra = {motor_id: (1020, False, False) for motor_id in node.ids}
    node.goals = {motor_id: (3000.,) for motor_id in node.ids}
    node.status_sec = node.extra_sec = node.goal_sec = 100.
    node.joints = {motor_id: (100_000_000_000, 0., 0.) for motor_id in node.ids}
    node.goal_pub, node.torque_pub = Mock(), Mock()
    node.goal_pub.get_subscription_count.return_value = 1
    node.torque_pub.get_subscription_count.return_value = 1
    node.get_topic_names_and_types = lambda: [(node.driver + '/command/goal', []), (node.driver + '/command/status', [])]
    node.count_publishers = lambda topic: 1
    node.get_logger = Mock()
    return node, clock


@pytest.mark.parametrize('velocity', [-100., -1., -.01, 0., .01, 1., 100.])
@pytest.mark.parametrize('limit', [2.69, 5., 20.])
def test_current_is_dissipative_quantized_and_bounded(velocity, limit):
    value = module.damping_current(velocity, 10., limit)
    assert math.isfinite(value) and abs(value) <= limit
    assert value * velocity <= 0
    assert abs(abs(value) / 2.69 - round(abs(value) / 2.69)) < 1e-10
    if velocity == 0:
        assert value == 0


@pytest.mark.parametrize('velocity', [-.2, -.05, 0., .05, .2])
def test_configured_damping_current(velocity):
    """ユーザー調整値に依存しない合成上限・減衰方向の確認。"""
    path = Path(__file__).resolve().parents[1] / 'config/dynamixel_neck_torque.yaml'
    config = yaml.safe_load(path.read_text())['/**']['ros__parameters']
    gains = module.numeric_pair(config['damping_gain'], 'damping_gain')
    limits = module.numeric_pair(config['max_current_ma'], 'max_current_ma')
    for gain, limit in zip(gains, limits):
        value = module.damping_current(velocity, gain, limit)
        assert abs(value) <= limit and value * velocity <= 0.
        if velocity == 0:
            assert value == 0.


@pytest.mark.parametrize('values', [[50, 50], [50., 50.], [50, 50.]])
def test_numeric_pair_accepts_integer_and_float(values):
    assert module.numeric_pair(values, 'test') == [50., 50.]


@pytest.mark.parametrize('values', [[True, True], ['50', '50'], [math.nan, 0], [0, math.inf], [1], 50])
def test_numeric_pair_rejects_invalid(values):
    with pytest.raises(ValueError):
        module.numeric_pair(values, 'test')


def test_gravity_at_rest_ramp_angle_and_no_return_target(rig):
    node, _ = rig
    node.enable_gravity_compensation = True
    node.gravity_cos_ma = [10.76, 0.]
    node.gravity_sin_ma = [0., -21.52]
    node.joints[52] = (100_000_000_000, 0., math.pi / 2)
    assert node.control_currents(100.) == [0., 0.]
    assert node.control_currents(100.5) == [5.38, -10.76]
    assert node.control_currents(101.) == [10.76, -21.52]
    node.joints[51] = (100_000_000_000, 0., math.pi)
    assert node.control_currents(101.)[0] == -10.76
    # 同じ実測角度・速度での履歴非依存。元姿勢への復帰目標なし
    node.joints[51] = (100_000_000_000, 0., 0.)
    assert node.control_currents(102.) == [10.76, -21.52]


def test_gravity_and_damping_share_current_limit(rig):
    node, _ = rig
    node.enable_gravity_compensation = True
    node.gravity_cos_ma = [20., -30.]
    node.joints = {51: (100_000_000_000, -100., 0.), 52: (100_000_000_000, 100., 0.)}
    values = node.control_currents(102.)
    assert 0 < values[0] <= 20. and -30. <= values[1] < 0


def test_zero_readback_before_torque_on_and_no_position_command(rig):
    node, clock = rig
    node.step()
    assert node.state == 'zero' and node.has_owned_output
    node.torque_pub.publish.assert_not_called()
    node.step()
    node.torque_pub.publish.assert_not_called()
    node.goals = {51: (0.,), 52: (0.,)}
    clock.now = node.goal_sec = 100.05
    node.step()
    assert node.state == 'enable'
    assert list(node.torque_pub.publish.call_args.args[0].torque) == [True, True]
    node.status = {motor_id: (True, False, True, 'current') for motor_id in node.ids}
    clock.now = node.status_sec = 100.1
    node.step()
    assert node.state == 'running'
    node.joints = {51: (100_100_000_000, 1., 0.), 52: (100_100_000_000, -2., 0.)}
    node.step()
    values = node.goal_pub.publish.call_args.args[0].current_ma
    assert values[0] < 0 < values[1]
    assert abs(values[0]) <= 20. and abs(values[1]) <= 30.
    for call in node.goal_pub.publish.call_args_list:
        message = call.args[0]
        assert list(message.id_list) == [51, 52]
        assert not message.position_deg and not message.velocity_deg_s and not message.pwm_percent
    for call in node.torque_pub.publish.call_args_list:
        message = call.args[0]
        assert list(message.id_list) == [51, 52]
        assert not message.mode and not message.error and not message.ping


@pytest.mark.parametrize('kind', ['already_on', 'position', 'error', 'ping', 'model', 'auto', 'reverse', 'conflict', 'no_driver'])
def test_unsafe_start_never_sends_commands(rig, kind):
    node, _ = rig
    if kind in ('already_on', 'position', 'error', 'ping'):
        node.status[51] = (kind == 'already_on', kind == 'error', kind != 'ping',
                           'cur_position' if kind == 'position' else 'current')
    elif kind in ('model', 'auto', 'reverse'):
        node.extra[51] = (999 if kind == 'model' else 1020, kind == 'auto', kind == 'reverse')
    elif kind == 'conflict':
        node.count_publishers = lambda topic: 2
    else:
        node.goal_pub.get_subscription_count.return_value = 0
    with pytest.raises(ValueError):
        node.step()
    assert not node.has_owned_output
    node.goal_pub.publish.assert_not_called()
    node.torque_pub.publish.assert_not_called()


@pytest.mark.parametrize('kind', ['stale_joints', 'stale_status', 'missing_id', 'mode', 'over_current', 'torque_off'])
def test_runtime_fault_never_sends_new_current(rig, kind):
    node, clock = rig
    node.state, node.has_owned_output = 'running', True
    node.status = {motor_id: (True, False, True, 'current') for motor_id in node.ids}
    node.goals = {51: (0.,), 52: (0.,)}
    if kind == 'stale_joints':
        clock.now = 100.21
    elif kind == 'stale_status':
        node.status_sec = 90.
    elif kind == 'missing_id':
        node.joints.pop(52)
    elif kind == 'mode':
        node.status[52] = (True, False, True, 'cur_position')
    elif kind == 'over_current':
        node.goals[52] = (31.,)
    else:
        node.status[52] = (False, False, True, 'current')
    with pytest.raises(ValueError):
        node.step()
    node.goal_pub.publish.assert_not_called()


@pytest.mark.parametrize('has_report', [True, False])
def test_shutdown_retries_zero_and_off_only(rig, monkeypatch, has_report):
    node, clock = rig
    node.has_owned_output = True
    node.status = {motor_id: (True, False, True, 'current') for motor_id in node.ids}
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)

    def spin(*_args, **_kwargs):
        clock.now += .1
        if has_report:
            node.status = {motor_id: (False, False, True, 'current') for motor_id in node.ids}
            node.status_sec = clock.now
            node.joints = {motor_id: (int(clock.now * 1e9), 0.) for motor_id in node.ids}

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    assert node.stop_output() is has_report
    assert node.state == 'stopping'
    assert node.torque_pub.publish.call_count >= 5
    assert all(list(call.args[0].torque) == [False, False] for call in node.torque_pub.publish.call_args_list)
    assert all(list(call.args[0].current_ma) == [0., 0.] for call in node.goal_pub.publish.call_args_list)


@pytest.mark.parametrize('kind', ['old', 'future', 'duplicate', 'nan', 'frame'])
def test_bad_measurement_does_not_refresh(rig, kind):
    node, _ = rig
    original = dict(node.joints)
    message = JointState(name=['51', '52'], position=[0., 0.], velocity=[1., 1.])
    message.header.frame_id = 'dynamixel_motor'
    message.header.stamp.sec = 101 if kind == 'future' else 99 if kind == 'old' else 100
    if kind == 'duplicate':
        message.name = ['51', '51']
    elif kind == 'nan':
        message.velocity = [math.nan, math.nan]
    elif kind == 'frame':
        message.header.frame_id = 'world'
    node.on_joints(message)
    assert node.joints == original


def test_no_shutdown_commands_without_ownership(rig):
    node, _ = rig
    assert node.stop_output()
    node.goal_pub.publish.assert_not_called()
    node.torque_pub.publish.assert_not_called()


def test_off_attempt_even_if_zero_current_publish_fails(rig, monkeypatch):
    node, _ = rig
    node.has_owned_output = True
    node.goal_pub.publish.side_effect = RuntimeError('模擬送信失敗')
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    with pytest.raises(RuntimeError):
        node.stop_output()
    message = node.torque_pub.publish.call_args.args[0]
    assert list(message.id_list) == [51, 52] and list(message.torque) == [False, False]
