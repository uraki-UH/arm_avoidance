"""ROS起動不要の操作判定・停止優先・周期処理・Gazebo状態表示の検証。"""

from contextlib import nullcontext
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dynamixel_sim_keyboard as keyboard
from dynamixel_sim_keyboard import gazebo_status_label, gazebo_key_help, gazebo_reset_feedback, operation_parameters


@pytest.mark.parametrize('action, mode, parameters', [
    ('enable', 'off', {'data': True}), ('enable', 'hold', {'data': False}),
    ('follow', 'hold', {'data': True}), ('follow', 'follow', {'data': False}),
    ('positive', 'hold', {}), ('negative', 'hold', {}), ('reset', 'off', {}),
])
def test_hardware_operation_parameters(action, mode, parameters):
    assert operation_parameters(action, {'hardware': {'mode': mode}}, {'hardware': 10.}, 10.5) == (parameters, '')
    rejected, reason = operation_parameters(action, {'hardware': {'mode': mode}}, {'hardware': 10.}, 10.500001)
    assert rejected is None and '未受信・失効' in reason


def test_sim_reset_is_independent_of_status_reception():
    assert operation_parameters('sim_reset', {}, {}, 10.) == ({}, '')


@pytest.mark.parametrize('mode, value', [('hold', True), ('avoidance', False)])
def test_avoidance_parameters_need_sim_and_demo_but_not_hardware(mode, value):
    latest = {'sim': {'mode': mode}, 'demo': {'state': 'running'}}
    received = dict.fromkeys(latest, 10.)
    assert operation_parameters('avoidance', latest, received, 10.5) == ({'data': value}, '')
    for key in ('sim', 'demo'):
        stale = dict(received, **{key: 9.})
        parameters, reason = operation_parameters('avoidance', latest, stale, 10.5)
        assert parameters is None and '失効' in reason


@pytest.mark.parametrize('state', ['waiting', 'fault', None])
def test_unready_demo_rejects_avoidance(state):
    latest = {'sim': {'mode': 'hold'}, 'demo': {'state': state}}
    parameters, reason = operation_parameters('avoidance', latest, dict.fromkeys(latest, 10.), 10.1)
    assert parameters is None and '準備待ち' in reason


@pytest.mark.parametrize('has_sim_status', [False, True])
def test_rejected_avoidance_keeps_polling_and_heartbeat(monkeypatch, has_sim_status):
    """操作拒否2回と終了要求1回。実ROS・端末・子プロセスの起動なし。"""
    clock = {'sec': 10., 'num_spins': 0}
    messages = []
    node = Mock()
    node.get_logger.return_value.info.side_effect = messages.append
    publishers = [Mock(), Mock()]
    node.create_publisher.side_effect = publishers

    def receive_initial_status(_kind, topic, callback, _qos):
        if topic.endswith('/control/status') and has_sim_status:
            callback(SimpleNamespace(data=json.dumps({'mode': 'hold'})))
        if topic.endswith('/avoidance/status'):
            callback(SimpleNamespace(data=json.dumps({'state': 'waiting'})))

    def spin_once(_node, timeout_sec):
        clock['num_spins'] += 1
        clock['sec'] += .2

    node.create_subscription.side_effect = receive_initial_status
    rclpy = ModuleType('rclpy')
    rclpy.init = Mock()
    rclpy.create_node = Mock(return_value=node)
    rclpy.ok = lambda: clock['num_spins'] < 3
    rclpy.spin_once = spin_once
    rclpy.shutdown = Mock()
    modules = {'rclpy': rclpy}
    for name in ('rclpy.signals', 'rclpy.utilities', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv'):
        modules[name] = ModuleType(name)
    modules['rclpy.signals'].SignalHandlerOptions = SimpleNamespace(NO='no_signals')
    modules['rclpy.utilities'].remove_ros_args = lambda: [
        'dynamixel_sim_keyboard.py', '--namespace', 'hardware', '--sim-namespace', 'sim_robot', '--tty-path', '/test_tty']
    modules['std_msgs.msg'].Empty = type('Empty', (), {})
    modules['std_msgs.msg'].String = type('String', (), {})
    modules['std_srvs.srv'].SetBool = SimpleNamespace(Request=lambda **kwargs: SimpleNamespace(**kwargs))
    modules['std_srvs.srv'].Trigger = SimpleNamespace(Request=lambda: SimpleNamespace())
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    operation = Mock()
    stop_requests = [Mock(is_pending=False) for _ in range(3)]
    stream = Mock()
    monkeypatch.setattr(keyboard, 'control_request', Mock(return_value=operation))
    monkeypatch.setattr(keyboard, 'stop_request', Mock(side_effect=stop_requests))
    monkeypatch.setattr(keyboard, 'terminal_input', lambda _: nullcontext(17))
    monkeypatch.setattr(keyboard, 'read_terminal_action', Mock(side_effect=['avoidance', 'avoidance', 'quit']))
    monkeypatch.setattr(keyboard.time, 'monotonic', lambda: clock['sec'])
    monkeypatch.setattr(keyboard.os, 'open', Mock(return_value=17))
    monkeypatch.setattr(keyboard.os, 'fdopen', Mock(return_value=stream))
    monkeypatch.setattr(keyboard.os, 'write', Mock())
    monkeypatch.setattr(keyboard.signal, 'signal', Mock())

    keyboard.main()

    assert clock['num_spins'] == 3
    operation.begin.assert_not_called()
    operation.cancel.assert_called_once()
    assert operation.poll.call_count == 3
    for request in stop_requests:
        assert request.poll.call_count == 3
    for request in stop_requests[:2]:
        request.begin.assert_called_once()
    for publisher in publishers:
        assert publisher.publish.call_count == 3
    assert any('未受信・失効' in message if not has_sim_status else '準備待ち' in message for message in messages)
    node.destroy_node.assert_called_once()
    rclpy.shutdown.assert_called_once()
    stream.close.assert_called_once()


def test_fault_with_unconfirmed_stop():
    latest = {'sim': {'mode': 'avoidance'}, 'safety': {'is_stop_latched': True},
              'demo': {'state': 'fault'}}
    assert gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1) == (
        'gazebo | mode=avoid | 停止解除待ち(B) | 静止=未確認 | 回避=異常')


@pytest.mark.parametrize('mode, display', [
    ('hold', 'hold'), ('avoidance', 'avoid'), ('leader', 'follow'),
    ('switching', '切替中'), ('stopped', 'stop'),
])
def test_short_modes(mode, display):
    latest = {'sim': {'mode': mode}, 'safety': {'is_stop_latched': False},
              'demo': {'state': 'idle'}}
    label = gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1)
    assert f'mode={display} |' in label
    assert '回避=開始待ち' in label
    assert '静止=' not in label
    assert '停止ロック' not in label


@pytest.mark.parametrize('key', ['sim', 'safety', 'demo'])
def test_stale_status_is_not_retained(key):
    latest = {'sim': {'mode': 'avoidance'}, 'safety': {
        'state': 'stopped', 'is_stop_latched': True, 'is_stop_applied': True,
        'is_stopped': True, 'state_age_sec': .1}, 'demo': {'state': 'idle'}}
    received = dict.fromkeys(latest, 10.)
    received[key] = 9.
    label = gazebo_status_label(latest, received, 10.1)
    field = {'sim': 'mode', 'safety': '停止状態', 'demo': '回避'}[key]
    assert f'{field}=未受信/失効' in label
    if key == 'safety':
        assert '確認済み' not in label


def test_stop_confirmation_requires_actual_fresh_stop():
    safety = {'state': 'stopped', 'is_stop_latched': True, 'is_stop_applied': True,
              'is_stopped': True, 'state_age_sec': .1}
    latest = {'safety': safety}
    received = {'safety': 10.}
    assert '静止=確認済み' in gazebo_status_label(latest, received, 10.1)
    safety['state_age_sec'] = 1.
    assert '静止=未確認' in gazebo_status_label(latest, received, 10.1)


def test_stopped_demo_after_reset_means_start_wait():
    latest = {'sim': {'mode': 'hold'}, 'safety': {'is_stop_latched': False},
              'demo': {'state': 'stopped'}}
    assert gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1) == (
        'gazebo | mode=hold | 回避=開始待ち')


def test_obstacle_wait_is_distinct_from_manual_stop():
    latest = {'sim': {'mode': 'avoidance'}, 'safety': {'is_stop_latched': False},
              'demo': {'state': 'running', 'phase': 'obstacle_wait', 'enable_obstacle_auto_resume': True}}
    label = gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1)
    assert '回避=障害物待ち（離れたら自動再開）' in label
    assert '停止解除待ち' not in label
    assert '障害物待ち' not in gazebo_status_label(latest, dict.fromkeys(latest, 10.), 11.)


def test_obstacle_wait_without_auto_resume_shows_hold_key():
    latest = {'sim': {'mode': 'avoidance'}, 'safety': {'is_stop_latched': False},
              'demo': {'state': 'running', 'phase': 'obstacle_wait', 'enable_obstacle_auto_resume': False}}
    label = gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1)
    assert '回避=障害物待ち（Aで保持へ）' in label
    assert '自動再開' not in label and '停止解除待ち' not in label


def test_waiting_for_reset_is_visible_even_if_demo_is_stopped():
    latest = {'sim': {'mode': 'stopped'}, 'safety': {'is_stop_latched': True},
              'demo': {'state': 'stopped'}}
    assert gazebo_status_label(latest, dict.fromkeys(latest, 10.), 10.1) == (
        'gazebo | mode=stop | 停止解除待ち(B) | 静止=未確認 | 回避=開始待ち')


def test_missing_or_invalid_status():
    label = gazebo_status_label({}, {}, 10.)
    assert 'None' not in label
    assert '停止状態=未受信/失効' in label
    assert '停止状態=不正' in gazebo_status_label({'safety': {}}, {'safety': 10.}, 10.1)


def test_help_includes_toggle_reset_stop_and_exit():
    assert gazebo_key_help == 'A:回避/保持  B:停止解除→保持  Space:両方停止  Ctrl+C:停止して終了'


def test_stop_reason_stays_on_status_line_and_expires():
    latest = {'sim': {'mode': 'stopped',
                     'detail': '切替操作の拒否: switch_start_demo / 開始時の点群余裕不足:\n31.5 mm'}}
    label = gazebo_status_label(latest, {'sim': 1.}, 1.1)
    assert '理由=回避開始拒否: 開始時の点群余裕不足: 31.5 mm' in label
    assert '\n' not in label
    assert '理由=' not in gazebo_status_label(latest, {'sim': 1.}, 2.)


def test_reset_rejection_visible_without_overwriting_stop_reason():
    message = 'sim_reset: 拒否・応答不正 / Gazeboの実測停止未確認\n静止継続待ち'
    latest = {'sim': {'mode': 'stopped', 'detail': 'ソフト停止要求'},
              'sim_operation': gazebo_reset_feedback(message)}
    label = gazebo_status_label(latest, {'sim': 1.}, 1.1)
    assert '理由=ソフト停止要求' in label
    assert 'B=拒否・応答不正 / Gazeboの実測停止未確認 静止継続待ち' in label
    assert '\n' not in label
    latest['sim']['mode'] = 'hold'
    assert 'B=' not in gazebo_status_label(latest, {'sim': 1.}, 1.1)
    assert gazebo_reset_feedback('enable: 受付済み') is None
