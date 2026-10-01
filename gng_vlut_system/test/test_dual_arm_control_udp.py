"""ROSノード・通信を伴わないUDP出力許可とGazebo停止連動の検証。"""

from concurrent.futures import Future
from unittest.mock import Mock, call

import pytest
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from test_dual_arm_control import control, stopped_safety


@pytest.fixture
def udp_control(control):
    control.udp = Mock()
    control.udp.is_enabled = False
    control.udp.enable.side_effect = lambda _now: setattr(control.udp, 'is_enabled', True)
    control.udp.disable.side_effect = lambda _detail: setattr(control.udp, 'is_enabled', False)
    return control


def controller_state(sec=12, nanosec=0):
    """実測・終点と区別可能なcontroller補間目標の入力。"""
    message = JointTrajectoryControllerState()
    message.header.stamp.sec, message.header.stamp.nanosec = sec, nanosec
    message.joint_names = ['L_joint1', 'R_joint1']
    message.desired.positions = [0.1, -0.2]
    message.actual.positions = [1.0, -1.0]
    return message


def test_hardware_on_requires_explicit_request_and_ready_hold(udp_control):
    response = udp_control.on_hardware(SetBool.Request(data=True), SetBool.Response())
    assert response.success
    assert udp_control.udp.is_enabled
    assert udp_control.udp.method_calls == [call.poll(100.0), call.enable(100.0)]
    udp_control.command.publish.assert_not_called()


def test_leader_key_reset_to_hold_keeps_udp_disabled(udp_control):
    udp_control.udp.is_enabled = True
    udp_control.stop('Space')
    stopped_safety(udp_control)
    response = udp_control.on_leader_mode(SetBool.Request(data=True), SetBool.Response())
    assert response.success
    udp_control.future = None
    udp_control.phase = 'reset_wait_active'
    udp_control.safety['is_stop_latched'] = False
    udp_control.advance(100.0)
    assert udp_control.model.mode == 'hold'
    assert not udp_control.udp.is_enabled
    udp_control.udp.enable.assert_not_called()


@pytest.mark.parametrize('mode', ['leader', 'avoidance', 'stopped'])
def test_hardware_on_rejects_non_hold_mode(udp_control, mode):
    udp_control.model.mode = mode
    response = udp_control.on_hardware(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    udp_control.udp.enable.assert_not_called()


@pytest.mark.parametrize('phase', ['stopped', 'switch_stop_demo', 'reset_wait_active'])
def test_hardware_on_rejects_non_idle_phase(udp_control, phase):
    udp_control.phase = phase
    response = udp_control.on_hardware(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    udp_control.udp.enable.assert_not_called()


@pytest.mark.parametrize('condition', [
    'motion', 'stale_state', 'stale_safety', 'stale_terminal', 'latched_stop',
    'inactive_controller', 'pending_stop', 'pending_operation', 'unconfirmed_operation',
])
def test_hardware_on_rejects_incomplete_safety_confirmation(udp_control, condition):
    if condition == 'motion':
        udp_control.model.is_stationary.return_value = False
    elif condition == 'stale_state':
        udp_control.model.is_fresh.return_value = False
    elif condition == 'stale_safety':
        udp_control.safety_sec = 99.0
    elif condition == 'stale_terminal':
        udp_control.heartbeat_sec = 99.0
    elif condition == 'latched_stop':
        udp_control.safety['is_stop_latched'] = True
    elif condition == 'inactive_controller':
        udp_control.safety['has_active_commands'] = False
    elif condition == 'pending_stop':
        udp_control.is_stop_required = True
    elif condition == 'pending_operation':
        udp_control.future = Future()
    elif condition == 'unconfirmed_operation':
        udp_control.has_unconfirmed_operation = True
    response = udp_control.on_hardware(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    assert not udp_control.udp.is_enabled
    udp_control.udp.enable.assert_not_called()
    udp_control.command.publish.assert_not_called()


@pytest.mark.parametrize('method', ['poll', 'enable'])
@pytest.mark.parametrize('error', [ValueError, OSError])
def test_hardware_on_reports_transport_rejection(udp_control, method, error):
    getattr(udp_control.udp, method).side_effect = error('UDP状態不正')
    response = udp_control.on_hardware(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    assert not udp_control.udp.is_enabled
    udp_control.command.publish.assert_not_called()


@pytest.mark.parametrize('phase', ['idle', 'stopped', 'switch_stop_demo', 'reset_wait_active'])
def test_hardware_off_disables_output_without_fresh_state(udp_control, phase):
    udp_control.phase, udp_control.safety_sec, udp_control.heartbeat_sec = phase, 99.0, 99.0
    udp_control.udp.is_enabled = True
    udp_control.model.is_stationary.return_value = False
    udp_control.future = Future()
    response = udp_control.on_hardware(SetBool.Request(data=False), SetBool.Response())
    assert response.success
    assert not udp_control.udp.is_enabled
    udp_control.udp.disable.assert_called_once()
    udp_control.udp.poll.assert_not_called()
    udp_control.udp.enable.assert_not_called()


def test_space_disables_udp_before_waiting_for_gazebo_stop(udp_control):
    udp_control.udp.is_enabled = True
    udp_control.service_clients['safety/stop'].service_is_ready.return_value = False
    response = udp_control.on_stop(None, Trigger.Response())
    assert response.success
    assert not udp_control.udp.is_enabled
    assert udp_control.phase == 'stopped'
    assert udp_control.model.mode == 'stopped'
    assert udp_control.is_stop_required
    udp_control.udp.disable.assert_called_once()
    udp_control.command.publish.assert_not_called()


def test_external_safety_latch_disables_udp_immediately(udp_control):
    udp_control.udp.is_enabled = True
    udp_control.on_safety(String(data='{"is_stop_latched": true}'))
    assert not udp_control.udp.is_enabled
    assert udp_control.phase == 'stopped'
    assert udp_control.model.mode == 'stopped'
    udp_control.udp.disable.assert_called_once()


def test_udp_stop_send_failure_keeps_gazebo_stop_latched(udp_control):
    udp_control.udp.disable.side_effect = OSError('UDP停止送信の失敗')
    response = udp_control.on_stop(None, Trigger.Response())
    assert response.success
    assert udp_control.phase == 'stopped'
    assert udp_control.model.mode == 'stopped'
    assert udp_control.is_stop_required
    assert 'UDP停止送信の失敗' in udp_control.detail


@pytest.mark.parametrize('method', ['poll', 'tick'])
@pytest.mark.parametrize('error', [ValueError, OSError])
def test_udp_periodic_failure_requests_both_outputs_stop(udp_control, method, error):
    udp_control.udp.is_enabled = True
    getattr(udp_control.udp, method).side_effect = error('UDP通信の失敗')
    udp_control.tick()
    assert not udp_control.udp.is_enabled
    assert udp_control.phase == 'stopped'
    assert udp_control.model.mode == 'stopped'
    assert udp_control.is_stop_required
    udp_control.udp.disable.assert_called_once()
    udp_control.service_clients['safety/stop'].call_async.assert_called_once()


def test_reset_completion_keeps_udp_disabled_until_new_hardware_request(udp_control):
    udp_control.udp.is_enabled = True
    udp_control.on_stop(None, Trigger.Response())
    stopped_safety(udp_control)
    response = udp_control.on_reset(None, Trigger.Response())
    assert response.success
    assert not udp_control.udp.is_enabled
    udp_control.phase = 'reset_activate'
    udp_control.future.set_result(Trigger.Response(success=True))
    udp_control.safety['is_stop_latched'] = False
    udp_control.advance(100.0)
    assert udp_control.phase == 'idle'
    assert udp_control.model.mode == 'hold'
    assert not udp_control.udp.is_enabled
    udp_control.udp.enable.assert_not_called()


def test_controller_desired_is_forwarded_without_actual_position(udp_control):
    message = controller_state()
    udp_control.on_controller_state(message)
    udp_control.udp.update_target.assert_called_once()
    names, positions, stamp, now = udp_control.udp.update_target.call_args.args
    assert list(names) == ['L_joint1', 'R_joint1']
    assert list(positions) == [0.1, -0.2]
    assert (stamp, now) == (12.0, 100.0)
    udp_control.udp.enable.assert_not_called()
    udp_control.command.publish.assert_not_called()


def test_measured_joint_state_is_not_forwarded_as_udp_target(udp_control):
    message = JointState()
    message.header.stamp.sec = 12
    message.name, message.position, message.velocity = ['L_joint1'], [1.0], [0.0]
    udp_control.on_state(message)
    udp_control.udp.update_target.assert_not_called()
    udp_control.udp.enable.assert_not_called()


@pytest.mark.parametrize('is_enabled', [False, True])
@pytest.mark.parametrize('sec,nanosec', [(11, 499_000_000), (12, 101_000_000)])
def test_stale_or_future_controller_target_is_rejected(udp_control, is_enabled, sec, nanosec):
    udp_control.udp.is_enabled = is_enabled
    udp_control.on_controller_state(controller_state(sec, nanosec))
    assert not udp_control.udp.is_enabled
    assert (udp_control.phase == 'stopped') == is_enabled
    udp_control.udp.update_target.assert_not_called()
    udp_control.udp.disable.assert_called()


@pytest.mark.parametrize('is_enabled', [False, True])
@pytest.mark.parametrize('error', [ValueError, OSError])
def test_invalid_controller_target_disables_udp_and_stops_when_enabled(udp_control, is_enabled, error):
    udp_control.udp.is_enabled = is_enabled
    udp_control.udp.update_target.side_effect = error('関節目標の不正')
    udp_control.on_controller_state(controller_state())
    assert not udp_control.udp.is_enabled
    assert (udp_control.phase == 'stopped') == is_enabled
    udp_control.udp.enable.assert_not_called()


def test_invalid_target_and_stop_send_failure_still_request_gazebo_stop(udp_control):
    udp_control.udp.is_enabled = True
    udp_control.udp.update_target.side_effect = ValueError('関節目標の不正')
    udp_control.udp.disable.side_effect = OSError('UDP停止送信の失敗')
    udp_control.on_controller_state(controller_state())
    assert udp_control.phase == 'stopped'
    assert udp_control.model.mode == 'stopped'
    assert udp_control.is_stop_required
    udp_control.command.publish.assert_not_called()
