"""ROSノード起動を伴わない制御FSMの停止優先・切替拒否の検証。"""

from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys

import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import dual_arm_control as control_module


@pytest.fixture
def control(monkeypatch):
    monkeypatch.setattr(control_module, 'time', SimpleNamespace(monotonic=lambda: 100.0))
    node = control_module.dual_arm_control.__new__(control_module.dual_arm_control)
    node.model = Mock()
    node.model.mode = 'hold'
    node.model.is_fresh.return_value = True
    node.model.is_stationary.return_value = True
    node.model.has_fresh_leader.return_value = True
    node.model.command.return_value = None
    node.model.stop.side_effect = lambda: setattr(node.model, 'mode', 'stopped')
    node.model.enter.side_effect = lambda mode, _now: setattr(node.model, 'mode', mode)
    node.phase, node.detail = 'idle', ''
    node.target_mode = 'hold'
    node.safety = {'is_stop_latched': False, 'has_active_commands': True, 'state_age_sec': 0.0}
    node.demo = {'state': 'idle', 'run_generation': 0}
    node.safety_sec = node.demo_sec = node.heartbeat_sec = 100.0
    node.joint_stamp = node.mode_stamp = 12.0
    node.last_avoidance_stamp = float('-inf')
    node.avoidance_sec = 100.0
    node.future = node.stop_future = None
    node.deadline, node.settle_stamp, node.last_status_sec = 105.0, 11.0, 100.0
    node.has_unconfirmed_operation = False
    node.has_initialized_mode = True
    node.is_stop_required = False
    node.expected_run_generation = 1
    node.stop_deadline = 0.0
    node.command, node.status = Mock(), Mock()
    node.service_clients = {}
    for name in ('avoidance/start', 'avoidance/stop', 'safety/stop', 'safety/reset', 'switch'):
        client = Mock()
        client.service_is_ready.return_value = True
        client.srv_type = Trigger
        client.call_async.return_value = Future()
        node.service_clients[name] = client
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=12_000_000_000)))
    return node


def stopped_safety(control):
    """受付結果とは別の、実測停止確認を満たす診断値。"""
    control.phase, control.model.mode = 'stopped', 'stopped'
    control.safety = {'state': 'stopped', 'is_stop_latched': True, 'is_stop_applied': True,
                      'is_stopped': True, 'has_active_commands': True, 'state_age_sec': 0.0}


def test_late_success_after_stop_cannot_restore_mode(control):
    control.future = Future()
    control.future.set_result(Trigger.Response(success=True))
    control.stop('停止キー')
    control.advance(100.0)
    assert control.future is None
    assert control.phase == 'stopped'
    assert control.model.mode == 'stopped'
    control.model.enter.assert_not_called()
    control.command.publish.assert_not_called()


def test_unconfirmed_operation_keeps_stop_and_blocks_reset(control):
    control.future = Future()
    control.phase, control.deadline = 'switch_start_demo', 99.0
    control.advance(100.0)
    assert control.has_unconfirmed_operation
    assert control.phase == 'stopped'
    control.future.set_result(Trigger.Response(success=True))
    control.advance(100.0)
    stopped_safety(control)
    response = control.on_reset(None, Trigger.Response())
    assert not response.success
    control.service_clients['avoidance/stop'].call_async.assert_not_called()


@pytest.mark.parametrize('phase', ['switch_stop_demo', 'switch_settle', 'switch_start_demo',
                                  'switch_wait_running', 'reset_clear', 'stopped'])
def test_mode_request_during_unsafe_phase_rejected(control, phase):
    control.phase = phase
    response = control.on_leader_mode(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    control.model.enter.assert_not_called()
    control.service_clients['avoidance/stop'].call_async.assert_not_called()


def test_mode_request_with_pending_operation_rejected(control):
    control.future = Future()
    response = control.on_avoidance_mode(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    control.model.enter.assert_not_called()


@pytest.mark.parametrize('field', ['safety_sec', 'heartbeat_sec'])
def test_mode_request_requires_fresh_safety_and_terminal(control, field):
    setattr(control, field, 99.0)
    response = control.on_avoidance_mode(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    control.model.enter.assert_not_called()


def test_leader_requires_fresh_input(control):
    control.model.has_fresh_leader.return_value = False
    response = control.on_leader_mode(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    control.model.enter.assert_not_called()


def test_request_stops_demo_before_selecting_leader(control):
    response = control.on_leader_mode(SetBool.Request(data=True), SetBool.Response())
    assert response.success
    assert control.target_mode == 'leader'
    assert control.phase == 'switch_stop_demo'
    control.model.enter.assert_called_once_with('hold', 100.0)
    control.service_clients['avoidance/stop'].call_async.assert_called_once()
    control.service_clients['avoidance/start'].call_async.assert_not_called()


def test_mode_request_without_stop_service_latches_stop(control):
    control.service_clients['avoidance/stop'].service_is_ready.return_value = False
    response = control.on_leader_mode(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    assert control.phase == 'stopped'
    assert control.model.mode == 'stopped'


def test_hardware_on_is_rejected_without_any_command(control):
    response = control.on_hardware(SetBool.Request(data=True), SetBool.Response())
    assert not response.success
    control.command.publish.assert_not_called()
    control.model.enter.assert_not_called()


def test_hardware_off_is_idempotent(control):
    response = control.on_hardware(SetBool.Request(data=False), SetBool.Response())
    assert response.success
    control.command.publish.assert_not_called()


def test_late_avoidance_trajectory_is_ignored_while_stopped(control):
    control.stop('停止キー')
    control.on_avoidance(None)
    control.command.publish.assert_not_called()


def test_late_avoidance_trajectory_is_ignored_in_leader_mode(control):
    control.model.mode = 'leader'
    control.on_avoidance(None)
    control.command.publish.assert_not_called()


def test_reset_requires_measured_stop_not_only_latch(control):
    stopped_safety(control)
    control.safety['is_stopped'] = False
    response = control.on_reset(None, Trigger.Response())
    assert not response.success
    control.service_clients['avoidance/stop'].call_async.assert_not_called()


def test_reset_requires_stationary_fresh_joint_state(control):
    stopped_safety(control)
    control.model.is_stationary.return_value = False
    response = control.on_reset(None, Trigger.Response())
    assert not response.success
    control.service_clients['avoidance/stop'].call_async.assert_not_called()


def test_failed_operation_never_publishes_command(control):
    control.future = Future()
    control.future.set_result(Trigger.Response(success=False))
    control.phase = 'switch_start_demo'
    control.tick()
    assert control.phase == 'stopped'
    control.command.publish.assert_not_called()


def test_heartbeat_loss_overrides_pending_operation(control):
    control.heartbeat_sec = 99.0
    control.future = Future()
    control.future.set_result(Trigger.Response(success=True))
    control.phase = 'switch_start_demo'
    control.tick()
    assert control.phase == 'stopped'
    assert control.model.mode == 'stopped'
    control.model.enter.assert_not_called()
    control.command.publish.assert_not_called()


def test_invalid_state_stops_before_next_tick(control):
    control.model.update_state.return_value = False
    control.on_state(JointState())
    assert control.phase == 'stopped'
    assert control.is_stop_required


def test_invalid_leader_stops_before_next_tick(control):
    control.model.mode = 'leader'
    control.model.update_leader.return_value = False
    control.on_leader(JointState())
    assert control.phase == 'stopped'
    assert control.is_stop_required


def test_unselected_leader_input_cannot_change_mode(control):
    control.model.update_leader.return_value = False
    control.on_leader(JointState())
    assert control.phase == 'idle'
    assert control.model.mode == 'hold'


def test_external_stop_overrides_mode_operation(control):
    control.future = Future()
    control.phase = 'switch_start_demo'
    control.on_safety(String(data='{"is_stop_latched": true}'))
    assert control.phase == 'stopped'
    assert control.model.mode == 'stopped'
    assert control.future is not None


@pytest.mark.parametrize('phase', ['idle', 'switch_stop_demo', 'switch_settle',
                                  'switch_start_demo', 'switch_wait_running'])
def test_safety_expiry_blocks_hold_and_pending_mode_switch(control, phase):
    control.phase, control.safety_sec = phase, 99.0
    control.target_mode = 'avoidance'
    control.tick()
    assert control.phase == 'stopped'
    assert control.model.mode == 'stopped'
    control.service_clients['avoidance/start'].call_async.assert_not_called()
    control.command.publish.assert_not_called()


def test_reset_waits_for_pending_stop_response(control):
    stopped_safety(control)
    control.stop_future = Future()
    response = control.on_reset(None, Trigger.Response())
    assert not response.success
    control.service_clients['avoidance/stop'].call_async.assert_not_called()


def test_start_response_waits_for_current_running_status(control):
    control.future = Future()
    control.future.set_result(Trigger.Response(success=True))
    control.phase = 'switch_start_demo'
    control.advance(100.0)
    assert control.phase == 'switch_wait_running'
    assert control.model.mode == 'hold'
    assert control.demo['state'] == 'idle'
    control.model.enter.assert_not_called()


def test_previous_run_status_cannot_open_avoidance_gate(control):
    control.phase = 'switch_wait_running'
    control.expected_run_generation = 2
    control.demo = {'state': 'running', 'run_generation': 1, 'run_start_stamp_sec': 11.0}
    control.advance(100.0)
    assert control.phase == 'switch_wait_running'
    assert control.model.mode == 'hold'
    control.model.enter.assert_not_called()


def test_current_running_status_opens_avoidance_gate(control):
    control.phase = 'switch_wait_running'
    control.demo = {'state': 'running', 'run_generation': 1, 'run_start_stamp_sec': 12.0}
    control.advance(100.0)
    assert control.phase == 'idle'
    assert control.model.mode == 'avoidance'
    control.model.enter.assert_called_once_with('avoidance', 100.0)


def test_old_idle_status_does_not_terminate_new_avoidance(control):
    control.model.mode = 'avoidance'
    control.expected_run_generation = 2
    control.demo = {'state': 'running', 'run_generation': 2, 'run_start_stamp_sec': 12.0}
    control.on_demo(String(data='{"state":"idle", "run_generation":1, "run_start_stamp_sec":11.0}'))
    control.tick()
    assert control.model.mode == 'avoidance'
    assert control.phase == 'idle'
    control.model.enter.assert_not_called()


def test_reset_activation_returns_hold_without_restarting_demo(control):
    control.model.mode = 'stopped'
    control.phase = 'reset_activate'
    control.future = Future()
    control.future.set_result(Trigger.Response(success=True))
    control.advance(100.0)
    assert control.model.mode == 'hold'
    assert control.phase == 'idle'
    control.service_clients['avoidance/start'].call_async.assert_not_called()


@pytest.mark.parametrize('data', ['{}', '{"state":"unknown", "run_generation":1}',
                                '{"state":"running", "run_generation":2}',
                                '{"state":"running", "run_generation":true}'])
def test_invalid_current_demo_status_forces_stop(control, data):
    control.model.mode = 'avoidance'
    control.on_demo(String(data=data))
    assert control.phase == 'stopped'
    control.tick()
    assert control.phase == 'stopped'
    assert control.model.mode == 'stopped'
    control.command.publish.assert_not_called()


def test_boolean_generation_cannot_confirm_running(control):
    control.phase = 'switch_wait_running'
    control.demo = {'state': 'running', 'run_generation': True, 'run_start_stamp_sec': 12.0}
    control.tick()
    assert control.model.mode != 'avoidance'
    control.model.enter.assert_not_called()


def test_unconfirmed_stop_request_blocks_reset_even_after_measured_stop(control):
    stopped_safety(control)
    control.stop_future = Future()
    control.stop_deadline = 99.0
    control.tick()
    assert control.has_unconfirmed_operation
    response = control.on_reset(None, Trigger.Response())
    assert not response.success
    control.service_clients['avoidance/stop'].call_async.assert_not_called()


def test_reset_waits_for_active_status_after_activation_response(control):
    control.model.mode, control.phase = 'stopped', 'reset_activate'
    control.safety['has_active_commands'] = False
    control.future = Future()
    control.future.set_result(Trigger.Response(success=True))
    control.advance(100.0)
    assert control.phase == 'reset_wait_active'
    assert control.model.mode == 'stopped'
    control.tick()
    assert control.phase == 'reset_wait_active'
    control.model.enter.assert_not_called()
    control.command.publish.assert_not_called()
    control.safety['has_active_commands'] = True
    control.advance(100.0)
    assert control.phase == 'idle'
    assert control.model.mode == 'hold'
    control.model.enter.assert_called_once_with('hold', 100.0)


def test_reset_active_status_timeout_keeps_output_stopped(control):
    control.model.mode, control.phase = 'stopped', 'reset_wait_active'
    control.safety['has_active_commands'] = False
    control.deadline = 99.0
    control.tick()
    assert control.phase == 'stopped'
    assert control.model.mode == 'stopped'
    control.model.enter.assert_not_called()
    control.command.publish.assert_not_called()
