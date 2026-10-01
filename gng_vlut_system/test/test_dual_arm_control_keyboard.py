"""ROS接続不要の統合操作、停止優先、端末復元、期限付き終了の回帰検証。"""

from concurrent.futures import Future
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import pty
import signal
import sys
import termios
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import dual_arm_control_keyboard as keyboard


class test_keys(unittest.TestCase):
    def test_stop_and_exit_keys(self):
        for key in (b' ',):
            with self.subTest(key=key):
                self.assertEqual(keyboard.control_key_action(key), 'stop')
        for key in (b'\x03', b'\x04', b''):
            with self.subTest(key=key):
                self.assertEqual(keyboard.control_key_action(key), 'quit')

    def test_mode_keys_and_ignored_keys(self):
        for key, action in ((b'a', 'avoidance'), (b'l', 'leader'), (b'h', 'hardware')):
            for variant in (key, key.upper()):
                with self.subTest(key=variant):
                    self.assertEqual(keyboard.control_key_action(variant), action)
        for key in (b'z', b'\n', b'\r', b'\x1b', b'r', b'R', b'q', b'Q', b's', b'S'):
            self.assertIsNone(keyboard.control_key_action(key))

    def test_escape_sequences_never_enable_modes(self):
        for sequence in (b'\x1b[A', b'\x1bOA', b'\x1b[1;2A', b'\x1b[H', b'\x1b[1;5R', b'\x1bl'):
            with self.subTest(sequence=sequence):
                decoder = keyboard.terminal_key_decoder()
                for value in sequence:
                    self.assertIsNone(decoder.read_action(bytes([value])))
                self.assertEqual(decoder.read_action(b'A'), 'avoidance')

    def test_bracketed_paste_never_enables_modes(self):
        decoder = keyboard.terminal_key_decoder()
        for value in b'\x1b[200~AaLlHhRr\x1b[201~':
            self.assertIsNone(decoder.read_action(bytes([value])))
        self.assertFalse(decoder.is_paste)
        self.assertEqual(decoder.read_action(b'L'), 'leader')

    def test_stop_and_exit_override_escape_or_paste(self):
        for prefix in (b'\x1b', b'\x1b[', b'\x1b[200~'):
            for key, action in ((b' ', 'stop'), (b'\x03', 'quit'), (b'', 'quit')):
                with self.subTest(prefix=prefix, key=key):
                    decoder = keyboard.terminal_key_decoder()
                    for value in prefix:
                        decoder.read_action(bytes([value]))
                    self.assertEqual(decoder.read_action(key), action)

    def test_long_escape_retains_mode_suppression(self):
        decoder = keyboard.terminal_key_decoder()
        for value in b'\x1b[' + b'1;' * 100 + b'A':
            self.assertIsNone(decoder.read_action(bytes([value])))
        self.assertEqual(decoder.read_action(b'H'), 'hardware')


class test_live_status(unittest.TestCase):
    def setUp(self):
        self.status = {'mode': 'hold', 'enable_hardware_output': False, 'detail': ''}

    def test_modes_toggle_from_latest_state(self):
        for mode in ('hold', 'avoidance', 'leader', 'stopped'):
            for action in ('avoidance', 'leader'):
                with self.subTest(mode=mode, action=action):
                    self.status['mode'] = mode
                    self.assertEqual(keyboard.toggle_value(action, self.status, .1), mode != action)

    def test_hardware_toggle_uses_boolean_state(self):
        for enable_hardware_output in (False, True):
            self.status['enable_hardware_output'] = enable_hardware_output
            self.assertEqual(keyboard.toggle_value('hardware', self.status, .1), not enable_hardware_output)

    def test_half_second_boundary(self):
        self.assertTrue(keyboard.toggle_value('avoidance', self.status, .5))
        self.assertIsNone(keyboard.toggle_value('avoidance', self.status, .500001))

    def test_invalid_ages_reject_toggle_and_display(self):
        for age in (-.1, .500001, None, True, '0', float('nan'), float('inf'), {}, [], 10 ** 1000):
            with self.subTest(age_type=type(age).__name__):
                self.assertIsNone(keyboard.toggle_value('avoidance', self.status, age))
                self.assertIn('未受信・失効', keyboard.control_status_label(self.status, age))

    def test_invalid_status_never_toggles(self):
        for status in (None, [], '', 1, {}, {'mode': [], 'enable_hardware_output': False},
                       {'mode': {}, 'enable_hardware_output': False},
                       {'mode': 'unknown', 'enable_hardware_output': False},
                       {'mode': 'hold', 'enable_hardware_output': 1}):
            with self.subTest(status=status):
                self.assertIsNone(keyboard.toggle_value('leader', status, .1))
                label = keyboard.control_status_label(status, .1)
                self.assertTrue('未受信・失効' in label or '診断形式不正' in label)

    def test_unknown_action_never_toggles(self):
        self.assertIsNone(keyboard.toggle_value('reset', self.status, .1))

    def test_detail_is_single_line_and_bounded(self):
        self.status['detail'] = 'a\nb\r' + 'x' * 200
        label = keyboard.control_status_label(self.status, .1)
        self.assertNotIn('\n', label)
        self.assertNotIn('\r', label)
        self.assertLessEqual(len(label), 200)

    def test_mode_label_never_claims_measured_stop(self):
        self.status['mode'] = 'stopped'
        label = keyboard.control_status_label(self.status, .1)
        self.assertIn('停止要求中・ラッチ', label)
        self.assertNotIn('実測停止: 確認済み', label)

    def test_switching_label_without_toggle_permission(self):
        self.status['mode'] = 'switching'
        self.assertIn('切替中', keyboard.control_status_label(self.status, .1))
        for action in ('avoidance', 'leader', 'hardware'):
            self.assertIsNone(keyboard.toggle_value(action, self.status, .1))


class test_control_request(unittest.TestCase):
    def setUp(self):
        self.now_sec = 10.0
        self.messages = []
        self.future = Future()
        self.client = Mock()
        self.client.service_is_ready.return_value = True
        self.client.call_async.return_value = self.future
        self.payload = object()
        self.request = keyboard.control_request(self.messages.append, now=lambda: self.now_sec)

    def begin(self):
        return self.request.begin(self.client, self.payload, '回避ON')

    def test_idle_has_no_service_calls(self):
        self.request.poll()
        self.client.service_is_ready.assert_not_called()

    def test_busy_rejects_new_request_and_keeps_deadline(self):
        self.assertTrue(self.begin())
        self.request.poll()
        self.now_sec = 11.0
        self.assertFalse(self.request.begin(Mock(), object(), 'リーダーON'))
        self.assertEqual(self.request.deadline, 12.0)
        self.assertIn('処理中', self.messages[-1])
        self.client.call_async.assert_called_once_with(self.payload)

    def test_cancel_discards_pending_response(self):
        self.begin()
        self.request.poll()
        self.request.cancel()
        self.assertTrue(self.future.cancelled())
        self.assertFalse(self.request.is_pending)
        self.assertIsNone(self.request.future)
        self.assertIn('停止優先', self.messages[-1])
        self.request.poll()
        self.client.call_async.assert_called_once()

    def test_unavailable_service_has_finite_deadline(self):
        self.client.service_is_ready.return_value = False
        self.begin()
        self.request.poll()
        self.now_sec = 12.0
        self.request.poll()
        self.assertFalse(self.request.is_pending)
        self.assertIn('応答未確認', self.messages[-1])
        self.client.call_async.assert_not_called()

    def test_unanswered_future_cancelled_at_deadline(self):
        self.begin()
        self.request.poll()
        self.now_sec = 12.0
        self.request.poll()
        self.assertTrue(self.future.cancelled())
        self.assertFalse(self.request.is_pending)

    def test_acceptance_clears_busy_state(self):
        self.begin()
        self.request.poll()
        self.future.set_result(SimpleNamespace(success=True, message='受付'))
        self.request.poll()
        self.assertFalse(self.request.is_pending)
        self.assertIn('受付済み', self.messages[-1])
        self.assertTrue(self.begin())

    def test_rejection_and_invalid_responses(self):
        for response in (None, SimpleNamespace(success=False, message='未許可'), SimpleNamespace(success=1)):
            with self.subTest(response=response):
                self.future = Future()
                self.client.call_async.return_value = self.future
                self.begin()
                self.request.poll()
                self.future.set_result(response)
                self.request.poll()
                self.assertFalse(self.request.is_pending)
                self.assertIn('拒否・応答不正', self.messages[-1])

    def test_send_and_response_exceptions_end_request(self):
        self.client.call_async.side_effect = RuntimeError('送信失敗')
        self.begin()
        self.request.poll()
        self.assertFalse(self.request.is_pending)
        self.assertIn('送信失敗', self.messages[-1])
        self.client.call_async.side_effect = None
        self.begin()
        self.request.poll()
        self.future.set_exception(RuntimeError('応答失敗'))
        self.request.poll()
        self.assertFalse(self.request.is_pending)
        self.assertIn('応答失敗', self.messages[-1])


class console_fixture:
    """実ROS・子プロセスなしのmain試験用サービス、時計、PTY。"""

    def __init__(self, keys, *, has_control_status=True, is_stop_ready=True, stop_delay_sec=.08,
                 signal_after_spin=None, has_read_error=False, control_mode='hold'):
        self.keys = list(keys)
        self.has_control_status = has_control_status
        self.control_mode = control_mode
        self.is_stop_ready = is_stop_ready
        self.stop_delay_sec = stop_delay_sec
        self.signal_after_spin = signal_after_spin
        self.has_read_error = has_read_error
        self.now_sec = 0.0
        self.is_running = True
        self.num_spins = 0
        self.clients = {}
        self.subscriptions = {}
        self.pending = []
        self.messages = []
        self.heartbeat_times = []
        self.handlers = {}
        self.node = Mock()
        self.node.create_client.side_effect = self.create_client
        self.node.create_subscription.side_effect = self.create_subscription
        self.node.create_publisher.return_value = SimpleNamespace(publish=lambda _: self.heartbeat_times.append(self.now_sec))
        self.rclpy = ModuleType('rclpy')
        self.rclpy.init = Mock()
        self.rclpy.create_node = Mock(return_value=self.node)
        self.rclpy.ok = lambda: self.is_running
        self.rclpy.spin_once = self.spin_once
        self.rclpy.shutdown = Mock(side_effect=self.shutdown)

    def shutdown(self):
        self.is_running = False

    def create_client(self, service_type, name):
        client = Mock()
        client.service_is_ready.return_value = self.is_stop_ready if name.endswith('/stop') else True
        client.call_async.side_effect = lambda request: self.call_async(name, request)
        self.clients[name.rsplit('/', 1)[-1]] = client
        return client

    def call_async(self, name, request):
        future = Future()
        delay_sec = self.stop_delay_sec if name.endswith('/stop') else math.inf
        self.pending.append((future, self.now_sec + delay_sec, request))
        return future

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = (callback, qos)
        if topic.endswith('/control/status') and self.has_control_status:
            callback(SimpleNamespace(data=json.dumps({'mode': self.control_mode, 'enable_hardware_output': False, 'detail': ''})))
        return Mock()

    def spin_once(self, node, timeout_sec):
        self.num_spins += 1
        if self.num_spins > 200:
            raise AssertionError('試験ループの上限超過')
        self.now_sec += .02
        for future, ready_sec, _ in self.pending:
            if not future.done() and self.now_sec >= ready_sec:
                future.set_result(SimpleNamespace(success=True, message='受付'))
        if self.signal_after_spin is not None and self.num_spins == 1:
            self.handlers[self.signal_after_spin](self.signal_after_spin, None)

    def register_signal(self, signum, handler):
        previous = self.handlers.get(signum)
        self.handlers[signum] = handler
        return previous

    def read_key(self, descriptor, num_bytes):
        if self.has_read_error:
            self.has_read_error = False
            raise OSError('模擬端末切断')
        return self.keys.pop(0)

    def run(self, extra_arguments=()):
        modules = {'rclpy': self.rclpy}
        for name in ('rclpy.qos', 'rclpy.signals', 'rclpy.utilities', 'std_msgs', 'std_msgs.msg', 'std_srvs', 'std_srvs.srv'):
            modules[name] = ModuleType(name)
        modules['rclpy.qos'].QoSProfile = lambda **kwargs: SimpleNamespace(**kwargs)
        modules['rclpy.qos'].DurabilityPolicy = SimpleNamespace(VOLATILE='volatile')
        modules['rclpy.signals'].SignalHandlerOptions = SimpleNamespace(NO='no_signals')
        modules['rclpy.utilities'].remove_ros_args = lambda args: args[:args.index('--ros-args')] if '--ros-args' in args else args
        modules['std_msgs.msg'].Empty = type('Empty', (), {})
        modules['std_msgs.msg'].String = type('String', (), {})
        modules['std_srvs.srv'].SetBool = SimpleNamespace(Request=lambda **kwargs: SimpleNamespace(**kwargs))
        modules['std_srvs.srv'].Trigger = SimpleNamespace(Request=lambda: SimpleNamespace())
        real_control_request, real_stop_request = keyboard.control_request, keyboard.stop_request
        master, slave = pty.openpty()
        previous = termios.tcgetattr(slave)
        arguments = ['--namespace', 'sim_topo_dual_arm_max', '--tty-path', os.ttyname(slave), *extra_arguments]
        try:
            with ExitStack() as stack:
                stack.enter_context(patch.dict(sys.modules, modules))
                stack.enter_context(patch.object(keyboard.time, 'monotonic', lambda: self.now_sec))
                stack.enter_context(patch.object(keyboard, 'control_request', lambda emit: real_control_request(emit, now=lambda: self.now_sec)))
                stack.enter_context(patch.object(keyboard, 'stop_request', lambda client, factory, emit: real_stop_request(client, factory, emit, now=lambda: self.now_sec)))
                stack.enter_context(patch.object(keyboard.select, 'select', lambda *args: ([slave] if self.keys or self.has_read_error else [], [], [])))
                stack.enter_context(patch.object(keyboard.os, 'read', self.read_key))
                stack.enter_context(patch.object(keyboard.os, 'write', lambda _, data: self.messages.append(data.decode('utf-8')) or len(data)))
                stack.enter_context(patch.object(keyboard.signal, 'signal', self.register_signal))
                result = keyboard.main(arguments)
            self.has_restored_terminal = termios.tcgetattr(slave) == previous
            return result
        finally:
            os.close(master)
            os.close(slave)


class test_console_main(unittest.TestCase):
    def test_stopped_leader_key_is_labelled_hold_reset_not_following(self):
        fixture = console_fixture([b'l', b'\x03'], control_mode='stopped')
        self.assertEqual(fixture.run(), 0)
        fixture.clients['leader'].call_async.assert_called_once()
        self.assertTrue(fixture.clients['leader'].call_async.call_args.args[0].data)
        self.assertIn('停止解除→ホールド: 送信待ち', ''.join(fixture.messages))
        self.assertNotIn('リーダーフォロワーON: 送信待ち', ''.join(fixture.messages))
        self.assert_clean_exit(fixture)

    def assert_clean_exit(self, fixture):
        self.assertTrue(fixture.has_restored_terminal)
        fixture.node.destroy_node.assert_called_once()
        fixture.rclpy.shutdown.assert_called_once()
        self.assertTrue(all(handler is None for handler in fixture.handlers.values()))

    def test_exit_keys_request_stop_and_restore_terminal(self):
        for key in (b'\x03', b'\x04', b''):
            with self.subTest(key=key):
                fixture = console_fixture([key])
                self.assertEqual(fixture.run(), 0)
                fixture.clients['stop'].call_async.assert_called_once()
                self.assert_clean_exit(fixture)

    def test_stop_preempts_pending_mode_change(self):
        fixture = console_fixture([b'a', b' ', b'\x03'])
        self.assertEqual(fixture.run(), 0)
        fixture.clients['avoidance'].call_async.assert_called_once()
        self.assertTrue(fixture.pending[0][0].cancelled())
        fixture.clients['stop'].call_async.assert_called_once()
        self.assertIn('停止優先', ''.join(fixture.messages))
        self.assert_clean_exit(fixture)

    def test_busy_mode_change_never_sends_second_operation(self):
        fixture = console_fixture([b'a', b'l', b'\x03'])
        self.assertEqual(fixture.run(), 0)
        fixture.clients['avoidance'].call_async.assert_called_once()
        fixture.clients['leader'].call_async.assert_not_called()
        self.assertIn('前の要求を処理中', ''.join(fixture.messages))

    def test_pending_stop_rejects_mode_change(self):
        fixture = console_fixture([b' ', b'a', b'\x03'])
        self.assertEqual(fixture.run(), 0)
        fixture.clients['avoidance'].call_async.assert_not_called()
        self.assertIn('停止要求を優先中', ''.join(fixture.messages))

    def test_unknown_status_rejects_toggle_but_allows_stop(self):
        fixture = console_fixture([b'a', b'l', b'h', b'\x03'], has_control_status=False)
        self.assertEqual(fixture.run(), 0)
        for name in ('avoidance', 'leader', 'hardware'):
            fixture.clients[name].call_async.assert_not_called()
        fixture.clients['stop'].call_async.assert_called_once()

    def test_removed_keys_never_send_operations_or_exit(self):
        fixture = console_fixture([b'r', b'R', b'q', b'Q', b's', b'S', b'\x03'])
        self.assertEqual(fixture.run(), 0)
        self.assertNotIn('reset', fixture.clients)
        for name in ('avoidance', 'leader', 'hardware'):
            fixture.clients[name].call_async.assert_not_called()
        self.assertEqual(fixture.keys, [])
        fixture.clients['stop'].call_async.assert_called_once()

    def test_missing_stop_service_times_out_with_failure(self):
        fixture = console_fixture([b'\x03'], is_stop_ready=False)
        self.assertEqual(fixture.run(), 2)
        fixture.clients['stop'].call_async.assert_not_called()
        self.assertGreaterEqual(fixture.now_sec, 2.0)
        self.assertLess(fixture.now_sec, 2.1)
        self.assert_clean_exit(fixture)

    def test_unanswered_stop_times_out_and_cancels_future(self):
        fixture = console_fixture([b'\x03'], stop_delay_sec=math.inf)
        self.assertEqual(fixture.run(), 2)
        self.assertTrue(fixture.pending[0][0].cancelled())
        self.assertLess(fixture.now_sec, 2.1)
        self.assert_clean_exit(fixture)

    def test_os_signals_request_stop_before_exit(self):
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signum=signum):
                fixture = console_fixture([], signal_after_spin=signum)
                self.assertEqual(fixture.run(), 0)
                fixture.clients['stop'].call_async.assert_called_once()
                self.assert_clean_exit(fixture)

    def test_terminal_read_error_requests_stop(self):
        fixture = console_fixture([], has_read_error=True)
        self.assertEqual(fixture.run(), 0)
        fixture.clients['stop'].call_async.assert_called_once()
        self.assert_clean_exit(fixture)

    def test_heartbeat_uses_wall_clock_during_wait(self):
        fixture = console_fixture([b'\x03'], stop_delay_sec=.36)
        self.assertEqual(fixture.run(), 0)
        self.assertGreaterEqual(len(fixture.heartbeat_times), 3)
        for previous, current in zip(fixture.heartbeat_times, fixture.heartbeat_times[1:]):
            self.assertGreaterEqual(current - previous, .1 - 1e-8)
            self.assertLessEqual(current - previous, .12 + 1e-8)

    def test_ros_remapping_arguments_reach_rclpy_only(self):
        fixture = console_fixture([b'\x03'])
        self.assertEqual(fixture.run(['--ros-args', '-r', '__node:=console_test']), 0)
        self.assertEqual(fixture.rclpy.init.call_args.kwargs['args'][-3:], ['--ros-args', '-r', '__node:=console_test'])
        self.assertEqual(fixture.rclpy.init.call_args.kwargs['signal_handler_options'], 'no_signals')
        self.assertTrue(all(qos.durability == 'volatile' and qos.depth == 1 for _, qos in fixture.subscriptions.values()))


if __name__ == '__main__':
    unittest.main()
