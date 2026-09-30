"""ROS接続不要の停止キー・診断表示・非同期要求・端末復元検証。"""

from contextlib import contextmanager
import json
import os
from pathlib import Path
import pty
import select
import sys
import tempfile
import termios
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from gazebo_stop_keyboard import is_fresh_age, key_action, status_label, stop_request, terminal_input


class test_key_action(unittest.TestCase):
    def test_stop_keys(self):
        for key in (b' ', b's', b'S'):
            with self.subTest(key=key):
                self.assertEqual(key_action(key), 'stop')

    def test_exit_keys_include_control_and_eof(self):
        for key in (b'q', b'Q', b'\x03', b'\x04', b''):
            with self.subTest(key=key):
                self.assertEqual(key_action(key), 'quit')

    def test_other_keys_have_no_action(self):
        for key in (b'r', b'R', b'\r', b'\n', b'\x1b', b'\x1a', b'1'):
            with self.subTest(key=key):
                self.assertIsNone(key_action(key))


class test_status_label(unittest.TestCase):
    def setUp(self):
        self.status = {'state': 'stopped', 'is_stop_latched': True,
                       'is_stop_applied': True, 'is_stopped': True, 'state_age_sec': 0.1}

    def test_fresh_complete_state_confirms_measured_stop(self):
        self.assertEqual(status_label(self.status, 0.1), '実測停止: 確認済み')

    def test_age_boundary_includes_half_second(self):
        self.status['state_age_sec'] = 0.5
        self.assertEqual(status_label(self.status, 0.5), '実測停止: 確認済み')
        self.assertTrue(is_fresh_age(0))
        self.assertFalse(is_fresh_age(0.500001))

    def test_local_receive_expiry_invalidates_old_confirmation(self):
        for value in (0.500001, -0.001, None, True, '0', float('nan'), float('inf')):
            with self.subTest(value=value):
                self.assertEqual(status_label(self.status, value), '停止状態: 未受信・失効')

    def test_remote_state_expiry_keeps_latch_unconfirmed(self):
        for value in (0.500001, -0.001, None, True, '0', float('nan'), float('inf'), {}, []):
            with self.subTest(value=value):
                self.status['state_age_sec'] = value
                self.assertEqual(status_label(self.status, 0.1), '停止ラッチ: ON / 実測停止: 未確認')

    def test_huge_json_integer_is_not_a_fresh_age(self):
        value = json.loads('1' + '0' * 1000)
        self.assertFalse(is_fresh_age(value))
        self.status['state_age_sec'] = value
        self.assertEqual(status_label(self.status, 0.1), '停止ラッチ: ON / 実測停止: 未確認')
        self.assertEqual(status_label(self.status, value), '停止状態: 未受信・失効')

    def test_missing_or_nonboolean_conditions_never_confirm_stop(self):
        for name in ('is_stop_latched', 'is_stop_applied', 'is_stopped'):
            for value in (None, False, 1, 'true'):
                with self.subTest(name=name, value=value):
                    status = dict(self.status, **{name: value})
                    self.assertNotEqual(status_label(status, 0.1), '実測停止: 確認済み')
            status = self.status.copy()
            del status[name]
            self.assertNotEqual(status_label(status, 0.1), '実測停止: 確認済み')

    def test_nonstopped_state_never_confirms_stop(self):
        for state in ('running', 'stop_requested', 'stop_unconfirmed', None):
            with self.subTest(state=state):
                self.status['state'] = state
                self.assertEqual(status_label(self.status, 0.1), '停止ラッチ: ON / 実測停止: 未確認')

    def test_malformed_json_structures_and_off_state(self):
        for status in (None, [], 'stopped', 1, False):
            with self.subTest(status=status):
                self.assertEqual(status_label(status, 0.1), '停止状態: 未受信・失効')
        self.assertEqual(status_label({}, 0.1), '停止状態: 診断形式不正')
        self.assertEqual(status_label({'is_stop_latched': False}, 0.1), '停止ラッチ: OFF')


class test_stop_request(unittest.TestCase):
    def setUp(self):
        self.now_sec = 10.0
        self.messages = []
        self.future = Mock()
        self.future.done.return_value = False
        self.client = Mock()
        self.client.service_is_ready.return_value = True
        self.client.call_async.return_value = self.future
        self.factory = Mock(return_value=object())
        self.request = stop_request(self.client, self.factory, self.messages.append,
                                    now=lambda: self.now_sec)

    def test_idle_poll_has_no_service_call(self):
        self.request.poll()
        self.client.service_is_ready.assert_not_called()
        self.client.call_async.assert_not_called()

    def test_acceptance_is_not_physical_stop(self):
        self.request.begin()
        self.request.poll()
        self.future.done.return_value = True
        self.future.result.return_value = SimpleNamespace(success=True)
        self.request.poll()
        self.assertFalse(self.request.is_pending)
        self.assertTrue(self.request.has_succeeded)
        self.assertIn('受付済み（停止完了とは別）', self.messages[-1])
        self.client.call_async.assert_called_once_with(self.factory.return_value)

    def test_rejected_and_invalid_responses(self):
        for response in (None, SimpleNamespace(success=False), SimpleNamespace(success=1)):
            with self.subTest(response=response):
                self.request.begin()
                self.request.poll()
                self.future.done.return_value = True
                self.future.result.return_value = response
                self.request.poll()
                self.assertFalse(self.request.is_pending)
                self.assertFalse(self.request.has_succeeded)
                self.assertIn('拒否・応答不正', self.messages[-1])

    def test_unavailable_service_times_out_without_send(self):
        self.client.service_is_ready.return_value = False
        self.request.begin()
        self.request.poll()
        self.now_sec += 2.0
        self.request.poll()
        self.assertFalse(self.request.is_pending)
        self.assertFalse(self.request.has_succeeded)
        self.client.call_async.assert_not_called()
        self.assertIn('応答未確認', self.messages[-1])

    def test_unanswered_request_cancels_future_at_deadline(self):
        self.request.begin()
        self.request.poll()
        self.now_sec += 2.0
        self.request.poll()
        self.future.cancel.assert_called_once_with()
        self.assertFalse(self.request.is_pending)
        self.assertFalse(self.request.has_succeeded)

    def test_repeated_key_keeps_single_request_and_deadline(self):
        self.request.begin()
        self.request.poll()
        first_deadline = self.request.deadline
        self.now_sec += 1.0
        self.request.begin()
        self.request.poll()
        self.assertEqual(self.request.deadline, first_deadline)
        self.assertEqual(self.messages.count('停止要求: 送信待ち'), 1)
        self.client.call_async.assert_called_once()

    def test_new_key_after_response_starts_new_request(self):
        self.request.begin()
        self.request.poll()
        self.future.done.return_value = True
        self.future.result.return_value = SimpleNamespace(success=True)
        self.request.poll()
        self.now_sec += 1.0
        self.request.begin()
        self.assertFalse(self.request.has_succeeded)
        self.assertTrue(self.request.is_pending)
        self.assertIsNone(self.request.future)
        self.assertEqual(self.request.deadline, 13.0)
        self.request.poll()
        self.assertEqual(self.client.call_async.call_count, 2)

    def test_service_and_factory_exceptions_end_request(self):
        for failing_call in (self.client.service_is_ready, self.client.call_async, self.factory):
            with self.subTest(failing_call=failing_call):
                failing_call.side_effect = RuntimeError('試験用失敗')
                self.request.begin()
                self.request.poll()
                self.assertFalse(self.request.is_pending)
                self.assertFalse(self.request.has_succeeded)
                self.assertIn('失敗 / 試験用失敗', self.messages[-1])
                failing_call.side_effect = None

    def test_future_result_exception_ends_request(self):
        self.request.begin()
        self.request.poll()
        self.future.done.return_value = True
        self.future.result.side_effect = RuntimeError('試験用応答失敗')
        self.request.poll()
        self.assertFalse(self.request.is_pending)
        self.assertFalse(self.request.has_succeeded)
        self.assertIn('試験用応答失敗', self.messages[-1])


@contextmanager
def temporary_terminal():
    """子プロセスを持たない一時PTYの確保と解放。"""
    master, slave = pty.openpty()
    try:
        with os.fdopen(slave, 'rb', buffering=0) as stream:
            yield master, stream
    finally:
        os.close(master)


class test_terminal_input(unittest.TestCase):
    def test_keys_available_without_enter_and_settings_restored(self):
        with temporary_terminal() as (master, stream):
            previous = termios.tcgetattr(stream.fileno())
            with terminal_input(stream) as descriptor:
                current = termios.tcgetattr(descriptor)
                self.assertEqual(current[3] & (termios.ICANON | termios.ECHO | termios.ISIG), 0)
                os.write(master, b's\x03')
                self.assertTrue(select.select([descriptor], [], [], 1.0)[0])
                self.assertEqual(os.read(descriptor, 2), b's\x03')
            self.assertEqual(termios.tcgetattr(stream.fileno()), previous)

    def test_settings_restored_after_exception(self):
        with temporary_terminal() as (_, stream):
            previous = termios.tcgetattr(stream.fileno())
            with self.assertRaisesRegex(RuntimeError, '試験用例外'):
                with terminal_input(stream):
                    raise RuntimeError('試験用例外')
            self.assertEqual(termios.tcgetattr(stream.fileno()), previous)

    def test_nonterminal_rejected(self):
        with tempfile.TemporaryFile() as stream:
            with self.assertRaisesRegex(ValueError, '対話端末が必要'):
                with terminal_input(stream):
                    self.fail('非端末の誤受付')


if __name__ == '__main__':
    unittest.main()
