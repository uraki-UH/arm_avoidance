#!/usr/bin/env python3
"""親launch端末からのモード選択と、最優先のソフト停止要求。"""

import argparse
import json
import math
import os
import select
import signal
import sys
import time

from gazebo_stop_keyboard import is_fresh_age, simulation_namespace, status_label, stop_request, terminal_input


def control_key_action(key):
    """通常操作3キーと、UDP許可・端末終了の分類。"""
    if key == b' ':
        return 'stop'
    if key in (b'\x03', b'\x04', b''):
        return 'quit'
    return {b'a': 'avoidance', b'l': 'leader', b'h': 'hardware'}.get(key.lower())


class terminal_key_decoder:
    """矢印・機能キーの末尾や貼付文字列による、意図しないモード変更の抑制。"""

    def __init__(self):
        self.escape = b''
        self.is_paste = False
        self.paste_tail = b''

    def read_action(self, key):
        action = control_key_action(key)
        if action in ('stop', 'quit'):
            return action
        if self.is_paste:
            self.paste_tail = (self.paste_tail + key)[-6:]
            if self.paste_tail == b'\x1b[201~':
                self.is_paste = False
                self.paste_tail = b''
            return None
        if key == b'\x1b':
            self.escape = key
            return None
        if self.escape == b'\x1b':
            self.escape = self.escape + key if key in (b'[', b'O') else b''
            return None
        if self.escape.startswith(b'\x1b['):
            if len(key) == 1 and 0x40 <= key[0] <= 0x7e:
                self.is_paste = self.escape == b'\x1b[200' and key == b'~'
                self.escape = b''
            elif len(self.escape) < 32:
                self.escape += key
            return None
        if self.escape:
            self.escape = b''
            return None
        return control_key_action(key)


def control_status_label(status, receive_age_sec):
    """ライブ制御状態の表示。サービス受付と実測停止の区別。"""
    if not isinstance(status, dict) or not is_fresh_age(receive_age_sec):
        return '制御状態: 未受信・失効'
    mode = status.get('mode')
    enable_hardware_output = status.get('enable_hardware_output')
    labels = {'hold': '保持', 'avoidance': '回避', 'leader': 'リーダーフォロワー',
              'switching': '切替中', 'stopped': '停止要求中・ラッチ'}
    if not isinstance(mode, str) or mode not in labels or not isinstance(enable_hardware_output, bool):
        return '制御状態: 診断形式不正'
    detail = str(status.get('detail', '')).replace('\n', ' ').replace('\r', ' ')[:160]
    return ('モード: ' + labels[mode] + ' / 実機出力: '
            + ('ON' if enable_hardware_output else 'OFF') + (' / ' + detail if detail else ''))


def toggle_value(action, status, receive_age_sec):
    """新鮮な制御状態に限定した、次のON/OFF値の決定。"""
    if not isinstance(status, dict) or not is_fresh_age(receive_age_sec):
        return None
    if (status.get('mode') not in ('hold', 'avoidance', 'leader', 'stopped')
            or not isinstance(status.get('enable_hardware_output'), bool)):
        return None
    if action in ('avoidance', 'leader'):
        return status['mode'] != action
    if action == 'hardware':
        return not status['enable_hardware_output']
    return None


class control_request:
    """非停止操作一件の期限付き待機。停止要求による待機の破棄。"""

    def __init__(self, emit, now=time.monotonic):
        self.emit, self.now = emit, now
        self.future = None
        self.client = None
        self.request = None
        self.is_pending = False
        self.deadline = 0.0
        self.label = ''

    def begin(self, client, request, label):
        if self.is_pending:
            self.emit('操作: 前の要求を処理中')
            return False
        self.client, self.request, self.label = client, request, label
        self.future = None
        self.is_pending = True
        self.deadline = self.now() + 2.0
        self.emit(label + ': 送信待ち')
        return True

    def cancel(self):
        if self.future is not None:
            self.future.cancel()
        if self.is_pending:
            self.emit(self.label + ': 応答待機の中断（停止優先）')
        self.future = None
        self.is_pending = False

    def poll(self):
        if not self.is_pending:
            return
        try:
            if self.future is not None and self.future.done():
                response = self.future.result()
                has_succeeded = response is not None and response.success is True
                detail = str(getattr(response, 'message', '')).replace('\n', ' ').replace('\r', ' ')[:160]
                self.emit(self.label + ': ' + ('受付済み' if has_succeeded else '拒否・応答不正')
                          + (' / ' + detail if detail else ''))
                self.future = None
                self.is_pending = False
            elif self.now() >= self.deadline:
                if self.future is not None:
                    self.future.cancel()
                self.future = None
                self.is_pending = False
                self.emit(self.label + ': 応答未確認')
            elif self.future is None and self.client.service_is_ready():
                self.future = self.client.call_async(self.request)
        except Exception as error:
            self.future = None
            self.is_pending = False
            self.emit(self.label + ': 失敗 / ' + str(error))


def main(argv=None):
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import Empty, String
    from std_srvs.srv import SetBool, Trigger

    ros_arguments = list(sys.argv if argv is None else ['dual_arm_control_keyboard.py', *argv])
    parser = argparse.ArgumentParser(description='統合操作: A 回避・保持 / L 追従・保持・停止解除 / Space 停止 / Ctrl-C 終了 / H UDP')
    parser.add_argument('--namespace', required=True, type=simulation_namespace)
    parser.add_argument('--tty-path', required=True, help='親launch端末のTTYパス')
    args = parser.parse_args(remove_ros_args(args=ros_arguments)[1:])
    descriptor = None
    try:
        descriptor = os.open(args.tty_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        if not os.isatty(descriptor):
            raise ValueError('指定先が対話端末ではありません')
        stream = os.fdopen(descriptor, 'r+b', buffering=0)
        descriptor = None
    except (OSError, ValueError) as error:
        if descriptor is not None:
            os.close(descriptor)
        parser.error('操作端末を開けません: ' + str(error))

    def emit(message):
        """launchの出力配管とは独立した、元端末への短い操作表示。"""
        try:
            os.write(stream.fileno(), ('\r\n' + message + '\r\n').encode('utf-8'))
        except (OSError, ValueError):
            pass

    node = None
    handlers = {}
    exit_state = {'is_requested': False}
    control = {'status': None, 'received_sec': -math.inf}
    safety = {'status': None, 'received_sec': -math.inf}
    is_initialized = False
    request = None

    def request_exit(_signum, _frame):
        exit_state['is_requested'] = True

    def receive_status(message, latest):
        try:
            latest['status'] = json.loads(message.data)
        except (ValueError, TypeError):
            latest['status'] = None
        latest['received_sec'] = time.monotonic()

    try:
        rclpy.init(args=ros_arguments, signal_handler_options=SignalHandlerOptions.NO)
        is_initialized = True
        node = rclpy.create_node('dual_arm_control_keyboard_' + str(os.getpid()))
        prefix = '/' + args.namespace
        clients = {name: node.create_client(SetBool, prefix + '/control/' + name)
                   for name in ('avoidance', 'leader', 'hardware')}
        stop_client = node.create_client(Trigger, prefix + '/control/stop')
        live_qos = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(String, prefix + '/control/status',
                                 lambda message: receive_status(message, control), live_qos)
        node.create_subscription(String, prefix + '/safety/status',
                                 lambda message: receive_status(message, safety), live_qos)
        heartbeat = node.create_publisher(Empty, prefix + '/control/heartbeat', 1)
        request = stop_request(stop_client, Trigger.Request, emit)
        operation = control_request(emit)
        for item in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[item] = signal.signal(item, request_exit)
        emit('対象: ' + prefix + ' / 操作: この端末を前面にしてキー入力（Enter不要）')
        emit('A: 回避・ホールド / L: 追従・ホールド / Space: ソフト停止')
        emit('開始はホールドからのみ。停止中のLは解除のみ、ホールド確認後にAまたはLで開始')
        emit('Ctrl-C: 停止要求後に終了 / H: UDP出力ON/OFF')
        emit('H: UDP設定・実測・開始姿勢の確認後のみ許可。表示する実測停止はGazeboのみ、実機停止は未確認')
        emit('ソフト停止は物理非常停止の代替ではありません')
        previous_labels = (None, None)
        next_heartbeat_sec = time.monotonic()
        is_exiting = False
        decoder = terminal_key_decoder()
        with terminal_input(stream) as terminal_descriptor:
            while rclpy.ok():
                action = None
                if exit_state['is_requested']:
                    action = 'quit'
                elif not is_exiting:
                    try:
                        if select.select([terminal_descriptor], [], [], 0)[0]:
                            action = decoder.read_action(os.read(terminal_descriptor, 1))
                    except BlockingIOError:
                        pass
                    except (OSError, ValueError):
                        action = 'quit'
                if action in ('stop', 'quit'):
                    operation.cancel()
                    if action == 'stop' or not is_exiting:
                        request.begin()
                    if action == 'quit':
                        is_exiting = True
                elif action is not None and not is_exiting:
                    if request.is_pending:
                        emit('操作: 停止要求を優先中')
                    else:
                        enable_mode = toggle_value(action, control['status'],
                                                   time.monotonic() - control['received_sec'])
                        if enable_mode is None:
                            emit('操作: 制御状態が未受信・失効のためON/OFF変更不可。停止キーは有効')
                        else:
                            label = {'avoidance': '回避', 'leader': 'リーダーフォロワー', 'hardware': '実機出力'}[action]
                            label = ('停止解除→ホールド' if action == 'leader' and control['status']['mode'] == 'stopped'
                                     else label + ('ON' if enable_mode else 'OFF'))
                            operation.begin(clients[action], SetBool.Request(data=enable_mode),
                                            label)
                request.poll()
                operation.poll()
                now_sec = time.monotonic()
                if now_sec >= next_heartbeat_sec:
                    heartbeat.publish(Empty())
                    next_heartbeat_sec = now_sec + 0.1
                rclpy.spin_once(node, timeout_sec=0.02)
                labels = (control_status_label(control['status'], time.monotonic() - control['received_sec']),
                          'Gazebo / ' + status_label(safety['status'], time.monotonic() - safety['received_sec']))
                for label, previous_label in zip(labels, previous_labels):
                    if label != previous_label:
                        emit(label)
                previous_labels = labels
                if is_exiting and not request.is_pending:
                    emit('入力監視: 終了（要求受付と実測停止は別）。launch全体の終了待ち')
                    return 0 if request.has_succeeded else 2
        return 2
    except (OSError, ValueError, RuntimeError) as error:
        emit('入力監視: 異常終了 / ' + str(error))
        # 端末切断・入力例外時の期限付き停止要求。通信断時の停止保証なし
        if request is not None and rclpy.ok():
            request.begin()
            try:
                while request.is_pending and rclpy.ok():
                    request.poll()
                    rclpy.spin_once(node, timeout_sec=0.02)
            except Exception:
                pass
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if is_initialized and rclpy.ok():
            rclpy.shutdown()
        for item, handler in handlers.items():
            signal.signal(item, handler)
        stream.close()


if __name__ == '__main__':
    raise SystemExit(main())
