#!/usr/bin/env python3
"""専用ターミナルからのGazebo停止要求。解除・再開機能なし。"""

import argparse
from contextlib import contextmanager
import json
import math
import os
import re
import select
import signal
import sys
import termios
import time


def simulation_namespace(value):
    """実機名前空間への誤送信を防ぐsim_形式の検査。"""
    if not re.fullmatch(r'sim_[A-Za-z][A-Za-z0-9_]*', value):
        raise argparse.ArgumentTypeError('sim_で始まる単一のGazebo名前空間が必要です')
    return value


def key_action(key):
    """停止キーと、停止要求付き終了キーの分類。"""
    if key in (b' ', b's', b'S'):
        return 'stop'
    if key in (b'q', b'Q', b'\x03', b'\x04', b''):
        return 'quit'
    return None


@contextmanager
def terminal_input(stream):
    """Enter不要の入力と、正常終了・例外時の端末設定復元。"""
    if not stream.isatty():
        raise ValueError('対話端末が必要です。Dockerでは docker exec -it を使用してください')
    descriptor = stream.fileno()
    previous = termios.tcgetattr(descriptor)
    current = termios.tcgetattr(descriptor)
    current[0] &= ~(termios.IXON | termios.ICRNL)
    current[3] &= ~(termios.ICANON | termios.ECHO | termios.IEXTEN | termios.ISIG)
    current[6][termios.VMIN], current[6][termios.VTIME] = 1, 0
    try:
        termios.tcsetattr(descriptor, termios.TCSANOW, current)
        yield descriptor
    finally:
        termios.tcsetattr(descriptor, termios.TCSANOW, previous)


def is_fresh_age(value):
    """実測診断の経過時間検査。"""
    try:
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value) and 0 <= value <= 0.5)
    except OverflowError:
        return False


def status_label(status, receive_age_sec):
    """サービス受付とは独立した実測停止の表示判定。"""
    if not isinstance(status, dict) or not is_fresh_age(receive_age_sec):
        return '停止状態: 未受信・失効'
    if (status.get('state') == 'stopped' and status.get('is_stop_latched') is True
            and status.get('is_stop_applied') is True and status.get('is_stopped') is True
            and is_fresh_age(status.get('state_age_sec'))):
        return '実測停止: 確認済み'
    if status.get('is_stop_latched') is True:
        return '停止ラッチ: ON / 実測停止: 未確認'
    if status.get('is_stop_latched') is False:
        return '停止ラッチ: OFF'
    return '停止状態: 診断形式不正'


class stop_request:
    """期限付きの非同期停止要求。連打中の要求重複と無期限待機の防止。"""

    def __init__(self, client, request_factory, emit, now=time.monotonic):
        self.client, self.request_factory, self.emit, self.now = client, request_factory, emit, now
        self.future = None
        self.is_pending = False
        self.has_succeeded = False
        self.deadline = 0.0

    def begin(self):
        if self.is_pending:
            return
        self.future = None
        self.is_pending, self.has_succeeded = True, False
        self.deadline = self.now() + 2.0
        self.emit('停止要求: 送信待ち')

    def poll(self):
        if not self.is_pending:
            return
        try:
            if self.future is not None and self.future.done():
                response = self.future.result()
                self.has_succeeded = response is not None and response.success is True
                self.is_pending = False
                self.emit('停止要求: 受付済み（停止完了とは別）' if self.has_succeeded else '停止要求: 拒否・応答不正')
            elif self.now() >= self.deadline:
                if self.future is not None:
                    self.future.cancel()
                self.is_pending = False
                self.emit('停止要求: 応答未確認。停止したとは判断できません')
            elif self.future is None and self.client.service_is_ready():
                self.future = self.client.call_async(self.request_factory())
        except Exception as error:
            self.is_pending, self.has_succeeded = False, False
            self.emit('停止要求: 失敗 / ' + str(error))


def emit(message):
    """端末切断時にも停止要求を妨げない表示。"""
    try:
        print(message, flush=True)
    except OSError:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description='Gazebo専用キーボード停止。Space/S: 停止、Q/Ctrl-C: 停止要求後に終了')
    parser.add_argument('--namespace', required=True, type=simulation_namespace)
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        parser.error('対話端末が必要です。Dockerでは docker exec -it を使用してください')

    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    handlers = {}
    exit_state = {'is_requested': False}
    latest = {'status': None, 'received_sec': -math.inf}

    def request_exit(_signum, _frame):
        exit_state['is_requested'] = True

    def on_status(message):
        try:
            latest['status'] = json.loads(message.data)
        except (ValueError, TypeError):
            latest['status'] = None
        latest['received_sec'] = time.monotonic()

    try:
        node = rclpy.create_node('gazebo_stop_keyboard_' + str(os.getpid()))
        client = node.create_client(Trigger, '/' + args.namespace + '/safety/stop')
        # 過去のtransient_local診断を起動直後の実測として扱わない、ライブ更新のみの購読
        node.create_subscription(String, '/' + args.namespace + '/safety/status', on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        request = stop_request(client, Trigger.Request, emit)
        for item in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            handlers[item] = signal.signal(item, request_exit)
        emit('対象: /' + args.namespace)
        emit('この端末を前面にして Space / S: 停止（Enter不要）。Q / Ctrl-C: 停止要求後に終了')
        emit('解除・再開操作なし。PC停止・通信断に対応する物理非常停止の代替ではありません')
        previous_label = None
        is_exiting = False
        with terminal_input(sys.stdin) as descriptor:
            while rclpy.ok():
                action = None
                if exit_state['is_requested']:
                    action = 'quit'
                elif not is_exiting:
                    try:
                        if select.select([descriptor], [], [], 0)[0]:
                            action = key_action(os.read(descriptor, 1))
                    except OSError:
                        action = 'quit'
                if action == 'stop':
                    request.begin()
                if not is_exiting and (action == 'quit' or exit_state['is_requested']):
                    request.begin()
                    is_exiting = True
                request.poll()
                rclpy.spin_once(node, timeout_sec=0.02)
                label = status_label(latest['status'], time.monotonic() - latest['received_sec'])
                if label != previous_label:
                    emit(label)
                    previous_label = label
                if is_exiting and not request.is_pending:
                    emit('入力監視: 終了（要求受付のみ。実測停止の保証なし）')
                    return 0 if request.has_succeeded else 2
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for item, handler in handlers.items():
            signal.signal(item, handler)


if __name__ == '__main__':
    raise SystemExit(main())
