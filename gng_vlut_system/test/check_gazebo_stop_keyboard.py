#!/usr/bin/env python3
"""隔離ROS・模擬停止サービス・疑似端末によるキー入力の有限試験。Gazebo駆動なし。"""

import argparse
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import termios
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '97' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('ROS_DOMAIN_ID=97 / ROS_LOCALHOST_ONLY=1 が必要')
    if args.output.exists():
        parser.error('既存結果への上書き禁止')
    args.output.mkdir(parents=True)
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('keyboard_stop_test_server')
    report = {'kind': 'mock_service_pty', 'cases': [], 'processes': [], 'command': sys.argv,
              'result': 'failed'}
    services, publishers = [], {}
    state = {'num_calls': 0, 'allow_stop': True, 'status': 'running'}
    begin = time.monotonic()

    def on_stop(_request, response):
        state['num_calls'] += 1
        response.success = state['allow_stop']
        response.message = '模擬停止要求'
        return response

    def publish_status():
        value = {'state': state['status'], 'is_stop_latched': state['status'] != 'running',
                 'is_stop_applied': state['status'] == 'stopped',
                 'is_stopped': state['status'] == 'stopped', 'state_age_sec': 0.0}
        for publisher in publishers.values():
            publisher.publish(String(data=json.dumps(value)))

    def spin_until(predicate, pump=lambda: None, max_sec=10):
        deadline = time.monotonic() + max_sec
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            pump()
            if predicate():
                return
            if time.monotonic() > deadline or time.monotonic() - begin > 75:
                raise TimeoutError('キー入力試験の待機上限')

    def check_case(name, namespace, action, *, expected_calls, expected_code):
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        output = bytearray()
        command = [sys.executable, '-B', str(Path(__file__).resolve().parents[1] / 'scripts/gazebo_stop_keyboard.py'),
                   '--namespace', namespace]
        process = subprocess.Popen(command, stdin=slave, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        record = {'pid': process.pid, 'command': command}
        report['processes'].append(record)
        num_before = state['num_calls']

        def pump():
            while select.select([process.stdout], [], [], 0)[0]:
                data = os.read(process.stdout.fileno(), 65536)
                if not data:
                    break
                output.extend(data)

        def contains(value):
            return value.encode() in output

        try:
            spin_until(lambda: contains('停止状態:') or contains('停止ラッチ:'), pump)
            if action == 'space_s_q':
                os.write(master, b' ')
                spin_until(lambda: state['num_calls'] == num_before + 1 and contains('受付済み'), pump)
                assert not contains('実測停止: 確認済み'), '受付だけで実測停止の誤表示'
                state['status'] = 'stopped'
                spin_until(lambda: contains('実測停止: 確認済み'), pump)
                os.write(master, b'S')
                spin_until(lambda: state['num_calls'] == num_before + 2, pump)
                spin_until(lambda: output.count('受付済み'.encode()) == 2, pump)
                os.write(master, b'q')
            elif action == 'signal':
                process.send_signal(signal.SIGTERM)
            else:
                os.write(master, action)
            spin_until(lambda: process.poll() is not None, pump)
            pump()
            assert process.returncode == expected_code, output.decode(errors='replace')
            assert state['num_calls'] - num_before == expected_calls
            assert termios.tcgetattr(slave) == original, '端末設定の復元漏れ'
            report['cases'].append({'name': name, 'is_success': True, 'num_calls': expected_calls})
        finally:
            # この試験が起動した単体Pythonだけを対象とする段階終了
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
            record['returncode'] = process.returncode
            pump()
            process.stdout.close()
            (args.output / (name + '.log')).write_bytes(output)
            os.close(master)
            os.close(slave)

    try:
        settle = time.monotonic() + 2
        spin_until(lambda: time.monotonic() >= settle)
        report['baseline_nodes'] = [name for name in node.get_node_names() if name != node.get_name()]
        if report['baseline_nodes']:
            raise RuntimeError('専用domain97に既存ノードあり。試験中止')
        namespaces = ('sim_topo_dual_arm_max', 'sim_topo_dual_arm_max_long')
        for namespace in namespaces:
            services.append(node.create_service(Trigger, '/' + namespace + '/safety/stop', on_stop))
            publishers[namespace] = node.create_publisher(String, '/' + namespace + '/safety/status', 1)
        timer = node.create_timer(0.05, publish_status)
        check_case('max_space_s_q', namespaces[0], 'space_s_q', expected_calls=3, expected_code=0)
        state['status'] = 'running'
        check_case('long_ctrl_c', namespaces[1], b'\x03', expected_calls=1, expected_code=0)
        check_case('max_sigterm', namespaces[0], 'signal', expected_calls=1, expected_code=0)
        state['allow_stop'] = False
        check_case('max_rejected', namespaces[0], b'q', expected_calls=1, expected_code=2)
        node.destroy_service(services.pop())
        check_case('long_unavailable', namespaces[1], b'q', expected_calls=0, expected_code=2)
        settle = time.monotonic() + 2
        spin_until(lambda: time.monotonic() >= settle)
        report['remaining_client_nodes'] = [name for name in node.get_node_names() if name != node.get_name()]
        if report['remaining_client_nodes']:
            raise RuntimeError('試験クライアントの終了未確認')
        report['result'] = 'passed'
    except Exception as error:
        report['error'] = repr(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        report['wall_sec'] = time.monotonic() - begin
        (args.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False), flush=True)
    return int(report['result'] != 'passed')


if __name__ == '__main__':
    raise SystemExit(main())
