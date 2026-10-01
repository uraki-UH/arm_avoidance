"""隔離ROSの模擬ドライバによる首トルクlaunch・終了指令の結合確認。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from dynamixel_handler_msgs.msg import DynamixelExtra, DynamixelGoal, DynamixelStatus
from sensor_msgs.msg import JointState
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--case', choices=('sigint', 'sigterm', 'parent_kill', 'stale', 'unconfigured', 'disallowed', 'wrong_mode', 'no_off_report'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config-file', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '217':
        raise RuntimeError('実機から隔離したROS_DOMAIN_ID=217が必要です')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros')
    config = {'allow_hardware_output': True, 'max_current_ma': [20., 30.],
              'damping_gain': [10., 20.]}
    if args.config_file:
        config = yaml.safe_load(args.config_file.read_text())['/**']['ros__parameters']
        if config.get('driver_namespace', '/dynamixel') != '/dynamixel':
            raise ValueError('模擬ドライバの名前空間は/dynamixel固定です')
    if args.case == 'unconfigured':
        config['max_current_ma'] = [0., 0.]
    if args.case == 'disallowed':
        config['allow_hardware_output'] = False
    config_path = args.output / 'mock.yaml'
    config_path.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}}))
    command = ['ros2', 'launch', 'gng_vlut_system', 'dynamixel_neck_torque.launch.py',
               'config_file:=' + str(config_path)]
    rclpy.init()
    node = rclpy.create_node('neck_torque_mock_driver')
    events = []
    current = [3000., 3000.]
    torque = [False, False]
    mode = 'cur_position' if args.case == 'wrong_mode' else 'current'
    status_pub = node.create_publisher(DynamixelStatus, '/dynamixel/state/status', 1)
    goal_pub = node.create_publisher(DynamixelGoal, '/dynamixel/state/goal', 1)
    extra_pub = node.create_publisher(DynamixelExtra, '/dynamixel/state/extra', 1)
    joint_pub = node.create_publisher(JointState, '/dynamixel/fresh_joint_states', 1)

    def on_goal(message):
        assert list(message.id_list) == [51, 52]
        assert not message.position_deg and not message.velocity_deg_s and not message.pwm_percent
        current[:] = message.current_ma
        assert all(abs(value) <= limit for value, limit in zip(current, config['max_current_ma']))
        events.append(('current', list(current)))

    def on_status(message):
        assert list(message.id_list) == [51, 52]
        assert not message.mode and not message.error and not message.ping
        if any(message.torque):
            assert current == [0., 0.]
        torque[:] = message.torque
        events.append(('torque', list(torque)))

    node.create_subscription(DynamixelGoal, '/dynamixel/command/goal', on_goal, 1)
    node.create_subscription(DynamixelStatus, '/dynamixel/command/status', on_status, 1)
    process = None
    is_triggered = False
    started = time.monotonic()
    log_path = args.output / 'launch.log'
    try:
        with log_path.open('w') as log:
            print('起動: ' + ' '.join(command), flush=True)
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while time.monotonic() - started < 20.:
                if not (is_triggered and args.case == 'stale'):
                    reported_torque = [True, True] if is_triggered and args.case == 'no_off_report' else torque
                    status_pub.publish(DynamixelStatus(id_list=[51, 52], torque=reported_torque,
                        error=[False, False], ping=[True, True], mode=[mode, mode]))
                    goal_pub.publish(DynamixelGoal(id_list=[51, 52], current_ma=current))
                    extra = DynamixelExtra(id_list=[51, 52], model_number=[1020, 1020])
                    extra.drive_mode.torque_on_by_goal_update = [False, False]
                    extra.drive_mode.reverse_mode = [False, False]
                    extra_pub.publish(extra)
                    joints = JointState(name=['51', '52'], position=[.3, -.2],
                                        velocity=[1., -1.5] if all(torque) else [0., 0.])
                    joints.header.frame_id = 'dynamixel_motor'
                    joints.header.stamp = node.get_clock().now().to_msg()
                    joint_pub.publish(joints)
                rclpy.spin_once(node, timeout_sec=.01)
                if not is_triggered and current[0] < 0 < current[1]:
                    is_triggered = True
                    if args.case != 'stale':
                        process.send_signal(signal.SIGKILL if args.case == 'parent_kill' else
                                            signal.SIGTERM if args.case == 'sigterm' else signal.SIGINT)
                if process.poll() is not None and node.count_publishers('/dynamixel/command/goal') == 0:
                    # 終了直前のDDS指令の取り込み
                    until = time.monotonic() + .2
                    while time.monotonic() < until:
                        rclpy.spin_once(node, timeout_sec=.02)
                    break
            assert process.poll() is not None and node.count_publishers('/dynamixel/command/goal') == 0, '所有ノードの終了待機時間超過'
    finally:
        if process is not None:
            # launchの先行終了時も所有プロセスグループ内の子だけを回収
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        print('所有launch・模擬ドライバ: 停止済み', flush=True)

    # SIGTERMでlaunchの転送が先に閉じた場合もノード自身のログを照合
    log = log_path.read_text() + ''.join(path.read_text() for path in (args.output / 'ros').glob('python3_*.log'))
    if args.case in ('wrong_mode', 'unconfigured', 'disallowed'):
        assert not events, events
        assert '首トルク停止:' in log
    else:
        assert is_triggered and torque == [False, False] and current == [0., 0.], events
        assert len(events) < 100, '終了時指令の過剰再送'
        first_off = next(idx for idx, event in enumerate(events) if event == ('torque', [False, False]))
        assert all(event[1] == ([False, False] if event[0] == 'torque' else [0., 0.]) for event in events[first_off:])
        assert ('OFF未確認' if args.case in ('stale', 'no_off_report') else 'OFF報告あり') in log, log
    print(json.dumps({'case': args.case, 'num_commands': len(events), 'is_triggered': is_triggered,
                      'last_current_ma': current, 'last_torque': torque, 'result': 'pass'}), flush=True)


if __name__ == '__main__':
    main()
