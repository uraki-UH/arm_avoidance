#!/usr/bin/env python3
"""単一PTYのGazebo操作とlocalhost UDP往復・遮断・後始末の有限試験。"""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import socket
import time
import xml.etree.ElementTree as element_tree

from check_dual_arm_control import control_trial, run


class udp_trial(control_trial):
    """既存の所有プロセス管理を利用した模擬受信機。外部IPへの送信なし。"""

    def __init__(self, args, report, launch, begin, cancel_state, joint_names):
        super().__init__(args, report, launch, begin, cancel_state, joint_names)
        from control_msgs.msg import JointTrajectoryControllerState
        from rclpy.qos import qos_profile_sensor_data
        self.udp_socket = None
        self.feedback_address = None
        self.independent_names = []
        self.feedback_values = None
        self.enable_feedback = False
        self.is_receiver_enabled = False
        self.next_feedback_sec = 0.0
        self.last_feedback_sec = None
        self.num_commands = self.num_enables = self.num_stops = self.num_feedback = 0
        self.num_matched_commands = 0
        self.num_commands_at_stop = None
        self.last_stop_sec = None
        self.first_command = self.last_command = None
        self.command_history = deque(maxlen=256)
        self.desired_history = deque(maxlen=256)
        self.pending_commands = deque()
        self.packet_log = (args.output / 'udp_packets.jsonl').open('w', encoding='utf-8')
        self.desired_log = (args.output / 'controller_desired.jsonl').open('w', encoding='utf-8')
        self.node.create_subscription(JointTrajectoryControllerState,
            self.namespace + '/dual_arm_controller/state', self.on_controller_state, qos_profile_sensor_data)

    def prepare_command(self, command):
        """loopback限定の2ポート予約と、試験固有の関節対応設定。"""
        import yaml
        params_path = Path(next(item.split(':=', 1)[1] for item in command
                               if item.startswith('params_file:=')))
        params = yaml.safe_load(params_path.read_text())['/**']['ros__parameters']
        root = element_tree.parse(params['urdf_path']).getroot()
        self.independent_names = [item.get('name') for item in root.findall('joint')
                                  if item.get('type') != 'fixed' and item.find('mimic') is None]
        if len(self.independent_names) != 19 or len(set(self.independent_names)) != 19:
            raise ValueError('試験対象の独立関節数・重複の不整合')
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('127.0.0.1', 0))
        self.udp_socket.setblocking(False)
        robot_port = self.udp_socket.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            listen_port = reservation.getsockname()[1]
        self.feedback_address = ('127.0.0.1', listen_port)
        config = {'robot_ip': '127.0.0.1', 'robot_port': robot_port,
                  'listen_ip': '127.0.0.1', 'listen_port': listen_port,
                  'feedback_source_port': robot_port, 'joint_names': self.independent_names,
                  'joint_scales': [1.0] * 19, 'joint_offsets_deg': [0.0] * 19,
                  'enable_packet': 'ENABLE\n', 'stop_packet': 'STOP\n',
                  'max_command_age_sec': 0.5, 'max_feedback_age_sec': 0.5,
                  'update_hz': 50.0, 'max_joint_velocity': 0.3,
                  'max_start_dev_rad': 0.05, 'max_tracking_dev_rad': 0.2}
        config_path = self.args.output / 'udp_config.yaml'
        config_path.write_text(yaml.safe_dump({'dual_arm_udp': config}, allow_unicode=True),
                               encoding='utf-8')
        self.report['udp'] = {'robot_ip': '127.0.0.1', 'robot_port': robot_port,
                              'listen_port': listen_port, 'joint_names': self.independent_names,
                              'feedback_source_port': robot_port,
                              'physical_hardware': 'not_connected',
                              'conversion': 'round(position_rad * 180 / pi * 10)'}
        return command + ['udp_config:=' + str(config_path.resolve()), 'allow_remote_udp:=false']

    @staticmethod
    def to_units(positions):
        """bridge実装に依存しない0.1度単位の最近接丸め変換。"""
        return [int(round(value * 180.0 / math.pi * 10.0)) for value in positions]

    def packet_event(self, direction, packet, **values):
        self.packet_log.write(json.dumps({'direction': direction, 'stage': self.stage,
            'wall_sec': time.monotonic() - self.begin, 'packet': packet.decode('ascii'),
            **values}, allow_nan=False, ensure_ascii=False) + '\n')
        self.packet_log.flush()

    def on_controller_state(self, message):
        now = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.desired_log.write(json.dumps({'wall_sec': now - self.begin, 'sim_sec': stamp,
            'stage': self.stage, 'joint_names': list(message.joint_names),
            'desired_positions': list(message.desired.positions)}, allow_nan=False) + '\n')
        self.desired_log.flush()
        if not self.independent_names:
            return
        positions = message.desired.positions
        if len(message.joint_names) != len(positions) or len(set(message.joint_names)) != len(positions):
            self.callback_error = 'controller desiredの関節名・角度配列不整合'
            return
        values = dict(zip(message.joint_names, positions))
        if any(name not in values or not math.isfinite(values[name]) for name in self.independent_names):
            self.callback_error = 'controller desiredの独立関節不足・非有限値'
            return
        ordered = [values[name] for name in self.independent_names]
        self.desired_history.append((now, self.to_units(ordered), stamp, ordered))

    def verify_commands(self):
        """UDPとROS購読の配送順の差を考慮した、受信CSV全件の独立照合。"""
        now = time.monotonic()
        while self.pending_commands:
            received_sec, values = self.pending_commands[0]
            match = next((item for item in self.desired_history
                          if abs(item[0] - received_sec) <= 0.6 and item[1] == values), None)
            if match is not None:
                self.num_matched_commands += 1
                self.pending_commands.popleft()
            elif now - received_sec <= 0.6:
                break
            else:
                self.report['udp']['unmatched_command'] = values
                self.report['udp']['recent_desired'] = [item[1:] for item in self.desired_history
                                                       if abs(item[0] - received_sec) <= 0.6]
                raise AssertionError('UDP指令とcontroller desiredの順序・0.1度変換の不一致')

    def pump_udp(self):
        if self.udp_socket is None:
            return
        while True:
            try:
                packet, address = self.udp_socket.recvfrom(65536)
            except BlockingIOError:
                break
            if address[0] != '127.0.0.1':
                raise AssertionError('loopback以外からの試験UDP受信')
            self.packet_event('received', packet, source_port=address[1])
            value = packet.decode('ascii').strip()
            if value == 'ENABLE':
                self.num_enables += 1
                self.is_receiver_enabled = True
            elif value == 'STOP':
                self.num_stops += 1
                self.is_receiver_enabled = False
                self.num_commands_at_stop = self.num_commands
                self.last_stop_sec = time.monotonic()
            else:
                if not self.is_receiver_enabled:
                    raise AssertionError('ENABLE前またはSTOP後のUDP角度指令')
                fields = value.rstrip(',').split(',')
                if len(fields) != len(self.independent_names):
                    raise AssertionError('UDP指令の関節数不一致: ' + value)
                values = [int(item) for item in fields]
                if any(str(number) != item for number, item in zip(values, fields)):
                    raise AssertionError('UDP指令の整数CSV形式不一致')
                self.num_commands += 1
                self.first_command = self.first_command or list(values)
                self.last_command = list(values)
                self.command_history.append((time.monotonic(), list(values)))
                self.feedback_values = list(values)
                self.pending_commands.append((time.monotonic(), list(values)))
        now = time.monotonic()
        if self.enable_feedback and now >= self.next_feedback_sec and self.is_fresh():
            if self.feedback_values is None:
                self.feedback_values = self.to_units([self.positions[name] for name in self.independent_names])
            packet = ('agl,' + ','.join(str(value) for value in self.feedback_values) + '\n').encode('ascii')
            self.udp_socket.sendto(packet, self.feedback_address)
            self.packet_event('feedback', packet, destination_port=self.feedback_address[1])
            self.num_feedback += 1
            self.last_feedback_sec = now
            self.next_feedback_sec = now + 0.02
        self.verify_commands()

    def spin(self, allow_launch_exit=False):
        self.pump_udp()
        super().spin(allow_launch_exit=allow_launch_exit)
        self.pump_udp()

    def wait_duration(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        self.wait(lambda: time.monotonic() >= deadline, duration_sec + 2.0)

    def check_output_off(self, num_commands, num_enables=None):
        if self.control.get('enable_hardware_output') is not False or self.num_commands != num_commands:
            raise AssertionError('UDP出力OFF中のCSV送信またはON状態')
        if num_enables is not None and self.num_enables != num_enables:
            raise AssertionError('UDP出力OFF中のENABLE送信')

    def enable_output(self):
        num_enables, num_commands = self.num_enables, self.num_commands
        self.key(b'h')
        self.wait(lambda: self.control.get('enable_hardware_output') is True
                  and self.num_enables == num_enables + 1 and self.num_commands >= num_commands + 2, 8)
        if self.control.get('udp', {}).get('is_physical_stop_confirmed') is not False:
            raise AssertionError('模擬受信だけによる実機停止確認の誤表示')

    def execute(self):
        self.set_stage('startup')
        self.wait(lambda: self.is_hold_ready() and 'Enter不要' in self.launch.transcript
                  and self.demo.get('state') == 'idle' and isinstance(self.control.get('udp'), dict)
                  and self.control['udp'].get('has_fresh_target') is True
                  and self.control['udp'].get('has_fresh_feedback') is False, 90)
        topic = self.namespace + '/dual_arm_controller/joint_trajectory'
        if self.node.count_publishers(topic) != 1:
            raise AssertionError('最終軌道publisherが単一ではありません')
        if any(self.node.count_publishers(item) for item in
               ('/dynamixel/command/goal', '/dynamixel/shortcut', '/leader/dynamixel/command/goal')):
            raise AssertionError('実機Dynamixel指令publisherの存在')
        if self.control['udp'].get('is_remote') is not False:
            raise AssertionError('localhost限定モードの未確認')
        self.wait_duration(0.6)
        self.check_output_off(0, 0)
        self.report['checks']['startup_no_command_or_enable_packet'] = True
        self.report['initial_positions'] = dict(self.positions)

        self.set_stage('hardware_without_feedback')
        transcript_start = len(self.launch.transcript)
        self.key(b'h')
        self.wait(lambda: '実機出力ON: 拒否・応答不正' in self.launch.transcript[transcript_start:], 5)
        self.wait_duration(0.2)
        self.check_output_off(0, 0)
        self.report['checks']['hardware_without_feedback_rejected'] = dict(self.control['udp'])

        self.set_stage('hardware_with_feedback')
        self.enable_feedback = True
        self.wait(lambda: self.control.get('udp', {}).get('has_fresh_feedback') is True
                  and self.control.get('udp', {}).get('has_fresh_target') is True, 5)
        self.enable_output()
        self.report['checks']['explicit_hardware_on'] = {'num_enable_packets': self.num_enables,
                                                       'num_command_packets': self.num_commands}

        self.set_stage('leader_to_udp')
        initial = self.positions['L_joint2']
        initial_units = self.to_units([initial])[0]
        joint_idx = self.independent_names.index('L_joint2')
        self.leader_target = initial + 0.06
        self.wait(lambda: self.control.get('has_fresh_leader') is True)
        self.key(b'l')
        self.wait(lambda: self.control.get('mode') == 'leader'
                  and self.positions['L_joint2'] - initial > 0.005
                  and abs(self.velocities['L_joint2']) > 0.01
                  and self.last_command[joint_idx] - initial_units >= 3, 20)
        if self.control.get('enable_hardware_output') is not True:
            raise AssertionError('リーダー追従中のUDP出力喪失')
        self.report['checks']['leader_gazebo_and_udp_movement'] = {
            'joint': 'L_joint2', 'udp_joint_idx': joint_idx, 'target_delta_rad': 0.06,
            'observed_delta_rad': self.positions['L_joint2'] - initial,
            'udp_delta_units': self.last_command[joint_idx] - initial_units,
            'unit_deg': 0.1, 'num_matched_commands': self.num_matched_commands}

        self.set_stage('space_stop')
        num_stops = self.num_stops
        stop_stamp = self.joint_stamp
        self.low_since_sim_sec = None
        self.key(b' ')
        self.wait(lambda: self.is_stopped() and self.joint_stamp - stop_stamp >= 0.25
                  and self.num_stops > num_stops
                  and self.control.get('enable_hardware_output') is False, 30)
        self.wait_duration(0.7)
        self.check_output_off(self.num_commands_at_stop)
        self.report['checks']['space_stop_packet_and_csv_cutoff'] = {
            'num_stop_packets': self.num_stops, 'num_commands_after_stop': 0,
            'safety': dict(self.safety)}

        num_commands, num_enables = self.num_commands, self.num_enables
        self.check_leader_resume()
        self.wait_duration(0.7)
        self.check_output_off(num_commands, num_enables)
        self.report['checks']['leader_resume_no_udp_packets'] = dict(self.control)

        self.set_stage('feedback_timeout')
        self.enable_output()
        num_stops = self.num_stops
        self.enable_feedback = False
        last_feedback_sec = self.last_feedback_sec
        self.event('feedback_disabled')
        self.wait(lambda: self.num_stops > num_stops and self.is_stopped()
                  and self.control.get('enable_hardware_output') is False, 30)
        stop_delay_sec = self.last_stop_sec - last_feedback_sec
        if stop_delay_sec > 1.5:
            raise AssertionError('UDP実測途絶からSTOP受信までの遅延超過')
        self.wait_duration(0.7)
        self.check_output_off(self.num_commands_at_stop)
        self.report['checks']['feedback_timeout_stops_udp_and_gazebo'] = {
            'stop_packet_delay_sec': stop_delay_sec, 'num_commands_after_stop': 0,
            'safety': dict(self.safety), 'udp': dict(self.control['udp'])}

        self.set_stage('conversion_all_packets')
        self.wait(lambda: not self.pending_commands, 2)
        if self.num_commands == 0 or self.num_matched_commands != self.num_commands:
            raise AssertionError('UDP全指令のcontroller desired照合不足')
        self.report['checks']['ordered_radians_to_deci_degrees'] = {
            'num_commands': self.num_commands, 'num_matched_commands': self.num_matched_commands,
            'first_command': self.first_command, 'last_command': self.last_command}

        self.set_stage('ctrl_c_exit')
        self.key(b'\x03')
        self.wait(lambda: self.launch.process.poll() is not None, 20, allow_launch_exit=True)
        self.wait(lambda: not self.launch.track(), 12, allow_launch_exit=True)
        if self.launch.process.returncode != 0:
            raise AssertionError('Ctrl+C終了後のlaunch終了コード: ' + str(self.launch.process.returncode))
        if not self.launch.is_terminal_restored():
            raise AssertionError('Ctrl+C終了後のtermios復元不一致')
        self.report['checks']['ctrl_c_and_terminal_restored'] = True
        self.set_stage('completed')

    def close(self):
        self.report.setdefault('udp', {}).update(num_commands=self.num_commands,
            num_matched_commands=self.num_matched_commands, num_feedback=self.num_feedback,
            num_enable_packets=self.num_enables, num_stop_packets=self.num_stops)
        if self.udp_socket is not None:
            self.udp_socket.close()
        self.packet_log.close()
        self.desired_log.close()
        super().close()


def main():
    parser = argparse.ArgumentParser(description='Gazebo統合操作とlocalhost UDP模擬受信機の有限試験')
    parser.add_argument('--robot', choices=('max', 'max_long'), default='max')
    parser.add_argument('--params-file', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-sec', type=float, default=180)
    args = parser.parse_args()
    args.output = args.output.resolve()
    return run(args, trial_factory=udp_trial)


if __name__ == '__main__':
    raise SystemExit(main())
