#!/usr/bin/env python3
"""同一PTYの単一launchによるGazebo統合操作・実測・後始末の有限試験。"""

import argparse
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import termios
import time
import traceback
import xml.etree.ElementTree as element_tree

from check_gazebo_software_stop import owned_launch, port_is_listening, process_snapshot, save_json


class pty_launch(owned_launch):
    """親launchとkeyboardに共通の試験専用端末。既存端末への変更なし。"""

    def __init__(self, output, baseline):
        super().__init__(output, baseline)
        self.master = self.slave = None
        self.original_terminal = None
        self.terminal_log = None
        self.transcript = ''

    def start(self, command):
        self.master, self.slave = pty.openpty()
        os.set_blocking(self.master, False)
        self.original_terminal = termios.tcgetattr(self.slave)
        self.terminal_log = (self.output / 'keyboard.log').open('wb')
        child_env = dict(os.environ, ROS2CLI_NO_DAEMON='1', ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='96',
                         GAZEBO_MASTER_URI='http://127.0.0.1:11369', GNG_STOP_TEST_OWNER=self.marker)
        self.log = (self.output / 'gazebo.log').open('w', encoding='utf-8')
        # 所有cleanupと外側runner双方の対象となるPGID継承。stdinだけがPTY。
        self.process = subprocess.Popen(command, stdin=self.slave, stdout=self.log,
                                        stderr=subprocess.STDOUT, env=child_env, start_new_session=False)
        save_json(self.output / 'command.json', command)
        save_json(self.output / 'terminal.json', {'slave_path': os.ttyname(self.slave),
                  'initial_flags': self.original_terminal[:6]})
        self.track()

    def read_terminal(self):
        if self.master is None:
            return
        while select.select([self.master], [], [], 0)[0]:
            try:
                data = os.read(self.master, 65536)
            except BlockingIOError:
                break
            if not data:
                break
            self.terminal_log.write(data)
            self.terminal_log.flush()
            self.transcript += data.decode('utf-8', errors='replace')

    def send_key(self, key):
        if self.master is None or self.process is None or self.process.poll() is not None:
            raise RuntimeError('操作端末またはlaunchが終了済み')
        os.write(self.master, key)

    def is_terminal_restored(self):
        return self.slave is not None and termios.tcgetattr(self.slave) == self.original_terminal

    def cleanup(self):
        # 端末所有keyboard経由の正常終了を優先。SIGINT/TERMによるDDS後始末中断の抑制
        if self.process is not None and self.process.poll() is None and self.master is not None:
            try:
                self.send_key(b'\x03')
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline and self.process.poll() is None:
                    self.read_terminal()
                    time.sleep(0.05)
            except OSError:
                pass
        return super().cleanup()

    def close_terminal(self):
        self.read_terminal()
        if self.terminal_log is not None:
            self.terminal_log.close()
        for descriptor in (self.master, self.slave):
            if descriptor is not None:
                os.close(descriptor)
        self.master = self.slave = None


class control_trial:
    """キー操作の応答と、Gazebo全関節の実測による独立判定。"""

    def __init__(self, args, report, launch, begin, cancel_state, joint_names):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        self.rclpy, self.joint_type = rclpy, JointState
        self.args, self.report, self.launch = args, report, launch
        self.begin, self.cancel_state = begin, cancel_state
        self.namespace = '/sim_ToPoDualArm' if args.robot == 'topodualarm' else '/sim_topo_dual_arm_' + args.robot
        self.node = rclpy.create_node('dual_arm_control_check_' + str(os.getpid()))
        self.joint_names = joint_names
        self.positions, self.velocities = {}, {}
        self.control, self.safety, self.demo = {}, {}, {}
        self.joint_stamp = self.low_since_sim_sec = None
        self.joint_sec = self.control_sec = self.safety_sec = -math.inf
        self.num_joint_samples = 0
        self.leader_target = None
        self.next_leader_sec = self.next_track_sec = 0.0
        self.stage = 'preflight'
        self.callback_error = None
        self.events = (args.output / 'events.jsonl').open('w', encoding='utf-8')
        self.samples = (args.output / 'joint_samples.jsonl').open('w', encoding='utf-8')
        self.last_sample_stamp = -math.inf
        self.node.create_subscription(JointState, self.namespace + '/joint_states',
                                      self.on_joint, qos_profile_sensor_data)
        for name, callback in (('control/status', self.on_control), ('safety/status', self.on_safety),
                               ('avoidance/status', self.on_demo)):
            self.node.create_subscription(String, self.namespace + '/' + name, callback, 10)
        # ハーネスからの出力は模擬leader実測のみ。最終JTCへの直接指令なし。
        self.leader = self.node.create_publisher(JointState, '/leader/joint_states', 10)

    def event(self, name, **values):
        self.events.write(json.dumps({'event': name, 'stage': self.stage,
                          'wall_sec': time.monotonic() - self.begin, 'sim_sec': self.joint_stamp,
                          **values}, ensure_ascii=False, allow_nan=False) + '\n')
        self.events.flush()

    def set_stage(self, name):
        self.stage = self.report['stage'] = name
        self.event('stage')
        print(json.dumps({'stage': name, 'robot': self.args.robot}), flush=True)

    def parse_status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('状態JSONがobject以外')
            return value
        except (TypeError, ValueError) as error:
            self.callback_error = str(error)
            return {}

    def on_control(self, message):
        self.control, self.control_sec = self.parse_status(message), time.monotonic()

    def on_safety(self, message):
        self.safety, self.safety_sec = self.parse_status(message), time.monotonic()

    def on_demo(self, message):
        self.demo = self.parse_status(message)

    def on_joint(self, message):
        if len(message.name) != len(message.position) or len(message.name) != len(message.velocity):
            self.callback_error = '関節実測の配列長不一致'
            return
        positions, velocities = dict(zip(message.name, message.position)), dict(zip(message.name, message.velocity))
        if not all(name in positions and name in velocities and math.isfinite(positions[name])
                   and math.isfinite(velocities[name]) for name in self.joint_names):
            self.callback_error = '全関節の有限実測不足'
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.joint_stamp is not None and stamp <= self.joint_stamp:
            return
        now = time.monotonic()
        max_velocity = max(abs(velocities[name]) for name in self.joint_names)
        if max_velocity > 0.01 or now - self.joint_sec > 0.5:
            self.low_since_sim_sec = None
        if max_velocity <= 0.01 and self.low_since_sim_sec is None:
            self.low_since_sim_sec = stamp
        self.positions, self.velocities = positions, velocities
        self.joint_stamp, self.joint_sec = stamp, now
        self.num_joint_samples += 1
        if stamp - self.last_sample_stamp >= 0.02:
            self.samples.write(json.dumps({'wall_sec': now - self.begin, 'sim_sec': stamp,
                'stage': self.stage, 'max_velocity_rad_sec': max_velocity,
                'positions': positions}, allow_nan=False) + '\n')
            self.samples.flush()
            self.last_sample_stamp = stamp

    def graph_names(self):
        own_name = self.node.get_fully_qualified_name()
        return sorted(namespace.rstrip('/') + '/' + name for name, namespace in
                      self.node.get_node_names_and_namespaces()
                      if namespace.rstrip('/') + '/' + name != own_name)

    def spin(self, allow_launch_exit=False):
        now = time.monotonic()
        if self.cancel_state['is_cancelled']:
            raise InterruptedError('試験中断要求')
        if now - self.begin >= self.args.timeout_sec:
            raise TimeoutError('統合試験の全体期限超過')
        if self.leader_target is not None and now >= self.next_leader_sec:
            message = self.joint_type()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.name, message.position = ['L_joint2'], [self.leader_target]
            self.leader.publish(message)
            self.next_leader_sec = now + 0.05
        self.rclpy.spin_once(self.node, timeout_sec=0.01)
        self.launch.read_terminal()
        if self.callback_error:
            raise RuntimeError(self.callback_error)
        if self.launch.process is not None and self.launch.process.poll() is not None and not allow_launch_exit:
            raise RuntimeError('launchの予期しない終了: ' + str(self.launch.process.returncode))
        if now >= self.next_track_sec:
            self.launch.track()
            self.next_track_sec = now + 0.5

    def wait(self, predicate, max_sec=20, allow_launch_exit=False):
        deadline = time.monotonic() + max_sec
        while True:
            self.spin(allow_launch_exit=allow_launch_exit)
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(self.stage + ': 段階期限超過')

    def key(self, value):
        self.launch.send_key(value)
        self.event('key', value=value.decode('ascii'))

    def is_fresh(self):
        return self.joint_stamp is not None and time.monotonic() - self.joint_sec <= 0.5

    def is_stationary(self):
        return (self.is_fresh() and self.low_since_sim_sec is not None
                and self.joint_stamp - self.low_since_sim_sec >= 0.25 - 1e-9)

    def is_stopped(self):
        age = self.safety.get('state_age_sec')
        return (self.is_stationary() and time.monotonic() - self.safety_sec <= 0.5
                and self.safety.get('state') == 'stopped'
                and self.safety.get('is_stop_latched') is True
                and self.safety.get('is_stop_applied') is True and self.safety.get('is_stopped') is True
                and isinstance(age, (int, float)) and not isinstance(age, bool)
                and math.isfinite(age) and 0 <= age <= 0.5)

    def is_hold_ready(self):
        return (self.is_stationary() and time.monotonic() - self.control_sec <= 0.5
                and self.control.get('mode') == 'hold' and self.control.get('phase') == 'idle'
                and self.control.get('is_ready') is True and self.safety.get('is_stop_latched') is False)

    def observe_hold(self, name, enable_latch_check):
        initial, started = dict(self.positions), self.joint_stamp
        max_drift = max_velocity = 0.0
        deadline = time.monotonic() + 12
        while self.joint_stamp - started < 0.5:
            self.spin()
            if not self.is_fresh() or (enable_latch_check and not self.is_stopped()):
                raise AssertionError(name + ': 実測停止の失効')
            max_drift = max(max_drift, max(abs(self.positions[item] - initial[item]) for item in self.joint_names))
            max_velocity = max(max_velocity, max(abs(self.velocities[item]) for item in self.joint_names))
            if max_drift > 0.01 or max_velocity > 0.01:
                raise AssertionError(name + ': 保持時の再運動')
            if time.monotonic() > deadline:
                raise TimeoutError(name + ': 物理時刻進行不足')
        self.report['checks'][name] = {'duration_sim_sec': self.joint_stamp - started,
            'max_drift_rad': max_drift, 'max_velocity_rad_sec': max_velocity}

    def check_leader_resume(self):
        """廃止キー無効・停止解除後保持・再度Lでの追従開始の実端末検証。"""
        self.set_stage('removed_keys')
        for key in (b'r', b'q', b's'):
            self.key(key)
        self.observe_hold('removed_keys_keep_stop', True)
        self.set_stage('reset_to_hold_without_leader')
        self.leader_target = None
        self.wait(lambda: self.control.get('has_fresh_leader') is False, 5)
        self.key(b'l')
        self.wait(self.is_hold_ready, 30)
        self.observe_hold('reset_to_hold_without_input', False)
        self.set_stage('leader_without_input')
        transcript_start = len(self.launch.transcript)
        self.key(b'l')
        self.wait(lambda: 'リーダーフォロワーON: 拒否・応答不正'
                  in self.launch.transcript[transcript_start:], 5)
        if not self.is_hold_ready():
            raise AssertionError('入力なしの追従拒否後にホールドが解除')
        self.report['checks']['leader_without_input_keeps_hold'] = True
        self.set_stage('leader_resume')
        initial = self.positions['L_joint2']
        self.leader_target = initial + 0.04
        self.wait(lambda: self.control.get('has_fresh_leader') is True, 5)
        self.key(b'l')
        self.wait(lambda: self.control.get('mode') == 'leader'
                  and self.control.get('phase') == 'idle'
                  and self.positions['L_joint2'] - initial > 0.005, 30)
        if self.control.get('enable_hardware_output') is not False:
            raise AssertionError('L再開によるUDP自動再送')
        self.report['checks']['leader_resume_without_udp_restart'] = dict(self.control)
        self.check_cross_mode_rejection('leader', b'a', '回避ON')
        self.set_stage('leader_off_hold')
        self.key(b'l')
        self.wait(self.is_hold_ready, 30)
        self.observe_hold('leader_off_hold_with_input', False)

    def check_cross_mode_rejection(self, mode, key, label):
        """動作中の別モードキーによる暗黙ホールド・自動開始の拒否。"""
        self.set_stage('cross_mode_rejection_from_' + mode)
        transcript_start = len(self.launch.transcript)
        self.key(key)
        self.wait(lambda: label + ': 拒否・応答不正' in self.launch.transcript[transcript_start:], 5)
        if self.control.get('mode') != mode or self.control.get('phase') != 'idle':
            raise AssertionError('別モードキー拒否時の現在モード変更')
        self.report['checks']['cross_mode_rejected_from_' + mode] = dict(self.control)

    def execute(self):
        self.set_stage('startup')
        self.wait(lambda: self.is_hold_ready() and 'Enter不要' in self.launch.transcript
                  and self.demo.get('state') == 'idle', 90)
        topic = self.namespace + '/dual_arm_controller/joint_trajectory'
        if self.node.count_publishers(topic) != 1:
            raise AssertionError('最終軌道publisherが単一ではありません')
        if any(self.node.count_publishers(topic) for topic in
               ('/dynamixel/command/goal', '/dynamixel/shortcut', '/leader/dynamixel/command/goal')):
            raise AssertionError('実機コマンドpublisherの存在')
        self.report['checks']['single_output_and_no_hardware'] = True
        self.report['initial_positions'] = dict(self.positions)
        self.set_stage('leader_key')
        initial = self.positions['L_joint2']
        self.leader_target = initial + 0.06
        self.wait(lambda: self.control.get('has_fresh_leader') is True)
        self.key(b'l')
        self.wait(lambda: self.control.get('mode') == 'leader' and
                  self.positions['L_joint2'] - initial > 0.005 and abs(self.velocities['L_joint2']) > 0.01)
        self.report['checks']['leader_movement'] = {'joint': 'L_joint2', 'target_delta_rad': 0.06,
            'observed_delta_rad': self.positions['L_joint2'] - initial,
            'velocity_rad_sec': self.velocities['L_joint2']}
        self.set_stage('space_stop')
        stop_stamp = self.joint_stamp
        self.low_since_sim_sec = None
        self.key(b' ')
        self.wait(lambda: self.is_stopped() and self.joint_stamp - stop_stamp >= 0.25, 30)
        self.report['checks']['space_physical_stop'] = dict(self.safety)
        self.observe_hold('stop_hold_with_leader_input', True)
        self.check_leader_resume()
        self.set_stage('hardware_key')
        transcript_start = len(self.launch.transcript)
        self.key(b'h')
        self.wait(lambda: '実機送信は無効です' in self.launch.transcript[transcript_start:]
                  and 'ON拒否' in self.launch.transcript[transcript_start:], 5)
        if self.control.get('enable_hardware_output') is not False:
            raise AssertionError('実機出力ON拒否後の状態不正')
        self.report['checks']['hardware_on_rejected'] = True
        if self.args.check_avoidance:
            self.set_stage('optional_avoidance_key')
            try:
                self.key(b'a')
                self.wait(lambda: self.control.get('mode') in ('avoidance', 'stopped'), 12)
                is_mode_on = self.control.get('mode') == 'avoidance'
                self.report['optional_avoidance'] = {
                    'result': 'on_confirmed' if is_mode_on else 'stopped_before_on',
                    'on_control': dict(self.control), 'on_demo': dict(self.demo)}
                if is_mode_on:
                    self.check_cross_mode_rejection('avoidance', b'l', 'リーダーフォロワーON')
                    self.key(b'a')
                    self.wait(lambda: self.is_hold_ready() or self.control.get('mode') == 'stopped', 12)
                    self.report['optional_avoidance'].update(
                        result='on_off_confirmed' if self.is_hold_ready() else 'stopped_during_off',
                        off_control=dict(self.control), off_demo=dict(self.demo))
            except (AssertionError, RuntimeError, TimeoutError) as error:
                self.report.setdefault('optional_avoidance', {}).update(
                    result='not_confirmed', error=str(error), control=dict(self.control), demo=dict(self.demo))
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
        self.events.close()
        self.samples.close()
        self.node.destroy_node()


def run(args, trial_factory=control_trial):
    args.output.mkdir(parents=True, exist_ok=True)
    if any((args.output / name).exists() for name in ('report.json', 'ownership.json', 'command.json')):
        raise FileExistsError('既存試験結果への上書き禁止')
    begin = time.monotonic()
    report = {'result': 'failed', 'stage': 'preflight', 'robot': args.robot, 'checks': {},
              'criteria': {'max_stop_velocity_rad_sec': 0.01, 'min_confirm_sim_sec': 0.25,
                           'max_state_age_sec': 0.5, 'max_hold_drift_rad': 0.01}}
    baseline = process_snapshot()
    launch = pty_launch(args.output, baseline)
    trial = rclpy = None
    cancel_state = {'is_cancelled': False}

    def cancel(_signum, _frame):
        cancel_state['is_cancelled'] = True

    handlers = {item: signal.signal(item, cancel) for item in (signal.SIGINT, signal.SIGTERM)}
    try:
        if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise ValueError('ROS_DOMAIN_ID=96とROS_LOCALHOST_ONLY=1が必要')
        if not math.isfinite(args.timeout_sec) or not 0 < args.timeout_sec <= 180:
            raise ValueError('timeout-secは有限の正数かつ180秒以内が必要')
        if port_is_listening():
            raise RuntimeError('専用Gazebo port11369が既に使用中')
        import yaml
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        package = Path(__file__).resolve().parents[1]
        robot_name = 'ToPoDualArm' if args.robot == 'topodualarm' else 'topo_dual_arm_' + args.robot
        params_file = args.params_file or package / 'config' / (robot_name + '.yaml')
        params = yaml.safe_load(params_file.read_text())['/**']['ros__parameters']
        if params['robot_name'] != robot_name:
            raise ValueError('robotとparams-fileの機種不一致')
        root = element_tree.parse(params['urdf_path']).getroot()
        joints = [item.get('name') for item in root.findall('joint') if item.get('type') != 'fixed']
        target = next(item for item in root.findall('joint') if item.get('name') == 'L_joint2')
        min_position, max_position = float(target.find('limit').get('lower')), float(target.find('limit').get('upper'))
        if not min_position < 0 < 0.06 < max_position:
            raise ValueError('検証用小角度の可動域不足')
        demo_filename = 'topodualarm_gazebo_demo.yaml' if args.robot == 'topodualarm' else 'dual_arm_gazebo_demo.yaml'
        demo = yaml.safe_load((package / 'config' / demo_filename).read_text())
        demo['dual_arm_gazebo_demo'].update(enable_viewer=False, enable_gui=False, enable_auto_start=False)
        demo_path = args.output / 'demo_config.yaml'
        demo_path.write_text(yaml.safe_dump(demo, allow_unicode=True), encoding='utf-8')
        save_json(args.output / 'baseline_processes.json', [row for row in baseline.values()
                  if any(word in row['command'] for word in ('ros2', 'gzserver', 'gzclient', 'gazebo', '_ros2_daemon'))])
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        trial = trial_factory(args, report, launch, begin, cancel_state, joints)
        discovery_deadline = time.monotonic() + 1.5
        while time.monotonic() < discovery_deadline:
            trial.spin()
        report['baseline_ros_nodes'] = trial.graph_names()
        if report['baseline_ros_nodes']:
            raise RuntimeError('専用domain96に既存ノードあり。起動・操作の中止')
        command = ['ros2', 'launch', 'gng_vlut_system', 'dual_arm_control.launch.py',
                   'robot:=' + args.robot, 'params_file:=' + str(params_file.resolve()), 'gui:=false',
                   'demo_config:=' + str(demo_path.resolve()), 'enable_keyboard:=true',
                   'gazebo_master_uri:=http://127.0.0.1:11369']
        if hasattr(trial, 'prepare_command'):
            command = trial.prepare_command(command)
        report['command'] = command
        launch.start(command)
        trial.execute()
        report['result'] = 'passed'
    except BaseException as error:
        report.update(result='failed', error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    finally:
        try:
            report['cleanup'] = launch.cleanup()
            launch.read_terminal()
            if launch.slave is not None:
                report['cleanup']['is_terminal_restored'] = launch.is_terminal_restored()
                report['cleanup']['is_success'] &= report['cleanup']['is_terminal_restored']
            if trial is not None:
                report.update(last_control=dict(trial.control), last_safety=dict(trial.safety),
                              last_demo=dict(trial.demo), last_positions=dict(trial.positions),
                              num_joint_samples=trial.num_joint_samples)
                if launch.process is not None:
                    graph_deadline = time.monotonic() + 15
                    while trial.graph_names() and time.monotonic() < graph_deadline:
                        rclpy.spin_once(trial.node, timeout_sec=0.1)
                    report['remaining_ros_nodes'] = trial.graph_names()
                    report['cleanup']['is_success'] &= not report['remaining_ros_nodes'] and not port_is_listening()
            if not report['cleanup']['is_success']:
                report.update(result='failed', cleanup_error='試験前状態への復元未確認')
        except BaseException as error:
            report.update(result='failed', cleanup_error=f'{type(error).__name__}: {error}')
        finally:
            if trial is not None:
                trial.close()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            launch.close_terminal()
            for item, handler in handlers.items():
                signal.signal(item, handler)
            report['wall_sec'] = time.monotonic() - begin
            save_json(args.output / 'report.json', report)
            save_json(args.output / 'metrics.json', {'is_success': int(report['result'] == 'passed'),
                      'is_cleanup_success': int(report.get('cleanup', {}).get('is_success', False)),
                      'wall_sec': report['wall_sec'], 'num_checks': len(report['checks']),
                      'num_joint_samples': report.get('num_joint_samples', 0)})
            print(json.dumps({'result': report['result'], 'stage': report['stage'],
                  'report': str(args.output / 'report.json'), 'error': report.get('error'),
                  'cleanup_error': report.get('cleanup_error')}, ensure_ascii=False), flush=True)
    return int(report['result'] != 'passed')


def main():
    parser = argparse.ArgumentParser(description='単一PTYのGazebo統合操作試験。実機driver・指令なし')
    parser.add_argument('--robot', choices=('max', 'max_long', 'topodualarm'), default='max')
    parser.add_argument('--params-file', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout-sec', type=float, default=180)
    parser.add_argument('--check-avoidance', action='store_true', help='Aキー経路の参考観測。合格条件から分離')
    args = parser.parse_args()
    args.output = args.output.resolve()
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
