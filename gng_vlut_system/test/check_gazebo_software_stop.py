#!/usr/bin/env python3
"""隔離Gazeboにおける停止ラッチ・物理停止・明示的再開の有限試験。"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import time
import traceback
import uuid
import xml.etree.ElementTree as element_tree


def save_json(path, value):
    """非有限値を許可しない試験記録の保存。"""
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def process_snapshot():
    """PID再利用の判別に必要なLinuxプロセス情報。"""
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            result[int(entry.name)] = {
                'pid': int(entry.name), 'ppid': int(fields[1]), 'pgid': int(fields[2]),
                'start_ticks': int(fields[19]), 'state': fields[0],
                'command': (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip(),
            }
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return result


def port_is_listening():
    """専用Gazebo masterのTCP待受確認。"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        connection.settimeout(0.2)
        return connection.connect_ex(('127.0.0.1', 11369)) == 0


class owned_launch:
    """既存PIDを保護したlaunch子孫の追跡・段階終了。"""

    def __init__(self, output, baseline):
        self.output = output
        self.baseline = {pid: row['start_ticks'] for pid, row in baseline.items()}
        self.marker = 'gazebo_stop_' + uuid.uuid4().hex
        self.process = None
        self.owned = {}
        self.temp_dirs = set()
        self.log = None

    def start(self, command):
        child_env = dict(os.environ, ROS2CLI_NO_DAEMON='1', ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='96',
                         GAZEBO_MASTER_URI='http://127.0.0.1:11369', GNG_STOP_TEST_OWNER=self.marker)
        self.log = (self.output / 'gazebo.log').open('w', encoding='utf-8')
        # 外側runnerの最終killpgでも回収可能なPGID継承。終了処理は所有PIDだけが対象。
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                        env=child_env, start_new_session=False)
        save_json(self.output / 'command.json', command)
        self.track()

    def track(self):
        current = process_snapshot()
        has_changed = False
        while True:
            added = False
            for pid, row in current.items():
                if pid == os.getpid() or self.baseline.get(pid) == row['start_ticks']:
                    continue
                if pid in self.owned and self.owned[pid]['start_ticks'] == row['start_ticks']:
                    continue
                is_root = (self.process is not None and pid == self.process.pid
                           and (pid not in self.owned or self.owned[pid]['start_ticks'] == row['start_ticks']))
                parent = self.owned.get(row['ppid'])
                has_owned_parent = parent is not None and current.get(row['ppid'], {}).get('start_ticks') == parent['start_ticks']
                has_marker = False
                if not is_root and not has_owned_parent:
                    try:
                        has_marker = ('GNG_STOP_TEST_OWNER=' + self.marker).encode() in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
                    except (FileNotFoundError, ProcessLookupError, PermissionError):
                        pass
                if is_root or has_owned_parent or has_marker:
                    self.owned[pid] = row
                    for match in re.finditer(r'/tmp/dual_arm_gazebo_demo_[A-Za-z0-9_]+', row['command']):
                        self.temp_dirs.add(match.group(0))
                    added = has_changed = True
            if not added:
                break
        if has_changed or not (self.output / 'ownership.json').exists():
            save_json(self.output / 'ownership.json', {
                'checker_pid': os.getpid(), 'checker_pgid': os.getpgrp(), 'marker': self.marker,
                'launch_pid': self.process.pid if self.process else None,
                'launch_pgid': current.get(self.process.pid, {}).get('pgid') if self.process else None,
                'owned_processes': list(self.owned.values()), 'owned_temp_dirs': sorted(self.temp_dirs),
            })
        return {pid: row for pid, row in current.items() if pid in self.owned
                and row['start_ticks'] == self.owned[pid]['start_ticks'] and row['state'] != 'Z'}

    def cleanup(self):
        signals = []
        for stop_signal, duration in ((signal.SIGINT, 5.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            alive = self.track()
            if not alive:
                break
            signal_targets = alive
            if stop_signal == signal.SIGINT:
                # 初回はlaunch経由の正常終了。子への重複SIGINTによる後始末中断の防止。
                signal_targets = ({self.process.pid: alive[self.process.pid]}
                                  if self.process is not None and self.process.pid in alive else {})
            for pid, row in signal_targets.items():
                # 直前のstarttime一致による他プロセスへの誤送信防止。
                if process_snapshot().get(pid, {}).get('start_ticks') != row['start_ticks']:
                    continue
                try:
                    os.kill(pid, stop_signal)
                    signals.append({'pid': pid, 'signal': stop_signal.name})
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if self.process is not None:
                    self.process.poll()
                if not self.track():
                    break
                time.sleep(0.1)
        if self.process is not None:
            try:
                self.process.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                pass
        remaining = self.track()
        removed = []
        if not remaining:
            for name in self.temp_dirs:
                path = Path(name)
                if path.parent == Path('/tmp') and path.name.startswith('dual_arm_gazebo_demo_') and path.is_dir() and not path.is_symlink():
                    shutil.rmtree(path)
                    removed.append(name)
        if self.log is not None:
            self.log.close()
        return {'is_success': not remaining, 'remaining_processes': list(remaining.values()),
                'signals': signals, 'removed_owned_temp_dirs': removed}


class stop_trial:
    """ROS実測とサービス応答を分離した試験状態。"""

    def __init__(self, args, report, launch, begin, cancel_state):
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String, Bool
        from rosgraph_msgs.msg import Clock
        from std_srvs.srv import Trigger, Empty
        from controller_manager_msgs.srv import SwitchController, ListControllers
        from control_msgs.msg import JointTrajectoryControllerState
        from trajectory_msgs.msg import JointTrajectory
        self.rclpy = rclpy
        self.args, self.report, self.launch = args, report, launch
        self.begin, self.cancel_state = begin, cancel_state
        self.node = rclpy.create_node('gazebo_software_stop_check_' + str(os.getpid()))
        self.namespace = '/' + args.namespace
        self.positions, self.velocities, self.safety, self.demo = {}, {}, {}, {}
        self.gng_status, self.controller_sample = {}, {}
        self.is_stop_latched = None
        self.joint_stamp = self.clock_sec = None
        self.joint_wall_sec = self.safety_wall_sec = 0.0
        self.low_since_sim_sec = None
        self.last_log_sim_sec = None
        self.last_track_wall_sec = 0.0
        self.num_joint_samples = self.num_commands = 0
        self.stage = 'preflight'
        self.events = (args.output / 'events.jsonl').open('w', encoding='utf-8')
        self.samples = (args.output / 'joint_samples.jsonl').open('w', encoding='utf-8')
        self.all_joint_names, self.command_joint_names, self.limits = [], [], {}
        self.callback_error = None
        self.hold_reference = None
        self.hold_violation = None
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            self.node.create_subscription(JointState, self.namespace + '/joint_states', self.on_joint, qos_profile_sensor_data),
            self.node.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data),
            self.node.create_subscription(String, self.namespace + '/safety/status', self.on_safety, latch_qos),
            self.node.create_subscription(Bool, self.namespace + '/safety/is_stop_latched', self.on_latch, latch_qos),
            self.node.create_subscription(String, self.namespace + '/avoidance/status', self.on_demo, 10),
            self.node.create_subscription(String, self.namespace + '/avoidance/gng_status', self.on_gng_status, 10),
            self.node.create_subscription(JointTrajectoryControllerState, self.namespace + '/dual_arm_controller/controller_state', self.on_controller_sample, 10),
            self.node.create_subscription(JointTrajectory, self.namespace + '/dual_arm_controller/joint_trajectory', self.on_command, 10),
        ]
        self.command = self.node.create_publisher(JointTrajectory, self.namespace + '/dual_arm_controller/joint_trajectory', 10)
        self.clients = {name: self.node.create_client(Trigger, self.namespace + '/' + name)
                        for name in ('safety/stop', 'safety/reset', 'avoidance/start')}
        self.clients['switch'] = self.node.create_client(SwitchController, self.namespace + '/controller_manager/switch_controller')
        self.clients['controllers'] = self.node.create_client(ListControllers, self.namespace + '/controller_manager/list_controllers')
        self.clients['pause'] = self.node.create_client(Empty, '/pause_physics')
        self.clients['unpause'] = self.node.create_client(Empty, '/unpause_physics')

    def event(self, name, **fields):
        value = {'event': name, 'stage': self.stage, 'wall_sec': time.monotonic() - self.begin,
                 'sim_sec': self.clock_sec, **fields}
        self.events.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + '\n')
        self.events.flush()

    def set_stage(self, stage):
        self.stage = stage
        self.report['stage'] = stage
        self.event('stage')
        print(json.dumps({'stage': stage, 'motion_source': self.args.motion_source}, ensure_ascii=False), flush=True)

    def on_clock(self, message):
        self.clock_sec = message.clock.sec + message.clock.nanosec * 1e-9

    def on_latch(self, message):
        self.is_stop_latched = bool(message.data)

    def on_safety(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('安全状態の形式不正')
            self.safety = value
            self.safety_wall_sec = time.monotonic()
        except (ValueError, TypeError) as error:
            self.callback_error = str(error)

    def on_demo(self, message):
        try:
            self.demo = json.loads(message.data)
        except (ValueError, TypeError) as error:
            self.callback_error = str(error)

    def on_command(self, _message):
        self.num_commands += 1

    def on_gng_status(self, message):
        self.gng_status = json.loads(message.data)

    def on_controller_sample(self, message):
        self.controller_sample = {
            'sim_sec': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            'desired_positions': dict(zip(message.joint_names, message.reference.positions)),
            'actual_positions': dict(zip(message.joint_names, message.feedback.positions)),
        }

    def on_joint(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        positions, velocities = dict(zip(message.name, message.position)), dict(zip(message.name, message.velocity))
        if not self.all_joint_names or not all(name in positions and name in velocities and
                math.isfinite(positions[name]) and math.isfinite(velocities[name]) for name in self.all_joint_names):
            self.low_since_sim_sec = None
            return
        if self.joint_stamp is not None and stamp <= self.joint_stamp:
            return
        max_velocity = max(abs(velocities[name]) for name in self.all_joint_names)
        if max_velocity > 0.01 or (self.joint_stamp is not None and stamp - self.joint_stamp > 0.1):
            self.low_since_sim_sec = None
        if max_velocity <= 0.01 and self.low_since_sim_sec is None:
            self.low_since_sim_sec = stamp
        self.positions, self.velocities = positions, velocities
        self.joint_stamp, self.joint_wall_sec = stamp, time.monotonic()
        self.num_joint_samples += 1
        if self.hold_reference is not None:
            max_drift = max(abs(positions[name] - self.hold_reference[name]) for name in self.all_joint_names)
            if max_velocity > 0.01 or max_drift > 0.01:
                self.hold_violation = {'stage': self.stage, 'sim_sec': stamp,
                                       'max_velocity_rad_sec': max_velocity, 'max_drift_rad': max_drift,
                                       'positions': positions, 'velocities': velocities,
                                       'hold_reference': dict(self.hold_reference),
                                       'last_controller_sample': dict(self.controller_sample)}
        if self.last_log_sim_sec is None or stamp - self.last_log_sim_sec >= 0.02:
            self.samples.write(json.dumps({'sim_sec': stamp, 'stage': self.stage, 'positions': positions,
                                          'velocities': velocities, 'max_velocity_rad_sec': max_velocity}, allow_nan=False) + '\n')
            self.last_log_sim_sec = stamp

    def spin(self, *, allow_demo_fault=True):
        if self.cancel_state['is_cancelled']:
            raise KeyboardInterrupt('試験中断')
        if time.monotonic() - self.begin > self.args.timeout_sec:
            raise TimeoutError('試験全体の実時間上限')
        if self.launch.process is not None and self.launch.process.poll() is not None:
            raise RuntimeError(f'Gazebo launch終了: {self.launch.process.returncode}')
        self.rclpy.spin_once(self.node, timeout_sec=0.02)
        if self.callback_error:
            raise RuntimeError(self.callback_error)
        if self.hold_violation:
            self.report['hold_violation'] = self.hold_violation
            raise AssertionError('停止保持期間の再運動: ' + str(self.hold_violation))
        if not allow_demo_fault and self.demo.get('state') == 'fault':
            raise RuntimeError('通常デモ失敗: ' + str(self.demo.get('error')))
        if time.monotonic() - self.last_track_wall_sec >= 0.5:
            self.launch.track()
            self.last_track_wall_sec = time.monotonic()

    def wait(self, predicate, max_sec=30, *, allow_demo_fault=True):
        deadline = time.monotonic() + max_sec
        while True:
            self.spin(allow_demo_fault=allow_demo_fault)
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{self.stage}: 待機上限 {max_sec:g} 秒')

    def call(self, name, request=None, max_sec=10):
        client = self.clients[name]
        self.wait(client.service_is_ready, max_sec)
        request = request if request is not None else client.srv_type.Request()
        future = client.call_async(request)
        self.wait(future.done, max_sec)
        response = future.result()
        if response is None:
            raise RuntimeError('サービス応答なし: ' + name)
        self.event('service_response', service=name, success=getattr(response, 'success', None),
                   ok=getattr(response, 'ok', None), message=getattr(response, 'message', None))
        return response

    def is_fresh(self):
        return self.joint_stamp is not None and self.clock_sec is not None and time.monotonic() - self.joint_wall_sec <= 0.5

    def is_stationary(self):
        return self.is_fresh() and self.low_since_sim_sec is not None and self.joint_stamp - self.low_since_sim_sec >= 0.25

    def is_physically_stopped(self):
        age = self.safety.get('state_age_sec')
        return (self.is_stationary() and self.is_stop_latched is True and self.safety.get('is_stopped') is True
                and self.safety.get('is_stop_applied') is True and self.safety.get('state') == 'stopped'
                and isinstance(age, (float, int)) and math.isfinite(age) and age <= 0.5
                and time.monotonic() - self.safety_wall_sec <= 0.5)

    def controller_state(self):
        return self.call('controllers').controller

    def switch(self, *, enable_controller):
        request = self.clients['switch'].srv_type.Request()
        start_field = 'activate_controllers' if hasattr(request, 'activate_controllers') else 'start_controllers'
        stop_field = 'deactivate_controllers' if hasattr(request, 'deactivate_controllers') else 'stop_controllers'
        setattr(request, start_field, ['dual_arm_controller'] if enable_controller else [])
        setattr(request, stop_field, [] if enable_controller else ['dual_arm_controller'])
        request.strictness = 2
        request.timeout.sec = 3
        return self.call('switch', request)

    def publish_step(self, delta_rad):
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        if not self.is_fresh():
            raise RuntimeError('全関節軌道の生成前に実測失効')
        target = [self.positions[name] for name in self.command_joint_names]
        joint = 'L_joint2'
        idx = self.command_joint_names.index(joint)
        lower, upper = self.limits[joint]
        if not lower + 0.01 < target[idx] + delta_rad < upper - 0.01:
            delta_rad = -delta_rad
        if not lower + 0.01 < target[idx] + delta_rad < upper - 0.01:
            raise RuntimeError('検証用小変位の関節範囲不足')
        message = JointTrajectory()
        message.joint_names = list(self.command_joint_names)
        for seconds, positions in ((0, target), (2, [value + delta_rad if item_idx == idx else value for item_idx, value in enumerate(target)])):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.velocities = [0.0] * len(target)
            point.accelerations = [0.0] * len(target)
            point.time_from_start.sec = seconds
            message.points.append(point)
        self.command.publish(message)
        self.event('trajectory_sent', joint=joint, delta_rad=delta_rad, num_joints=len(target), duration_sim_sec=2.0)
        return joint, target[idx], delta_rad

    def observe_hold(self, name, duration_sim_sec=0.75, *, require_latch):
        if not self.is_stationary():
            raise AssertionError(name + ': 保持検証開始前の停止実測不足')
        started, initial = self.joint_stamp, dict(self.positions)
        max_velocity, max_drift = 0.0, 0.0
        deadline = time.monotonic() + 30
        while self.joint_stamp - started < duration_sim_sec:
            self.spin()
            if not self.is_fresh():
                raise RuntimeError(name + ': 関節実測失効')
            current_velocity = max(abs(self.velocities[item]) for item in self.all_joint_names)
            max_velocity = max(max_velocity, current_velocity)
            max_drift = max(max_drift, max(abs(self.positions[item] - initial[item]) for item in self.all_joint_names))
            if current_velocity > 0.01 or max_drift > 0.01:
                raise AssertionError(name + ': 停止後の再運動')
            if require_latch and not self.is_physically_stopped():
                raise AssertionError(name + ': 停止確認の失効')
            if time.monotonic() >= deadline:
                raise TimeoutError(name + ': /clock進行不足')
        value = {'duration_sim_sec': self.joint_stamp - started, 'max_velocity_rad_sec': max_velocity,
                 'max_position_drift_rad': max_drift}
        self.report['checks'][name] = value
        self.event('hold_verified', check=name, **value)

    def require_stop(self, check_name='physical_stop'):
        request_clock = self.clock_sec
        self.low_since_sim_sec = None
        response = self.call('safety/stop')
        if not response.success:
            raise AssertionError('停止要求が拒否')
        self.report['checks']['stop_request_accepted'] = True
        self.wait(lambda: self.is_physically_stopped() and self.clock_sec - request_clock >= 0.25, 45)
        self.report['checks'][check_name] = {'request_clock_sec': request_clock,
            'confirmed_clock_sec': self.clock_sec, 'status': dict(self.safety),
            'observed_low_velocity_sim_sec': self.joint_stamp - self.low_since_sim_sec}
        self.wait(lambda: self.demo.get('state') == 'stopped' and self.demo.get('phase') == 'software_stop', 5)

    def execute(self):
        self.set_stage('startup')
        self.wait(lambda: self.is_stationary() and self.demo.get('state') == 'idle'
                  and self.safety.get('has_active_commands') is True and self.is_stop_latched is False
                  and self.command.get_subscription_count() > 0, 180)
        controllers = self.controller_state()
        active = [item.name for item in controllers if any(interface.endswith('/position') for interface in item.claimed_interfaces)]
        if active != ['dual_arm_controller']:
            raise AssertionError('想定外の指令controller: ' + str(active))
        self.report['initial_positions'] = dict(self.positions)
        self.set_stage('motion_' + self.args.motion_source)
        initial = dict(self.positions)
        if self.args.motion_source == 'demo':
            response = self.call('avoidance/start')
            if not response.success:
                raise AssertionError('通常デモ開始拒否: ' + response.message)
            self.wait(lambda: self.demo.get('state') == 'running' and
                      max(abs(self.positions[name] - initial[name]) for name in self.command_joint_names) > 0.005
                      and max(abs(self.velocities[name]) for name in self.all_joint_names) > 0.01,
                      90, allow_demo_fault=False)
        else:
            joint, position, delta = self.publish_step(0.04)
            self.wait(lambda: delta * (self.positions[joint] - position) > abs(delta) * 0.002
                      and abs(self.velocities[joint]) > 0.01, 30)
        self.report['checks']['motion_observed'] = {'source': self.args.motion_source, 'sim_sec': self.clock_sec,
            'max_displacement_rad': max(abs(self.positions[name] - initial[name]) for name in self.command_joint_names),
            'max_velocity_rad_sec': max(abs(self.velocities[name]) for name in self.all_joint_names)}
        self.set_stage('stop_and_command_block')
        self.require_stop()
        # サービス応答待ち・controller切替中も含む、再運動の連続監視。
        self.hold_reference = dict(self.positions)
        if self.call('avoidance/start').success:
            raise AssertionError('ラッチ中のデモ開始が受理')
        self.publish_step(-0.04)
        self.observe_hold('command_ignored_while_stopped', require_latch=True)
        if self.call('safety/reset').success:
            raise AssertionError('active controllerのまま解除成功')
        self.report['checks']['active_reset_rejected'] = True
        self.set_stage('controller_deactivation')
        if not self.switch(enable_controller=False).ok:
            raise AssertionError('controller deactivate失敗')
        self.wait(lambda: self.safety.get('has_active_commands') is False, 5)
        if any(interface.endswith('/position') for item in self.controller_state() for interface in item.claimed_interfaces):
            raise AssertionError('deactivate後の位置指令interface残留')
        if self.switch(enable_controller=True).ok:
            raise AssertionError('ラッチ中のcontroller activate成功')
        if next(item for item in self.controller_state() if item.name == 'dual_arm_controller').state != 'inactive':
            raise AssertionError('activation拒否後のcontroller状態不正')
        self.report['checks']['latched_activation_rejected'] = True
        self.set_stage('paused_physics')
        self.call('pause')
        self.wait(lambda: self.safety.get('state') == 'stop_unconfirmed' and self.safety.get('is_stopped') is False
                  and (self.safety.get('state_age_sec') or 0) > 0.5, 5)
        paused_clock = self.clock_sec
        if self.call('safety/reset').success:
            raise AssertionError('物理停止中の失効実測で解除成功')
        if self.call('safety/stop').success is not True or self.safety.get('is_stopped') is True:
            raise AssertionError('停止要求応答と実測停止の区別不成立')
        self.report['checks']['pause_invalidates_stop_confirmation'] = dict(self.safety)
        if self.clock_sec != paused_clock:
            raise AssertionError('pause中に/clockが進行')
        self.call('unpause')
        self.wait(self.is_physically_stopped, 45)
        self.set_stage('explicit_reset_reactivation')
        command_count = self.num_commands
        if not self.call('safety/reset').success:
            raise AssertionError('deactivateと新鮮な停止実測後の解除拒否')
        self.wait(lambda: self.is_stop_latched is False and self.safety.get('is_stop_latched') is False, 5)
        self.observe_hold('reset_without_activation_holds', 0.3, require_latch=False)
        if not self.switch(enable_controller=True).ok:
            raise AssertionError('解除後のcontroller activate失敗')
        self.observe_hold('reactivation_does_not_restore_old_goal', require_latch=False)
        if self.demo.get('state') != 'stopped' or self.num_commands != command_count:
            raise AssertionError('解除後にproducerまたは古い指令が自動復活')
        self.set_stage('explicit_new_command')
        self.hold_reference = None
        joint, position, delta = self.publish_step(0.04)
        self.wait(lambda: delta * (self.positions[joint] - position) > abs(delta) * 0.002
                  and abs(self.velocities[joint]) > 0.01, 30)
        self.report['checks']['new_command_motion'] = {'joint': joint, 'displacement_rad': self.positions[joint] - position}
        self.set_stage('final_stop')
        self.require_stop('final_physical_stop')
        self.observe_hold('final_stopped', 0.3, require_latch=True)

    def graph_names(self):
        return sorted(namespace.rstrip('/') + '/' + name for name, namespace in self.node.get_node_names_and_namespaces()
                      if name != self.node.get_name())

    def close(self):
        self.events.close()
        self.samples.close()
        self.node.destroy_node()


def run(args):
    """環境検査・所有プロセスの追跡・全終了経路の結果保存。"""
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / 'report.json').exists() or (args.output / 'ownership.json').exists():
        raise FileExistsError('既存試験結果への上書き禁止')
    begin = time.monotonic()
    report = {'result': 'failed', 'stage': 'preflight', 'motion_source': args.motion_source,
              'namespace': args.namespace, 'checks': {}, 'trajectory_interface': 'topic',
              'criteria': {'max_stop_velocity_rad_sec': 0.01, 'min_confirm_sim_sec': 0.25,
                           'max_state_age_sec': 0.5, 'max_hold_drift_rad': 0.01}}
    baseline = process_snapshot()
    launch = owned_launch(args.output, baseline)
    trial, rclpy = None, None
    cancel_state = {'is_cancelled': False}

    def cancel(_signum, _frame):
        cancel_state['is_cancelled'] = True

    handlers = {item: signal.signal(item, cancel) for item in (signal.SIGINT, signal.SIGTERM)}
    try:
        if os.environ.get('ROS_DOMAIN_ID') != '96' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise ValueError('ROS_DOMAIN_ID=96とROS_LOCALHOST_ONLY=1が必要')
        if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0 or args.timeout_sec > 540:
            raise ValueError('timeout-secは有限の正数かつ540秒以内が必要')
        if port_is_listening():
            raise RuntimeError('専用Gazebo port11369が既に使用中')
        import yaml
        import rclpy
        package = Path(__file__).resolve().parents[1]
        params = yaml.safe_load(args.params_file.read_text())['/**']['ros__parameters']
        if params['robot_name'] not in ('topo_dual_arm_max', 'topo_dual_arm_max_long') or args.namespace != 'sim_' + params['robot_name']:
            raise ValueError('機種名とsim_名前空間の不一致')
        demo = yaml.safe_load((package / 'config/dual_arm_gazebo_demo.yaml').read_text())
        if demo['dual_arm_gazebo_demo'].get('namespace') not in ('', args.namespace, None):
            raise ValueError('既定demoの名前空間が試験対象と不一致')
        demo['dual_arm_gazebo_demo'].update(enable_viewer=False, enable_gui=False, enable_auto_start=False)
        demo_path = args.output / 'demo_config.yaml'
        demo_path.write_text(yaml.safe_dump(demo, allow_unicode=True), encoding='utf-8')
        save_json(args.output / 'baseline_processes.json', [row for row in baseline.values()
                  if any(word in row['command'] for word in ('ros2', 'gzserver', 'gzclient', 'gazebo', '_ros2_daemon'))])
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        trial = stop_trial(args, report, launch, begin, cancel_state)
        for joint in element_tree.parse(params['urdf_path']).getroot().findall('joint'):
            if joint.get('type') == 'fixed':
                continue
            name = joint.get('name')
            trial.all_joint_names.append(name)
            if joint.find('mimic') is None:
                trial.command_joint_names.append(name)
                limit = joint.find('limit')
                # 連続回転関節の角度上下限なし。検証指令の対象はL_joint2のみ
                trial.limits[name] = ((-math.inf, math.inf) if joint.get('type') == 'continuous' else
                                      (float(limit.get('lower')), float(limit.get('upper'))))
        if 'L_joint2' not in trial.command_joint_names:
            raise ValueError('検証対象関節の不在')
        discovery_deadline = time.monotonic() + 1.5
        while time.monotonic() < discovery_deadline:
            trial.spin()
        report['baseline_ros_nodes'] = trial.graph_names()
        if report['baseline_ros_nodes']:
            raise RuntimeError('専用domain96に既存ノードあり。起動・サービス操作を中止')
        command = ['ros2', 'launch', 'gng_vlut_system', 'dual_arm_gng_lidar_demo.launch.py',
                   'gui:=false', 'enable_auto_start:=false', 'enable_dynamixel_leader:=false',
                   'enable_external_control:=false', 'gazebo_master_uri:=http://127.0.0.1:11369',
                   'params_file:=' + str(args.params_file.resolve()), 'demo_config:=' + str(demo_path.resolve())]
        report['command'] = command
        launch.start(command)
        trial.execute()
        report['result'] = 'passed'
    except BaseException as error:
        report.update(result='failed', error=f'{type(error).__name__}: {error}', traceback=traceback.format_exc())
    finally:
        try:
            report['cleanup'] = launch.cleanup()
            if trial is not None:
                report.update(last_safety=dict(trial.safety), last_demo=dict(trial.demo),
                              last_gng_status=dict(trial.gng_status), last_controller_sample=dict(trial.controller_sample),
                              last_positions=dict(trial.positions), num_joint_samples=trial.num_joint_samples)
                if launch.process is not None:
                    # 所有プロセス終了後のDDS discovery反映待ち。残存判定の条件は不変
                    graph_begin = time.monotonic()
                    graph_deadline = graph_begin + 15.0
                    while trial.graph_names() and time.monotonic() < graph_deadline:
                        rclpy.spin_once(trial.node, timeout_sec=0.1)
                    report['remaining_ros_nodes'] = trial.graph_names()
                    report['cleanup']['graph_settle_wall_sec'] = time.monotonic() - graph_begin
                    report['cleanup']['is_success'] &= not report['remaining_ros_nodes'] and not port_is_listening()
            if not report['cleanup']['is_success']:
                report.update(result='failed', cleanup_error='起動前状態への復元未確認')
        except BaseException as error:
            report.update(result='failed', cleanup_error=f'{type(error).__name__}: {error}')
        finally:
            if trial is not None:
                trial.close()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
            for item, handler in handlers.items():
                signal.signal(item, handler)
            report['wall_sec'] = time.monotonic() - begin
            save_json(args.output / 'report.json', report)
            save_json(args.output / 'metrics.json', {'is_success': int(report['result'] == 'passed'),
                'is_cleanup_success': int(report.get('cleanup', {}).get('is_success', False)),
                'wall_sec': report['wall_sec'], 'num_joint_samples': report.get('num_joint_samples', 0),
                'num_checks': len(report['checks'])})
            print(json.dumps({'result': report['result'], 'stage': report['stage'],
                              'report': str(args.output / 'report.json'), 'error': report.get('error'),
                              'cleanup_error': report.get('cleanup_error')}, ensure_ascii=False), flush=True)
    return int(report['result'] != 'passed')


def main():
    parser = argparse.ArgumentParser(description='Gazebo専用ソフトウェア停止の物理実測試験')
    parser.add_argument('--params-file', type=Path, required=True)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--motion-source', choices=('demo', 'direct'), default='demo')
    parser.add_argument('--timeout-sec', type=float, default=540)
    args = parser.parse_args()
    args.output = args.output.resolve()
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())
