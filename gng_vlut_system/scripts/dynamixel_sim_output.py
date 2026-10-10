#!/usr/bin/env python3
"""Gazebo・実機・共通追従目標からのDynamixel出力と実測停止ラッチ。"""
import json
import math
from pathlib import Path
import signal
import time
import xml.etree.ElementTree as et

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from control_msgs.msg import JointTrajectoryControllerState
from dynamixel_handler_msgs.msg import DynamixelGoal, DynamixelStatus, DynamixelExtra
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool, Trigger
import yaml

from joint_command_model import joint_command_model, dynamixel_mapping


class dynamixel_sim_output(Node):
    def __init__(self):
        super().__init__('dynamixel_sim_output')
        defaults = {'urdf_path': '', 'mapping_file': '',
            'joint_names': ['L_joint1', 'L_joint2', 'L_joint3', 'L_joint4', 'L_joint5', 'L_joint6', 'L_joint7'],
            'driver_namespace': '/dynamixel', 'sim_namespace': '/sim_ToPoDualArm',
            'allow_hardware_output': False, 'allow_sim_follow': False,
            'target_source': 'gazebo', 'leader_mapping_file': '',
            'target_topic': '/robot_follow/follower_target', 'target_session_topic': '/robot_follow/config',
            'allow_joint_state_follow': False,
            'leader_driver_namespace': '/dynamixel', 'leader_topic': '/leader/joint_states',
            'allow_leader_follow': False, 'enable_relative_follow': True,
            'enable_torque_off_on_exit': False,
            'publish_hz': 20., 'max_state_age_sec': .3, 'max_status_age_sec': 1.5,
            'max_command_age_sec': .3, 'max_heartbeat_age_sec': .4,
            'max_velocity': math.radians(2.), 'max_acceleration': math.radians(30.),
            'max_current_ma': 0.,
            'max_excursion': math.radians(5.), 'max_follow_dev_th': math.radians(3.),
            'max_start_dev_th': math.radians(1.), 'max_stop_velocity_th': math.radians(.5),
            'min_stop_duration_sec': .3, 'jog_step': math.radians(1.), 'max_prepare_sec': 3.}
        self.config = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        if self.config['target_source'] not in ('gazebo', 'leader', 'joint_state'):
            raise ValueError('target_sourceはgazebo・leader・joint_stateが必要です')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('実機監視にはuse_sim_time=falseが必要です')
        for name, default in defaults.items():
            if isinstance(default, float) and (not math.isfinite(self.config[name]) or
                    (self.config[name] < 0 if name == 'max_current_ma' else self.config[name] <= 0)):
                raise ValueError('有限の正数が必要です: '+name)
        self.model = joint_command_model(self.config['urdf_path'])
        mapping = yaml.safe_load(Path(self.config['mapping_file']).read_text())['/**']['ros__parameters']
        self.mapping = dynamixel_mapping(self.model, mapping)
        self.names = self.config['joint_names']
        if (not self.names or len(set(self.names)) != len(self.names) or
                any(name not in self.model.independent_names or name not in self.mapping.entries for name in self.names)):
            raise ValueError('出力関節にはID対応済みの独立関節が必要です')
        joint_types = {item.get('name'): item.get('type') for item in et.parse(self.config['urdf_path']).getroot().findall('joint')}
        for name in self.names:
            if joint_types[name] not in ('revolute', 'continuous'):
                raise ValueError('初期実機試験の対象は回転関節のみです')
            scale = abs(self.mapping.entries[name][1])
            if (math.degrees(self.config['max_velocity'])/scale < 1.374 or
                    math.degrees(self.config['max_acceleration'])/scale < 21.4577):
                raise ValueError('profile上限がXシリーズの最小非ゼロ値より小さいため起動不可')
        if self.config['target_source'] == 'joint_state' and any('gripper' in name for name in self.names):
            raise ValueError('グリッパーの実機追従は校正確認前の対象外です')
        self.ids = [self.mapping.entries[name][0] for name in self.names]
        self.motor_names = {str(motor_id): name for motor_id, name in zip(self.ids, self.names)}
        self.leader_measured, self.leader_velocity, self.leader_stamps = {}, {}, {}
        self.leader_anchor, self.follower_anchor = {}, {}
        self.target_session_id = ''
        self.target_role = 'none'
        if self.config['target_source'] == 'leader':
            leader_config = yaml.safe_load(Path(self.config['leader_mapping_file']).read_text())['/**']['ros__parameters']
            self.leader_mapping = dynamixel_mapping(self.model, leader_config)
            if any(name not in self.leader_mapping.entries for name in self.names):
                raise ValueError('選択関節のリーダーID対応が不足しています')
            if any('gripper' in name for name in self.names):
                raise ValueError('グリッパーの実機追従は校正確認前の対象外です')
            self.leader_ids = [self.leader_mapping.entries[name][0] for name in self.names]
            if (self.config['driver_namespace'].rstrip('/') == self.config['leader_driver_namespace'].rstrip('/')
                    and set(self.ids) & set(self.leader_ids)):
                raise ValueError('同一バス内のリーダー・フォロワーID重複です')
        self.measured, self.velocity, self.stamps = {}, {}, {}
        self.motor_status, self.drive_modes = {}, {}
        self.motor_series = {}
        self.motor_models = {}
        self.status_sec = self.extra_sec = self.heartbeat_sec = -math.inf
        self.sim_sec = self.sim_control_sec = self.sim_safety_sec = -math.inf
        self.last_sim_stamp = -1
        self.sim_target = {}
        self.sim_control, self.sim_safety = {}, {}
        self.mode, self.detail = 'off', '実機出力OFF'
        self.anchor, self.target, self.commanded = {}, {}, {}
        self.has_owned_output = False
        self.is_stop_latched = False
        self.is_torque_off_latched = False
        self.torque_off_sec = -math.inf
        self.low_since = None
        self.last_tick = time.monotonic()
        self.goal_echo, self.goal_sec = None, -math.inf
        self.prepare_sec = self.stop_sec = 0.
        self.stop_feedback_stamps = {}
        self.has_sent_torque_on = False
        self.has_pending_hold = False
        driver = self.config['driver_namespace'].rstrip('/')
        sim = self.config['sim_namespace'].rstrip('/')
        self.goal_pub = self.torque_pub = None
        if self.config['allow_hardware_output']:
            self.goal_pub = self.create_publisher(DynamixelGoal, driver+'/command/goal', 1)
            self.torque_pub = self.create_publisher(DynamixelStatus, driver+'/command/status', 1)
        self.status_pub = self.create_publisher(String, 'status', 1)
        self.create_subscription(JointState, driver+'/fresh_joint_states', self.on_measured, qos_profile_sensor_data)
        self.create_subscription(DynamixelStatus, driver+'/state/status', self.on_motor_status, 1)
        self.create_subscription(DynamixelExtra, driver+'/state/extra', self.on_extra, 1)
        self.create_subscription(DynamixelGoal, driver+'/state/goal', self.on_goal, 1)
        if self.config['target_source'] == 'joint_state':
            self.create_subscription(String, self.config['target_session_topic'], self.on_target_session,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(JointState, self.config['target_topic'], self.on_joint_target, qos_profile_sensor_data)
        elif self.config['target_source'] == 'leader':
            self.create_subscription(JointState, self.config['leader_topic'], self.on_leader, qos_profile_sensor_data)
        else:
            self.create_subscription(JointTrajectoryControllerState, sim+'/dual_arm_controller/controller_state', self.on_sim_target, qos_profile_sensor_data)
            self.create_subscription(String, sim+'/control/status', lambda msg: self.on_sim_status(msg, False), 1)
            self.create_subscription(String, sim+'/safety/status', lambda msg: self.on_sim_status(msg, True), 1)
            self.create_subscription(Bool, sim+'/safety/is_stop_latched', self.on_sim_stop, 1)
        self.create_subscription(Empty, 'heartbeat', self.on_heartbeat, 1)
        self.create_service(SetBool, 'enable', self.on_enable)
        self.create_service(SetBool, 'follow', self.on_follow)
        self.create_service(Trigger, 'stop', self.on_stop)
        self.create_service(Trigger, 'torque_off', self.on_torque_off)
        self.create_service(Trigger, 'reset', self.on_reset)
        self.create_service(Trigger, 'jog_positive', lambda req, res: self.on_jog(res, 1.))
        self.create_service(Trigger, 'jog_negative', lambda req, res: self.on_jog(res, -1.))
        self.create_timer(1./self.config['publish_hz'], self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_heartbeat(self, _message):
        self.heartbeat_sec = time.monotonic()

    def has_fresh_state(self):
        now = time.time_ns()
        return all(name in self.stamps and 0 <= (now-self.stamps[name])*1e-9 < self.config['max_state_age_sec'] for name in self.names)

    def has_fresh_leader(self):
        now = time.time_ns()
        return all(name in self.leader_stamps and
                   0 <= (now-self.leader_stamps[name])*1e-9 < self.config['max_command_age_sec'] for name in self.names)

    def on_leader(self, message):
        """共通入力で校正済みのリーダー実測。ID換算・表示トピック配信なし。"""
        if (message.header.frame_id != 'dynamixel_leader' or
                not len(message.name) == len(message.position) == len(message.velocity) or
                len(set(message.name)) != len(message.name)):
            self.leader_stamps.clear()
            return
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if not 0 <= (time.time_ns()-stamp)*1e-9 < self.config['max_command_age_sec']:
            return
        for name, position, velocity in zip(message.name, message.position, message.velocity):
            if name not in self.names or stamp <= self.leader_stamps.get(name, -1):
                continue
            if not all(map(math.isfinite, (position, velocity))):
                self.leader_stamps.pop(name, None)
                continue
            self.leader_measured[name], self.leader_velocity[name], self.leader_stamps[name] = position, velocity, stamp

    def on_target_session(self, message):
        """構成切替時の停止ラッチと旧世代目標の破棄。"""
        try:
            data = json.loads(message.data)
            session = data['session_id']
            profile = data['profiles'][data['profile']]
            if not isinstance(session, str) or not session or profile['follower_source'] not in ('none', 'r', 's') or profile['follow_mode'] not in ('relative', 'absolute'):
                raise ValueError('共通追従構成の形式不正')
            if session == self.target_session_id:
                return
            if self.target_session_id or self.has_owned_output:
                self.stop('接続構成の変更。停止解除・追従の明示再開が必要')
            self.target_session_id, self.target_role = session, profile['follower_source']
            self.config['enable_relative_follow'] = profile['follow_mode'] == 'relative'
            self.leader_stamps.clear()
            self.leader_anchor, self.follower_anchor = {}, {}
        except (ValueError, KeyError, TypeError):
            self.stop('共通追従構成の受信不正')
            self.target_session_id, self.target_role = '', 'none'
            self.leader_stamps.clear()

    def on_joint_target(self, message):
        """構成世代・単一配信元・実測時刻付きの共通目標。"""
        if (not self.target_session_id or self.target_role == 'none' or
                message.header.frame_id != 'robot_follow_target:'+self.target_session_id or
                self.count_publishers(self.config['target_topic']) != 1 or
                self.count_publishers(self.config['target_session_topic']) != 1):
            self.leader_stamps.clear()
            return
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if (not 0 <= (time.time_ns()-stamp)*1e-9 < self.config['max_command_age_sec'] or
                len(message.name) != len(message.position) or len(set(message.name)) != len(message.name)):
            return
        pose = dict(zip(message.name, message.position))
        if any(name not in pose or not math.isfinite(pose[name]) for name in self.names):
            self.leader_stamps.clear()
            return
        if any(stamp <= self.leader_stamps.get(name, -1) for name in self.names):
            return
        self.leader_measured = {name: pose[name] for name in self.names}
        self.leader_stamps = dict.fromkeys(self.names, stamp)

    def leader_target(self):
        if not self.has_fresh_leader():
            raise ValueError('リーダー実測の未受信・欠測・失効')
        if self.config['enable_relative_follow']:
            return {name: self.follower_anchor[name]+self.leader_measured[name]-self.leader_anchor[name]
                    for name in self.names}
        return {name: self.leader_measured[name] for name in self.names}

    def on_measured(self, message):
        if (message.header.frame_id != 'dynamixel_motor' or
                not len(message.name) == len(message.position) == len(message.velocity) or
                len(set(message.name)) != len(message.name)):
            return
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if not 0 <= (time.time_ns()-stamp)*1e-9 < self.config['max_state_age_sec']:
            return
        has_sample = False
        has_position_motion = False
        for motor_name, position, velocity in zip(message.name, message.position, message.velocity):
            name = self.motor_names.get(motor_name)
            if name is None or stamp <= self.stamps.get(name, -1) or not all(map(math.isfinite, (position, velocity))):
                continue
            _, scale, offset = self.mapping.entries[name]
            value = (position+math.radians(offset))*scale
            if name in self.stamps:
                elapsed_sec = (stamp-self.stamps[name])*1e-9
                has_position_motion |= abs(value-self.measured[name])/elapsed_sec > self.config['max_stop_velocity_th']
            self.measured[name] = value
            self.velocity[name] = velocity*scale
            self.stamps[name] = stamp
            has_sample = True
        if not has_sample:
            return
        now = time.monotonic()
        if has_position_motion or not self.has_fresh_state() or any(abs(self.velocity[name]) > self.config['max_stop_velocity_th'] for name in self.names):
            self.low_since = None
        elif self.low_since is None:
            self.low_since = now

    def on_motor_status(self, message):
        if not len(message.id_list) == len(message.torque) == len(message.error) == len(message.ping) == len(message.mode):
            return
        self.motor_status = {motor_id: (torque, error, ping, mode) for motor_id, torque, error, ping, mode in
                             zip(message.id_list, message.torque, message.error, message.ping, message.mode)}
        self.status_sec = time.monotonic()

    def on_extra(self, message):
        if not len(message.id_list) == len(message.model) == len(message.drive_mode.profile_configuration) == len(message.drive_mode.torque_on_by_goal_update):
            return
        self.motor_series = dict(zip(message.id_list, message.model))
        self.motor_models = dict(zip(message.id_list, message.model_number))
        self.drive_modes = dict(zip(message.id_list, zip(message.drive_mode.profile_configuration,
                                                        message.drive_mode.torque_on_by_goal_update)))
        self.extra_sec = time.monotonic()

    def on_goal(self, message):
        if len(message.id_list) == len(message.position_deg) == len(message.profile_vel_deg_s) == len(message.profile_acc_deg_ss) == len(message.current_ma):
            self.goal_echo = dict(zip(message.id_list, zip(message.position_deg, message.profile_vel_deg_s, message.profile_acc_deg_ss, message.current_ma)))
            self.goal_sec = time.monotonic()

    def on_sim_status(self, message, is_safety):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
        except ValueError:
            return
        if is_safety:
            self.sim_safety, self.sim_safety_sec = value, time.monotonic()
        else:
            self.sim_control, self.sim_control_sec = value, time.monotonic()

    def on_sim_stop(self, message):
        if message.data and self.has_owned_output:
            self.stop('Gazebo停止ラッチ')

    def on_sim_target(self, message):
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if stamp <= self.last_sim_stamp:
            return
        self.last_sim_stamp = stamp
        if len(message.joint_names) != len(message.reference.positions) or len(set(message.joint_names)) != len(message.joint_names):
            return
        values = dict(zip(message.joint_names, message.reference.positions))
        if not all(name in values and math.isfinite(values[name]) for name in self.names):
            return
        self.sim_target = {name: values[name] for name in self.names}
        self.sim_sec = time.monotonic()

    def is_stationary(self):
        return self.has_fresh_state() and self.low_since is not None and time.monotonic()-self.low_since >= self.config['min_stop_duration_sec']

    def check_ready(self):
        now = time.monotonic()
        if self.config['target_source'] == 'joint_state' and (not self.target_session_id or self.target_role == 'none'):
            raise ValueError('実機フォロワーの入力元が未選択')
        if self.config['target_source'] == 'joint_state':
            driver = self.config['driver_namespace'].rstrip('/')
            if any(self.count_publishers(driver+topic) > 1 for topic in ('/command/goal', '/command/status')):
                raise ValueError('実機出力の配信元重複')
        if not self.has_fresh_state():
            raise ValueError('実測失効。fresh_joint_states対応ドライバと位置・速度の同時読取りが必要です')
        if now-self.heartbeat_sec > self.config['max_heartbeat_age_sec']:
            raise ValueError('操作端末のheartbeat失効')
        if now-self.status_sec > self.config['max_status_age_sec']:
            raise ValueError('モータ状態の失効')
        for name, motor_id in zip(self.names, self.ids):
            status = self.motor_status.get(motor_id)
            if not status or status[1] or not status[2] or status[3] != 'cur_position':
                if self.has_owned_output and status and status[3] != 'cur_position':
                    self.on_torque_off(None, Trigger.Response())
                    self.detail = f'ID {motor_id}: 電流制御モードの逸脱・トルクOFF'
                raise ValueError(f'ID {motor_id}: 通信・エラー・電流ベース位置制御モードの確認が必要です')
            if not self.model.bounds[name][0] <= self.measured[name] <= self.model.bounds[name][1]:
                raise ValueError(f'{name}: 実測角がURDF範囲外。対応表の符号・原点を確認してください')

    def check_target(self, values):
        for name, value in values.items():
            if (not math.isfinite(value) or not self.model.bounds[name][0] <= value <= self.model.bounds[name][1]
                    or abs(value-self.anchor[name]) > self.config['max_excursion']):
                raise ValueError('試験範囲・URDF可動域を超える目標: '+name)

    def send_goal(self, values):
        if self.goal_pub is None or self.is_torque_off_latched:
            return
        ids, angles = self.mapping.convert(values)
        scales = {self.mapping.entries[name][0]: abs(self.mapping.entries[name][1]) for name in values}
        def profile(value, unit):
            # Xシリーズの分解能とドライバの切上げ換算に合わせた、上限内の非ゼロ値
            count = math.floor(value/unit)
            if count < 1:
                raise ValueError('profile上限がXシリーズの最小非ゼロ値より小さいため送信不可')
            return count*unit*(1.-1e-10)
        self.goal_pub.publish(DynamixelGoal(id_list=ids, position_deg=angles,
            current_ma=[profile(self.config['max_current_ma'], 2.69)]*len(ids),
            profile_vel_deg_s=[profile(math.degrees(self.config['max_velocity'])/scales[motor_id], 1.374) for motor_id in ids],
            profile_acc_deg_ss=[profile(math.degrees(self.config['max_acceleration'])/scales[motor_id], 21.4577) for motor_id in ids]))

    def stop(self, detail):
        if self.is_torque_off_latched:
            return
        if not self.is_stop_latched:
            self.stop_sec = time.monotonic()
            self.stop_feedback_stamps = dict(self.stamps)
            self.low_since = None
            self.has_pending_hold = not self.has_fresh_state()
            if self.has_owned_output and self.has_fresh_state():
                self.target = dict(self.measured)
                self.send_goal(self.target)
        self.is_stop_latched = True
        self.mode, self.detail = 'stopped', detail

    def on_stop(self, _request, response):
        self.stop('キーボード／サービス停止要求')
        response.success, response.message = True, '停止ラッチ受付。実測停止はstatusで確認'
        return response

    def has_torque_off_report(self):
        # キャッシュを含む既存statusによるOFF報告。実測静止・独立した電源遮断とは別判定
        return (self.is_torque_off_latched and self.status_sec > self.torque_off_sec and
                time.monotonic()-self.status_sec < self.config['max_status_age_sec'] and
                all(motor_id in self.motor_status and not self.motor_status[motor_id][0]
                    and self.motor_status[motor_id][2] for motor_id in self.ids))

    def send_torque_off(self):
        if self.torque_pub is not None:
            self.torque_pub.publish(DynamixelStatus(id_list=self.ids, torque=[False]*len(self.ids)))

    def on_torque_off(self, _request, response):
        if not self.is_torque_off_latched:
            self.torque_off_sec = time.monotonic()
            self.stop_feedback_stamps = dict(self.stamps)
            self.low_since = None
        self.is_torque_off_latched = self.is_stop_latched = True
        self.mode = 'torque_off'
        self.target, self.commanded = {}, {}
        self.has_pending_hold = False
        self.detail = ('トルクOFFラッチ・位置指令停止。腕の支持とOFF報告を確認'
                       if self.torque_pub is not None else 'トルクOFF未送信: allow_hardware_output=false')
        self.send_torque_off()
        response.success, response.message = self.torque_pub is not None, self.detail
        return response

    def on_enable(self, request, response):
        if not request.data:
            return self.on_stop(request, response)
        try:
            if not self.config['allow_hardware_output']:
                raise ValueError('監視モード。allow_hardware_output=false')
            if self.mode != 'off' or self.is_stop_latched:
                raise ValueError('有効化にはOFF・停止ラッチ解除が必要です')
            if self.config['max_current_ma'] < 2.69:
                raise ValueError('許容電流max_current_maの明示設定が必要です。未設定での有効化は不可')
            self.check_ready()
            if not self.is_stationary():
                raise ValueError('実測静止待ち')
            if time.monotonic()-self.extra_sec > 10.:
                raise ValueError('drive mode情報の失効')
            for motor_id in self.ids:
                if self.motor_models.get(motor_id) not in (1020, 1120):
                    raise ValueError('電流制限の換算確認済み機種はXM430-W350・XM540-W270のみです')
                if self.motor_series.get(motor_id) != 'X' or self.drive_modes.get(motor_id) != ('velocity_based', False):
                    raise ValueError('速度基準profile・goalによる自動torque ON無効が必要です')
            if self.count_publishers(self.config['driver_namespace']+'/command/goal') != 1:
                raise ValueError('別のDynamixel指令publisherが存在します')
            self.anchor = self.target = dict(self.measured)
            self.commanded = dict(self.measured)
            self.mode, self.detail = 'preparing', '実測保持目標と低速profileの読返し待ち'
            self.has_sent_torque_on = False
            self.prepare_sec = time.monotonic()
            self.send_goal(self.target)
            self.has_owned_output = True
            response.success, response.message = True, self.detail
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def on_reset(self, _request, response):
        response.success = (self.mode in ('stopped', 'torque_off') and self.is_stationary() and
                            (not self.is_torque_off_latched or self.has_torque_off_report()))
        if response.success:
            self.mode, self.detail, self.is_stop_latched = 'off', '解除済み・出力OFF。Hで再有効化', False
            self.has_owned_output = False
            self.is_torque_off_latched = False
        response.message = self.detail if response.success else '新鮮な実測停止の確認待ち'
        return response

    def on_jog(self, response, direction):
        try:
            self.check_ready()
            if self.mode != 'hold' or len(self.names) != 1 or not self.is_stationary():
                raise ValueError('単関節・保持・実測静止が必要です')
            target = {self.names[0]: self.measured[self.names[0]]+direction*self.config['jog_step']}
            self.check_target(target)
            self.target = target
            self.mode, self.detail = 'jog', '単関節の小動作'
            response.success, response.message = True, self.detail
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def on_follow(self, request, response):
        is_leader = self.config.get('target_source', 'gazebo') in ('leader', 'joint_state')
        source = ('共通追従目標' if self.config['target_source'] == 'joint_state' else 'リーダー') if is_leader else 'Gazebo'
        try:
            if not request.data:
                if self.mode != 'follow':
                    raise ValueError(source+'追従中ではありません')
                self.target, self.mode, self.detail = dict(self.measured), 'hold', source+'追従解除・現在姿勢保持'
            else:
                self.check_ready()
                permission = ('allow_joint_state_follow' if self.config['target_source'] == 'joint_state' else 'allow_leader_follow') if is_leader else 'allow_sim_follow'
                if not self.config[permission] or self.mode != 'hold' or not self.is_stationary():
                    raise ValueError('追従許可・保持・実測静止が必要です')
                if is_leader:
                    if not self.has_fresh_leader():
                        raise ValueError('リーダー実測の未受信・欠測・失効')
                    self.leader_anchor, self.follower_anchor = dict(self.leader_measured), dict(self.measured)
                    target = self.leader_target()
                else:
                    self.check_sim()
                    target = self.sim_target
                if any(abs(target[name]-self.measured[name]) > self.config['max_start_dev_th'] for name in self.names):
                    raise ValueError(source+'目標と実機の開始姿勢差が大きすぎます')
                self.commanded = dict(self.measured)
                self.mode, self.detail = 'follow', source+'目標への追従'
            response.success, response.message = True, self.mode
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def check_sim(self):
        now = time.monotonic()
        if (any(now-value > self.config['max_command_age_sec'] for value in
                (self.sim_sec, self.sim_control_sec, self.sim_safety_sec)) or
                self.sim_control.get('mode') not in ('hold', 'avoidance', 'leader') or
                self.sim_safety.get('is_stop_latched') is not False):
            raise ValueError('Gazebo目標・状態の失効または停止')

    def prepare_output(self, now):
        """有効化準備中の保持目標・profile読返しとトルクONの確認。"""
        if now-self.prepare_sec > self.config['max_prepare_sec']:
            raise ValueError('保持目標・torque ONの読返し期限超過')
        if any(abs(self.target[name]-self.measured[name]) > self.config['max_start_dev_th'] for name in self.names):
            raise ValueError('有効化準備中の実測姿勢変化')
        ids, angles = self.mapping.convert(self.target)
        has_echo = self.goal_sec > self.prepare_sec and self.goal_echo and all(
            motor_id in self.goal_echo and abs(self.goal_echo[motor_id][0]-angle) < .1 and
            0 < self.goal_echo[motor_id][1] <= math.degrees(self.config['max_velocity'])/abs(self.mapping.entries[name][1])+.01 and
            0 < self.goal_echo[motor_id][2] <= math.degrees(self.config['max_acceleration'])/abs(self.mapping.entries[name][1])+.01 and
            0 < self.goal_echo[motor_id][3] <= self.config['max_current_ma']+.001
            for motor_id, angle in zip(ids, angles) for name in self.names if self.mapping.entries[name][0] == motor_id)
        if has_echo and not self.has_sent_torque_on:
            self.torque_pub.publish(DynamixelStatus(id_list=self.ids, torque=[True]*len(self.ids)))
            self.has_sent_torque_on = True
        elif has_echo and self.status_sec > self.prepare_sec and all(self.motor_status[motor_id][0] for motor_id in self.ids):
            self.mode, self.detail = 'hold', '実機保持。J/K: 小動作 / F: 追従'
        self.send_goal(self.target)

    def advance_output(self, now, duration):
        """電流・目標・実測偏差の確認後の速度制限つき出力。"""
        if (now-self.goal_sec > self.config['max_status_age_sec'] or not self.goal_echo or
                any(motor_id not in self.goal_echo or not 0 < self.goal_echo[motor_id][3] <= self.config['max_current_ma']+.001
                    for motor_id in self.ids)):
            self.on_torque_off(None, Trigger.Response())
            self.detail = '電流目標の読返し失効・上限逸脱によるトルクOFF'
            raise ValueError('電流目標の読返し失効・上限逸脱')
        if any(not self.motor_status[motor_id][0] for motor_id in self.ids):
            raise ValueError('実機トルクOFF')
        if self.mode == 'follow':
            if self.config.get('target_source', 'gazebo') in ('leader', 'joint_state'):
                self.target = self.leader_target()
            else:
                self.check_sim()
                self.target = dict(self.sim_target)
        self.check_target(self.target)
        if any(abs(self.commanded[name]-self.measured[name]) > self.config['max_follow_dev_th'] for name in self.names):
            raise ValueError('実機追従偏差の超過')
        self.commanded = self.model.step(self.commanded, self.target, duration, self.config['max_velocity'])
        self.send_goal(self.commanded)
        if self.mode == 'jog' and self.is_stationary() and all(abs(self.target[name]-self.measured[name]) < math.radians(.15) for name in self.names):
            self.mode, self.detail = 'hold', '小動作完了・実測静止'

    def hold_stopped_output(self):
        """停止後の新実測による保持目標の確定と再送。旧到達目標への再開なし。"""
        if not self.target or any(self.stamps.get(name, -1) <= self.stop_feedback_stamps.get(name, -1) for name in self.names):
            return
        if self.has_pending_hold:
            self.target = dict(self.measured)
            self.has_pending_hold = False
        self.send_goal(self.target)

    def publish_status(self):
        """出力状態と独立した実測停止確認の配信。"""
        is_stopped = self.is_stop_latched and self.is_stationary() and all(
            self.stamps.get(name, -1) > self.stop_feedback_stamps.get(name, -1) for name in self.names)
        self.status_pub.publish(String(data=json.dumps({'mode': self.mode, 'detail': self.detail,
            'allow_hardware_output': self.config['allow_hardware_output'], 'joint_names': self.names, 'ids': self.ids,
            'is_stop_latched': self.is_stop_latched, 'is_stopped': is_stopped,
            'is_torque_off_latched': self.is_torque_off_latched,
            'has_torque_off_report': self.has_torque_off_report(),
            'is_stationary': self.is_stationary(),
            'has_fresh_state': self.has_fresh_state(), 'positions': self.measured,
            'velocities': self.velocity, 'commanded': self.commanded,
            'has_owned_output': self.has_owned_output,
            'target_source': self.config['target_source'], 'target_role': self.target_role, 'target_session_id': self.target_session_id,
            **({'has_fresh_target': self.has_fresh_leader(), 'target_positions': self.leader_measured}
               if self.config['target_source'] == 'joint_state' else {}),
            **({'leader_ids': self.leader_ids, 'has_fresh_leader': self.has_fresh_leader(),
                'leader_positions': self.leader_measured, 'enable_relative_follow': self.config['enable_relative_follow']}
               if self.config.get('target_source', 'gazebo') == 'leader' else {})}, ensure_ascii=False)))

    def tick(self):
        now = time.monotonic()
        duration = min(now-self.last_tick, 2./self.config['publish_hz'])
        self.last_tick = now
        if not self.has_fresh_state():
            self.low_since = None
        try:
            if self.is_torque_off_latched:
                self.send_torque_off()
            elif self.mode in ('preparing', 'hold', 'jog', 'follow'):
                self.check_ready()
                if self.count_publishers(self.config['driver_namespace']+'/command/goal') != 1:
                    raise ValueError('実機指令publisherの競合')
                if self.mode == 'preparing':
                    self.prepare_output(now)
                else:
                    self.advance_output(now, duration)
            elif self.mode == 'stopped' and self.has_owned_output and self.has_fresh_state():
                self.hold_stopped_output()
        except ValueError as error:
            self.stop(str(error))
        self.publish_status()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exit_state = {'is_requested': False}
    handlers = {kind: signal.signal(kind, lambda *_: exit_state.update(is_requested=True))
                for kind in (signal.SIGINT, signal.SIGTERM)}
    node = None
    try:
        node = dynamixel_sim_output()
        while rclpy.ok() and not exit_state['is_requested']:
            rclpy.spin_once(node, timeout_sec=.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                if node.config['enable_torque_off_on_exit'] and (node.has_owned_output or node.is_torque_off_latched):
                    node.on_torque_off(None, Trigger.Response())
                    deadline = time.monotonic()+1.
                    while rclpy.ok() and time.monotonic() < deadline and not node.has_torque_off_report():
                        node.send_torque_off()
                        rclpy.spin_once(node, timeout_sec=.05)
                    if not node.has_torque_off_report():
                        node.get_logger().warning('終了時のトルクOFF未確認。独立停止手段で確認してください')
                else:
                    node.stop('出力ノード終了')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for kind, handler in handlers.items():
            signal.signal(kind, handler)


if __name__ == '__main__':
    main()
