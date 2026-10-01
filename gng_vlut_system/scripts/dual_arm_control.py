#!/usr/bin/env python3
"""Gazebo回避・追従と、明示許可時のUDP目標出力。"""

import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from controller_manager_msgs.srv import SwitchController
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml

from dual_arm_mode_model import mode_model
from dual_arm_udp_output import udp_output
from gazebo_stop_keyboard import status_label


class dual_arm_control(Node):
    def __init__(self):
        super().__init__('dual_arm_control')
        if not self.get_namespace().startswith('/sim_') or '/' in self.get_namespace()[1:]:
            raise ValueError('対応機種のGazebo専用名前空間が必要です')
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('Gazeboのuse_sim_timeが必要です')
        urdf_path = self.declare_parameter('urdf_path', '').value
        max_velocity = self.declare_parameter('max_leader_velocity', 0.3).value
        self.model = mode_model(urdf_path, max_joint_velocity=max_velocity)
        udp_config = self.declare_parameter('udp_config', '').value
        if self.get_namespace() not in ('/sim_topo_dual_arm_max', '/sim_topo_dual_arm_max_long') and udp_config:
            raise ValueError('この機種の実機UDP仕様は未検証です')
        allow_remote_udp = self.declare_parameter('allow_remote_udp', False).value
        self.udp = None
        if udp_config:
            config = yaml.safe_load(Path(udp_config).read_text())
            self.udp = udp_output(urdf_path, config['dual_arm_udp'], allow_remote_udp=allow_remote_udp)
        self.phase, self.detail = 'idle', '保持。A:回避 / L:追従 / Space:停止'
        self.target_mode = 'hold'
        self.safety, self.demo = {}, {}
        self.safety_sec = self.demo_sec = self.heartbeat_sec = -math.inf
        self.joint_stamp = self.mode_stamp = 0.0
        self.last_avoidance_stamp = -math.inf
        self.avoidance_sec = -math.inf
        self.expected_run_generation = 0
        self.future = self.stop_future = None
        self.deadline = self.settle_stamp = self.last_status_sec = 0.0
        self.has_unconfirmed_operation = False
        self.is_stop_required = False
        self.has_initialized_mode = False
        self.stop_deadline = 0.0
        self.command = self.create_publisher(JointTrajectory, 'dual_arm_controller/joint_trajectory', 1)
        self.status = self.create_publisher(String, 'control/status', 1)
        self.create_subscription(JointState, 'joint_states', self.on_state, qos_profile_sensor_data)
        self.create_subscription(JointState, 'leader_joint_states', self.on_leader, qos_profile_sensor_data)
        self.create_subscription(JointTrajectory, 'control/avoidance_trajectory', self.on_avoidance, 1)
        self.create_subscription(String, 'safety/status', self.on_safety, QoSProfile(depth=1))
        self.create_subscription(String, 'avoidance/status', self.on_demo, 1)
        self.create_subscription(Empty, 'control/heartbeat', self.on_heartbeat, 1)
        if self.udp is not None:
            self.create_subscription(JointTrajectoryControllerState, 'dual_arm_controller/state',
                                     self.on_controller_state, qos_profile_sensor_data)
        self.service_clients = {name: self.create_client(Trigger, name) for name in
                        ('avoidance/start', 'avoidance/stop', 'safety/stop', 'safety/reset')}
        self.service_clients['switch'] = self.create_client(SwitchController, 'controller_manager/switch_controller')
        self.create_service(SetBool, 'control/avoidance', self.on_avoidance_mode)
        self.create_service(SetBool, 'control/leader', self.on_leader_mode)
        self.create_service(SetBool, 'control/hardware', self.on_hardware)
        self.create_service(Trigger, 'control/stop', self.on_stop)
        self.create_service(Trigger, 'control/reset', self.on_reset)
        self.timer = self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def decode(message):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {}
        except (TypeError, ValueError):
            return {}

    def on_safety(self, message):
        self.safety, self.safety_sec = self.decode(message), time.monotonic()
        if self.safety.get('is_stop_latched') is True and not self.phase.startswith('reset_'):
            if self.phase != 'stopped':
                self.stop('Gazebo停止ラッチ。UDP出力も無効化')

    def on_demo(self, message):
        value = self.decode(message)
        generation = value.get('run_generation')
        if (type(generation) is int and generation < self.expected_run_generation
                and (self.model.mode == 'avoidance' or self.phase == 'switch_wait_running')):
            return
        if self.model.mode == 'avoidance' and (type(generation) is not int
                or generation != self.expected_run_generation
                or value.get('state') not in ('running', 'completed', 'stopped', 'idle', 'fault')):
            self.stop('回避診断の形式・開始世代不正')
            return
        self.demo, self.demo_sec = value, time.monotonic()

    def on_heartbeat(self, _message):
        self.heartbeat_sec = time.monotonic()

    def on_controller_state(self, message):
        # Gazebo controllerの補間済み目標。実測角の転送や終点への即時ジャンプなし
        if self.udp is None:
            return
        is_enabled = self.udp.is_enabled
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        try:
            if not -0.1 <= self.joint_stamp - stamp <= 0.5:
                raise ValueError('UDP元のcontroller目標時刻が失効しています')
            self.udp.update_target(message.joint_names, message.desired.positions, stamp, time.monotonic())
        except (ValueError, OSError) as error:
            if is_enabled:
                self.stop('UDP目標異常: ' + str(error))
            else:
                try:
                    self.udp.disable(str(error))
                except (ValueError, OSError):
                    pass

    def on_state(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.model.update_state(message.name, message.position, message.velocity, stamp, time.monotonic()):
            self.joint_stamp = stamp
        elif self.has_initialized_mode:
            self.stop('関節実測の形式・時刻不正')

    def on_leader(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if not self.model.update_leader(message.name, message.position, stamp, time.monotonic()) and self.model.mode == 'leader':
            self.stop('リーダー入力の形式・時刻不正')

    def is_ready(self, now):
        return (now - self.safety_sec <= 0.5 and self.safety.get('is_stop_latched') is False
                and self.safety.get('has_active_commands') is True and self.model.is_fresh(now)
                and now - self.heartbeat_sec <= 0.5 and not self.is_stop_required)

    def call(self, name, phase, request=None):
        client = self.service_clients[name]
        if not client.service_is_ready():
            raise RuntimeError('サービス未接続: ' + name)
        self.future = client.call_async(request if request is not None else client.srv_type.Request())
        self.phase, self.deadline = phase, time.monotonic() + 5.0

    def request_mode(self, name, enable_mode, response):
        target = name if enable_mode else 'hold'
        now = time.monotonic()
        if self.future is not None or self.phase != 'idle' or self.has_unconfirmed_operation:
            response.success, response.message = False, '処理中または停止ラッチ中です。状態を確認してください'
        elif not self.is_ready(now):
            response.success, response.message = False, 'Gazebo実測・停止状態・端末の新鮮な更新が必要です'
        elif enable_mode and self.model.mode != 'hold':
            response.success, response.message = False, '開始はホールドからのみ可能です。現在のモードをOFFにしてください'
        elif not enable_mode and self.model.mode not in ('hold', name):
            response.success, response.message = False, '別モードからの切替はできません。現在のモードをOFFにしてください'
        elif enable_mode and name == 'leader' and not self.model.has_fresh_leader(now):
            response.success, response.message = False, 'リーダーの新鮮な関節入力が必要です'
        elif not enable_mode and self.model.mode == 'hold':
            response.success, response.message = True, 'ホールドを継続しています'
        else:
            try:
                self.model.enter('hold', now)
                self.target_mode, self.settle_stamp = target, self.joint_stamp
                self.call('avoidance/stop', 'switch_stop_demo')
                response.success, response.message = True, '切替要求の受付。完了はcontrol/status'
                self.detail = '保持姿勢での切替待ち: ' + target
            except (ValueError, RuntimeError) as error:
                self.stop(str(error))
                response.success, response.message = False, str(error)
        return response

    def on_avoidance_mode(self, request, response):
        return self.request_mode('avoidance', request.data, response)

    def on_leader_mode(self, request, response):
        if request.data and self.phase == 'stopped':
            response = self.on_reset(None, response)
            if response.success:
                self.detail = 'Lによる停止解除の確認中。解除後はホールド、UDP出力OFF'
                response.message = '停止解除要求の受付。ホールド確認後、もう一度Lで追従開始'
            return response
        if not request.data and self.phase.startswith('reset_'):
            self.stop('追従OFFによる解除取消。停止ラッチの保持')
            response.success, response.message = True, self.detail
            return response
        return self.request_mode('leader', request.data, response)

    def on_hardware(self, request, response):
        if self.udp is None:
            response.success = not request.data
            response.message = ('実機送信は無効です。UDP設定未指定のためON拒否'
                                if request.data else '実機送信: OFF')
            return response
        now = time.monotonic()
        try:
            if not request.data:
                self.udp.disable('HによるUDP出力OFF。実機停止完了は未確認')
            else:
                if (self.phase != 'idle' or self.future is not None or self.has_unconfirmed_operation
                        or self.model.mode != 'hold'
                        or not self.is_ready(now) or not self.model.is_stationary(now)):
                    raise ValueError('UDP出力ONには保持モードと新鮮な静止確認が必要です')
                self.udp.poll(now)
                self.udp.enable(now)
            response.success = True
            response.message = 'UDP出力: ' + ('ON' if request.data else 'OFF。実機停止完了は未確認')
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
        return response

    def stop(self, detail):
        self.model.stop()
        self.phase, self.detail, self.is_stop_required = 'stopped', detail, True
        if self.udp is not None:
            try:
                self.udp.disable(detail)
            except (ValueError, OSError) as error:
                self.detail += ' / UDP停止送信の失敗: ' + str(error)
        # 送信済み非同期要求は完了まで追跡。停止後の遅延応答によるモード復帰なし

    def on_stop(self, _request, response):
        self.stop('ソフト停止要求。実停止確認はsafety/status')
        response.success, response.message = True, self.detail
        return response

    def on_reset(self, _request, response):
        now = time.monotonic()
        if (self.phase != 'stopped' or self.future is not None or self.stop_future is not None
                or self.has_unconfirmed_operation
                or status_label(self.safety, now - self.safety_sec) != '実測停止: 確認済み'
                or not self.model.is_stationary(now) or now - self.heartbeat_sec > 0.5):
            response.success, response.message = False, '新鮮な実測停止確認と未処理要求の完了が必要です'
            return response
        try:
            self.is_stop_required = False
            self.call('avoidance/stop', 'reset_stop_demo')
            response.success, response.message = True, '解除要求の受付。解除後も保持、モードの再選択が必要'
        except RuntimeError as error:
            self.stop(str(error))
            response.success, response.message = False, str(error)
        return response

    def switch_controller(self, enable_controller, phase):
        request = SwitchController.Request()
        if hasattr(request, 'activate_controllers'):
            request.activate_controllers = ['dual_arm_controller'] if enable_controller else []
            request.deactivate_controllers = [] if enable_controller else ['dual_arm_controller']
        else:
            request.start_controllers = ['dual_arm_controller'] if enable_controller else []
            request.stop_controllers = [] if enable_controller else ['dual_arm_controller']
        request.strictness, request.timeout.sec = 2, 3
        self.call('switch', phase, request)

    def advance(self, now):
        if self.future is not None:
            if not self.future.done():
                if now > self.deadline:
                    self.has_unconfirmed_operation = True
                    self.stop('操作の応答未確認。旧操作の完了を保証できないため再起動が必要')
                return
            future, self.future = self.future, None
            if self.phase == 'stopped':
                return
            response = future.result()
            if response is None or not getattr(response, 'success', getattr(response, 'ok', False)):
                raise RuntimeError('切替操作の拒否: ' + self.phase)
            if self.phase == 'switch_stop_demo':
                self.phase, self.deadline = 'switch_settle', now + 5
            elif self.phase == 'switch_start_demo':
                self.phase, self.deadline = 'switch_wait_running', now + 5
            elif self.phase == 'reset_stop_demo':
                self.switch_controller(False, 'reset_deactivate')
            elif self.phase == 'reset_deactivate':
                self.phase, self.deadline = 'reset_wait_stopped', now + 5
            elif self.phase == 'reset_clear':
                self.phase, self.deadline = 'reset_wait_clear', now + 5
            elif self.phase == 'reset_activate':
                self.phase, self.deadline = 'reset_wait_active', now + 5
        if self.phase == 'switch_settle':
            if self.model.is_stationary(now) and self.joint_stamp - self.settle_stamp >= 0.25:
                if self.target_mode == 'avoidance':
                    generation = self.demo.get('run_generation')
                    if now - self.demo_sec > 0.5 or type(generation) is not int or generation < 0:
                        raise RuntimeError('回避デモの新鮮な開始世代が必要です')
                    self.expected_run_generation = generation + 1
                    self.call('avoidance/start', 'switch_start_demo')
                else:
                    self.model.enter(self.target_mode, now)
                    self.phase, self.detail = 'idle', self.target_mode + '選択済み'
            elif now > self.deadline:
                raise RuntimeError('切替時の停止保持を確認できません')
        elif self.phase == 'switch_wait_running':
            if (now - self.demo_sec <= 0.5 and type(self.demo.get('run_generation')) is int
                    and self.demo.get('run_generation') == self.expected_run_generation):
                if self.demo.get('state') != 'running':
                    raise RuntimeError('回避開始後の実行状態が不正です')
                stamp = self.demo.get('run_start_stamp_sec')
                if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                        or stamp < self.settle_stamp or stamp > self.joint_stamp + 0.1):
                    raise RuntimeError('回避開始時刻が不正です')
                self.model.enter('avoidance', now)
                self.mode_stamp, self.last_avoidance_stamp = stamp, -math.inf
                self.avoidance_sec = now
                self.phase, self.detail = 'idle', '回避デモON'
            elif now > self.deadline:
                raise RuntimeError('新しい回避開始世代の状態未確認')
        elif self.phase == 'reset_wait_stopped':
            if (self.safety.get('has_active_commands') is False
                    and status_label(self.safety, now - self.safety_sec) == '実測停止: 確認済み'):
                self.call('safety/reset', 'reset_clear')
            elif now > self.deadline:
                raise RuntimeError('controller停止後の実測停止未確認')
        elif self.phase == 'reset_wait_clear':
            if now - self.safety_sec <= 0.5 and self.safety.get('is_stop_latched') is False:
                self.switch_controller(True, 'reset_activate')
            elif now > self.deadline:
                raise RuntimeError('停止ラッチ解除の状態未確認')
        elif self.phase == 'reset_wait_active':
            if self.is_ready(now) and self.model.is_stationary(now):
                self.model.enter('hold', now)
                self.has_initialized_mode = True
                self.phase, self.detail = 'idle', '解除済み・ホールド。Aで回避、Lで追従。UDP出力OFF'
            elif now > self.deadline:
                raise RuntimeError('controller再activationまたは実測静止の状態未確認')

    def on_avoidance(self, message):
        now = time.monotonic()
        if self.phase != 'idle' or self.model.mode != 'avoidance' or not self.is_ready(now):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if stamp < self.mode_stamp or stamp <= self.last_avoidance_stamp:
            return
        try:
            if not -0.1 <= self.joint_stamp - stamp <= 0.5:
                raise ValueError('回避軌道の時刻失効')
            if set(message.joint_names) != set(self.model.model.independent_names) or not message.points:
                raise ValueError('回避軌道の関節不足')
            previous_sec = -1.0
            for point in message.points:
                values = self.model.model.canonical_positions(message.joint_names, point.positions)
                for name, value in values.items():
                    min_position, max_position, _ = self.model.model.bounds[name]
                    if not min_position - 1e-6 <= value <= max_position + 1e-6:
                        raise ValueError('回避軌道の可動域逸脱')
                duration = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                if duration <= previous_sec or duration < 0:
                    raise ValueError('回避軌道の区間時刻不正')
                for values in (point.velocities, point.accelerations):
                    if values and (len(values) != len(message.joint_names) or not all(map(math.isfinite, values))):
                        raise ValueError('回避軌道の速度・加速度不正')
                previous_sec = duration
            self.last_avoidance_stamp, self.avoidance_sec = stamp, now
            # 生成時刻は旧モードの入力除外用。controllerへの区間実行は受信時起点
            message.header.stamp = Time()
            self.command.publish(message)
        except ValueError as error:
            self.stop(str(error))

    def publish_positions(self, positions):
        message = JointTrajectory()
        message.joint_names = list(self.model.model.independent_names)
        point = JointTrajectoryPoint()
        point.positions = [positions[name] for name in message.joint_names]
        point.time_from_start.nanosec = 50_000_000
        message.points = [point]
        self.command.publish(message)

    def tick(self):
        now = time.monotonic()
        try:
            if self.udp is not None:
                self.udp.poll(now)
            if math.isfinite(self.heartbeat_sec) and now - self.heartbeat_sec > 0.5:
                self.stop('操作端末の接続失効')
            if (self.has_initialized_mode and (self.phase == 'idle' or self.phase.startswith('switch_'))
                    and not self.is_ready(now)):
                self.stop('実測または停止状態の更新失効')
            if (self.phase.startswith('reset_') and (now - self.safety_sec > 0.5
                    or not self.model.is_fresh(now) or now - self.heartbeat_sec > 0.5)):
                self.stop('解除中の実測・停止状態・操作端末の更新失効')
            if not self.has_initialized_mode and self.phase == 'idle' and self.is_ready(now):
                self.model.enter('hold', now)
                self.has_initialized_mode = True
            self.advance(now)
            if self.model.mode == 'avoidance':
                if now - self.demo_sec > 0.5 or self.demo.get('state') == 'fault':
                    self.stop('回避状態の失効・異常: ' + str(self.demo.get('error', '')))
                elif self.demo.get('state') in ('completed', 'stopped', 'idle'):
                    self.model.enter('hold', now)
                    self.detail = '回避終了・保持'
                elif now - self.avoidance_sec > 0.5:
                    self.stop('回避指令の更新失効')
            if self.model.mode in ('hold', 'leader') and self.is_ready(now):
                values = self.model.command(now, self.get_clock().now().nanoseconds * 1e-9)
                if values is not None:
                    self.publish_positions(values)
            if self.udp is not None:
                self.udp.tick(now)
        except Exception as error:
            self.stop(str(error))
        if self.stop_future is not None and (self.stop_future.done() or now > self.stop_deadline):
            if not self.stop_future.done():
                self.has_unconfirmed_operation = True
                self.stop_future.cancel()
            self.stop_future = None
        if (self.is_stop_required and (self.safety.get('is_stop_latched') is not True or now - self.safety_sec > 0.5)
                and self.stop_future is None and self.service_clients['safety/stop'].service_is_ready()):
            self.stop_future = self.service_clients['safety/stop'].call_async(Trigger.Request())
            self.stop_deadline = now + 2
        if now - self.last_status_sec >= 0.1:
            udp_status = self.udp.status(now) if self.udp is not None else None
            value = {'mode': self.model.mode if self.phase in ('idle', 'stopped') else 'switching',
                     'phase': self.phase, 'enable_hardware_output': self.udp is not None and self.udp.is_enabled,
                     'udp': udp_status,
                     'detail': self.detail, 'has_fresh_leader': self.model.has_fresh_leader(now),
                     'is_ready': self.is_ready(now)}
            self.status.publish(String(data=json.dumps(value, ensure_ascii=False)))
            self.last_status_sec = now


def main():
    rclpy.init()
    node = None
    try:
        node = dual_arm_control()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                if node.udp is not None:
                    node.udp.close()
            except (ValueError, OSError) as error:
                node.get_logger().error('UDP終了時の停止パケット送信未確認: ' + str(error))
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
