#!/usr/bin/env python3
"""ID51・52専用の電流制限付き重力補償・粘性抵抗と終了時OFF要求。"""
import ctypes
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterDescriptor
from dynamixel_handler_msgs.msg import DynamixelExtra, DynamixelGoal, DynamixelStatus
from sensor_msgs.msg import JointState


def numeric_pair(values, name):
    """整数・小数配列の共通化。ID51・52の有限な数値2要素。"""
    if (not hasattr(values, '__len__') or len(values) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in values)):
        raise ValueError(f'{name}: ID51・52の有限な数値2要素が必要です')
    return [float(v) for v in values]


def damping_current(velocity, gain, max_current, gravity_current=0.):
    """補償と減衰の合成電流[mA]。2.69 mA刻み・絶対値切捨て。"""
    value = max(-max_current, min(max_current, gravity_current - gain * velocity))
    return math.copysign(math.floor(abs(value) / 2.69) * 2.69, value)


class neck_torque(Node):
    ids = (51, 52)

    def __init__(self):
        super().__init__('dynamixel_neck_torque')
        # 起動時の設定固定。パラメータ表示だけ変更された状態の防止
        fixed = ParameterDescriptor(read_only=True)
        numeric = ParameterDescriptor(read_only=True, dynamic_typing=True)
        self.driver = self.declare_parameter('driver_namespace', '/dynamixel', fixed).value.rstrip('/')
        allow_output = self.declare_parameter('allow_hardware_output', False, fixed).value
        def pair(name):
            return numeric_pair(self.declare_parameter(name, [0.0, 0.0], numeric).value, name)
        self.max_current = pair('max_current_ma')
        self.gains = pair('damping_gain')
        self.enable_gravity_compensation = self.declare_parameter('enable_gravity_compensation', False, fixed).value
        self.gravity_cos_ma = pair('gravity_cos_ma')
        self.gravity_sin_ma = pair('gravity_sin_ma')
        self.gravity_ramp_sec = self.declare_parameter('gravity_ramp_sec', 1.0, numeric).value
        if (isinstance(self.gravity_ramp_sec, bool) or not isinstance(self.gravity_ramp_sec, (int, float))
                or not math.isfinite(self.gravity_ramp_sec) or self.gravity_ramp_sec <= 0):
            raise ValueError('gravity_ramp_sec: 正の有限値が必要です')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('実機監視にはuse_sim_time=falseが必要です')
        if not allow_output:
            raise ValueError('出力未許可: YAMLのallow_hardware_outputを確認してください')
        if (len(self.max_current) != 2 or len(self.gains) != 2
                or any(not math.isfinite(v) or v < 2.69 for v in self.max_current)
                or any(not math.isfinite(v) or v <= 0 for v in self.gains)):
            raise ValueError('ID51・52それぞれのmax_current_maとdamping_gainの設定が必要です')
        if self.enable_gravity_compensation:
            amplitudes = [math.hypot(c, s) for c, s in zip(self.gravity_cos_ma, self.gravity_sin_ma)]
            if not any(amplitudes) or any(a > limit for a, limit in zip(amplitudes, self.max_current)):
                raise ValueError('重力補償係数が未設定、または補償振幅がmax_current_maを超えています')
        if not self.driver.startswith('/') or self.driver == '':
            raise ValueError('driver_namespaceには絶対名前空間が必要です')
        self.state = 'waiting'
        self.has_owned_output = False
        self.joints, self.status, self.extra, self.goals = {}, {}, {}, {}
        self.status_sec = self.extra_sec = self.goal_sec = -math.inf
        self.stage_sec = time.monotonic()
        self.goal_pub = self.create_publisher(DynamixelGoal, self.driver + '/command/goal', 1)
        self.torque_pub = self.create_publisher(DynamixelStatus, self.driver + '/command/status', 1)
        self.create_subscription(JointState, self.driver + '/fresh_joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(DynamixelStatus, self.driver + '/state/status', self.on_status, 1)
        self.create_subscription(DynamixelExtra, self.driver + '/state/extra', self.on_extra, 1)
        self.create_subscription(DynamixelGoal, self.driver + '/state/goal', self.on_goal, 1)

    def on_joints(self, message):
        if (message.header.frame_id != 'dynamixel_motor'
                or len(set(message.name)) != len(message.name)
                or not len(message.name) == len(message.position) == len(message.velocity)):
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if not 0 <= (time.time_ns() - stamp) * 1e-9 < .2:
            return
        for name, position, velocity in zip(message.name, message.position, message.velocity):
            if name not in ('51', '52') or not math.isfinite(position) or not math.isfinite(velocity):
                continue
            motor_id = int(name)
            if stamp > self.joints.get(motor_id, (-1, 0.))[0]:
                self.joints[motor_id] = (stamp, velocity, position)

    def read_rows(self, ids, *columns):
        if len(set(ids)) != len(ids) or any(len(column) != len(ids) for column in columns):
            return None
        rows = dict(zip(ids, zip(*columns)))
        return {motor_id: rows[motor_id] for motor_id in self.ids if motor_id in rows}

    def on_status(self, message):
        rows = self.read_rows(message.id_list, message.torque, message.error, message.ping, message.mode)
        if rows is not None:
            self.status, self.status_sec = rows, time.monotonic()

    def on_extra(self, message):
        rows = self.read_rows(message.id_list, message.model_number,
                              message.drive_mode.torque_on_by_goal_update, message.drive_mode.reverse_mode)
        if rows is not None:
            self.extra, self.extra_sec = rows, time.monotonic()

    def on_goal(self, message):
        rows = self.read_rows(message.id_list, message.current_ma)
        if rows is not None and all(math.isfinite(row[0]) for row in rows.values()):
            self.goals, self.goal_sec = rows, time.monotonic()

    def has_fresh_state(self):
        return all(motor_id in self.joints and
                   0 <= (time.time_ns() - self.joints[motor_id][0]) * 1e-9 < .2 for motor_id in self.ids)

    def has_feedback(self):
        now = time.monotonic()
        return (self.has_fresh_state() and now - self.status_sec < 1.5
                and now - self.extra_sec < 10. and now - self.goal_sec < 1.5
                and all(all(motor_id in rows for motor_id in self.ids)
                        for rows in (self.status, self.extra, self.goals)))

    def check_driver(self):
        if not self.has_feedback():
            raise ValueError('実測・モータ状態・電流目標の欠測または失効')
        for motor_id in self.ids:
            torque, error, ping, mode = self.status[motor_id]
            model, auto_torque, reverse = self.extra[motor_id]
            if error or not ping or mode != 'current':
                raise ValueError(f'ID{motor_id}: mode={mode}, error={error}, ping={ping}（必要: current・エラーなし・通信正常）')
            if model != 1020 or auto_torque or reverse:
                raise ValueError(f'ID{motor_id}: XM430-W350・Goal更新時自動ON無効・Reverse無効が必要')
        for topic, _ in self.get_topic_names_and_types():
            if (topic.startswith(self.driver + '/command/') or topic.startswith(self.driver + '/commands/')
                    or topic == self.driver + '/shortcut'):
                num_own = int(topic in (self.driver + '/command/goal', self.driver + '/command/status'))
                if self.count_publishers(topic) > num_own:
                    raise ValueError('別のDynamixel指令publisherの存在: ' + topic)
        if self.goal_pub.get_subscription_count() == 0 or self.torque_pub.get_subscription_count() == 0:
            raise ValueError('Dynamixel指令の受信先なし')

    def send_current(self, values):
        self.goal_pub.publish(DynamixelGoal(id_list=list(self.ids), current_ma=list(values)))

    def send_torque(self, enable_torque):
        self.torque_pub.publish(DynamixelStatus(id_list=list(self.ids), torque=[enable_torque, enable_torque]))

    def step(self):
        now = time.monotonic()
        if self.state == 'waiting' and not self.has_feedback():
            if now - self.stage_sec > 12.:
                raise ValueError('起動に必要な状態の受信待機時間超過')
            return
        self.check_driver()
        if self.state == 'waiting':
            if any(self.status[motor_id][0] for motor_id in self.ids):
                raise ValueError('開始前の51・52のトルクOFFが必要。既存ON状態の引継ぎなし')
            self.has_owned_output = True
            self.state, self.stage_sec = 'zero', now
            self.send_current([0., 0.])
            return
        if self.state == 'zero':
            if any(self.status[motor_id][0] for motor_id in self.ids):
                raise ValueError('準備中の予期しないトルクON')
            self.send_current([0., 0.])
            if self.goal_sec > self.stage_sec and all(abs(self.goals[motor_id][0]) < .01 for motor_id in self.ids):
                self.state, self.stage_sec = 'enable', now
                self.send_torque(True)
            elif now - self.stage_sec > 3.:
                raise ValueError('ゼロ電流目標の読返し待機時間超過')
            return
        if self.state == 'enable':
            if any(abs(self.goals[motor_id][0]) >= .01 for motor_id in self.ids):
                raise ValueError('トルクON準備中のゼロ電流目標の逸脱')
            self.send_current([0., 0.])
            if self.status_sec > self.stage_sec and all(self.status[motor_id][0] for motor_id in self.ids):
                self.state, self.stage_sec = 'running', now
                mode = '重力補償＋減衰' if self.enable_gravity_compensation else '減衰のみ（静止時0 mA）'
                self.get_logger().info(f'ID51・52: {mode} | Ctrl+C: トルクOFF')
            elif now - self.stage_sec > 3.:
                raise ValueError('トルクON報告の待機時間超過')
            return
        if self.state == 'running':
            if not all(self.status[motor_id][0] for motor_id in self.ids):
                raise ValueError('運転中のトルクOFF。自動再開なし')
            if any(abs(self.goals[motor_id][0]) > limit + .01 for motor_id, limit in zip(self.ids, self.max_current)):
                raise ValueError('電流目標の上限逸脱')
            self.send_current(self.control_currents(now))

    def control_currents(self, now):
        """モータ角度の重力項と減衰。目標姿勢・積分・自動増量なし。"""
        ramp = min(1., max(0., (now - self.stage_sec) / self.gravity_ramp_sec))
        values = []
        for idx, motor_id in enumerate(self.ids):
            _, velocity, position = self.joints[motor_id]
            gravity = 0.
            if self.enable_gravity_compensation:
                gravity = ramp * (self.gravity_cos_ma[idx] * math.cos(position)
                                  + self.gravity_sin_ma[idx] * math.sin(position))
            values.append(damping_current(velocity, self.gains[idx], self.max_current[idx], gravity))
        return values

    def stop_output(self):
        """通常・異常終了時のゼロ電流とOFFの再送。状態報告は機器キャッシュを含む。"""
        if not self.has_owned_output:
            return True
        self.state = 'stopping'
        start = time.monotonic()
        next_send = start
        while rclpy.ok() and time.monotonic() - start < 3.:
            if time.monotonic() >= next_send:
                next_send = time.monotonic() + .1
                try:
                    self.send_current([0., 0.])
                finally:
                    self.send_torque(False)
            rclpy.spin_once(self, timeout_sec=.05)
            if (time.monotonic() - start > .5 and self.status_sec > start
                    and self.has_fresh_state()
                    and all(motor_id in self.status and not self.status[motor_id][0] for motor_id in self.ids)):
                self.get_logger().info('ID51・52: OFF報告あり（キャッシュを含む）')
                return True
        self.get_logger().error('ID51・52: OFF未確認。首を支持し、独立した停止手段で確認してください')
        return False


def main():
    is_exiting = False

    def request_exit(*_args):
        nonlocal is_exiting
        is_exiting = True

    handlers = {kind: signal.signal(kind, request_exit) for kind in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    # Linuxの親終了通知。launch先行終了による子ノードの孤立・電流出力継続の防止
    parent_pid = os.getppid()
    if parent_pid == 1 or ctypes.CDLL(None, use_errno=True).prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        print('起動拒否: 親終了通知を設定できません', flush=True)
        return 1
    if os.getppid() != parent_pid:
        is_exiting = True
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node, result = None, 0
    try:
        node = neck_torque()
        next_tick = time.monotonic()
        while rclpy.ok() and not is_exiting:
            rclpy.spin_once(node, timeout_sec=.02)
            if not is_exiting and time.monotonic() >= next_tick:
                node.step()
                next_tick = time.monotonic() + .02
    except Exception as error:
        rclpy.logging.get_logger('dynamixel_neck_torque').error('首トルク停止: ' + str(error))
        result = 1
    finally:
        try:
            if node is not None and not node.stop_output():
                result = 2
        except Exception as error:
            print('ID51・52: OFF未確認。終了処理の通信異常: ' + str(error), flush=True)
            result = 2
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for kind, handler in handlers.items():
                signal.signal(kind, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
