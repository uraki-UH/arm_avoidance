#!/usr/bin/env python3
"""両腕14軸の重力補償付き手動操作。既定は実機出力なしの計算表示。"""
import json
import math
from pathlib import Path
import time

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from dynamixel_neck_torque import neck_torque, main
from gravity_compensation import gravity_model
from joint_command_model import joint_command_model, dynamixel_mapping


class hand_guiding(neck_torque):
    ids = tuple(range(31, 38)) + tuple(range(41, 48))
    node_name = 'dynamixel_hand_guiding'
    label = '両腕31〜37・41〜47'
    supported_models = (1020, 1120)
    control_label = '手動操作（重力補償＋減衰・位置復帰なし）'
    names = tuple(f'{side}_joint{idx}' for side in ('R', 'L') for idx in range(1, 8))

    def __init__(self):
        Node.__init__(self, self.node_name)
        fixed = ParameterDescriptor(read_only=True)
        numeric = ParameterDescriptor(read_only=True, dynamic_typing=True)
        def param(name, value, descriptor=fixed):
            return self.declare_parameter(name, value, descriptor).value
        def values(name, default):
            result = param(name, [default] * len(self.ids), numeric)
            if (len(result) != len(self.ids) or any(isinstance(v, bool) or not isinstance(v, (int, float))
                    or not math.isfinite(v) or v < 0 for v in result)):
                raise ValueError(name + ': 関節順の非負・有限14要素が必要です')
            return list(map(float, result))
        self.driver = param('driver_namespace', '/dynamixel').rstrip('/')
        self.allow_hardware_output = param('allow_hardware_output', False)
        self.has_verified_calibration = param('has_verified_calibration', False)
        self.max_current = values('max_current_ma', 0.)
        self.torque_nm_per_ma = values('torque_nm_per_ma', 0.)
        self.gains = values('damping_gain', .02)
        self.gravity_ramp_sec = param('gravity_ramp_sec', 2., numeric)
        self.max_velocity_th = param('max_velocity_th', .5, numeric)
        self.max_start_velocity_th = param('max_start_velocity_th', .02, numeric)
        if (self.get_parameter('use_sim_time').value or not self.driver.startswith('/') or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
                    for v in (self.gravity_ramp_sec, self.max_velocity_th, self.max_start_velocity_th))):
            raise ValueError('実時刻・絶対namespace・正の有限な立上げ時間／速度が必要です')
        self.model = joint_command_model(param('urdf_path', ''))
        config = yaml.safe_load(Path(param('mapping_file', '')).read_text())['/**']['ros__parameters']
        self.mapping = dynamixel_mapping(self.model, config)
        if (any(name not in self.mapping.entries for name in self.names) or
                tuple(self.mapping.entries[name][0] for name in self.names) != self.ids or
                any(abs(self.mapping.entries[name][1]) != 1. for name in self.names)):
            raise ValueError('両腕ID・関節名・±1の角度換算が必要です')
        self.gravity = gravity_model(self.get_parameter('urdf_path').value, self.names)
        self.enable_gravity_compensation = True
        if self.allow_hardware_output and (not self.has_verified_calibration or
                any(limit < 2.69 or factor <= 0 for limit, factor in zip(self.max_current, self.torque_nm_per_ma))):
            raise ValueError('実機出力拒否: 校正確認・関節別の電流上限とトルク換算係数が必要です')
        self.setup_io(self.allow_hardware_output)
        self.report_pub = self.create_publisher(String, '~/status', 1)
        self.report_sec = -math.inf
        self.get_logger().info('手動操作: ' + ('実機出力許可・状態確認待ち' if self.allow_hardware_output else 'プレビューのみ（トルク変更なし）'))

    def on_joints(self, message):
        # 無効・欠損情報による旧実測の再利用防止。首用の受信経路とは独立
        if (message.header.frame_id != 'dynamixel_motor' or len(set(message.name)) != len(message.name)
                or not len(message.name) == len(message.position) == len(message.velocity)):
            self.joints.clear()
            return
        for name, q, velocity in zip(message.name, message.position, message.velocity):
            if name in tuple(map(str, self.ids)) and not (math.isfinite(q) and math.isfinite(velocity)):
                self.joints.pop(int(name), None)
        super().on_joints(message)

    def support_currents(self, ramp=1.):
        positions = {}
        for name, motor_id in zip(self.names, self.ids):
            _, scale, offset = self.mapping.entries[name]
            _, velocity, q = self.joints[motor_id]
            q = (q + math.radians(offset)) * scale
            min_q, max_q, _ = self.model.bounds[name]
            if not min_q <= q <= max_q:
                raise ValueError(f'ID{motor_id}: 関節角{q:.3f} radがURDF範囲[{min_q:.3f}, {max_q:.3f}]外。原点・符号の校正確認が必要')
            if abs(velocity) > self.max_velocity_th:
                raise ValueError(f'ID{motor_id}: モータ角速度{velocity:.3f} rad/sの上限超過')
            positions[name] = q
        torques, _ = self.gravity.evaluate(positions)
        currents = None
        if all(self.torque_nm_per_ma):
            currents = []
            for idx, (name, motor_id) in enumerate(zip(self.names, self.ids)):
                _, scale, _ = self.mapping.entries[name]
                # モータ座標での支持トルクと粘性抵抗。位置目標・積分項なし
                torque = ramp * torques[name] * scale - self.gains[idx] * self.joints[motor_id][1]
                current = torque / self.torque_nm_per_ma[idx]
                if self.allow_hardware_output and abs(current) > self.max_current[idx]:
                    raise ValueError(f'ID{motor_id}: 支持電流の上限超過。補償不足のまま継続なし')
                currents.append(math.copysign(math.floor(abs(current) / 2.69) * 2.69, current))
        return torques, currents

    def check_driver(self):
        super().check_driver()
        self.support_currents()
        if self.state in ('waiting', 'zero', 'enable') and any(abs(self.joints[id_value][1]) > self.max_start_velocity_th for id_value in self.ids):
            raise ValueError('開始前の実測静止確認が必要です')

    def control_currents(self, now):
        ramp = min(1., max(0., (now - self.stage_sec) / self.gravity_ramp_sec))
        return self.support_currents(ramp)[1]

    def step(self):
        if self.allow_hardware_output:
            super().step()
        now = time.monotonic()
        if now - self.report_sec < .5:
            return
        self.report_sec = now
        report = {'state': self.state if self.allow_hardware_output else 'preview',
                  'has_fresh_state': self.has_fresh_state(), 'id_list': list(self.ids),
                  'allow_hardware_output': self.allow_hardware_output,
                  'has_verified_calibration': self.has_verified_calibration}
        if report['has_fresh_state']:
            try:
                report['gravity_nm'], report['current_ma'] = self.support_currents()
            except ValueError as error:
                report['error'] = str(error)
        self.report_pub.publish(String(data=json.dumps(report, ensure_ascii=False, allow_nan=False)))


if __name__ == '__main__':
    raise SystemExit(main(hand_guiding))
